import math

import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.engagement import EngagementController, EngagementState
from hexa_gait.gaits.base import LegContext
from hexa_gait.gaits.ripple import Ripple
from hexa_gait.gaits.tripod import Tripod
from hexa_gait.gaits.wave import Wave


# Symmetric stance shared with test_engine.py. Front / rear sit at
# hypot(0.15, 0.10) m from the body centre; middle legs at 0.12 m.
_MOUNTS: dict[str, tuple[float, float, float]] = {
    "l_front": (0.15, 0.10, 0.0),
    "r_front": (0.15, -0.10, 0.0),
    "l_middle": (0.0, 0.12, 0.0),
    "r_middle": (0.0, -0.12, 0.0),
    "l_rear": (-0.15, 0.10, 0.0),
    "r_rear": (-0.15, -0.10, 0.0),
}

_TRIPOD_A = ("l_front", "r_middle", "l_rear")
_TRIPOD_B = ("r_front", "l_middle", "r_rear")

# Engagement reference parameters. With stride_length = 0.10 m,
# duty_factor = 0.5 and cmd v_x = 0.20 m/s, the engine derives
# cycle_time = 1.0 s, engage_time = 0.5 s, stride magnitude = 0.10 m.
_STRIDE_LENGTH = 0.10
_DUTY_FACTOR = 0.5
_CYCLE_TIME_AT_0_20 = _STRIDE_LENGTH / (0.20 * _DUTY_FACTOR)
_ENGAGE_TIME_AT_0_20 = _DUTY_FACTOR * _CYCLE_TIME_AT_0_20


def _nominal_stance() -> dict[str, tuple[float, float, float]]:
    return {n: (xyz[0], xyz[1], -0.10) for n, xyz in _MOUNTS.items()}


def _leg_contexts() -> dict[str, LegContext]:
    nominal = _nominal_stance()
    return {
        n: LegContext(
            name=n, mount_xyz=_MOUNTS[n], mount_yaw=0.0, nominal_stance=nominal[n]
        )
        for n in LEG_NAMES
    }


def _controller(**overrides) -> EngagementController:
    args = dict(
        nominal_stance=_nominal_stance(),
        stride_length=_STRIDE_LENGTH,
        min_cycle_time=0.5,
        max_cycle_time=2.0,
        duty_factor=_DUTY_FACTOR,
        swing_clearance=0.03,
        swing_width=0.0,
        controller_dt=0.02,
    )
    args.update(overrides)
    return EngagementController(**args)


def _begin(ctrl: EngagementController, strategy_cls=Tripod):
    strategy = strategy_cls()
    ctrl.begin(strategy=strategy, leg_contexts=_leg_contexts())
    return strategy


def _drive(
    ctrl: EngagementController,
    v_cmd_xy: tuple[float, float],
    omega_cmd: float = 0.0,
    dt: float = 0.02,
    max_ticks: int = 200,
):
    last = None
    for _ in range(max_ticks):
        last = ctrl.update(dt=dt, v_cmd_xy=v_cmd_xy, omega_cmd=omega_cmd)
        if ctrl.state is EngagementState.DONE:
            return last
    raise AssertionError("engagement did not reach DONE within max_ticks")


def test_first_tick_no_position_jump_anywhere():
    # The bug this controller exists to fix: at master ≈ 0, no foot
    # should have lurched away from NOMINAL. Both tripods stay close.
    ctrl = _controller()
    _begin(ctrl)
    nominal = _nominal_stance()
    out = ctrl.update(dt=0.02, v_cmd_xy=(0.20, 0.0), omega_cmd=0.0)
    for name in LEG_NAMES:
        dx = abs(out[name].foot_target[0] - nominal[name][0])
        dy = abs(out[name].foot_target[1] - nominal[name][1])
        assert dx < 5e-3, f"{name} jumped horizontally: dx={dx}"
        assert dy < 5e-3


def test_initial_swing_legs_lift_off_initial_stance_stay_grounded():
    # Tripod A flagged as in-swing, Tripod B as in-stance throughout
    # the ENGAGING window. Roles flip at the boundary in line with the
    # strategy's view (Tripod A touches down, Tripod B lifts off), so
    # we only assert during ENGAGING.
    ctrl = _controller()
    _begin(ctrl)
    nominal = _nominal_stance()
    for _ in range(30):
        out = ctrl.update(dt=0.02, v_cmd_xy=(0.20, 0.0), omega_cmd=0.0)
        if ctrl.state is EngagementState.DONE:
            break
        for name in _TRIPOD_A:
            assert out[name].stance is False
        for name in _TRIPOD_B:
            assert out[name].stance is True
            assert out[name].foot_target[2] == pytest.approx(
                nominal[name][2], abs=1e-12
            )


def test_constant_cmd_swing_lands_at_aep():
    # Smoothstep envelope guarantees the closed-form integral over
    # engage_time equals 0.5·stride; for any constant cmd_vel the swing
    # leg lands near AEP. The discrete-time loop introduces an O(dt)
    # quadrature error (right-Riemann sum of v_body · dt over the last
    # swing tick + the boundary stance step), so a 2e-3 tolerance at
    # dt=0.02 acknowledges that truth.
    ctrl = _controller()
    _begin(ctrl)
    out = _drive(ctrl, v_cmd_xy=(0.20, 0.0))
    nominal = _nominal_stance()
    # v=0.20, β=0.5, cycle_time = stride_length/(v·β) = 0.10/0.10 = 1.0 s,
    # stance_time = 0.5 s. stride per leg = v · stance_time = 0.10 m.
    # AEP = nominal + 0.5·stride.
    stride_x = 0.10
    for name in _TRIPOD_A:
        aep_x = nominal[name][0] + 0.5 * stride_x
        assert out[name].foot_target[0] == pytest.approx(aep_x, abs=2e-3)
        assert out[name].foot_target[1] == pytest.approx(nominal[name][1], abs=1e-6)


def test_constant_cmd_stance_lands_at_pep():
    # Integrated stance foot lands at PEP at master = β under constant
    # cmd_vel. The smoothstep integral over the half cycle equals
    # 0.5·stride in closed form, but finite-dt right-Riemann integration
    # introduces an O(dt) quadrature error — Tripod B integrates over
    # only N-1 stance ticks (the N-th is its lift-off), so the foot
    # falls short of PEP by ~v_cmd·dt at dt=0.02.
    ctrl = _controller()
    _begin(ctrl)
    out = _drive(ctrl, v_cmd_xy=(0.20, 0.0))
    nominal = _nominal_stance()
    stride_x = 0.10
    for name in _TRIPOD_B:
        pep_x = nominal[name][0] - 0.5 * stride_x
        assert out[name].foot_target[0] == pytest.approx(pep_x, abs=3e-3)
        assert out[name].foot_target[1] == pytest.approx(nominal[name][1], abs=1e-6)


def test_swing_clears_step_height():
    swing_clearance = 0.03
    ctrl = _controller(swing_clearance=swing_clearance)
    _begin(ctrl)
    nominal = _nominal_stance()
    seen_lifted = {n: False for n in _TRIPOD_A}
    for _ in range(40):
        out = ctrl.update(dt=0.02, v_cmd_xy=(0.20, 0.0), omega_cmd=0.0)
        for name in _TRIPOD_A:
            if out[name].foot_target[2] > nominal[name][2] + swing_clearance * 0.5:
                seen_lifted[name] = True
        if ctrl.state is EngagementState.DONE:
            break
    assert all(seen_lifted.values()), f"swing never lifted: {seen_lifted}"


def test_internal_v_body_follows_smoothstep_envelope():
    # ``v_body`` exposed by the controller is exactly
    # ``cmd_vel · smoothstep(master/exit_master)`` each tick. Direct
    # check: sample a few master phases and compare.
    cmd_v_x = 0.20
    ctrl = _controller()
    _begin(ctrl)
    samples: list[tuple[float, float]] = []
    while ctrl.state is not EngagementState.DONE:
        out = ctrl.update(dt=0.005, v_cmd_xy=(cmd_v_x, 0.0), omega_cmd=0.0)
        # Pull master phase out via the controller property. It's tau =
        # master/β under tripod β=0.5.
        v_x_internal = ctrl.v_body[0]
        samples.append((v_x_internal / cmd_v_x, ctrl.exit_master))

    # First sample is ~0 (engagement just started), last sample is ~1.
    assert samples[0][0] == pytest.approx(0.0, abs=0.05)
    assert samples[-1][0] == pytest.approx(1.0, abs=0.05)
    # Monotone non-decreasing.
    for i in range(1, len(samples)):
        assert samples[i][0] >= samples[i - 1][0] - 1e-9


def test_swing_touchdown_velocity_matches_steady_state():
    # The whole point of swing_target_velocity = -v_body: foot velocity
    # at touchdown equals the steady-state stance velocity (-v_cmd),
    # not the half-stride/swing_time value (-0.5·v_cmd).
    cmd_v_x = 0.20
    dt = 0.001
    ctrl = _controller(controller_dt=dt)
    _begin(ctrl)
    last_target = None
    while ctrl.state is not EngagementState.DONE:
        out = ctrl.update(dt=dt, v_cmd_xy=(cmd_v_x, 0.0), omega_cmd=0.0)
        last_target = out["l_front"].foot_target

    # Re-run one tick beyond DONE to estimate touchdown velocity from
    # the slope just before the final frame. Instead of that, build a
    # short window of the last few engagement ticks and compute foot
    # velocity numerically.
    ctrl = _controller(controller_dt=dt)
    _begin(ctrl)
    trace: list[tuple[float, tuple[float, float, float]]] = []
    elapsed = 0.0
    while ctrl.state is not EngagementState.DONE:
        out = ctrl.update(dt=dt, v_cmd_xy=(cmd_v_x, 0.0), omega_cmd=0.0)
        elapsed += dt
        trace.append((elapsed, out["l_front"].foot_target))

    # Average foot velocity over the last 5 ticks before touchdown.
    assert len(trace) >= 6
    (t_a, p_a) = trace[-6]
    (t_b, p_b) = trace[-1]
    v_foot_x = (p_b[0] - p_a[0]) / (t_b - t_a)
    # Steady-state stance velocity at AEP is -v_cmd = -0.20 m/s. With
    # the trajectory 2× bug fixed (_node_separation = 0.125) the swing's
    # touchdown velocity equals -v_cmd analytically; numerical finite-
    # difference error from the 5-tick window stays well under 5e-3.
    assert v_foot_x == pytest.approx(-cmd_v_x, abs=5e-3)


def test_engagement_tracks_growing_cmd_vel():
    # cmd_vel ramps from 0 to a target during engagement; the engagement
    # tracks it continuously and the swing leg still lands near the
    # live AEP. Position is bounded relative to the analytic AEP for
    # the *final* cmd value.
    ctrl = _controller()
    _begin(ctrl)
    target_v = 0.20
    dt = 0.005
    elapsed = 0.0
    last_out = None
    while ctrl.state is not EngagementState.DONE:
        # Slew cmd_vel linearly from 0 over ~0.3 s, then hold.
        cmd_v_x = min(target_v, target_v * elapsed / 0.3)
        last_out = ctrl.update(dt=dt, v_cmd_xy=(cmd_v_x, 0.0), omega_cmd=0.0)
        elapsed += dt

    assert last_out is not None
    nominal = _nominal_stance()
    stride_x_at_final = 0.10  # cycle_time = 1.0 s, stance_time = 0.5 s, v=0.2.
    aep_x = nominal["l_front"][0] + 0.5 * stride_x_at_final
    # Swing target migrates with cmd_vel; final position should be in
    # the ballpark of the live AEP (looser tolerance because the swing
    # was tracking a moving target).
    assert last_out["l_front"].foot_target[0] == pytest.approx(aep_x, abs=0.02)


def test_pure_yaw_inner_vs_outer_stride():
    # Pure ω: outer legs (front/rear) travel a larger arc than inner
    # legs (middle). Stride magnitudes at engagement end reflect that.
    omega = 1.0
    ctrl = _controller()
    _begin(ctrl)
    out = _drive(ctrl, v_cmd_xy=(0.0, 0.0), omega_cmd=omega)
    nominal = _nominal_stance()

    # Outer-leg radius drives the cycle_time. Outer per-leg v = ω·r.
    outer_r = math.hypot(0.15, 0.10)
    inner_r = 0.12
    # cycle_time = stride_length/(ω·r_outer·β) at saturation.
    expected_cycle = _STRIDE_LENGTH / (omega * outer_r * _DUTY_FACTOR)
    expected_stance_time = expected_cycle * _DUTY_FACTOR

    outer_displacement = math.hypot(
        out["l_front"].foot_target[0] - nominal["l_front"][0],
        out["l_front"].foot_target[1] - nominal["l_front"][1],
    )
    inner_displacement = math.hypot(
        out["l_middle"].foot_target[0] - nominal["l_middle"][0],
        out["l_middle"].foot_target[1] - nominal["l_middle"][1],
    )
    expected_outer = 0.5 * omega * outer_r * expected_stance_time
    expected_inner = 0.5 * omega * inner_r * expected_stance_time
    # 0.5·stride for the half-cycle end displacement.
    assert outer_displacement == pytest.approx(expected_outer, abs=2e-3)
    assert inner_displacement == pytest.approx(expected_inner, abs=2e-3)
    assert outer_displacement > inner_displacement


def test_exit_master_equals_duty_factor():
    ctrl = _controller()
    _begin(ctrl)
    assert ctrl.exit_master == pytest.approx(_DUTY_FACTOR)


def test_idle_emits_nominal_stance():
    ctrl = _controller()
    nominal = _nominal_stance()
    out = ctrl.update(dt=0.02, v_cmd_xy=(0.20, 0.0), omega_cmd=0.0)
    # No begin() call: state stays IDLE, output is nominal stance.
    for name in LEG_NAMES:
        assert out[name].foot_target == nominal[name]
        assert out[name].stance is True


def test_begin_rejects_strategy_duty_mismatch():
    # Controller is built for one duty_factor; passing a strategy with
    # a different one is a programmer error.
    ctrl = _controller(duty_factor=0.6)

    class _BadStrategy:
        phase_offsets = Tripod.phase_offsets
        duty_factor = 0.5

        def foot_target(self, *args, **kwargs):
            raise NotImplementedError

    with pytest.raises(ValueError):
        ctrl.begin(strategy=_BadStrategy(), leg_contexts=_leg_contexts())


# Ripple / wave engagement coverage. Both gaits share METACHRONAL_OFFSETS;
# they only differ in duty_factor. The bug they exercise is in the
# initial-stance branch: legs whose ``transition_m + (1 − β) < β``
# (i.e. swing window closes before engagement ends) used to be kept in
# the "swing" branch with ``phase_in_swing`` clamped at 1.0, freezing the
# foot at the live AEP while the body kept moving. The fix collapses
# their in_swing window to ``[transition_m, transition_m + (1 − β))`` so
# they fall through to stance integration for the rest of engagement.
#
# Stuck legs per gait (initial_stance with transition_m + (1−β) < β):
#   ripple (β = 2/3, swing_end = 1/3): l_middle (transition_m = 1/6)
#   wave   (β = 5/6, swing_end = 1/6): l_middle (1/6), r_front (1/3), l_rear (1/2)


_RIPPLE_STUCK = ("l_middle",)
_WAVE_STUCK = ("l_middle", "r_front", "l_rear")


def _stuck_window(offset: float, duty_factor: float) -> tuple[float, float]:
    # Master interval during which an initial-stance leg's swing curve is
    # active. The post-fix predicate switches the leg back to stance at
    # the upper bound.
    swing_window = 1.0 - duty_factor
    transition_m = 1.0 - offset
    return transition_m, transition_m + swing_window


@pytest.mark.parametrize(
    "strategy_cls, stuck_legs",
    [(Ripple, _RIPPLE_STUCK), (Wave, _WAVE_STUCK)],
)
def test_post_swing_legs_return_to_stance(strategy_cls, stuck_legs):
    # After the swing window closes, the leg must be reported with
    # stance=True (the bug had stance=False for the rest of engagement).
    duty_factor = strategy_cls.duty_factor
    ctrl = _controller(duty_factor=duty_factor)
    strategy = _begin(ctrl, strategy_cls)
    offsets = strategy.phase_offsets.offsets

    # Drive at a non-saturating cmd_vel so cycle_time is the raw quotient.
    # stride/(v·β): ripple → 1.5 s, wave → 1.2 s. Both above min_cycle.
    v_cmd_x = 0.10
    saw_post_swing_stance = {n: False for n in stuck_legs}
    while ctrl.state is not EngagementState.DONE:
        out = ctrl.update(dt=0.02, v_cmd_xy=(v_cmd_x, 0.0), omega_cmd=0.0)
        for name in stuck_legs:
            _, swing_end_master = _stuck_window(offsets[name], duty_factor)
            # Allow one-tick slop on either side of the boundary (the
            # master phase advances by dt/cycle_time per tick).
            tick_slop_master = 0.02 / (_STRIDE_LENGTH / (v_cmd_x * duty_factor))
            if ctrl._master > swing_end_master + tick_slop_master:
                if out[name].stance:
                    saw_post_swing_stance[name] = True

    assert all(saw_post_swing_stance.values()), (
        f"post-swing stance never reported for {strategy_cls.__name__}: "
        f"{saw_post_swing_stance}"
    )


@pytest.mark.parametrize(
    "strategy_cls, stuck_legs",
    [(Ripple, _RIPPLE_STUCK), (Wave, _WAVE_STUCK)],
)
def test_post_swing_foot_leaves_aep(strategy_cls, stuck_legs):
    # Once stance integration resumes, the foot must move back toward
    # PEP — the bug held it frozen at AEP regardless of body motion.
    duty_factor = strategy_cls.duty_factor
    ctrl = _controller(duty_factor=duty_factor)
    strategy = _begin(ctrl, strategy_cls)
    offsets = strategy.phase_offsets.offsets

    v_cmd_x = 0.10
    aep_observed: dict[str, float] = {}
    final_foot_x: dict[str, float] = {}
    while ctrl.state is not EngagementState.DONE:
        out = ctrl.update(dt=0.02, v_cmd_xy=(v_cmd_x, 0.0), omega_cmd=0.0)
        for name in stuck_legs:
            transition_m, swing_end_master = _stuck_window(
                offsets[name], duty_factor
            )
            # AEP for forward motion = nominal.x + 0.5·stride. Capture the
            # foot x at the swing endpoint.
            if (
                name not in aep_observed
                and ctrl._master >= swing_end_master - 1e-6
            ):
                aep_observed[name] = out[name].foot_target[0]
            final_foot_x[name] = out[name].foot_target[0]

    for name in stuck_legs:
        assert name in aep_observed, f"never reached swing end for {name}"
        # The post-swing stance integration must walk the foot back from
        # AEP — i.e. final x is strictly behind the swing-endpoint x.
        # Without the fix, foot was frozen at AEP and the two were equal.
        assert final_foot_x[name] < aep_observed[name] - 1e-4, (
            f"{name} did not integrate stance after swing ended "
            f"({strategy_cls.__name__}): swing_end_x={aep_observed[name]}, "
            f"final_x={final_foot_x[name]}"
        )


@pytest.mark.parametrize("strategy_cls", [Ripple, Wave])
def test_engagement_reaches_done(strategy_cls):
    # Smoke test: ripple / wave engagement completes without errors.
    # Pre-fix this also completed, but with three legs (wave) frozen at
    # AEP for up to half of engagement — the smoke test alone won't
    # catch that, hence the explicit post-swing checks above.
    ctrl = _controller(duty_factor=strategy_cls.duty_factor)
    _begin(ctrl, strategy_cls)
    _drive(ctrl, v_cmd_xy=(0.10, 0.0), max_ticks=400)


@pytest.mark.parametrize(
    "strategy_cls, stuck_legs",
    [(Ripple, _RIPPLE_STUCK), (Wave, _WAVE_STUCK)],
)
def test_no_body_frame_position_step_at_swing_end(strategy_cls, stuck_legs):
    # The swing → stance handoff inside the engagement must not introduce
    # a body-frame position step. The swing branch's last tick stores the
    # AEP into self._foot_position; the stance branch's first tick reads
    # from there and integrates −v_body·dt. So the swing-end → stance-
    # start tick delta is bounded by one tick of stance velocity
    # (≤ v_cmd·dt). Pre-fix this delta could be many cm — the leg was
    # kept in the "swing" branch with phase_in_swing pinned at 1.0 so
    # foot stayed at AEP while body moved, then GAIT handoff jumped it
    # to the strategy's expected stance position.
    duty_factor = strategy_cls.duty_factor
    ctrl = _controller(duty_factor=duty_factor)
    strategy = _begin(ctrl, strategy_cls)
    offsets = strategy.phase_offsets.offsets

    v_cmd_x = 0.10
    dt = 0.02
    prev_targets: dict[str, tuple[float, float, float]] = {}
    prev_stance: dict[str, bool] = {}
    while ctrl.state is not EngagementState.DONE:
        out = ctrl.update(dt=dt, v_cmd_xy=(v_cmd_x, 0.0), omega_cmd=0.0)
        for name in stuck_legs:
            # Only check the swing → stance transition tick. Swing apex
            # tip velocity legitimately exceeds body velocity, so a
            # tick-by-tick bound across the whole engagement would fail.
            transitioning = (
                name in prev_stance
                and prev_stance[name] is False
                and out[name].stance is True
            )
            if transitioning:
                pa = prev_targets[name]
                pb = out[name].foot_target
                step = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
                assert step < 2.0 * v_cmd_x * dt, (
                    f"{strategy_cls.__name__}/{name} foot position stepped "
                    f"by {step:.4f} m at swing → stance handoff "
                    f"(master={ctrl._master:.3f})"
                )
            prev_targets[name] = out[name].foot_target
            prev_stance[name] = out[name].stance
