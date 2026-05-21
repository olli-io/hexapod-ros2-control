import math

import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.engagement import EngagementController, EngagementState
from hexa_gait.gaits.base import LegContext, StrideParams
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
# cycle_time = 1.0 s. One full engagement cycle therefore lasts 1.0 s
# (master 0 → 1.0).
_STRIDE_LENGTH = 0.10
_DUTY_FACTOR = 0.5
_CYCLE_TIME_AT_0_20 = _STRIDE_LENGTH / (0.20 * _DUTY_FACTOR)
_SWING_CLEARANCE = 0.03
_SWING_WIDTH = 0.0
_CONTROLLER_DT = 0.02


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
        swing_clearance=_SWING_CLEARANCE,
        swing_width=_SWING_WIDTH,
        controller_dt=_CONTROLLER_DT,
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
    max_ticks: int = 400,
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


def test_tripod_roles_during_initial_half_cycle():
    # During master < 0.5 (tripod's first touchdown), Tripod A is in
    # INITIAL_SWING and Tripod B in INITIAL_STANCE. After that they
    # swap: A enters GAIT_LIKE stance and B enters INITIAL_SWING. The
    # assertion window is therefore restricted to the first half cycle.
    ctrl = _controller()
    _begin(ctrl)
    nominal = _nominal_stance()
    while ctrl.state is not EngagementState.DONE and ctrl._master < 0.4:
        out = ctrl.update(dt=0.02, v_cmd_xy=(0.20, 0.0), omega_cmd=0.0)
        for name in _TRIPOD_A:
            assert out[name].stance is False
        for name in _TRIPOD_B:
            assert out[name].stance is True
            assert out[name].foot_target[2] == pytest.approx(
                nominal[name][2], abs=1e-12
            )


def test_constant_cmd_tripod_a_lands_at_pep():
    # Tripod A is initial-swing: it leaves NOMINAL at master = 0, lands
    # at AEP at master = 0.5, then runs GAIT_LIKE stance to PEP by
    # master = 1.0. End of engagement → foot at PEP.
    ctrl = _controller()
    _begin(ctrl)
    out = _drive(ctrl, v_cmd_xy=(0.20, 0.0))
    nominal = _nominal_stance()
    stride_x = 0.10  # v = 0.20, β = 0.5, stance_time = 0.5 s → stride 0.10.
    for name in _TRIPOD_A:
        pep_x = nominal[name][0] - 0.5 * stride_x
        assert out[name].foot_target[0] == pytest.approx(pep_x, abs=3e-3), (
            f"{name} did not reach PEP: got {out[name].foot_target[0]}, "
            f"expected {pep_x}"
        )
        assert out[name].foot_target[1] == pytest.approx(nominal[name][1], abs=1e-6)


def test_constant_cmd_tripod_b_lands_at_aep():
    # Tripod B is initial-stance: it integrates stance from NOMINAL to
    # PEP over master [0, 0.5] (smoothstep envelope delivers exactly
    # 0.5·stride), then swings to live AEP by master = 1.0. End of
    # engagement → foot at AEP.
    ctrl = _controller()
    _begin(ctrl)
    out = _drive(ctrl, v_cmd_xy=(0.20, 0.0))
    nominal = _nominal_stance()
    stride_x = 0.10
    for name in _TRIPOD_B:
        aep_x = nominal[name][0] + 0.5 * stride_x
        assert out[name].foot_target[0] == pytest.approx(aep_x, abs=3e-3), (
            f"{name} did not reach AEP: got {out[name].foot_target[0]}, "
            f"expected {aep_x}"
        )
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


def test_internal_v_body_smoothstep_then_holds():
    # The internal body velocity ramps via smoothstep over
    # [0, smoothstep_window], then holds at cmd_vel through the rest of
    # the cycle. For tripod the window is 0.5 (the earliest first
    # touchdown). The envelope must be monotone non-decreasing and
    # converge to 1.0 well before engagement ends.
    cmd_v_x = 0.20
    ctrl = _controller()
    _begin(ctrl)
    samples: list[float] = []
    saturated_masters: list[float] = []
    while ctrl.state is not EngagementState.DONE:
        ctrl.update(dt=0.005, v_cmd_xy=(cmd_v_x, 0.0), omega_cmd=0.0)
        envelope = ctrl.v_body[0] / cmd_v_x
        samples.append(envelope)
        if ctrl._master >= ctrl.smoothstep_window:
            saturated_masters.append(envelope)

    assert samples[0] == pytest.approx(0.0, abs=0.05)
    assert samples[-1] == pytest.approx(1.0, abs=1e-9)
    # Monotone non-decreasing across the whole cycle.
    for i in range(1, len(samples)):
        assert samples[i] >= samples[i - 1] - 1e-9
    # Hold at 1.0 once master passes the smoothstep window. The first
    # tick across the boundary may straddle the threshold, so allow a
    # one-tick warm-up before pinning the assertion.
    assert all(s == pytest.approx(1.0, abs=1e-9) for s in saturated_masters[1:])


def test_smoothstep_window_matches_first_touchdown():
    # Tripod's first touchdown is at master = swing_end = 0.5; for
    # ripple and wave the earliest first touchdown is at master = 1/6
    # (the leg with the largest initial-swing offset). The smoothstep
    # window must equal that value so the envelope saturates exactly
    # when the first leg touches down.
    for strategy_cls, expected in [
        (Tripod, 0.5),
        (Ripple, 1.0 / 6.0),
        (Wave, 1.0 / 6.0),
    ]:
        ctrl = _controller(duty_factor=strategy_cls.duty_factor)
        _begin(ctrl, strategy_cls)
        assert ctrl.smoothstep_window == pytest.approx(expected, abs=1e-12), (
            f"{strategy_cls.__name__}: smoothstep_window {ctrl.smoothstep_window} "
            f"!= expected {expected}"
        )


def test_swing_touchdown_velocity_matches_steady_state():
    # The whole point of swing_target_velocity = -v_body: foot velocity
    # at the swing → stance handover equals the steady-state stance
    # velocity (-v_cmd). With the new design the leg crosses into
    # GAIT_LIKE stance at master = 0.5 for tripod's initial-swing legs,
    # and the GAIT_LIKE Bezier stance runs the foot at exactly -v_cmd.
    # Numerical finite-difference at small dt resolves this cleanly.
    cmd_v_x = 0.20
    dt = 0.001
    ctrl = _controller(controller_dt=dt)
    _begin(ctrl)
    trace: list[tuple[float, tuple[float, float, float]]] = []
    elapsed = 0.0
    while ctrl.state is not EngagementState.DONE:
        out = ctrl.update(dt=dt, v_cmd_xy=(cmd_v_x, 0.0), omega_cmd=0.0)
        elapsed += dt
        trace.append((elapsed, out["l_front"].foot_target))

    assert len(trace) >= 6
    (t_a, p_a) = trace[-6]
    (t_b, p_b) = trace[-1]
    v_foot_x = (p_b[0] - p_a[0]) / (t_b - t_a)
    assert v_foot_x == pytest.approx(-cmd_v_x, abs=5e-3)


def test_engagement_tracks_growing_cmd_vel():
    # cmd_vel ramps from 0 to a target during engagement; the live AEP
    # used at each leg's first touchdown reflects the cmd_vel at that
    # moment. By master = 1.0 the cmd has long since saturated, so the
    # GAIT_LIKE branch parks Tripod A near PEP for the final cmd_vel
    # and Tripod B near AEP for the final cmd_vel.
    ctrl = _controller()
    _begin(ctrl)
    target_v = 0.20
    dt = 0.005
    elapsed = 0.0
    last_out = None
    while ctrl.state is not EngagementState.DONE:
        cmd_v_x = min(target_v, target_v * elapsed / 0.3)
        last_out = ctrl.update(dt=dt, v_cmd_xy=(cmd_v_x, 0.0), omega_cmd=0.0)
        elapsed += dt

    assert last_out is not None
    nominal = _nominal_stance()
    stride_x = 0.10  # at final cmd_v = 0.2, β=0.5: stance_time = 0.5, stride = 0.1.
    pep_x = nominal["l_front"][0] - 0.5 * stride_x
    aep_x = nominal["r_front"][0] + 0.5 * stride_x
    assert last_out["l_front"].foot_target[0] == pytest.approx(pep_x, abs=0.02)
    assert last_out["r_front"].foot_target[0] == pytest.approx(aep_x, abs=0.02)


def test_pure_yaw_inner_vs_outer_stride():
    # Pure ω: outer legs (front/rear) sweep a larger tangential arc
    # than inner legs (middle). At engagement end the foot of each
    # initial-stance leg has landed at its own AEP (= NOMINAL +
    # 0.5·v_leg·stance_time). Magnitudes scale with leg radius.
    omega = 1.0
    ctrl = _controller()
    _begin(ctrl)
    out = _drive(ctrl, v_cmd_xy=(0.0, 0.0), omega_cmd=omega)
    nominal = _nominal_stance()

    outer_r = math.hypot(0.15, 0.10)
    inner_r = 0.12
    expected_cycle = _STRIDE_LENGTH / (omega * outer_r * _DUTY_FACTOR)
    expected_stance_time = expected_cycle * _DUTY_FACTOR

    # Use Tripod B legs (initial-stance, foot at AEP at end). r_front is
    # outer, l_middle is inner.
    outer_displacement = math.hypot(
        out["r_front"].foot_target[0] - nominal["r_front"][0],
        out["r_front"].foot_target[1] - nominal["r_front"][1],
    )
    inner_displacement = math.hypot(
        out["l_middle"].foot_target[0] - nominal["l_middle"][0],
        out["l_middle"].foot_target[1] - nominal["l_middle"][1],
    )
    expected_outer = 0.5 * omega * outer_r * expected_stance_time
    expected_inner = 0.5 * omega * inner_r * expected_stance_time
    assert outer_displacement == pytest.approx(expected_outer, abs=3e-3)
    assert inner_displacement == pytest.approx(expected_inner, abs=3e-3)
    assert outer_displacement > inner_displacement


def test_exit_master_wraps_to_zero():
    # Engagement covers a full master cycle; the modular handoff phase
    # is 0.0 regardless of the active gait's duty factor.
    for strategy_cls in (Tripod, Ripple, Wave):
        ctrl = _controller(duty_factor=strategy_cls.duty_factor)
        _begin(ctrl, strategy_cls)
        assert ctrl.exit_master == pytest.approx(0.0)


def test_idle_emits_nominal_stance():
    ctrl = _controller()
    nominal = _nominal_stance()
    out = ctrl.update(dt=0.02, v_cmd_xy=(0.20, 0.0), omega_cmd=0.0)
    # No begin() call: state stays IDLE, output is nominal stance.
    for name in LEG_NAMES:
        assert out[name].foot_target == nominal[name]
        assert out[name].stance is True


def test_begin_rejects_strategy_duty_mismatch():
    ctrl = _controller(duty_factor=0.6)

    class _BadStrategy:
        phase_offsets = Tripod.phase_offsets
        duty_factor = 0.5

        def foot_target(self, *args, **kwargs):
            raise NotImplementedError

    with pytest.raises(ValueError):
        ctrl.begin(strategy=_BadStrategy(), leg_contexts=_leg_contexts())


# GAIT continuity at engagement end.
#
# The redesign's defining invariant: at master = 1.0 every leg sits on
# the curve that the strategy would produce for that phase. Since the
# clock hands off as master = 0.0 of the next cycle, that's exactly
# the position GAIT's first tick will evaluate. No position step.


def _expected_gait_foot(
    strategy_cls,
    name: str,
    master: float,
    v_cmd_x: float,
    stride_length: float = _STRIDE_LENGTH,
) -> tuple[float, float, float]:
    strategy = strategy_cls()
    legs = _leg_contexts()
    offsets = strategy.phase_offsets.offsets
    phase = (master + offsets[name]) % 1.0
    duty = strategy.duty_factor
    cycle_time = stride_length / (v_cmd_x * duty)
    stance_time = cycle_time * duty
    stride_vec = (v_cmd_x * stance_time, 0.0, 0.0)
    stride = StrideParams(
        stride_vector=stride_vec,
        cycle_time=cycle_time,
        duty_factor=duty,
        swing_clearance=_SWING_CLEARANCE,
        swing_width=_SWING_WIDTH,
        controller_dt=_CONTROLLER_DT,
    )
    return strategy.foot_target(phase, stride, legs[name])


@pytest.mark.parametrize("strategy_cls", [Tripod, Ripple, Wave])
def test_engagement_end_matches_strategy_for_constant_cmd(strategy_cls):
    # Every leg lands on its strategy-prescribed position by master = 1.0.
    v_cmd_x = 0.10
    ctrl = _controller(duty_factor=strategy_cls.duty_factor)
    _begin(ctrl, strategy_cls)
    out = _drive(ctrl, v_cmd_xy=(v_cmd_x, 0.0), max_ticks=600)

    for name in LEG_NAMES:
        expected = _expected_gait_foot(strategy_cls, name, master=0.0, v_cmd_x=v_cmd_x)
        got = out[name].foot_target
        # The discrete-time master clamp at 1.0 + the swing → GAIT_LIKE
        # boundary collapse account for the tolerance.
        for axis, label in enumerate("xyz"):
            assert got[axis] == pytest.approx(expected[axis], abs=4e-3), (
                f"{strategy_cls.__name__}/{name} {label}: got {got[axis]}, "
                f"expected {expected[axis]} (strategy at master=0)"
            )


@pytest.mark.parametrize("strategy_cls", [Tripod, Ripple, Wave])
def test_no_position_step_at_state_boundaries(strategy_cls):
    # At every per-leg state boundary (INITIAL_STANCE → swing, swing →
    # GAIT_LIKE) the foot position must be continuous. The previous
    # design failed this for ripple/wave initial-stance legs whose swing
    # window closed before exit_master — they stayed pinned at AEP for
    # the rest of engagement, then jumped to PEP-ish at GAIT handoff.
    # Both transitions show up as ``stance`` flag flips; bound the
    # cross-flip tick delta to the foot's stance / swing-initial speed
    # (which both stay under v_cmd at the boundary instants).
    v_cmd_x = 0.10
    dt = 0.005
    ctrl = _controller(duty_factor=strategy_cls.duty_factor, controller_dt=dt)
    _begin(ctrl, strategy_cls)

    max_boundary_step = 2.0 * v_cmd_x * dt
    prev_targets: dict[str, tuple[float, float, float]] = {}
    prev_stance: dict[str, bool] = {}
    while ctrl.state is not EngagementState.DONE:
        out = ctrl.update(dt=dt, v_cmd_xy=(v_cmd_x, 0.0), omega_cmd=0.0)
        for name in LEG_NAMES:
            pb = out[name].foot_target
            if name in prev_stance and prev_stance[name] != out[name].stance:
                pa = prev_targets[name]
                step = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
                assert step < max_boundary_step, (
                    f"{strategy_cls.__name__}/{name} stepped {step:.4f} m "
                    f"across stance flip at master={ctrl._master:.3f}"
                )
            prev_targets[name] = pb
            prev_stance[name] = out[name].stance


@pytest.mark.parametrize("strategy_cls", [Tripod, Ripple, Wave])
def test_engagement_reaches_done(strategy_cls):
    ctrl = _controller(duty_factor=strategy_cls.duty_factor)
    _begin(ctrl, strategy_cls)
    _drive(ctrl, v_cmd_xy=(0.10, 0.0), max_ticks=600)
    assert ctrl.state is EngagementState.DONE


def test_wave_engagement_gait_like_stance_world_invariant_under_velocity_step():
    # The engagement controller's GAIT_LIKE branch must integrate stance
    # legs against the internal body velocity — not rebuild them from
    # the closed-form stance Bezier. Drive a wave engagement far enough
    # that several legs sit in GAIT_LIKE stance, then step v_cmd. Each
    # such leg's world-frame foot position must hold across the step.
    ctrl = _controller(duty_factor=Wave.duty_factor)
    _begin(ctrl, Wave)

    v_x = 0.10
    dt = 0.005
    body_x = 0.0
    body_y = 0.0
    # Drive to master ≈ 0.7. With wave's first_touchdown masters at
    # 1/6, 1/3, 1/2, 2/3, ..., by master 0.7 four legs have already
    # entered GAIT_LIKE (r_rear, l_middle, r_front, l_rear).
    out = None
    while ctrl.state is not EngagementState.DONE and ctrl._master < 0.7:
        out = ctrl.update(dt=dt, v_cmd_xy=(v_x, 0.0), omega_cmd=0.0)
        # Internal body velocity has saturated to v_x by master ≥ 1/6.
        body_x += ctrl.v_body[0] * dt
        body_y += ctrl.v_body[1] * dt

    assert out is not None
    before: dict[str, tuple[float, float]] = {}
    for name in LEG_NAMES:
        if out[name].stance:
            before[name] = (
                body_x + out[name].foot_target[0],
                body_y + out[name].foot_target[1],
            )
    assert len(before) >= 3, f"expected ≥3 stance legs, got {len(before)}"

    v_new = (0.05, 0.05)
    out = ctrl.update(dt=dt, v_cmd_xy=v_new, omega_cmd=0.0)
    body_x += ctrl.v_body[0] * dt
    body_y += ctrl.v_body[1] * dt

    for name, (wx_before, wy_before) in before.items():
        if not out[name].stance:
            continue
        wx_after = body_x + out[name].foot_target[0]
        wy_after = body_y + out[name].foot_target[1]
        dx = abs(wx_after - wx_before)
        dy = abs(wy_after - wy_before)
        # 1 mm tolerance, same as the engine-side test.
        assert dx < 1e-3, f"{name} world dx={dx*1000:.2f} mm"
        assert dy < 1e-3, f"{name} world dy={dy*1000:.2f} mm"


@pytest.mark.parametrize("strategy_cls", [Ripple, Wave])
def test_legs_enter_stance_after_first_touchdown(strategy_cls):
    # After each leg's first touchdown master it transitions to
    # GAIT_LIKE, where stance=True whenever phase ≥ swing_end. This is
    # the symptom of the old "stuck at AEP" bug: legs whose swing
    # window closed before exit_master used to stay flagged stance=False
    # for the rest of engagement. The new design routes them through
    # the strategy's stance Bezier instead.
    duty_factor = strategy_cls.duty_factor
    ctrl = _controller(duty_factor=duty_factor)
    strategy = _begin(ctrl, strategy_cls)
    offsets = strategy.phase_offsets.offsets

    v_cmd_x = 0.10
    saw_stance = {n: False for n in LEG_NAMES}
    while ctrl.state is not EngagementState.DONE:
        out = ctrl.update(dt=0.02, v_cmd_xy=(v_cmd_x, 0.0), omega_cmd=0.0)
        for name in LEG_NAMES:
            phase = (ctrl._master + offsets[name]) % 1.0
            if phase >= 1.0 - duty_factor + 0.05 and out[name].stance:
                saw_stance[name] = True

    assert all(saw_stance.values()), (
        f"{strategy_cls.__name__}: legs never reported stance: {saw_stance}"
    )
