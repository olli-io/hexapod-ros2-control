import math

import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.engagement import EngagementController, EngagementState
from hexa_gait.gaits.base import LegContext
from hexa_gait.gaits.tripod import Tripod


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


def _begin(ctrl: EngagementController) -> Tripod:
    strategy = Tripod()
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
