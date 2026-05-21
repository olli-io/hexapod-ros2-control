import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.engine import Engine, EngineConfig, EngineState
from hexa_gait.gaits.base import LegContext
from hexa_gait.gaits.tripod import TRIPOD_OFFSETS, Tripod
from hexa_gait.initialize import (
    PAIR_ORDER,
    InitializeController,
    InitializeState,
)


COXA_TO_BOTTOM = 0.02
PAIR_TIME = 0.1
LIFT_TIME = 0.1
SWING_CLEARANCE = 0.02
PLACE_FEET_CLEARANCE = 0.001


def _nominal_stance() -> dict[str, tuple[float, float, float]]:
    return {
        "l_front": (0.15, 0.10, -0.10),
        "r_front": (0.15, -0.10, -0.10),
        "l_middle": (0.0, 0.12, -0.10),
        "r_middle": (0.0, -0.12, -0.10),
        "l_rear": (-0.15, 0.10, -0.10),
        "r_rear": (-0.15, -0.10, -0.10),
    }


def _initial_stance() -> dict[str, tuple[float, float, float]]:
    # Feet folded up well above the body, in some asymmetric layout
    # — verifies the controller never accidentally relies on
    # initial_stance matching nominal_stance in XY.
    return {
        "l_front": (0.05, 0.04, 0.08),
        "r_front": (0.05, -0.04, 0.08),
        "l_middle": (0.0, 0.05, 0.08),
        "r_middle": (0.0, -0.05, 0.08),
        "l_rear": (-0.05, 0.04, 0.08),
        "r_rear": (-0.05, -0.04, 0.08),
    }


def _ground_target(name: str) -> tuple[float, float, float]:
    # PLACE_FEET targets land the foot 1 mm above the floor (world z =
    # +PLACE_FEET_CLEARANCE) so the swing arc never scuffs the ground;
    # body-frame z therefore = -coxa_to_bottom + place_feet_clearance.
    nx, ny, _ = _nominal_stance()[name]
    return (nx, ny, -COXA_TO_BOTTOM + PLACE_FEET_CLEARANCE)


def _controller(**overrides) -> InitializeController:
    args = dict(
        initial_stance=_initial_stance(),
        nominal_stance=_nominal_stance(),
        coxa_to_bottom=COXA_TO_BOTTOM,
        pair_swing_time=PAIR_TIME,
        lift_body_time=LIFT_TIME,
        swing_clearance=SWING_CLEARANCE,
        place_feet_clearance=PLACE_FEET_CLEARANCE,
        swing_width=0.0,
        controller_dt=0.02,
    )
    args.update(overrides)
    return InitializeController(**args)


def _leg_contexts() -> dict[str, LegContext]:
    nominal = _nominal_stance()
    mounts = {
        "l_front": (0.15, 0.10, 0.0),
        "r_front": (0.15, -0.10, 0.0),
        "l_middle": (0.0, 0.12, 0.0),
        "r_middle": (0.0, -0.12, 0.0),
        "l_rear": (-0.15, 0.10, 0.0),
        "r_rear": (-0.15, -0.10, 0.0),
    }
    return {
        n: LegContext(name=n, mount_xyz=mounts[n], mount_yaw=0.0, nominal_stance=nominal[n])
        for n in LEG_NAMES
    }


def _engine_config() -> EngineConfig:
    return EngineConfig(
        stride_length=0.10,
        min_swing_time=0.25,
        max_swing_time=1.0,
        step_height=0.03,
        swing_width=0.0,
        controller_dt=0.02,
        cmd_zero_tol=1.0e-4,
        pause_debounce_delay=0.0,
        pause_to_reseat_delay=10.0,
        max_reset_time=0.6,
        init_pair_swing_time=PAIR_TIME,
        init_lift_body_time=LIFT_TIME,
        init_swing_clearance=SWING_CLEARANCE,
        init_place_feet_clearance=PLACE_FEET_CLEARANCE,
        # Reseat knobs (unused by these tests — Engine is constructed
        # without leg_specs/reseat_geometry).
        reseat_pose_settle_delay=0.1,
        reseat_height_change_threshold=0.001,
        reseat_pair_swing_time=0.1,
        reseat_pair_dwell_time=0.0,
        reseat_swing_clearance=0.02,
    )


# --- InitializeController unit tests --------------------------------------


def test_pair_order_is_middle_then_diagonals():
    # Documented behaviour for static-stability reasons (middle pair
    # first to keep CoM centred, then each diagonal). Regression guard
    # against reordering in initialize.py.
    assert PAIR_ORDER == (
        ("l_middle", "r_middle"),
        ("l_front", "r_rear"),
        ("r_front", "l_rear"),
    )


def test_controller_starts_in_place_feet():
    ctrl = _controller()
    assert ctrl.state is InitializeState.PLACE_FEET
    assert ctrl.done is False


def test_first_tick_only_active_pair_moves():
    ctrl = _controller()
    initial = _initial_stance()
    out = ctrl.update(dt=0.02)
    assert ctrl.state is InitializeState.PLACE_FEET
    # The first active pair is middle left/right.
    for name in ("l_middle", "r_middle"):
        assert out[name].foot_target != initial[name]
        assert out[name].stance is False
    # All other legs sit at exactly their initial_stance entry.
    for name in LEG_NAMES:
        if name in ("l_middle", "r_middle"):
            continue
        assert out[name].foot_target == initial[name]
        assert out[name].stance is True


def test_pairs_complete_in_order_and_snap_to_ground_targets():
    ctrl = _controller()
    initial = _initial_stance()
    dt = 0.02

    def _drain_pair(expected_active: tuple[str, str]) -> dict[str, tuple[float, float, float]]:
        out = None
        for _ in range(int(PAIR_TIME / dt) + 5):
            out = ctrl.update(dt=dt)
            # Active pair stays airborne while the arc plays out.
            if ctrl.state is InitializeState.PLACE_FEET:
                still_in_pair = any(
                    out[n].foot_target != _ground_target(n) for n in expected_active
                )
                if not still_in_pair:
                    break
            else:
                break
        return {n: out[n].foot_target for n in LEG_NAMES}

    # Pair 1 — middle pair lands on ground; everyone else still at initial.
    snap = _drain_pair(("l_middle", "r_middle"))
    for name in ("l_middle", "r_middle"):
        assert snap[name] == pytest.approx(_ground_target(name), abs=1e-9)
    for name in ("l_front", "r_front", "l_rear", "r_rear"):
        assert snap[name] == initial[name]

    # Pair 2 — front-left + rear-right diagonal lands; other diagonal still at initial.
    snap = _drain_pair(("l_front", "r_rear"))
    for name in ("l_middle", "r_middle", "l_front", "r_rear"):
        assert snap[name] == pytest.approx(_ground_target(name), abs=1e-9)
    for name in ("r_front", "l_rear"):
        assert snap[name] == initial[name]

    # Pair 3 — other diagonal lands; controller advances to LIFT_BODY.
    snap = _drain_pair(("r_front", "l_rear"))
    for name in LEG_NAMES:
        assert snap[name] == pytest.approx(_ground_target(name), abs=1e-9)
    assert ctrl.state is InitializeState.LIFT_BODY


def test_lift_body_monotonic_descent_in_body_frame_with_xy_held():
    ctrl = _controller()
    nominal = _nominal_stance()
    dt = 0.005

    # Drain PLACE_FEET first; that's covered separately above.
    while ctrl.state is InitializeState.PLACE_FEET:
        ctrl.update(dt=dt)
    assert ctrl.state is InitializeState.LIFT_BODY

    # LIFT_BODY starts from the PLACE_FEET endpoint (1 mm above floor).
    prev_z = {n: -COXA_TO_BOTTOM + PLACE_FEET_CLEARANCE for n in LEG_NAMES}
    while ctrl.state is InitializeState.LIFT_BODY:
        out = ctrl.update(dt=dt)
        for name in LEG_NAMES:
            x, y, z = out[name].foot_target
            # XY held exactly at standing positions throughout LIFT_BODY.
            assert x == pytest.approx(nominal[name][0], abs=1e-12)
            assert y == pytest.approx(nominal[name][1], abs=1e-12)
            # Body-frame z is monotonically more negative (foot pressed
            # further away from body ⇒ world-frame body rising).
            assert z <= prev_z[name] + 1e-12
            assert out[name].stance is True
            prev_z[name] = z

    # Final tick snaps to nominal exactly.
    for name in LEG_NAMES:
        x, y, z = ctrl.update(dt=dt)[name].foot_target
        assert (x, y, z) == pytest.approx(nominal[name], abs=1e-12)
    assert ctrl.state is InitializeState.DONE
    assert ctrl.done is True


def test_done_state_emits_nominal_forever():
    ctrl = _controller()
    nominal = _nominal_stance()
    for _ in range(500):
        ctrl.update(dt=0.02)
        if ctrl.state is InitializeState.DONE:
            break
    assert ctrl.state is InitializeState.DONE
    out = ctrl.update(dt=0.02)
    for name in LEG_NAMES:
        assert out[name].foot_target == nominal[name]
        assert out[name].stance is True
        assert out[name].phase == 0.0


def test_missing_initial_stance_raises():
    incomplete = dict(_initial_stance())
    incomplete.pop("l_rear")
    with pytest.raises(ValueError, match="initial_stance missing legs"):
        _controller(initial_stance=incomplete)


def test_nonpositive_timings_raise():
    with pytest.raises(ValueError, match="pair_swing_time"):
        _controller(pair_swing_time=0.0)
    with pytest.raises(ValueError, match="lift_body_time"):
        _controller(lift_body_time=-0.1)


# --- Engine integration tests ---------------------------------------------


def _engine() -> Engine:
    return Engine(
        config=_engine_config(),
        strategy=Tripod(),
        nominal_stance=_nominal_stance(),
        initial_stance=_initial_stance(),
        coxa_to_bottom=COXA_TO_BOTTOM,
        leg_contexts=_leg_contexts(),
    )


def test_engine_starts_in_folded_not_initialize():
    # Operator-gated cold start: the engine waits in FOLDED until
    # start_initialize() is called (typically from the joystick start
    # button via /gait/initialize), so power-on does not move the robot.
    engine = _engine()
    assert engine.state is EngineState.FOLDED


def test_folded_emits_initial_stance_and_ignores_cmd_vel():
    engine = _engine()
    initial = _initial_stance()
    # Multiple ticks with non-zero cmd_vel must all return the folded
    # foot positions and leave the engine in FOLDED.
    for _ in range(5):
        out = engine.update(dt=0.02, v_body_xy=(0.3, 0.2), omega_z=0.4)
        assert engine.state is EngineState.FOLDED
        for name in LEG_NAMES:
            assert out[name].foot_target == initial[name]
            assert out[name].stance is True


def test_start_initialize_transitions_folded_to_initialize():
    engine = _engine()
    assert engine.start_initialize() is True
    assert engine.state is EngineState.INITIALIZE
    # Re-trigger from INITIALIZE is a no-op.
    assert engine.start_initialize() is False
    assert engine.state is EngineState.INITIALIZE


def test_start_initialize_from_stand_is_a_noop():
    # After the cold-start has run once and the engine settled at
    # STAND, re-pressing the start button must not kick off another
    # initialize cycle — the legs are already standing and re-running
    # PLACE_FEET would lift them from nominal back to ground.
    engine = _engine()
    engine.start_initialize()
    for _ in range(60):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.STAND:
            break
    assert engine.state is EngineState.STAND
    assert engine.start_initialize() is False
    assert engine.state is EngineState.STAND


def test_engine_completes_initialize_then_enters_stand():
    engine = _engine()
    nominal = _nominal_stance()
    engine.start_initialize()
    # Total ladder: 3 * PAIR_TIME + LIFT_TIME = 0.4 s; one extra tick to
    # observe the STAND transition. dt = 0.02 ⇒ ≤ 25 ticks.
    for _ in range(60):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.STAND:
            break
    assert engine.state is EngineState.STAND
    out = engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    for name in LEG_NAMES:
        assert out[name].foot_target == nominal[name]
        assert out[name].stance is True


def test_cmd_vel_during_initialize_does_not_short_circuit_the_ladder():
    # Sending a non-zero cmd_vel during INITIALIZE must not bail to
    # ENGAGING / STOPPING — the cold-start ladder runs to completion.
    # Mirrors STOPPING's commit-to-completion contract.
    engine_baseline = _engine()
    engine_with_cmd = _engine()
    engine_baseline.start_initialize()
    engine_with_cmd.start_initialize()

    dt = 0.02
    baseline_out: list[dict] = []
    cmd_out: list[dict] = []
    for _ in range(int((3 * PAIR_TIME + LIFT_TIME) / dt) + 1):
        out_a = engine_baseline.update(dt=dt, v_body_xy=(0.0, 0.0), omega_z=0.0)
        out_b = engine_with_cmd.update(dt=dt, v_body_xy=(0.2, 0.0), omega_z=0.0)
        baseline_out.append({n: out_a[n].foot_target for n in LEG_NAMES})
        cmd_out.append({n: out_b[n].foot_target for n in LEG_NAMES})
        # Mid-INITIALIZE, the engine ignores cmd_vel and stays in
        # INITIALIZE — never ENGAGING / STOPPING — until the ladder ends.
        if engine_with_cmd.state is not EngineState.INITIALIZE:
            assert engine_with_cmd.state is EngineState.STAND
            break
        assert engine_with_cmd.state is EngineState.INITIALIZE

    # Foot targets emitted under the two cmd streams are identical
    # tick-for-tick — cmd_vel had no effect inside INITIALIZE.
    assert baseline_out == cmd_out
