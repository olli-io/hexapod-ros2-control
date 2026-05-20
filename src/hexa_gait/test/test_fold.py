import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.engine import Engine, EngineConfig, EngineState
from hexa_gait.fold import (
    PAIR_ORDER_REVERSED,
    FoldController,
    FoldState,
)
from hexa_gait.gaits.base import LegContext
from hexa_gait.gaits.tripod import Tripod
from hexa_gait.initialize import PAIR_ORDER


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
    nx, ny, _ = _nominal_stance()[name]
    return (nx, ny, -COXA_TO_BOTTOM + PLACE_FEET_CLEARANCE)


def _controller(**overrides) -> FoldController:
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
    return FoldController(**args)


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
        max_cycle_time=2.0,
        step_height=0.03,
        swing_width=0.0,
        controller_dt=0.02,
        cmd_zero_tol=1.0e-4,
        pause_debounce_delay=0.0,
        pause_to_reseat_delay=10.0,
        max_foot_speed=0.333,
        max_swing_time=0.6,
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


# --- FoldController unit tests --------------------------------------------


def test_pair_order_reversed_mirrors_initialize():
    # Documented behaviour: fold's LIFT_FEET reverses initialize's
    # PLACE_FEET pair order so the two ladders feel symmetric.
    assert PAIR_ORDER_REVERSED == tuple(reversed(PAIR_ORDER))


def test_controller_starts_in_lower_body():
    ctrl = _controller()
    assert ctrl.state is FoldState.LOWER_BODY
    assert ctrl.done is False


def test_lower_body_holds_xy_and_monotonically_raises_z_in_body_frame():
    # LOWER_BODY ramps body-frame z from nominal.z up to
    # -coxa_to_bottom + place_feet_clearance (less negative ⇒ foot
    # closer to body ⇒ world-frame body lowers onto its belly).
    ctrl = _controller()
    nominal = _nominal_stance()
    dt = 0.005

    prev_z = {n: nominal[n][2] for n in LEG_NAMES}
    while ctrl.state is FoldState.LOWER_BODY:
        out = ctrl.update(dt=dt)
        for name in LEG_NAMES:
            x, y, z = out[name].foot_target
            assert x == pytest.approx(nominal[name][0], abs=1e-12)
            assert y == pytest.approx(nominal[name][1], abs=1e-12)
            # Body-frame z increases (less negative) as the body
            # descends onto the belly.
            assert z >= prev_z[name] - 1e-12
            assert out[name].stance is True
            prev_z[name] = z

    # Snap to the LOWER_BODY endpoint at the boundary tick.
    assert ctrl.state is FoldState.LIFT_FEET
    for name in LEG_NAMES:
        nx, ny, _ = nominal[name]
        prev = prev_z[name]
        # LOWER_BODY endpoint == ground target.
        assert prev == pytest.approx(_ground_target(name)[2], abs=1e-12)


def test_first_lift_feet_tick_only_first_reverse_pair_moves():
    ctrl = _controller()
    dt = 0.005

    # Drain LOWER_BODY.
    while ctrl.state is FoldState.LOWER_BODY:
        ctrl.update(dt=dt)
    assert ctrl.state is FoldState.LIFT_FEET

    out = ctrl.update(dt=dt)
    # First active pair under the reversed order is the LAST entry of
    # PAIR_ORDER (i.e. ("r_front", "l_rear")).
    active = PAIR_ORDER_REVERSED[0]
    for name in active:
        # Active legs are mid-arc — not yet at initial.
        assert out[name].stance is False
        assert out[name].foot_target != _ground_target(name)
    for name in LEG_NAMES:
        if name in active:
            continue
        # All other legs sit at their ground target (LOWER_BODY endpoint).
        assert out[name].foot_target == pytest.approx(_ground_target(name), abs=1e-12)
        assert out[name].stance is True


def test_lift_feet_pairs_complete_in_reversed_order_and_snap_to_initial():
    ctrl = _controller()
    initial = _initial_stance()
    dt = 0.02

    # Drain LOWER_BODY.
    while ctrl.state is FoldState.LOWER_BODY:
        ctrl.update(dt=dt)

    def _drain_pair(expected_active: tuple[str, str]) -> dict[str, tuple[float, float, float]]:
        out = None
        for _ in range(int(PAIR_TIME / dt) + 5):
            out = ctrl.update(dt=dt)
            if ctrl.state is FoldState.LIFT_FEET:
                still_in_pair = any(
                    out[n].foot_target != initial[n] for n in expected_active
                )
                if not still_in_pair:
                    break
            else:
                break
        return {n: out[n].foot_target for n in LEG_NAMES}

    # Pair 1 — reverse[0] == ("r_front", "l_rear"); others still at ground.
    snap = _drain_pair(PAIR_ORDER_REVERSED[0])
    for name in PAIR_ORDER_REVERSED[0]:
        assert snap[name] == pytest.approx(initial[name], abs=1e-9)
    for name in PAIR_ORDER_REVERSED[1] + PAIR_ORDER_REVERSED[2]:
        assert snap[name] == pytest.approx(_ground_target(name), abs=1e-12)

    # Pair 2 — reverse[1] == ("l_front", "r_rear").
    snap = _drain_pair(PAIR_ORDER_REVERSED[1])
    for name in PAIR_ORDER_REVERSED[0] + PAIR_ORDER_REVERSED[1]:
        assert snap[name] == pytest.approx(initial[name], abs=1e-9)
    for name in PAIR_ORDER_REVERSED[2]:
        assert snap[name] == pytest.approx(_ground_target(name), abs=1e-12)

    # Pair 3 — middle pair; controller advances to DONE.
    snap = _drain_pair(PAIR_ORDER_REVERSED[2])
    for name in LEG_NAMES:
        assert snap[name] == pytest.approx(initial[name], abs=1e-9)
    assert ctrl.state is FoldState.DONE


def test_done_state_emits_initial_forever():
    ctrl = _controller()
    initial = _initial_stance()
    for _ in range(500):
        ctrl.update(dt=0.02)
        if ctrl.state is FoldState.DONE:
            break
    assert ctrl.state is FoldState.DONE
    out = ctrl.update(dt=0.02)
    for name in LEG_NAMES:
        assert out[name].foot_target == initial[name]
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


def _drive_to_stand(engine: Engine) -> None:
    """Run a fresh engine through INITIALIZE until it parks in STAND."""
    assert engine.start_initialize() is True
    for _ in range(200):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.STAND:
            return
    raise AssertionError(f"engine never reached STAND; last state={engine.state}")


def test_start_fold_rejected_from_folded():
    engine = _engine()
    assert engine.state is EngineState.FOLDED
    assert engine.start_fold() is False
    assert engine.state is EngineState.FOLDED


def test_start_fold_rejected_mid_initialize():
    engine = _engine()
    engine.start_initialize()
    assert engine.state is EngineState.INITIALIZE
    assert engine.start_fold() is False
    assert engine.state is EngineState.INITIALIZE


def test_start_fold_rejected_during_gait():
    # The user requirement: fold can only happen from STAND. A press
    # mid-walk must NOT abort the gait.
    engine = _engine()
    _drive_to_stand(engine)
    # Step into GAIT with a non-zero cmd_vel.
    engine.update(dt=0.02, v_body_xy=(0.1, 0.0), omega_z=0.0)
    assert engine.state in (EngineState.ENGAGING, EngineState.GAIT)
    assert engine.start_fold() is False
    assert engine.state in (EngineState.ENGAGING, EngineState.GAIT)


def test_start_fold_from_stand_transitions_to_folding():
    engine = _engine()
    _drive_to_stand(engine)
    assert engine.start_fold() is True
    assert engine.state is EngineState.FOLDING
    # Re-trigger from FOLDING is a no-op.
    assert engine.start_fold() is False
    assert engine.state is EngineState.FOLDING


def test_engine_completes_fold_then_settles_in_folded():
    engine = _engine()
    initial = _initial_stance()
    _drive_to_stand(engine)
    assert engine.start_fold() is True

    # Total ladder: LIFT_TIME + 3 * PAIR_TIME = 0.4 s; one extra tick
    # to observe the FOLDED transition. dt = 0.02 ⇒ ≤ 25 ticks.
    for _ in range(60):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.FOLDED:
            break
    assert engine.state is EngineState.FOLDED
    out = engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    for name in LEG_NAMES:
        assert out[name].foot_target == initial[name]
        assert out[name].stance is True


def test_cmd_vel_during_fold_does_not_short_circuit_the_ladder():
    # Sending a non-zero cmd_vel during FOLDING must not bail to
    # ENGAGING / STOPPING — the warm-shutdown commits to completion,
    # mirroring INITIALIZE's behaviour.
    baseline = _engine()
    with_cmd = _engine()
    _drive_to_stand(baseline)
    _drive_to_stand(with_cmd)
    baseline.start_fold()
    with_cmd.start_fold()

    dt = 0.02
    baseline_out: list[dict] = []
    cmd_out: list[dict] = []
    for _ in range(int((LIFT_TIME + 3 * PAIR_TIME) / dt) + 1):
        out_a = baseline.update(dt=dt, v_body_xy=(0.0, 0.0), omega_z=0.0)
        out_b = with_cmd.update(dt=dt, v_body_xy=(0.2, 0.0), omega_z=0.0)
        baseline_out.append({n: out_a[n].foot_target for n in LEG_NAMES})
        cmd_out.append({n: out_b[n].foot_target for n in LEG_NAMES})
        if with_cmd.state is not EngineState.FOLDING:
            assert with_cmd.state is EngineState.FOLDED
            break
        assert with_cmd.state is EngineState.FOLDING

    assert baseline_out == cmd_out


def test_engine_can_re_initialize_after_fold():
    # After a fold round-trip the engine should accept a second
    # start_initialize() and run a fresh ladder — verifies the
    # controllers are rebuilt rather than left half-consumed.
    engine = _engine()
    _drive_to_stand(engine)
    engine.start_fold()
    for _ in range(60):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.FOLDED:
            break
    assert engine.state is EngineState.FOLDED

    assert engine.start_initialize() is True
    for _ in range(200):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.STAND:
            break
    assert engine.state is EngineState.STAND


def test_start_fold_from_pausing_is_a_noop():
    # Mid-pause the engine is lowering the airborne legs — a fold press
    # during the transition must be ignored so the pause / paused flow
    # finishes cleanly.
    engine = _engine()
    _drive_to_stand(engine)
    # Drive into GAIT, then release the stick. pause_debounce_delay=0
    # in this config so the first cmd_zero tick is enough to enter
    # PAUSING.
    engine.update(dt=0.02, v_body_xy=(0.1, 0.0), omega_z=0.0)
    # Walk several ticks of GAIT, then drop to zero.
    for _ in range(5):
        engine.update(dt=0.02, v_body_xy=(0.1, 0.0), omega_z=0.0)
    engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    # The engine should now be PAUSING (or PAUSED if every airborne leg
    # landed in one tick).
    if engine.state is EngineState.PAUSING:
        assert engine.start_fold() is False
        assert engine.state is EngineState.PAUSING
