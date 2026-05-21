"""Engine integration tests for the RESEATING state.

These tests use the real YAML geometry (so reseat_nominal_stance
returns sensible body-frame positions) and a compact engine config
so the ladders complete in a handful of ticks.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.engine import (
    Engine,
    EngineConfig,
    EngineState,
    build_leg_contexts,
    initial_stance_from_yaml,
    nominal_stance_from_yaml,
    reseat_geometry_from_yaml,
)
from hexa_gait.gaits.tripod import Tripod
from hexa_kinematics.leg_specs import load_leg_specs


PAIR_TIME = 0.04
LIFT_TIME = 0.04
RESEAT_PAIR_TIME = 0.04
SETTLE_DELAY = 0.10


def _desc() -> Path:
    return Path(__file__).resolve().parents[2] / "hexa_description" / "config"


def _config() -> EngineConfig:
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
        init_swing_clearance=0.01,
        init_place_feet_clearance=0.001,
        reseat_pose_settle_delay=SETTLE_DELAY,
        reseat_height_change_threshold=0.001,
        reseat_pair_swing_time=RESEAT_PAIR_TIME,
        reseat_pair_dwell_time=0.0,
        reseat_swing_clearance=0.02,
    )


def _engine() -> Engine:
    desc = _desc()
    legs = load_leg_specs(desc / "geometry.yaml")
    nominal = nominal_stance_from_yaml(desc / "geometry.yaml", desc / "standing_pose.yaml")
    initial = initial_stance_from_yaml(desc / "geometry.yaml")
    leg_contexts = build_leg_contexts(desc / "geometry.yaml", desc / "standing_pose.yaml")
    reseat_geometry = reseat_geometry_from_yaml(
        desc / "geometry.yaml", desc / "standing_pose.yaml"
    )
    # coxa_to_bottom — pluck directly from YAML to avoid duplicating the
    # _load_coxa_to_bottom helper.
    import yaml

    with (desc / "geometry.yaml").open() as f:
        raw = yaml.safe_load(f)
    coxa_to_bottom = float(raw["body"]["coxa_to_bottom"])
    return Engine(
        config=_config(),
        strategy=Tripod(),
        nominal_stance=nominal,
        initial_stance=initial,
        coxa_to_bottom=coxa_to_bottom,
        leg_contexts=leg_contexts,
        leg_specs=legs,
        reseat_geometry=reseat_geometry,
    )


def _drive_to_stand(engine: Engine) -> None:
    assert engine.start_initialize() is True
    for _ in range(200):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.STAND:
            return
    raise AssertionError("engine did not reach STAND within 200 ticks")


def _drive_through_reseat(engine: Engine) -> int:
    """Tick from RESEATING back to STAND; returns ticks consumed."""
    for i in range(500):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.STAND:
            return i + 1
    raise AssertionError("engine did not return to STAND after reseat")


def test_set_target_height_does_not_fire_immediately():
    engine = _engine()
    _drive_to_stand(engine)
    engine.set_target_height(0.02)
    # One tick: settle timer at 0.02 s; threshold 0.10 — should still
    # be in STAND.
    engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.STAND


def test_height_settles_and_triggers_reseat():
    engine = _engine()
    _drive_to_stand(engine)
    engine.set_target_height(0.02)
    # Tick past the settle delay: at dt=0.02 and delay=0.10 s, the
    # 6th tick puts elapsed at 0.12 s ≥ 0.10 s and the reseat fires.
    for _ in range(20):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.RESEATING:
            break
    assert engine.state is EngineState.RESEATING


def test_reseat_completes_and_returns_to_stand_with_updated_nominal():
    engine = _engine()
    _drive_to_stand(engine)
    nominal_before = dict(engine._nominal)
    engine.set_target_height(0.02)
    # Wait for the ladder to start, then tick it to completion.
    for _ in range(500):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.RESEATING:
            break
    assert engine.state is EngineState.RESEATING
    _drive_through_reseat(engine)
    # Nominal updated: XY changed radially, Z stayed at default.
    for name in LEG_NAMES:
        nbx, nby, nbz = nominal_before[name]
        nax, nay, naz = engine._nominal[name]
        assert nbz == pytest.approx(naz, abs=1e-9)
        assert (nax, nay) != (nbx, nby)
    assert engine._applied_height == pytest.approx(0.02, abs=1e-9)


def test_height_change_resets_settle_timer():
    # If the target keeps slewing, the settle timer resets each time,
    # so reseat does not fire mid-ramp.
    engine = _engine()
    _drive_to_stand(engine)
    for i in range(10):
        # Slew height upward by 5 mm per tick — well above the
        # float-noise epsilon, so the settle timer never accumulates.
        engine.set_target_height(0.005 * (i + 1))
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.STAND
    # Now hold the height steady — reseat fires after the delay.
    final = 0.005 * 10
    engine.set_target_height(final)
    for _ in range(20):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is not EngineState.STAND:
            break
    assert engine.state in (EngineState.RESEATING, EngineState.STAND)


def test_held_dpad_at_one_mm_per_tick_does_not_fire_mid_press():
    # Regression: the teleop integrates pose.z at 0.05 m/s and
    # publishes at 50 Hz, so a held D-pad slews the target by exactly
    # 1 mm per tick. That used to sit right on the YAML dead-band
    # (``reseat_height_change_threshold = 0.001``) and the settle
    # timer accrued anyway, firing reseat mid-press. The fix uses a
    # tighter float-noise epsilon inside set_target_height so the
    # timer reliably resets on every per-tick D-pad step.
    engine = _engine()
    _drive_to_stand(engine)
    # Simulate the held D-pad for well past the settle delay.
    dt = 0.02
    ticks = int(round((SETTLE_DELAY + 0.10) / dt))
    z = 0.0
    for _ in range(ticks):
        z += 0.001
        engine.set_target_height(z)
        engine.update(dt=dt, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.STAND, (
        "reseat fired while D-pad was still held"
    )
    # Release: stop slewing, ride out the settle window — now it should fire.
    for _ in range(ticks):
        engine.set_target_height(z)
        engine.update(dt=dt, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.RESEATING:
            break
    assert engine.state is EngineState.RESEATING


def test_cmd_vel_during_reseat_is_held_until_done():
    # Commit-to-completion: cmd_vel arriving mid-reseat must not bail
    # to ENGAGING / STOPPING. Mirrors the INITIALIZE / FOLDING contract.
    engine = _engine()
    _drive_to_stand(engine)
    engine.set_target_height(0.02)
    for _ in range(500):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.RESEATING:
            break
    assert engine.state is EngineState.RESEATING
    # Now slam cmd_vel non-zero — engine must stay in RESEATING for at
    # least a couple of ticks (the ladder is 3 pairs × pair_swing_time
    # but the entering tick already advanced the controller). Stop
    # well before completion so the assertion never trips on the
    # natural reseat → STAND boundary.
    for _ in range(int(2 * RESEAT_PAIR_TIME / 0.02) - 1):
        engine.update(dt=0.02, v_body_xy=(0.2, 0.0), omega_z=0.0)
        # Reseat continues until done, regardless of cmd_vel.
        assert engine.state is EngineState.RESEATING


def test_pending_fold_defers_until_height_zero():
    # Two-press Start scheme: at lifted height, request_fold latches
    # _pending_fold but the engine must NOT fold until the height has
    # been snapped back to zero AND the reseat ladder has run to
    # completion at applied_height=0.
    engine = _engine()
    _drive_to_stand(engine)
    engine.set_target_height(0.02)
    # Let the lift settle and reseat fire.
    for _ in range(50):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.RESEATING:
            break
    assert engine.state is EngineState.RESEATING
    # Snap height back to zero and request a fold. Reseat must run to
    # completion at the new target (0), then engine returns to STAND,
    # then consumes _pending_fold and transitions to FOLDING.
    engine.set_target_height(0.0)
    assert engine.request_fold() is True
    # Tick through reseat completion AND the subsequent fold trigger.
    for _ in range(500):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.FOLDING:
            break
    assert engine.state is EngineState.FOLDING
    assert engine._applied_height == pytest.approx(0.0, abs=1e-9)


def test_request_fold_at_zero_height_folds_on_next_stand_tick():
    # When height is already at default, request_fold should kick off
    # FOLDING immediately on the next STAND tick. No reseat in the way.
    engine = _engine()
    _drive_to_stand(engine)
    assert engine.request_fold() is True
    engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.FOLDING


def test_request_fold_rejected_when_folded_or_folding():
    engine = _engine()
    # FOLDED: request_fold returns False (engine is already where
    # FOLDING would take it).
    assert engine.state is EngineState.FOLDED
    assert engine.request_fold() is False
    _drive_to_stand(engine)
    # Drive to FOLDING.
    assert engine.request_fold() is True
    engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.FOLDING
    # Mid-FOLDING: rejected.
    assert engine.request_fold() is False


def test_cmd_vel_preempts_pending_reseat_in_stand():
    # If the user pushes the stick while the settle delay is counting
    # down, the engine bails to ENGAGING (walking takes priority) and
    # leaves the pending reseat alone. When the user later returns to
    # STAND, the reseat fires if the height target still differs.
    engine = _engine()
    _drive_to_stand(engine)
    engine.set_target_height(0.02)
    # Halfway through the settle window:
    for _ in range(3):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.STAND
    # User pushes stick:
    engine.update(dt=0.02, v_body_xy=(0.2, 0.0), omega_z=0.0)
    assert engine.state is EngineState.ENGAGING


def test_reseat_with_zero_height_is_a_noop():
    # The threshold dead-band: a target within
    # reseat_height_change_threshold of applied does not fire.
    engine = _engine()
    _drive_to_stand(engine)
    engine.set_target_height(0.0005)  # below the 1 mm threshold
    for _ in range(50):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.STAND
    assert engine._applied_height == pytest.approx(0.0, abs=1e-12)


def test_engine_rejects_partial_reseat_kwargs():
    desc = _desc()
    nominal = nominal_stance_from_yaml(desc / "geometry.yaml", desc / "standing_pose.yaml")
    initial = initial_stance_from_yaml(desc / "geometry.yaml")
    leg_contexts = build_leg_contexts(desc / "geometry.yaml", desc / "standing_pose.yaml")
    # leg_specs without reseat_geometry must raise.
    with pytest.raises(ValueError, match="must be supplied together"):
        Engine(
            config=_config(),
            strategy=Tripod(),
            nominal_stance=nominal,
            initial_stance=initial,
            coxa_to_bottom=0.03,
            leg_contexts=leg_contexts,
            leg_specs=load_leg_specs(desc / "geometry.yaml"),
        )


def test_engine_initializes_with_no_reseat_kwargs():
    # Backward-compatible constructor: tests that pass none of the
    # reseat kwargs still build a valid engine — they just don't get
    # the RESEATING behaviour. This keeps the synthetic test_engine.py
    # / test_initialize.py / test_fold.py setups working.
    desc = _desc()
    nominal = nominal_stance_from_yaml(desc / "geometry.yaml", desc / "standing_pose.yaml")
    initial = initial_stance_from_yaml(desc / "geometry.yaml")
    leg_contexts = build_leg_contexts(desc / "geometry.yaml", desc / "standing_pose.yaml")
    engine = Engine(
        config=_config(),
        strategy=Tripod(),
        nominal_stance=nominal,
        initial_stance=initial,
        coxa_to_bottom=0.03,
        leg_contexts=leg_contexts,
    )
    # With no reseat geometry, set_target_height + settle should be
    # silently inert.
    assert engine.start_initialize() is True
    for _ in range(200):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.STAND:
            break
    engine.set_target_height(0.02)
    for _ in range(50):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    # Never enters RESEATING.
    assert engine.state is EngineState.STAND
