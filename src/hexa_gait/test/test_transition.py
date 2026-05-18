import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.transition import TransitionController, TransitionState


def _flat_stance() -> dict[str, tuple[float, float, float]]:
    # Simple symmetric six-leg layout sufficient for ladder testing.
    return {
        "l_front": (0.15, 0.10, -0.10),
        "r_front": (0.15, -0.10, -0.10),
        "l_middle": (0.0, 0.12, -0.10),
        "r_middle": (0.0, -0.12, -0.10),
        "l_rear": (-0.15, 0.10, -0.10),
        "r_rear": (-0.15, -0.10, -0.10),
    }


def _controller(**overrides):
    args = dict(
        nominal_stance=_flat_stance(),
        recenter_swing_time=0.4,
        swing_clearance=0.03,
        swing_width=0.0,
        controller_dt=0.02,
        touchdown_settle_time=0.0,
    )
    args.update(overrides)
    return TransitionController(**args)


def test_begin_skips_force_touchdown_when_all_grounded():
    ctrl = _controller()
    nominal = _flat_stance()
    ctrl.begin(last_targets=nominal, swing_flags={n: False for n in LEG_NAMES})
    assert ctrl.state is TransitionState.RECENTER


def test_force_touchdown_drives_airborne_legs_to_nominal_in_parallel():
    ctrl = _controller(recenter_swing_time=0.1, swing_clearance=0.03)
    nominal = _flat_stance()
    # Tripod-style stop: three legs airborne at non-nominal poses.
    swing_set = {"l_front", "r_middle", "l_rear"}
    targets = dict(nominal)
    for n in swing_set:
        # Offset XY and lift Z above ground so the arc has motion on
        # every axis.
        nx, ny, nz = nominal[n]
        targets[n] = (nx + 0.04, ny - 0.03, nz + 0.05)
    flags = {n: (n in swing_set) for n in LEG_NAMES}
    ctrl.begin(last_targets=targets, swing_flags=flags)
    assert ctrl.state is TransitionState.FORCE_TOUCHDOWN

    # Mid-FORCE_TOUCHDOWN (t = 0.05 s of 0.10 s): every airborne leg
    # should still be airborne and lifted at least above the higher of
    # its two endpoints by ``swing_clearance``. Grounded legs hold.
    out = ctrl.update(dt=0.05)
    assert ctrl.state is TransitionState.FORCE_TOUCHDOWN
    for n in swing_set:
        assert out[n].stance is False
        peak_floor = max(targets[n][2], nominal[n][2])
        assert out[n].foot_target[2] > peak_floor + 0.01
    for n in LEG_NAMES:
        if n in swing_set:
            continue
        assert out[n].foot_target == nominal[n]
        assert out[n].stance is True

    # Finish FORCE_TOUCHDOWN — airborne legs snap to nominal, all six
    # are grounded, controller advances to RECENTER.
    out = ctrl.update(dt=0.06)
    assert ctrl.state is TransitionState.RECENTER
    for n in swing_set:
        assert out[n].foot_target == pytest.approx(nominal[n], abs=1e-9)
        assert out[n].stance is True


def test_force_touchdown_lifts_leg_that_stopped_just_above_ground():
    # Regression: when a leg stops in late swing — almost touching down
    # — the recenter arc must still rise to swing_clearance, not skim
    # along the ground from stop pose to nominal.
    swing_clearance = 0.03
    ctrl = _controller(recenter_swing_time=0.1, swing_clearance=swing_clearance)
    nominal = _flat_stance()
    targets = dict(nominal)
    # 0.5 mm above the ground, displaced in XY — the worst case for a
    # linear interpolation, which would barely lift Z.
    nx, ny, nz = nominal["l_front"]
    targets["l_front"] = (nx + 0.05, ny + 0.03, nz + 0.0005)
    flags = {n: False for n in LEG_NAMES}
    flags["l_front"] = True
    ctrl.begin(last_targets=targets, swing_flags=flags)

    peak_z = nz  # Both endpoints sit essentially at nominal Z.
    seen_lifted = False
    while ctrl.state is TransitionState.FORCE_TOUCHDOWN:
        out = ctrl.update(dt=0.02)
        if ctrl.state is TransitionState.FORCE_TOUCHDOWN:
            # Mid-flight Z must clear nominal+swing_clearance with
            # margin — the arc apex is at least the clearance, even
            # though origin Z is essentially ground.
            if out["l_front"].foot_target[2] > peak_z + swing_clearance * 0.5:
                seen_lifted = True
    assert seen_lifted, "FORCE_TOUCHDOWN arc never lifted the leg above ground"


def test_force_touchdown_holds_grounded_legs_exactly_still():
    ctrl = _controller(recenter_swing_time=0.1)
    nominal = _flat_stance()
    # One airborne leg; the other five are grounded but offset from
    # nominal so RECENTER would have something to do later. They must
    # not budge a millimetre during FORCE_TOUCHDOWN.
    targets = {n: (nominal[n][0] + 0.01, nominal[n][1], nominal[n][2]) for n in LEG_NAMES}
    targets["l_front"] = (0.20, 0.12, -0.04)
    flags = {n: False for n in LEG_NAMES}
    flags["l_front"] = True
    initial_grounded = {n: targets[n] for n in LEG_NAMES if n != "l_front"}
    ctrl.begin(last_targets=targets, swing_flags=flags)

    # Run several ticks while still in FORCE_TOUCHDOWN — grounded legs
    # should emit their stop-time positions verbatim every tick.
    while ctrl.state is TransitionState.FORCE_TOUCHDOWN:
        out = ctrl.update(dt=0.02)
        for n, frozen in initial_grounded.items():
            assert out[n].foot_target == frozen
            assert out[n].stance is True


def test_recenter_visits_each_leg_in_canonical_order_when_all_stance():
    ctrl = _controller(recenter_swing_time=0.1)
    nominal = _flat_stance()
    # Start every leg slightly offset from nominal so RECENTER has work
    # to do; all legs grounded so FORCE_TOUCHDOWN is skipped and the
    # order falls through to the canonical LEG_NAMES sequence.
    offset = {
        n: (nominal[n][0] + 0.01, nominal[n][1] + 0.01, nominal[n][2])
        for n in LEG_NAMES
    }
    ctrl.begin(last_targets=offset, swing_flags={n: False for n in LEG_NAMES})

    visited: list[str] = []
    for _ in range(80):
        out = ctrl.update(dt=0.02)
        airborne = [n for n in LEG_NAMES if not out[n].stance]
        assert len(airborne) <= 1, f"two legs airborne at once: {airborne}"
        if airborne and (not visited or visited[-1] != airborne[0]):
            visited.append(airborne[0])
        if ctrl.state is TransitionState.STAND:
            break

    assert visited == list(LEG_NAMES)
    assert ctrl.state is TransitionState.STAND


def test_recenter_visits_only_originally_grounded_legs():
    ctrl = _controller(recenter_swing_time=0.05)
    nominal = _flat_stance()
    offset = {
        n: (nominal[n][0] + 0.01, nominal[n][1] + 0.01, nominal[n][2])
        for n in LEG_NAMES
    }
    # Tripod-style stop: three legs airborne, three grounded. The
    # airborne legs land during FORCE_TOUCHDOWN (in parallel), so
    # RECENTER must only iterate the originally-grounded ones.
    swing_set = {"l_front", "r_middle", "l_rear"}
    for n in swing_set:
        offset[n] = (offset[n][0], offset[n][1], nominal[n][2] + 0.02)
    flags = {n: (n in swing_set) for n in LEG_NAMES}
    ctrl.begin(last_targets=offset, swing_flags=flags)

    # Drain FORCE_TOUCHDOWN first so the visited-list focuses on RECENTER.
    while ctrl.state is TransitionState.FORCE_TOUCHDOWN:
        ctrl.update(dt=0.02)

    visited: list[str] = []
    for _ in range(200):
        out = ctrl.update(dt=0.02)
        airborne = [n for n in LEG_NAMES if not out[n].stance]
        assert len(airborne) <= 1, f"two legs airborne at once: {airborne}"
        if airborne and (not visited or visited[-1] != airborne[0]):
            visited.append(airborne[0])
        if ctrl.state is TransitionState.STAND:
            break

    expected = [n for n in LEG_NAMES if n not in swing_set]
    assert visited == expected
    assert ctrl.state is TransitionState.STAND


def test_stand_emits_nominal_for_all_legs():
    ctrl = _controller()
    nominal = _flat_stance()
    ctrl.begin(last_targets=nominal, swing_flags={n: False for n in LEG_NAMES})
    # 6 legs * 0.4s recenter_swing_time = 2.4s; 200 iters * 0.02s = 4s
    # is comfortably enough to reach STAND.
    for _ in range(200):
        ctrl.update(dt=0.02)
        if ctrl.state is TransitionState.STAND:
            break
    assert ctrl.state is TransitionState.STAND
    out = ctrl.update(dt=0.02)
    for name in LEG_NAMES:
        assert out[name].foot_target == nominal[name]
        assert out[name].stance is True
        assert out[name].phase == 0.0


def test_settle_state_holds_every_foot_still_after_force_touchdown():
    # After FORCE_TOUCHDOWN lands the airborne legs, the controller
    # must hold every foot still for ``touchdown_settle_time`` before
    # advancing to RECENTER. Lets the chassis stop rocking from the
    # touchdown impact before another leg lifts.
    ctrl = _controller(recenter_swing_time=0.1, touchdown_settle_time=0.08)
    nominal = _flat_stance()
    swing_set = {"l_front", "r_middle", "l_rear"}
    targets = dict(nominal)
    for n in swing_set:
        nx, ny, nz = nominal[n]
        targets[n] = (nx + 0.04, ny - 0.03, nz + 0.05)
    flags = {n: (n in swing_set) for n in LEG_NAMES}
    ctrl.begin(last_targets=targets, swing_flags=flags)

    # Drain FORCE_TOUCHDOWN; next state must be SETTLE, not RECENTER.
    while ctrl.state is TransitionState.FORCE_TOUCHDOWN:
        ctrl.update(dt=0.02)
    assert ctrl.state is TransitionState.SETTLE

    # Snapshot positions at the moment SETTLE begins; nothing should
    # move while we tick through the hold window.
    held = dict(ctrl.update(dt=0.0))
    settle_ticks = 0
    while ctrl.state is TransitionState.SETTLE:
        out = ctrl.update(dt=0.02)
        settle_ticks += 1
        for n in LEG_NAMES:
            assert out[n].foot_target == held[n].foot_target
            assert out[n].stance is True
            assert out[n].phase == 0.0
        assert settle_ticks < 20, "SETTLE never advanced"

    # 0.08 s / 0.02 s per tick = 4 ticks of SETTLE, plus the dt=0.0
    # snapshot that was already taken above.
    assert settle_ticks == 4
    assert ctrl.state is TransitionState.RECENTER


def test_settle_skipped_when_no_leg_was_airborne():
    # Settle exists to absorb force-touchdown impact. When every leg
    # was already on the ground at stop time, FORCE_TOUCHDOWN is
    # skipped — there is nothing to settle from, so begin() must drop
    # straight to RECENTER regardless of touchdown_settle_time.
    ctrl = _controller(touchdown_settle_time=0.5)
    nominal = _flat_stance()
    ctrl.begin(last_targets=nominal, swing_flags={n: False for n in LEG_NAMES})
    assert ctrl.state is TransitionState.RECENTER


def test_settle_skipped_when_settle_time_is_zero():
    # Zero settle time disables the hold; FORCE_TOUCHDOWN must hand
    # directly off to RECENTER on the same tick it finishes.
    ctrl = _controller(recenter_swing_time=0.05, touchdown_settle_time=0.0)
    nominal = _flat_stance()
    swing_set = {"l_front"}
    targets = dict(nominal)
    nx, ny, nz = nominal["l_front"]
    targets["l_front"] = (nx + 0.04, ny - 0.03, nz + 0.05)
    flags = {n: (n in swing_set) for n in LEG_NAMES}
    ctrl.begin(last_targets=targets, swing_flags=flags)

    while ctrl.state is TransitionState.FORCE_TOUCHDOWN:
        ctrl.update(dt=0.02)
    assert ctrl.state is TransitionState.RECENTER


def test_force_touchdown_decelerates_to_rest_at_landing():
    # The forced-touchdown swing arc must land at zero velocity so the
    # foot does not slam into the floor at the steady-state stance
    # velocity. We oversample the trajectory with a tight controller_dt
    # and check that the last airborne sample (one tick before snap)
    # is essentially at the nominal landing spot — a Bezier with
    # non-zero endpoint velocity still has measurable residual motion
    # over the final tick, while a rest-to-rest curve does not.
    swing_time = 0.1
    controller_dt = 0.001
    ctrl = _controller(
        recenter_swing_time=swing_time,
        controller_dt=controller_dt,
        touchdown_settle_time=0.0,
    )
    nominal = _flat_stance()
    targets = dict(nominal)
    nx, ny, nz = nominal["l_front"]
    stride = (0.08, -0.06, 0.04)
    targets["l_front"] = (nx + stride[0], ny + stride[1], nz + stride[2])
    flags = {n: False for n in LEG_NAMES}
    flags["l_front"] = True
    ctrl.begin(last_targets=targets, swing_flags=flags)

    samples: list[tuple[float, float, float]] = []
    while ctrl.state is TransitionState.FORCE_TOUCHDOWN:
        out = ctrl.update(dt=controller_dt)
        if ctrl.state is TransitionState.FORCE_TOUCHDOWN:
            samples.append(out["l_front"].foot_target)

    # With dt = 1 ms and swing_time = 100 ms, samples[-1] is at
    # phase ≈ 0.99 — far enough into the curve that any non-zero
    # touchdown velocity would still leave a > 0.5 mm displacement on
    # the largest-stride axis (the default would be |stride| / 100 =
    # 0.8 mm on x). A rest-to-rest curve has near-zero residual.
    last = samples[-1]
    nominal_xyz = nominal["l_front"]
    residual = max(abs(last[i] - nominal_xyz[i]) for i in range(3))
    assert residual < 1e-4, f"residual {residual:.6f} m above rest-to-rest floor"


