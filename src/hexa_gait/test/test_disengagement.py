import pytest

from hexa_gait.clock import LEG_NAMES, PhaseOffsets
from hexa_gait.gaits._common import METACHRONAL_OFFSETS
from hexa_gait.gaits.tripod import TRIPOD_OFFSETS
from hexa_gait.disengagement import DisengagementController, DisengagementState


# Per-gait duty factors mirror the values in hexa_gait/config/gait.yaml
# and the strategy classes. Defined locally so a future YAML retune
# does not silently break these tests.
TRIPOD_DUTY = 0.5
RIPPLE_DUTY = 2.0 / 3.0
WAVE_DUTY = 5.0 / 6.0

# Tripod offset groups (the two natural triples).
TRIPOD_A = {"l_front", "r_middle", "l_rear"}  # offset 0.0
TRIPOD_B = {"r_front", "l_middle", "r_rear"}  # offset 0.5


def _flat_stance() -> dict[str, tuple[float, float, float]]:
    # Simple symmetric six-leg layout sufficient for queue testing.
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
        swing_clearance=0.03,
        swing_width=0.0,
        controller_dt=0.02,
        max_foot_speed=0.333,
        min_swing_time=0.1,
        max_swing_time=0.4,
    )
    args.update(overrides)
    return DisengagementController(**args)


def _begin(
    ctrl: DisengagementController,
    *,
    last_targets,
    swing_flags,
    phase_offsets: PhaseOffsets = TRIPOD_OFFSETS,
    duty_factor: float = TRIPOD_DUTY,
    master_phase: float = 0.0,
) -> None:
    ctrl.begin(
        last_targets=last_targets,
        swing_flags=swing_flags,
        phase_offsets=phase_offsets,
        duty_factor=duty_factor,
        master_phase=master_phase,
    )


def _airborne(out) -> list[str]:
    return [n for n in LEG_NAMES if not out[n].stance]


def _offset_all_legs(nominal):
    # Body-frame X displacement of 2 cm per leg — enough to force a
    # visible swing for every leg.
    return {n: (nominal[n][0] + 0.02, nominal[n][1], nominal[n][2]) for n in LEG_NAMES}


def _phase_based_flags(
    phase_offsets: PhaseOffsets, duty_factor: float, master_phase: float
) -> dict[str, bool]:
    """Compute swing_flags the engine would emit at this master phase.

    Mirrors ``Engine._last_stance`` semantics: a leg is "airborne" iff
    its projected phase falls inside the swing window ``[0, 1 − β)``.
    """

    swing_window = 1.0 - duty_factor
    offsets = phase_offsets.offsets
    return {
        n: ((master_phase + offsets[n]) % 1.0) < swing_window for n in LEG_NAMES
    }


def _run_to_stand(ctrl: DisengagementController, dt: float = 0.02, max_ticks: int = 600):
    """Tick until STAND or assert if the queue never empties."""

    for _ in range(max_ticks):
        out = ctrl.update(dt=dt)
        if ctrl.state is DisengagementState.STAND:
            return out
    raise AssertionError(f"controller did not reach STAND after {max_ticks} ticks")


# -----------------------------------------------------------------------------
# begin() routing
# -----------------------------------------------------------------------------


def test_begin_routes_straight_to_stand_when_all_legs_at_nominal():
    ctrl = _controller()
    nominal = _flat_stance()
    _begin(ctrl, last_targets=nominal, swing_flags={n: False for n in LEG_NAMES})
    assert ctrl.state is DisengagementState.STAND


def test_begin_enters_running_when_work_remains():
    ctrl = _controller()
    nominal = _flat_stance()
    _begin(
        ctrl,
        last_targets=_offset_all_legs(nominal),
        swing_flags={n: False for n in LEG_NAMES},
    )
    assert ctrl.state is DisengagementState.RUNNING


def test_begin_rejects_invalid_duty_factor():
    ctrl = _controller()
    nominal = _flat_stance()
    with pytest.raises(ValueError):
        _begin(
            ctrl,
            last_targets=nominal,
            swing_flags={n: False for n in LEG_NAMES},
            duty_factor=0.0,
        )
    with pytest.raises(ValueError):
        _begin(
            ctrl,
            last_targets=nominal,
            swing_flags={n: False for n in LEG_NAMES},
            duty_factor=1.0,
        )


def test_begin_rejects_out_of_range_master_phase():
    ctrl = _controller()
    nominal = _flat_stance()
    with pytest.raises(ValueError):
        _begin(
            ctrl,
            last_targets=nominal,
            swing_flags={n: False for n in LEG_NAMES},
            master_phase=1.0,
        )


# -----------------------------------------------------------------------------
# Group ordering
# -----------------------------------------------------------------------------


def test_tripod_airborne_group_swings_first_then_stance_group():
    # Tripod A airborne mid-cycle, tripod B grounded but offset from
    # nominal. The queue must run A first, then B — neither group
    # should overlap because they are sequential.
    ctrl = _controller()
    nominal = _flat_stance()
    targets = _offset_all_legs(nominal)
    for n in TRIPOD_A:
        nx, ny, nz = nominal[n]
        targets[n] = (nx + 0.04, ny - 0.03, nz + 0.02)
    flags = {n: (n in TRIPOD_A) for n in LEG_NAMES}
    _begin(ctrl, last_targets=targets, swing_flags=flags)

    a_lifted_first = False
    b_lifted_only_after_a_landed = True
    a_landed = False
    seen_b_airborne_before_a_landed = False
    for _ in range(600):
        out = ctrl.update(dt=0.02)
        airborne = set(_airborne(out))
        if airborne:
            if airborne <= TRIPOD_A and not a_landed:
                a_lifted_first = True
            if airborne & TRIPOD_B and not a_landed:
                seen_b_airborne_before_a_landed = True
        if a_lifted_first and not airborne & TRIPOD_A:
            a_landed = True
        if ctrl.state is DisengagementState.STAND:
            break
    assert a_lifted_first, "Tripod A (airborne at stop time) never lifted"
    if seen_b_airborne_before_a_landed:
        b_lifted_only_after_a_landed = False
    assert b_lifted_only_after_a_landed, (
        "Tripod B started swinging before tripod A finished landing"
    )


def test_stance_groups_are_ordered_by_descending_phase():
    # Wave-style metachronal offsets at master=0: r_rear is in swing
    # (phase 0). The five remaining stance legs lift in descending-phase
    # order: l_middle (5/6), r_front (2/3), l_rear (1/2), r_middle
    # (1/3), l_front (1/6).
    ctrl = _controller()
    nominal = _flat_stance()
    expected_after_swing = ["l_middle", "r_front", "l_rear", "r_middle", "l_front"]
    _begin(
        ctrl,
        last_targets=_offset_all_legs(nominal),
        swing_flags=_phase_based_flags(METACHRONAL_OFFSETS, WAVE_DUTY, 0.0),
        phase_offsets=METACHRONAL_OFFSETS,
        duty_factor=WAVE_DUTY,
    )

    # Drop the very first observed airborne leg (r_rear is in the
    # swing window at master=0); collect the rest in lift-off order.
    observed: list[str] = []
    last_airborne: set[str] = set()
    for _ in range(600):
        out = ctrl.update(dt=0.02)
        airborne = set(_airborne(out))
        for n in airborne - last_airborne:
            observed.append(n)
        last_airborne = airborne
        if ctrl.state is DisengagementState.STAND:
            break
    assert observed[0] == "r_rear", (
        f"first leg to swing should be r_rear (in swing at master=0), got {observed[0]}"
    )
    assert observed[1:] == expected_after_swing, (
        f"stance order {observed[1:]} does not match descending-phase {expected_after_swing}"
    )


# -----------------------------------------------------------------------------
# Stability — at most gait-natural concurrent airborne legs
# -----------------------------------------------------------------------------


@pytest.mark.parametrize(
    "phase_offsets, duty_factor, max_airborne, gait",
    [
        (TRIPOD_OFFSETS, TRIPOD_DUTY, 3, "tripod"),
        (METACHRONAL_OFFSETS, RIPPLE_DUTY, 2, "ripple"),
        (METACHRONAL_OFFSETS, WAVE_DUTY, 1, "wave"),
    ],
)
def test_disengagement_never_exceeds_gait_natural_airborne_count(
    phase_offsets, duty_factor, max_airborne, gait
):
    # Each gait's swing window covers (1 − β) of the cycle. Sequential
    # groups are strictly more conservative than the gait's overlapping
    # swing windows, so the simultaneous-airborne count tops out at
    # the gait's mid-walk maximum: tripod 3, ripple 2, wave 1.
    ctrl = _controller()
    nominal = _flat_stance()
    _begin(
        ctrl,
        last_targets=_offset_all_legs(nominal),
        swing_flags={n: False for n in LEG_NAMES},
        phase_offsets=phase_offsets,
        duty_factor=duty_factor,
    )

    for _ in range(600):
        out = ctrl.update(dt=0.02)
        airborne = _airborne(out)
        assert len(airborne) <= max_airborne, (
            f"[{gait}] {len(airborne)} legs airborne at once ({airborne})"
        )
        if ctrl.state is DisengagementState.STAND:
            break
    else:
        raise AssertionError(f"[{gait}] never reached STAND")


@pytest.mark.parametrize(
    "phase_offsets, duty_factor, gait",
    [
        (TRIPOD_OFFSETS, TRIPOD_DUTY, "tripod"),
        (METACHRONAL_OFFSETS, RIPPLE_DUTY, "ripple"),
        (METACHRONAL_OFFSETS, WAVE_DUTY, "wave"),
    ],
)
def test_disengagement_keeps_at_least_three_legs_grounded(
    phase_offsets, duty_factor, gait
):
    ctrl = _controller()
    nominal = _flat_stance()
    _begin(
        ctrl,
        last_targets=_offset_all_legs(nominal),
        swing_flags={n: False for n in LEG_NAMES},
        phase_offsets=phase_offsets,
        duty_factor=duty_factor,
    )

    for _ in range(600):
        out = ctrl.update(dt=0.02)
        stance_count = sum(1 for n in LEG_NAMES if out[n].stance)
        assert stance_count >= 3, (
            f"[{gait}] only {stance_count} stance legs ({_airborne(out)})"
        )
        if ctrl.state is DisengagementState.STAND:
            break


# -----------------------------------------------------------------------------
# Stop-time bound: tripod is dramatically faster than the old ladder
# -----------------------------------------------------------------------------


def test_tripod_stop_time_is_bounded_by_two_swings():
    # Tripod has two offset groups. With max_swing_time=0.2 and the
    # adaptive timing for a 4 cm planar displacement at 0.333 m/s
    # (raw 0.12 s, clamped to 0.2 s), each group lands inside one
    # max_swing_time. Total must fit within 2 * max_swing_time plus a
    # small slack for the per-tick discretisation.
    max_swing = 0.2
    ctrl = _controller(min_swing_time=0.1, max_swing_time=max_swing)
    nominal = _flat_stance()
    _begin(
        ctrl,
        last_targets=_offset_all_legs(nominal),
        swing_flags={n: False for n in LEG_NAMES},
    )
    dt = 0.02
    elapsed = 0.0
    for _ in range(600):
        ctrl.update(dt=dt)
        elapsed += dt
        if ctrl.state is DisengagementState.STAND:
            break
    assert elapsed <= 2 * max_swing + 3 * dt, (
        f"tripod stop took {elapsed:.3f} s, exceeds 2*max_swing_time+slack"
    )


# -----------------------------------------------------------------------------
# Per-leg adaptive timing
# -----------------------------------------------------------------------------


def test_adaptive_timing_clamps_close_leg_to_min_swing():
    # A leg displaced by < min_swing_time * max_foot_speed lands in
    # min_swing_time, not faster.
    min_swing = 0.1
    ctrl = _controller(min_swing_time=min_swing, max_swing_time=0.4, max_foot_speed=0.333)
    nominal = _flat_stance()
    targets = dict(nominal)
    nx, ny, nz = nominal["l_front"]
    # 1 mm displacement, well below the min_swing * max_foot_speed
    # = 0.0333 m floor.
    targets["l_front"] = (nx + 0.001, ny, nz + 0.005)
    flags = {n: False for n in LEG_NAMES}
    flags["l_front"] = True
    _begin(ctrl, last_targets=targets, swing_flags=flags)

    ticks = 0
    while not out_airborne_empty(ctrl):
        ctrl.update(dt=0.02)
        ticks += 1
        if ticks > 50:
            break
    # 0.1 s / 0.02 s per tick = 5 ticks. Allow ±1 for the final snap.
    assert 4 <= ticks <= 6


def test_adaptive_timing_caps_far_leg_at_max_swing():
    # A leg displaced by more than max_swing_time * max_foot_speed
    # lands in max_swing_time, not slower.
    max_swing = 0.2
    ctrl = _controller(min_swing_time=0.1, max_swing_time=max_swing, max_foot_speed=0.1)
    nominal = _flat_stance()
    targets = dict(nominal)
    nx, ny, nz = nominal["l_front"]
    # 0.1 m displacement at 0.1 m/s would take 1.0 s; clamp to 0.2 s.
    targets["l_front"] = (nx + 0.10, ny, nz + 0.01)
    flags = {n: False for n in LEG_NAMES}
    flags["l_front"] = True
    _begin(ctrl, last_targets=targets, swing_flags=flags)

    ticks = 0
    while not out_airborne_empty(ctrl):
        ctrl.update(dt=0.02)
        ticks += 1
        if ticks > 50:
            break
    # 0.2 s / 0.02 s per tick = 10 ticks. ±1 for the snap.
    assert 9 <= ticks <= 11


def out_airborne_empty(ctrl: DisengagementController) -> bool:
    """True when no swing is currently in progress."""

    return not ctrl._swings  # type: ignore[attr-defined]


# -----------------------------------------------------------------------------
# Apex behaviour
# -----------------------------------------------------------------------------


def test_grounded_leg_lifts_full_clearance():
    # A grounded leg's apex sits at target_z + swing_clearance (the
    # full normal lift).
    swing_clearance = 0.03
    ctrl = _controller(swing_clearance=swing_clearance)
    nominal = _flat_stance()
    targets = dict(nominal)
    nx, ny, nz = nominal["l_front"]
    targets["l_front"] = (nx + 0.04, ny - 0.03, nz)
    _begin(
        ctrl,
        last_targets=targets,
        swing_flags={n: False for n in LEG_NAMES},
    )

    max_z = nz
    for _ in range(200):
        out = ctrl.update(dt=0.005)
        max_z = max(max_z, out["l_front"].foot_target[2])
        if ctrl.state is DisengagementState.STAND:
            break
    assert max_z == pytest.approx(nz + swing_clearance, abs=2e-4)


def test_airborne_leg_already_above_apex_threshold_does_not_bounce_higher():
    # A leg that stopped well above the apex threshold descends with
    # no extra lift — the max z along its trajectory equals origin_z.
    swing_clearance = 0.03
    ctrl = _controller(swing_clearance=swing_clearance)
    nominal = _flat_stance()
    target_z = nominal["l_front"][2]
    origin_z = target_z + swing_clearance + 0.01
    targets = dict(nominal)
    nx, ny, _ = nominal["l_front"]
    targets["l_front"] = (nx + 0.04, ny - 0.03, origin_z)
    flags = {n: False for n in LEG_NAMES}
    flags["l_front"] = True
    _begin(ctrl, last_targets=targets, swing_flags=flags)

    max_z = origin_z
    for _ in range(200):
        out = ctrl.update(dt=0.005)
        max_z = max(max_z, out["l_front"].foot_target[2])
        if ctrl.state is DisengagementState.STAND:
            break
    assert max_z <= origin_z + 1e-6, (
        f"trajectory peaked at {max_z:.5f} m, above origin_z {origin_z:.5f}"
    )


def test_airborne_leg_just_above_floor_still_rises_to_apex():
    # A leg between the floor and target_z + swing_clearance gets a
    # partial lift to the same apex a grounded leg would use. Worst
    # case for a linear interpolation that would skim along the floor.
    swing_clearance = 0.03
    ctrl = _controller(swing_clearance=swing_clearance)
    nominal = _flat_stance()
    target_z = nominal["l_front"][2]
    origin_z = target_z + 0.0005
    targets = dict(nominal)
    nx, ny, _ = nominal["l_front"]
    targets["l_front"] = (nx + 0.05, ny + 0.03, origin_z)
    flags = {n: False for n in LEG_NAMES}
    flags["l_front"] = True
    _begin(ctrl, last_targets=targets, swing_flags=flags)

    max_z = origin_z
    for _ in range(200):
        out = ctrl.update(dt=0.005)
        max_z = max(max_z, out["l_front"].foot_target[2])
        if ctrl.state is DisengagementState.STAND:
            break
    assert max_z == pytest.approx(target_z + swing_clearance, abs=2e-4)


# -----------------------------------------------------------------------------
# Rest-to-rest landing
# -----------------------------------------------------------------------------


def test_swing_decelerates_to_rest_at_landing():
    # The rest-to-rest swing arc must land at zero velocity so the
    # foot does not slam into the floor. Oversample with a tight
    # controller_dt; the last airborne sample should sit essentially
    # at the nominal landing spot.
    controller_dt = 0.001
    ctrl = _controller(
        min_swing_time=0.1,
        max_swing_time=0.4,
        controller_dt=controller_dt,
    )
    nominal = _flat_stance()
    targets = dict(nominal)
    nx, ny, nz = nominal["l_front"]
    stride = (0.08, -0.06, 0.04)
    targets["l_front"] = (nx + stride[0], ny + stride[1], nz + stride[2])
    flags = {n: False for n in LEG_NAMES}
    flags["l_front"] = True
    _begin(ctrl, last_targets=targets, swing_flags=flags)

    samples: list[tuple[float, float, float]] = []
    for _ in range(2000):
        out = ctrl.update(dt=controller_dt)
        if not out["l_front"].stance:
            samples.append(out["l_front"].foot_target)
        if ctrl.state is DisengagementState.STAND:
            break

    last = samples[-1]
    nominal_xyz = nominal["l_front"]
    residual = max(abs(last[i] - nominal_xyz[i]) for i in range(3))
    assert residual < 1e-4, f"residual {residual:.6f} m above rest-to-rest floor"


# -----------------------------------------------------------------------------
# Final position and steady-state STAND
# -----------------------------------------------------------------------------


@pytest.mark.parametrize(
    "phase_offsets, duty_factor, gait",
    [
        (TRIPOD_OFFSETS, TRIPOD_DUTY, "tripod"),
        (METACHRONAL_OFFSETS, RIPPLE_DUTY, "ripple"),
        (METACHRONAL_OFFSETS, WAVE_DUTY, "wave"),
    ],
)
def test_every_leg_ends_at_nominal(phase_offsets, duty_factor, gait):
    ctrl = _controller()
    nominal = _flat_stance()
    _begin(
        ctrl,
        last_targets=_offset_all_legs(nominal),
        swing_flags={n: False for n in LEG_NAMES},
        phase_offsets=phase_offsets,
        duty_factor=duty_factor,
    )
    out = _run_to_stand(ctrl)
    for name in LEG_NAMES:
        assert out[name].foot_target == pytest.approx(nominal[name], abs=1e-9), (
            f"[{gait}] {name} ended at {out[name].foot_target}"
        )
        assert out[name].stance is True


def test_stand_steady_state_emits_nominal():
    ctrl = _controller()
    nominal = _flat_stance()
    _begin(
        ctrl,
        last_targets=_offset_all_legs(nominal),
        swing_flags={n: False for n in LEG_NAMES},
    )
    _run_to_stand(ctrl)
    # An extra tick well past STAND must keep emitting nominal.
    out = ctrl.update(dt=0.02)
    for name in LEG_NAMES:
        assert out[name].foot_target == nominal[name]
        assert out[name].stance is True
        assert out[name].phase == 0.0


# -----------------------------------------------------------------------------
# No twitch on legs already at nominal
# -----------------------------------------------------------------------------


def test_legs_at_nominal_never_lift():
    # Five legs at nominal, one offset. Only the offset leg should
    # ever leave the ground.
    ctrl = _controller()
    nominal = _flat_stance()
    targets = dict(nominal)
    nx, ny, nz = nominal["r_middle"]
    targets["r_middle"] = (nx + 0.03, ny, nz)
    _begin(
        ctrl,
        last_targets=targets,
        swing_flags={n: False for n in LEG_NAMES},
    )

    lifted_legs: set[str] = set()
    for _ in range(400):
        out = ctrl.update(dt=0.02)
        for n in LEG_NAMES:
            if not out[n].stance:
                lifted_legs.add(n)
        if ctrl.state is DisengagementState.STAND:
            break
    assert lifted_legs == {"r_middle"}, (
        f"legs other than the off-nominal one were lifted: {lifted_legs}"
    )


def test_grounded_legs_hold_position_until_their_group_runs():
    # While the airborne group is mid-swing, every grounded leg must
    # stay exactly where it was — no drift while another leg moves.
    ctrl = _controller()
    nominal = _flat_stance()
    targets = _offset_all_legs(nominal)
    # Tripod A airborne with a visible offset; tripod B grounded but
    # also offset (their group hasn't run yet).
    for n in TRIPOD_A:
        nx, ny, nz = nominal[n]
        targets[n] = (nx + 0.04, ny - 0.03, nz + 0.02)
    flags = {n: (n in TRIPOD_A) for n in LEG_NAMES}
    frozen_b = {n: targets[n] for n in TRIPOD_B}
    _begin(ctrl, last_targets=targets, swing_flags=flags)

    saw_a_airborne = False
    for _ in range(50):
        out = ctrl.update(dt=0.02)
        a_airborne = any(not out[n].stance for n in TRIPOD_A)
        if a_airborne:
            saw_a_airborne = True
            for n in TRIPOD_B:
                assert out[n].foot_target == frozen_b[n]
                assert out[n].stance is True
        elif saw_a_airborne:
            return  # tripod A has finished, test passed
    raise AssertionError("tripod A never lifted")
