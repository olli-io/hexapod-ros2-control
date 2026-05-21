import math

import pytest

from hexa_control.body_velocity_limiter import BodyVelocityLimiter


DT = 0.02  # 50 Hz control tick


def test_construction_rejects_non_positive_accels():
    with pytest.raises(ValueError):
        BodyVelocityLimiter(accel_linear=0.0, accel_angular=1.0)
    with pytest.raises(ValueError):
        BodyVelocityLimiter(accel_linear=1.0, accel_angular=-0.1)


def test_accel_setters_reject_non_positive_values():
    """``accel_linear`` / ``accel_angular`` are mutable for per-gait
    retuning, but the validated property setters refuse non-positive
    values just like the constructor."""
    lim = BodyVelocityLimiter(accel_linear=1.0, accel_angular=1.0)
    with pytest.raises(ValueError):
        lim.accel_linear = 0.0
    with pytest.raises(ValueError):
        lim.accel_linear = -1.0
    with pytest.raises(ValueError):
        lim.accel_angular = 0.0
    with pytest.raises(ValueError):
        lim.accel_angular = -1.0
    # The originals stayed intact after the failed assignments.
    assert lim.accel_linear == 1.0
    assert lim.accel_angular == 1.0


def test_accel_setter_takes_effect_on_next_step():
    """Retuning ``accel_linear`` mid-life changes the per-tick cap on the
    very next ``step`` — there is no internal latch."""
    lim = BodyVelocityLimiter(accel_linear=1.0, accel_angular=1.0)
    v = lim.step((1.0, 0.0, 0.0), DT)
    assert v[0] == pytest.approx(1.0 * DT)
    lim.accel_linear = 4.0
    v = lim.step((1.0, 0.0, 0.0), DT)
    # Second tick advances by the new cap on top of the existing state.
    assert v[0] == pytest.approx(1.0 * DT + 4.0 * DT)


def test_construction_rejects_negative_snap_tols():
    with pytest.raises(ValueError):
        BodyVelocityLimiter(
            accel_linear=1.0, accel_angular=1.0, snap_tol_linear=-1e-6
        )
    with pytest.raises(ValueError):
        BodyVelocityLimiter(
            accel_linear=1.0, accel_angular=1.0, snap_tol_angular=-1e-6
        )


def test_linear_ramp_advances_at_constant_rate():
    """For a constant target far from rest the state advances by exactly
    ``accel_linear * dt`` per tick until it lands within one step of the
    target, then snaps."""
    accel = 1.0
    lim = BodyVelocityLimiter(accel_linear=accel, accel_angular=1.0)
    target = (1.0, 0.0, 0.0)
    step = accel * DT
    for n in range(1, 200):
        v = lim.step(target, DT)
        expected = min(n * step, 1.0)
        assert v[0] == pytest.approx(expected, abs=1e-12)
        assert v[1] == 0.0
        assert v[2] == 0.0
        if expected >= 1.0:
            break
    # Once landed, the state holds at the target.
    v = lim.step(target, DT)
    assert v == pytest.approx((1.0, 0.0, 0.0))


def test_angular_uses_independent_accel():
    """The angular branch has its own cap and does not consume the
    linear cap's budget."""
    accel_lin = 0.1
    accel_ang = 10.0
    lim = BodyVelocityLimiter(accel_linear=accel_lin, accel_angular=accel_ang)
    target = (0.0, 0.0, 3.0)
    step_ang = accel_ang * DT
    v = lim.step(target, DT)
    assert v[2] == pytest.approx(step_ang)
    # Linear stays at rest because the linear target is zero.
    assert v[0] == 0.0
    assert v[1] == 0.0
    v = lim.step(target, DT)
    assert v[2] == pytest.approx(2 * step_ang)


def test_landing_within_one_step_snaps_exactly():
    """When the remaining distance is less than ``accel * dt`` the
    limiter snaps to the target in one tick instead of overshooting."""
    accel = 1.0
    lim = BodyVelocityLimiter(accel_linear=accel, accel_angular=1.0)
    # Place the state one-third of a max-step below the target.
    lim.reset((1.0 - 0.5 * accel * DT, 0.0, 0.0))
    v = lim.step((1.0, 0.0, 0.0), DT)
    assert v[0] == 1.0


def test_direction_reversal_is_acceleration_bounded():
    """The bug fix: from ``+0.5`` toward ``-0.5`` the per-tick change
    never exceeds ``accel_linear * dt``, regardless of whether the
    target spent a tick at zero (deadband-style)."""
    accel = 1.0
    lim = BodyVelocityLimiter(accel_linear=accel, accel_angular=1.0)
    lim.reset((0.5, 0.0, 0.0))
    max_step = accel * DT

    # One tick of zero target (mimics the teleop deadband zeroing the
    # axis as the stick passes through centre), then full reverse.
    last = lim.state
    targets = [(0.0, 0.0, 0.0)] + [(-0.5, 0.0, 0.0)] * 1000
    for tgt in targets:
        v = lim.step(tgt, DT)
        # Bound the per-tick change on the linear vector. Allow a small
        # numerical slack for the final landing tick.
        delta_mag = math.hypot(v[0] - last[0], v[1] - last[1])
        assert delta_mag <= max_step + 1e-12
        last = v
        if v[0] <= -0.5 + 1e-9:
            break
    assert last[0] == pytest.approx(-0.5, abs=1e-9)


def test_vectorial_coupling_on_diagonal_reversal():
    """A diagonal direction flip slews on the linear vector magnitude
    — neither axis snaps independently to zero, and the per-tick
    vector change stays bounded by ``accel_linear * dt``."""
    accel = 1.0
    lim = BodyVelocityLimiter(accel_linear=accel, accel_angular=1.0)
    lim.reset((0.3, 0.3, 0.0))
    max_step = accel * DT

    last = lim.state
    for _ in range(1000):
        v = lim.step((-0.3, -0.3, 0.0), DT)
        delta_mag = math.hypot(v[0] - last[0], v[1] - last[1])
        assert delta_mag <= max_step + 1e-12
        last = v
        if math.hypot(v[0] + 0.3, v[1] + 0.3) < 1e-9:
            break
    assert last[0] == pytest.approx(-0.3, abs=1e-9)
    assert last[1] == pytest.approx(-0.3, abs=1e-9)


def test_release_from_walking_reaches_zero_in_finite_time():
    """A clean release (target = 0 from non-zero state) lands at exact
    zero in finite time, so the engine's ``cmd_zero_tol`` (1e-4 m/s)
    triggers without any special-case snap."""
    accel = 1.0
    lim = BodyVelocityLimiter(accel_linear=accel, accel_angular=1.0)
    lim.reset((0.3, 0.0, 0.0))
    # 0.3 / (1.0 * 0.02) = 15 ticks of capped step, plus one for the
    # final landing snap.
    ticks = 0
    for _ in range(50):
        v = lim.step((0.0, 0.0, 0.0), DT)
        ticks += 1
        if v == (0.0, 0.0, 0.0):
            break
    assert v == (0.0, 0.0, 0.0)
    assert ticks <= 16


def test_linear_response_is_isotropic():
    """A diagonal step of magnitude M settles in the same wall time as
    an axis-aligned step of magnitude M — the vectorial slew operates
    on the magnitude, not on per-axis components."""
    accel = 1.0
    target_mag = 0.2
    target_axis = (target_mag, 0.0, 0.0)
    target_diag = (
        target_mag / math.sqrt(2.0),
        target_mag / math.sqrt(2.0),
        0.0,
    )

    def ticks_to_reach(lim, target):
        for n in range(1, 10_000):
            v = lim.step(target, DT)
            if math.hypot(v[0], v[1]) >= target_mag - 1e-9:
                return n
        raise AssertionError("did not reach target")

    lim_axis = BodyVelocityLimiter(accel_linear=accel, accel_angular=1.0)
    lim_diag = BodyVelocityLimiter(accel_linear=accel, accel_angular=1.0)
    assert ticks_to_reach(lim_axis, target_axis) == ticks_to_reach(
        lim_diag, target_diag
    )


def test_snap_tol_zeroes_floating_dribble():
    """A sub-tolerance residue at the end of a ramp toward zero gets
    snapped exactly to zero, so the engine sees a crisp zero crossing."""
    accel = 1.0
    lim = BodyVelocityLimiter(
        accel_linear=accel,
        accel_angular=1.0,
        snap_tol_linear=1e-4,
        snap_tol_angular=1e-4,
    )
    # Pre-seed a sub-tolerance state then step toward zero. Even though
    # the cap-step alone would land it exactly at zero in one tick, the
    # snap_tol handles cases where it would land at, say, 1e-15.
    lim.reset((5e-5, 0.0, 5e-5))
    v = lim.step((0.0, 0.0, 0.0), DT)
    assert v == (0.0, 0.0, 0.0)


def test_reset_clears_state_to_default():
    lim = BodyVelocityLimiter(accel_linear=1.0, accel_angular=1.0)
    lim.step((1.0, 1.0, 1.0), DT)
    assert lim.state != (0.0, 0.0, 0.0)
    lim.reset()
    assert lim.state == (0.0, 0.0, 0.0)


def test_reset_to_arbitrary_value():
    lim = BodyVelocityLimiter(accel_linear=1.0, accel_angular=1.0)
    lim.reset((0.1, -0.2, 0.3))
    assert lim.state == (0.1, -0.2, 0.3)
    # Stepping toward the same value is a no-op (distance == 0 lands
    # via the snap branch).
    v = lim.step((0.1, -0.2, 0.3), DT)
    assert v == pytest.approx((0.1, -0.2, 0.3))


def test_nonpositive_dt_is_a_noop():
    lim = BodyVelocityLimiter(accel_linear=1.0, accel_angular=1.0)
    lim.reset((0.1, 0.0, 0.0))
    assert lim.step((1.0, 0.0, 0.0), 0.0) == (0.1, 0.0, 0.0)
    assert lim.step((1.0, 0.0, 0.0), -0.01) == (0.1, 0.0, 0.0)


def test_user_scenario_yaw_release_does_not_jump_vx():
    """Reproduces the migration-plan failure scenario: the limiter sees
    post-``scale_to_envelope`` output. While omega is held at max,
    ``scale_to_envelope`` suppresses v_x. When omega is released the
    envelope's v_x output jumps to its full demand in one tick. The
    rate cap must absorb that step instead of publishing it."""
    accel_lin = 1.0
    accel_ang = 20.0  # deliberately fast so omega can drop quickly
    lim = BodyVelocityLimiter(accel_linear=accel_lin, accel_angular=accel_ang)

    # Phase 1: post-envelope output with omega held — v_x is suppressed.
    # Settle for plenty of ticks in both axes.
    suppressed_vx = 0.08
    omega = 3.0
    for _ in range(500):
        lim.step((suppressed_vx, 0.0, omega), DT)
    assert lim.state[0] == pytest.approx(suppressed_vx, abs=1e-9)
    assert lim.state[2] == pytest.approx(omega, abs=1e-9)

    # Phase 2: user releases omega. scale_to_envelope no longer
    # suppresses, so the envelope's v_x output steps from 0.08 to 0.20
    # in one tick. The limiter must cap the per-tick step at
    # accel_lin * dt = 0.02 m/s, not jump to the new target.
    unsuppressed_vx = 0.20
    expected_after_one_tick = suppressed_vx + accel_lin * DT
    v = lim.step((unsuppressed_vx, 0.0, 0.0), DT)
    assert v[0] == pytest.approx(expected_after_one_tick, abs=1e-12)
    assert v[0] < unsuppressed_vx
    # And it converges to the new target within a finite horizon.
    last_vx = v[0]
    for _ in range(500):
        last_vx = lim.step((unsuppressed_vx, 0.0, 0.0), DT)[0]
        if last_vx >= unsuppressed_vx - 1e-9:
            break
    assert last_vx == pytest.approx(unsuppressed_vx, abs=1e-9)
