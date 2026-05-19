import math

import pytest

from hexa_control.body_velocity_limiter import BodyVelocityLimiter


DT = 0.02  # 50 Hz control tick


def test_construction_rejects_non_positive_accels():
    with pytest.raises(ValueError):
        BodyVelocityLimiter(max_linear_accel=0.0, max_angular_accel=1.0)
    with pytest.raises(ValueError):
        BodyVelocityLimiter(max_linear_accel=1.0, max_angular_accel=-0.1)


def test_step_input_ramps_at_exactly_max_accel():
    lim = BodyVelocityLimiter(max_linear_accel=0.5, max_angular_accel=2.0)
    target = (1.0, 0.0, 0.0)
    # One tick: change bounded by max_linear_accel * dt.
    v = lim.step(target, DT)
    assert v[0] == pytest.approx(0.5 * DT)
    assert v[1] == 0.0
    assert v[2] == 0.0
    # Steady ramp: each tick adds exactly max_linear_accel * dt until
    # the target is reached.
    for tick in range(2, 200):
        v = lim.step(target, DT)
        expected = min(0.5 * DT * tick, 1.0)
        assert v[0] == pytest.approx(expected, abs=1e-9)
        if v[0] >= 1.0 - 1e-12:
            break
    assert v == pytest.approx((1.0, 0.0, 0.0))


def test_angular_step_ramps_at_max_angular_accel():
    lim = BodyVelocityLimiter(max_linear_accel=0.5, max_angular_accel=2.0)
    target = (0.0, 0.0, 3.0)
    v = lim.step(target, DT)
    assert v[2] == pytest.approx(2.0 * DT)
    v = lim.step(target, DT)
    assert v[2] == pytest.approx(2.0 * DT * 2)


def test_target_within_slew_window_is_reached_in_one_tick():
    lim = BodyVelocityLimiter(max_linear_accel=1.0, max_angular_accel=10.0)
    # max linear delta per tick = 1.0 * 0.02 = 0.02; target is half that.
    v = lim.step((0.005, 0.005, 0.0), DT)
    assert v[0] == pytest.approx(0.005)
    assert v[1] == pytest.approx(0.005)


def test_direction_reversal_passes_through_zero():
    lim = BodyVelocityLimiter(max_linear_accel=1.0, max_angular_accel=10.0)
    lim.reset((0.5, 0.0, 0.0))
    # Command opposite direction; per-tick change capped at 1.0 * 0.02 = 0.02.
    seen_zero_crossing = False
    last_vx = 0.5
    for _ in range(100):
        v = lim.step((-0.5, 0.0, 0.0), DT)
        assert abs(v[0] - last_vx) <= 1.0 * DT + 1e-12
        if last_vx > 0.0 and v[0] <= 0.0:
            seen_zero_crossing = True
        last_vx = v[0]
        if v[0] <= -0.5 + 1e-9:
            break
    assert seen_zero_crossing
    assert v[0] == pytest.approx(-0.5)


def test_linear_slew_is_isotropic():
    """A diagonal slew of magnitude M finishes in the same wall time as
    an axis-aligned slew of magnitude M. If the limiter clamped each
    axis independently, the diagonal slew would finish √2× faster."""
    lin = 0.5
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
            mag = math.hypot(v[0], v[1])
            if mag >= target_mag - 1e-9:
                return n
        raise AssertionError("did not reach target")

    lim_axis = BodyVelocityLimiter(max_linear_accel=lin, max_angular_accel=2.0)
    lim_diag = BodyVelocityLimiter(max_linear_accel=lin, max_angular_accel=2.0)
    n_axis = ticks_to_reach(lim_axis, target_axis)
    n_diag = ticks_to_reach(lim_diag, target_diag)
    assert n_axis == n_diag


def test_reset_clears_state_to_default():
    lim = BodyVelocityLimiter(max_linear_accel=0.5, max_angular_accel=2.0)
    lim.step((1.0, 1.0, 1.0), DT)
    assert lim.state != (0.0, 0.0, 0.0)
    lim.reset()
    assert lim.state == (0.0, 0.0, 0.0)


def test_reset_to_arbitrary_value():
    lim = BodyVelocityLimiter(max_linear_accel=0.5, max_angular_accel=2.0)
    lim.reset((0.1, -0.2, 0.3))
    assert lim.state == (0.1, -0.2, 0.3)
    # Next step is bounded relative to the new state.
    v = lim.step((0.1, -0.2, 0.3), DT)
    assert v == pytest.approx((0.1, -0.2, 0.3))


def test_nonpositive_dt_is_a_noop():
    lim = BodyVelocityLimiter(max_linear_accel=0.5, max_angular_accel=2.0)
    lim.reset((0.1, 0.0, 0.0))
    assert lim.step((1.0, 0.0, 0.0), 0.0) == (0.1, 0.0, 0.0)
    assert lim.step((1.0, 0.0, 0.0), -0.01) == (0.1, 0.0, 0.0)


def test_user_scenario_yaw_release_does_not_jump_vx():
    """Reproduces the failure scenario in the migration plan: the
    limiter sees the post-scale_to_envelope output. While ω is held at
    max, ``scale_to_envelope`` suppresses v_x. When ω is released, the
    envelope's v_x output jumps to its full demand in one tick. The
    limiter must bound the per-tick change in v_x by max_linear_accel·dt
    even when omega itself is allowed to drop faster (different bound).
    """
    max_lin = 0.5
    max_ang = 5.0  # deliberately loose so omega can drop quickly
    lim = BodyVelocityLimiter(
        max_linear_accel=max_lin, max_angular_accel=max_ang
    )

    # Phase 1: post-envelope output while ω is held — v_x is suppressed.
    suppressed_vx = 0.08
    omega = 3.0
    for _ in range(200):
        lim.step((suppressed_vx, 0.0, omega), DT)
    assert lim.state[0] == pytest.approx(suppressed_vx, abs=1e-6)
    assert lim.state[2] == pytest.approx(omega, abs=1e-6)

    # Phase 2: user releases ω. scale_to_envelope no longer suppresses,
    # so the envelope's v_x output steps from 0.08 to 0.20 in one tick.
    unsuppressed_vx = 0.20
    last_vx = lim.state[0]
    for tick in range(50):
        v = lim.step((unsuppressed_vx, 0.0, 0.0), DT)
        per_tick_change = v[0] - last_vx
        assert per_tick_change <= max_lin * DT + 1e-12
        last_vx = v[0]
        if v[0] >= unsuppressed_vx - 1e-9:
            break
    assert last_vx == pytest.approx(unsuppressed_vx)
