import math

import pytest

from hexa_control.body_velocity_limiter import BodyVelocityLimiter


DT = 0.02  # 50 Hz control tick


def test_construction_rejects_non_positive_taus():
    with pytest.raises(ValueError):
        BodyVelocityLimiter(tau_linear=0.0, tau_angular=0.3)
    with pytest.raises(ValueError):
        BodyVelocityLimiter(tau_linear=0.3, tau_angular=-0.1)


def test_step_input_follows_first_order_envelope():
    """For a constant target the state tracks the closed-form
    first-order response: ``v[n] = target * (1 - exp(-n*dt/tau))``."""
    tau = 0.3
    lim = BodyVelocityLimiter(tau_linear=tau, tau_angular=tau)
    target = (1.0, 0.0, 0.0)
    for n in range(1, 500):
        v = lim.step(target, DT)
        expected = 1.0 - math.exp(-n * DT / tau)
        assert v[0] == pytest.approx(expected, abs=1e-12)
        assert v[1] == 0.0
        assert v[2] == 0.0
        if expected > 1.0 - 1e-9:
            break


def test_angular_uses_independent_tau():
    """Angular and linear branches each respect their own tau."""
    tau_lin = 1.0
    tau_ang = 0.1
    lim = BodyVelocityLimiter(tau_linear=tau_lin, tau_angular=tau_ang)
    target = (0.0, 0.0, 3.0)
    alpha_ang = 1.0 - math.exp(-DT / tau_ang)
    v = lim.step(target, DT)
    assert v[2] == pytest.approx(3.0 * alpha_ang)
    # Linear branch is untouched because the linear target is zero.
    assert v[0] == 0.0
    assert v[1] == 0.0
    v = lim.step(target, DT)
    expected = 3.0 * (1.0 - math.exp(-2 * DT / tau_ang))
    assert v[2] == pytest.approx(expected)


def test_single_tick_response_is_alpha_scaled():
    """A small demand from rest produces ``alpha * target`` in one tick —
    no slew-rate cap, the response is proportional to the error."""
    tau = 0.1
    lim = BodyVelocityLimiter(tau_linear=tau, tau_angular=tau)
    alpha = 1.0 - math.exp(-DT / tau)
    v = lim.step((0.5, 0.5, 0.0), DT)
    assert v[0] == pytest.approx(0.5 * alpha)
    assert v[1] == pytest.approx(0.5 * alpha)


def test_direction_reversal_passes_through_zero():
    tau = 0.1
    lim = BodyVelocityLimiter(tau_linear=tau, tau_angular=tau)
    lim.reset((0.5, 0.0, 0.0))
    alpha = 1.0 - math.exp(-DT / tau)
    seen_zero_crossing = False
    last_vx = 0.5
    for _ in range(1000):
        v = lim.step((-0.5, 0.0, 0.0), DT)
        # Per-tick change equals alpha * (target - current); bound it
        # with a small numerical slack.
        max_change = alpha * abs(-0.5 - last_vx) + 1e-12
        assert abs(v[0] - last_vx) <= max_change
        if last_vx > 0.0 >= v[0]:
            seen_zero_crossing = True
        last_vx = v[0]
        if v[0] <= -0.5 + 1e-9:
            break
    assert seen_zero_crossing
    assert last_vx == pytest.approx(-0.5, abs=1e-6)


def test_linear_response_is_isotropic():
    """A diagonal step of magnitude M settles in the same wall time as
    an axis-aligned step of magnitude M. Per-axis first-order filtering
    with a shared tau makes this fall out — each component approaches
    its target along the same exponential, so the vector magnitude
    rides the same envelope regardless of axis alignment."""
    tau = 0.2
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
            if math.hypot(v[0], v[1]) >= target_mag - 1e-6:
                return n
        raise AssertionError("did not reach target")

    lim_axis = BodyVelocityLimiter(tau_linear=tau, tau_angular=0.1)
    lim_diag = BodyVelocityLimiter(tau_linear=tau, tau_angular=0.1)
    assert ticks_to_reach(lim_axis, target_axis) == ticks_to_reach(
        lim_diag, target_diag
    )


def test_reset_clears_state_to_default():
    lim = BodyVelocityLimiter(tau_linear=0.3, tau_angular=0.3)
    lim.step((1.0, 1.0, 1.0), DT)
    assert lim.state != (0.0, 0.0, 0.0)
    lim.reset()
    assert lim.state == (0.0, 0.0, 0.0)


def test_reset_to_arbitrary_value():
    lim = BodyVelocityLimiter(tau_linear=0.3, tau_angular=0.3)
    lim.reset((0.1, -0.2, 0.3))
    assert lim.state == (0.1, -0.2, 0.3)
    # A step toward the same value is a no-op (target − state == 0).
    v = lim.step((0.1, -0.2, 0.3), DT)
    assert v == pytest.approx((0.1, -0.2, 0.3))


def test_nonpositive_dt_is_a_noop():
    lim = BodyVelocityLimiter(tau_linear=0.3, tau_angular=0.3)
    lim.reset((0.1, 0.0, 0.0))
    assert lim.step((1.0, 0.0, 0.0), 0.0) == (0.1, 0.0, 0.0)
    assert lim.step((1.0, 0.0, 0.0), -0.01) == (0.1, 0.0, 0.0)


def test_user_scenario_yaw_release_does_not_jump_vx():
    """Reproduces the migration-plan failure scenario: the filter sees
    post-``scale_to_envelope`` output. While ω is held at max,
    ``scale_to_envelope`` suppresses v_x. When ω is released the
    envelope's v_x output jumps to its full demand in one tick. The
    filter must absorb that step instead of publishing it, even though
    ω itself collapses faster (its tau is deliberately smaller)."""
    tau_lin = 0.3
    tau_ang = 0.05  # deliberately fast so omega can drop quickly
    lim = BodyVelocityLimiter(tau_linear=tau_lin, tau_angular=tau_ang)

    # Phase 1: post-envelope output with ω held — v_x is suppressed.
    # Settle for plenty of time constants in both axes.
    suppressed_vx = 0.08
    omega = 3.0
    for _ in range(2000):
        lim.step((suppressed_vx, 0.0, omega), DT)
    assert lim.state[0] == pytest.approx(suppressed_vx, abs=1e-9)
    assert lim.state[2] == pytest.approx(omega, abs=1e-9)

    # Phase 2: user releases ω. scale_to_envelope no longer suppresses,
    # so the envelope's v_x output steps from 0.08 to 0.20 in one tick.
    unsuppressed_vx = 0.20
    alpha_lin = 1.0 - math.exp(-DT / tau_lin)
    expected_after_one_tick = (
        suppressed_vx + alpha_lin * (unsuppressed_vx - suppressed_vx)
    )
    v = lim.step((unsuppressed_vx, 0.0, 0.0), DT)
    # First tick is alpha-scaled — the published v_x does not jump.
    assert v[0] == pytest.approx(expected_after_one_tick, abs=1e-12)
    assert v[0] < unsuppressed_vx
    # And it converges to the new target within a finite horizon.
    last_vx = v[0]
    for _ in range(2000):
        last_vx = lim.step((unsuppressed_vx, 0.0, 0.0), DT)[0]
        if last_vx >= unsuppressed_vx - 1e-6:
            break
    assert last_vx == pytest.approx(unsuppressed_vx, abs=1e-5)
