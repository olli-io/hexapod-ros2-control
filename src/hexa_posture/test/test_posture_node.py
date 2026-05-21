"""Regression tests for posture_node gating against /gait/state."""

import math

from geometry_msgs.msg import Point
from hexa_interfaces.msg import LegState, LegTargets
from hexa_posture.posture_node import (
    POSTURE_ACTIVE_STATES,
    _lpf_step_scalar,
    _lpf_step_xy,
    _max_swing_lift_z,
    _stance_centroid_xy,
)


def test_pause_trio_preserves_body_pose():
    # Regression: PAUSING zeroed body pose.z back to default because the
    # posture node treated `pausing` / `paused` / `resuming` as inactive
    # states and emitted IDENTITY. Pause must only affect the legs.
    for state in ("pausing", "paused", "resuming"):
        assert state in POSTURE_ACTIVE_STATES, (
            f"{state!r} missing from POSTURE_ACTIVE_STATES — "
            "posture node would publish IDENTITY and snap body pose to default"
        )


def test_reseating_preserves_body_pose():
    # The persistent height offset must keep applying while reseat walks
    # the feet to the new nominal — otherwise the body drops mid-ladder.
    assert "reseating" in POSTURE_ACTIVE_STATES


def test_walking_states_are_active():
    for state in ("stand", "engaging", "gait"):
        assert state in POSTURE_ACTIVE_STATES


def test_pre_stand_states_emit_identity():
    # Legs aren't at nominal footprint in these states — composing a
    # body-pose offset would push IK against the wrong configuration.
    for state in ("folded", "initialize", "folding"):
        assert state not in POSTURE_ACTIVE_STATES


def _leg(name, x, y, stance, z=0.0):
    leg = LegState()
    leg.leg_name = name
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    leg.foot_target = p
    leg.phase = 0.0
    leg.stance = stance
    return leg


def _targets(samples):
    """samples: iterable of (name, x, y, stance[, z]) tuples, length 6."""
    msg = LegTargets()
    msg.legs = [_leg(*s) for s in samples]
    return msg


def test_stance_centroid_is_mean_of_stance_legs():
    # Three stance legs at (1, 0), (0, 1), (-1, -1); three swing legs
    # with non-zero coords that must NOT contribute.
    msg = _targets(
        [
            ("l_front", 1.0, 0.0, True),
            ("l_middle", 0.0, 1.0, True),
            ("l_rear", -1.0, -1.0, True),
            ("r_front", 99.0, 99.0, False),
            ("r_middle", 99.0, 99.0, False),
            ("r_rear", 99.0, 99.0, False),
        ]
    )
    cx, cy = _stance_centroid_xy(msg)
    assert math.isclose(cx, 0.0, abs_tol=1e-12)
    assert math.isclose(cy, 0.0, abs_tol=1e-12)


def test_stance_centroid_returns_none_when_polygon_degenerate():
    # Only two stance legs — undefined polygon, helper must signal None
    # so the caller holds the previous filtered value.
    msg = _targets(
        [
            ("l_front", 0.1, 0.1, True),
            ("l_middle", -0.1, 0.0, True),
            ("l_rear", 0.0, 0.0, False),
            ("r_front", 0.0, 0.0, False),
            ("r_middle", 0.0, 0.0, False),
            ("r_rear", 0.0, 0.0, False),
        ]
    )
    assert _stance_centroid_xy(msg) is None


def test_max_swing_lift_picks_highest_swing_foot_above_stance_mean():
    # Two legs swinging at different heights; the higher one drives
    # the bounce — this is what makes ripple/surf overlap collapse
    # to the "main wave" instead of double-counting both airborne
    # legs.
    msg = _targets(
        [
            ("l_front", 0.0, 0.0, False, 0.04),  # swing, low
            ("l_middle", 0.0, 0.0, True, 0.0),
            ("l_rear", 0.0, 0.0, True, 0.0),
            ("r_front", 0.0, 0.0, False, 0.06),  # swing, apex
            ("r_middle", 0.0, 0.0, True, 0.0),
            ("r_rear", 0.0, 0.0, True, 0.0),
        ]
    )
    assert math.isclose(_max_swing_lift_z(msg), 0.06, abs_tol=1e-12)


def test_max_swing_lift_offsets_against_stance_mean():
    # If the stance feet aren't at z=0 (e.g. body lifted), the lift
    # must be measured relative to that ground level — otherwise
    # the body height would feed back into its own bounce.
    msg = _targets(
        [
            ("l_front", 0.0, 0.0, False, -0.04),  # swing apex at -0.04
            ("l_middle", 0.0, 0.0, True, -0.10),
            ("l_rear", 0.0, 0.0, True, -0.10),
            ("r_front", 0.0, 0.0, True, -0.10),
            ("r_middle", 0.0, 0.0, True, -0.10),
            ("r_rear", 0.0, 0.0, True, -0.10),
        ]
    )
    # Ground = mean stance z = -0.10; max swing = -0.04; lift = 0.06.
    assert math.isclose(_max_swing_lift_z(msg), 0.06, abs_tol=1e-12)


def test_max_swing_lift_zero_when_no_legs_swinging():
    # All-stance frame: the bounce signal is observed and quiet, NOT
    # missing — return 0.0 so GaitBounce treats it as the resting
    # altitude (body low at AEP).
    msg = _targets(
        [
            ("l_front", 0.0, 0.0, True),
            ("l_middle", 0.0, 0.0, True),
            ("l_rear", 0.0, 0.0, True),
            ("r_front", 0.0, 0.0, True),
            ("r_middle", 0.0, 0.0, True),
            ("r_rear", 0.0, 0.0, True),
        ]
    )
    assert _max_swing_lift_z(msg) == 0.0


def test_max_swing_lift_returns_none_when_polygon_degenerate():
    # Mid-handover: keep previous signal rather than emit a noisy
    # zero. Same gate as the centroid filter.
    msg = _targets(
        [
            ("l_front", 0.0, 0.0, True, 0.0),
            ("l_middle", 0.0, 0.0, True, 0.0),
            ("l_rear", 0.0, 0.0, False, 0.05),
            ("r_front", 0.0, 0.0, False, 0.05),
            ("r_middle", 0.0, 0.0, False, 0.05),
            ("r_rear", 0.0, 0.0, False, 0.05),
        ]
    )
    assert _max_swing_lift_z(msg) is None


def test_max_swing_lift_clamps_negative_to_zero():
    # If the highest "swinging" foot sits below the stance mean
    # (shouldn't happen mid-swing, but defends against transients
    # at lift-off / touchdown), clamp to 0 rather than feeding a
    # negative lift into GaitBounce.
    msg = _targets(
        [
            ("l_front", 0.0, 0.0, False, -0.01),
            ("l_middle", 0.0, 0.0, True, 0.0),
            ("l_rear", 0.0, 0.0, True, 0.0),
            ("r_front", 0.0, 0.0, True, 0.0),
            ("r_middle", 0.0, 0.0, True, 0.0),
            ("r_rear", 0.0, 0.0, True, 0.0),
        ]
    )
    assert _max_swing_lift_z(msg) == 0.0


def test_lpf_seeds_on_first_sample():
    # First raw sample should be adopted as-is — no transient from (0, 0).
    out = _lpf_step_xy(None, (0.02, -0.01), tau=0.1, dt=0.02)
    assert out == (0.02, -0.01)


def test_lpf_holds_previous_when_raw_missing():
    prev = (0.03, 0.01)
    assert _lpf_step_xy(prev, None, tau=0.1, dt=0.02) == prev


def test_lpf_scalar_seeds_on_first_sample():
    # First raw sample is adopted as-is — no ramp-up from 0 when the
    # gait engine has been running before posture catches up.
    assert _lpf_step_scalar(None, 0.05, tau=0.05, dt=0.02) == 0.05


def test_lpf_scalar_holds_previous_when_raw_missing():
    # Degenerate stance polygon mid-handover → hold previous so
    # GaitBounce sees a continuous signal instead of a zero blip.
    assert _lpf_step_scalar(0.03, None, tau=0.05, dt=0.02) == 0.03


def test_lpf_scalar_settles_after_a_few_tau():
    tau = 0.05
    dt = 0.005
    target = 0.06
    state = 0.0
    n_steps = int(round((4.0 * tau) / dt))
    for _ in range(n_steps):
        state = _lpf_step_scalar(state, target, tau=tau, dt=dt)
    assert abs(state - target) <= 0.05 * target


def test_lpf_settles_after_a_few_tau():
    # Step from (0, 0) to (c, 0). The continuous LPF reaches ~95% at
    # 3τ; the discrete `alpha = dt/(tau+dt)` form settles slightly
    # slower, so we check at 4τ where the residual is well under 5%.
    tau = 0.1
    dt = 0.005
    target = 0.04
    state = (0.0, 0.0)
    n_steps = int(round((4.0 * tau) / dt))
    for _ in range(n_steps):
        state = _lpf_step_xy(state, (target, 0.0), tau=tau, dt=dt)
    assert abs(state[0] - target) <= 0.05 * target
    assert math.isclose(state[1], 0.0, abs_tol=1e-12)
