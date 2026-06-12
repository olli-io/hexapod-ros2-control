import math

from hexa_posture import (
    AnimationContext,
    Breathing,
    GaitBounce,
    GaitSway,
    IDENTITY,
    Stack,
    Still,
)


def _ctx(
    t=0.0,
    walking=False,
    gait_phase=None,
    gait_name=None,
    support_centroid_xy=None,
    swing_lift_z=None,
):
    return AnimationContext(
        t=t,
        walking=walking,
        gait_phase=gait_phase,
        gait_name=gait_name,
        support_centroid_xy=support_centroid_xy,
        swing_lift_z=swing_lift_z,
    )


def test_still_always_identity():
    s = Still()
    assert s(_ctx(t=0.0)) == IDENTITY
    assert s(_ctx(t=10.0, walking=True)) == IDENTITY


def test_breathing_is_silent_while_walking():
    b = Breathing()
    assert b(_ctx(t=1.0, walking=True)) == IDENTITY
    assert b(_ctx(t=2.7, walking=True)) == IDENTITY


def test_breathing_emits_z_offset_while_idle():
    b = Breathing(amplitude=0.005, period=4.0)
    # Quarter-period — sin(pi/2) = 1, so z hits the amplitude peak.
    out = b(_ctx(t=1.0, walking=False))
    assert math.isclose(out.z, 0.005, abs_tol=1e-12)
    # All other axes stay zero.
    assert out.x == 0.0 and out.y == 0.0
    assert out.roll == 0.0 and out.pitch == 0.0 and out.yaw == 0.0


def test_breathing_completes_a_cycle():
    b = Breathing(amplitude=0.005, period=4.0)
    # At t=0 and t=period the sine returns to zero.
    assert math.isclose(b(_ctx(t=0.0)).z, 0.0, abs_tol=1e-12)
    assert math.isclose(b(_ctx(t=4.0)).z, 0.0, abs_tol=1e-9)


def test_stack_sums_layer_outputs():
    # Two breathing layers at the same instant should add their offsets.
    b1 = Breathing(amplitude=0.003, period=4.0)
    b2 = Breathing(amplitude=0.002, period=4.0)
    stack = Stack(layers=(b1, b2))
    out = stack(_ctx(t=1.0, walking=False))
    assert math.isclose(out.z, 0.005, abs_tol=1e-12)


def test_stack_with_no_layers_is_identity():
    assert Stack(layers=()).__call__(_ctx()) == IDENTITY


def test_gait_sway_identity_when_not_walking():
    sway = GaitSway(gain=1.0, strength=1.0)
    assert sway(_ctx(walking=False, support_centroid_xy=(0.02, -0.01))) == IDENTITY


def test_gait_sway_identity_when_centroid_missing():
    sway = GaitSway(gain=1.0, strength=1.0)
    assert sway(_ctx(walking=True, support_centroid_xy=None)) == IDENTITY


def test_gait_sway_default_strength_is_half():
    # Documenting the conservative default: out of the box GaitSway
    # applies only 50% of the centroid feedforward. Lifting this is
    # an explicit tuning step.
    assert GaitSway().strength == 0.5


def test_gait_sway_is_linear_in_centroid():
    gain = 0.8
    sway = GaitSway(gain=gain, strength=1.0)
    cx, cy = 0.02, -0.015
    out = sway(_ctx(walking=True, support_centroid_xy=(cx, cy)))
    assert math.isclose(out.x, gain * cx, abs_tol=1e-12)
    assert math.isclose(out.y, gain * cy, abs_tol=1e-12)
    assert out.z == 0.0
    assert out.roll == 0.0 and out.pitch == 0.0 and out.yaw == 0.0


def test_gait_sway_strength_scales_output():
    # Strength multiplies gain → output is gain * strength * centroid.
    gain, strength = 1.0, 0.5
    sway = GaitSway(gain=gain, strength=strength)
    cx, cy = 0.04, -0.02
    out = sway(_ctx(walking=True, support_centroid_xy=(cx, cy)))
    assert math.isclose(out.x, gain * strength * cx, abs_tol=1e-12)
    assert math.isclose(out.y, gain * strength * cy, abs_tol=1e-12)


def test_gait_sway_strength_zero_is_identity():
    sway = GaitSway(gain=1.0, strength=0.0)
    assert sway(_ctx(walking=True, support_centroid_xy=(0.05, 0.03))) == IDENTITY


def test_gait_bounce_identity_when_not_walking():
    bounce = GaitBounce(arc_height=0.02, step_height_ref=0.06)
    assert bounce(
        _ctx(walking=False, gait_name="tripod", swing_lift_z=0.06)
    ) == IDENTITY


def test_gait_bounce_identity_when_signal_missing():
    bounce = GaitBounce(arc_height=0.02, step_height_ref=0.06)
    assert bounce(
        _ctx(walking=True, gait_name="tripod", swing_lift_z=None)
    ) == IDENTITY


def test_gait_bounce_identity_when_gait_unknown():
    # No /gait/params seen yet → don't guess; the bounce is too
    # aggressive for any non-tripod gait, so default to silent.
    bounce = GaitBounce(arc_height=0.02, step_height_ref=0.06)
    assert bounce(
        _ctx(walking=True, gait_name=None, swing_lift_z=0.06)
    ) == IDENTITY


def test_gait_bounce_identity_under_non_tripod_gaits():
    # Tetrapod / ripple / crawl / surf all drove the chassis unstable
    # in testing — gate is hard, not a soft attenuation.
    bounce = GaitBounce(arc_height=0.02, step_height_ref=0.06)
    for name in ("tetrapod", "ripple", "crawl", "surf"):
        assert bounce(
            _ctx(walking=True, gait_name=name, swing_lift_z=0.06)
        ) == IDENTITY, f"GaitBounce must stay silent under {name!r}"


def test_gait_bounce_zero_at_rest():
    # All feet on the ground (no swing lift) → body at nominal Z.
    # This is the "leg at AEP" low point the spec calls out.
    bounce = GaitBounce(arc_height=0.02, step_height_ref=0.06)
    out = bounce(_ctx(walking=True, gait_name="tripod", swing_lift_z=0.0))
    assert out == IDENTITY


def test_gait_bounce_peaks_at_apex():
    # Foot at the swing arc apex → body at full arc_height.
    bounce = GaitBounce(arc_height=0.02, step_height_ref=0.06)
    out = bounce(_ctx(walking=True, gait_name="tripod", swing_lift_z=0.06))
    assert math.isclose(out.z, 0.02, abs_tol=1e-12)
    assert out.x == 0.0 and out.y == 0.0
    assert out.roll == 0.0 and out.pitch == 0.0 and out.yaw == 0.0


def test_gait_bounce_is_linear_in_swing_lift():
    bounce = GaitBounce(arc_height=0.02, step_height_ref=0.06)
    # Half-apex foot → half-height body lift.
    out = bounce(_ctx(walking=True, gait_name="tripod", swing_lift_z=0.03))
    assert math.isclose(out.z, 0.01, abs_tol=1e-12)


def test_gait_bounce_clamps_above_reference_height():
    # Signal exceeding step_height_ref shouldn't push the body
    # past arc_height — defends downstream PoseLimits against any
    # transient overshoot in /legs/targets.
    bounce = GaitBounce(arc_height=0.02, step_height_ref=0.06)
    out = bounce(_ctx(walking=True, gait_name="tripod", swing_lift_z=0.10))
    assert math.isclose(out.z, 0.02, abs_tol=1e-12)


def test_gait_bounce_arc_height_zero_is_identity():
    # Same off-switch convention as GaitSway.strength=0.
    bounce = GaitBounce(arc_height=0.0, step_height_ref=0.06)
    assert bounce(
        _ctx(walking=True, gait_name="tripod", swing_lift_z=0.06)
    ) == IDENTITY


def test_gait_bounce_stacks_with_gait_sway_on_independent_axes():
    # Sway emits (x, y); bounce emits z. The stack must preserve all
    # three axes — regression guard on pose.add composition.
    sway = GaitSway(gain=1.0, strength=1.0)
    bounce = GaitBounce(arc_height=0.02, step_height_ref=0.06)
    stack = Stack(layers=(sway, bounce))
    out = stack(
        _ctx(
            walking=True,
            gait_name="tripod",
            support_centroid_xy=(0.03, -0.02),
            swing_lift_z=0.06,
        )
    )
    assert math.isclose(out.x, 0.03, abs_tol=1e-12)
    assert math.isclose(out.y, -0.02, abs_tol=1e-12)
    assert math.isclose(out.z, 0.02, abs_tol=1e-12)


def test_gait_sway_stacks_additively_with_breathing():
    # Breathing is silent while walking, so the stack output should
    # equal GaitSway alone — regression on `pose.add` composition.
    sway = GaitSway(gain=1.0, strength=1.0)
    breath = Breathing(amplitude=0.005, period=4.0)
    stack = Stack(layers=(sway, breath))
    cx, cy = 0.03, 0.01
    out = stack(_ctx(t=1.0, walking=True, support_centroid_xy=(cx, cy)))
    assert math.isclose(out.x, cx, abs_tol=1e-12)
    assert math.isclose(out.y, cy, abs_tol=1e-12)
    assert math.isclose(out.z, 0.0, abs_tol=1e-12)
