import math

from hexa_posture import (
    AnimationContext,
    Breathing,
    IDENTITY,
    Stack,
    Still,
)


def _ctx(t=0.0, walking=False, gait_phase=None):
    return AnimationContext(t=t, walking=walking, gait_phase=gait_phase)


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
