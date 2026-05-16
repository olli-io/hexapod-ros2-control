import math

from hexa_posture import IDENTITY, BodyPose, PoseLimits, add, clamp, scale


def test_identity_is_all_zeros():
    assert IDENTITY == BodyPose()


def test_add_is_component_wise():
    a = BodyPose(x=0.01, y=0.02, z=0.03, roll=0.1, pitch=0.2, yaw=0.3)
    b = BodyPose(x=-0.005, y=0.0, z=0.01, roll=-0.05, pitch=0.0, yaw=0.1)
    s = add(a, b)
    assert math.isclose(s.x, 0.005)
    assert math.isclose(s.y, 0.02)
    assert math.isclose(s.z, 0.04)
    assert math.isclose(s.roll, 0.05)
    assert math.isclose(s.pitch, 0.2)
    assert math.isclose(s.yaw, 0.4)


def test_scale_is_uniform():
    p = BodyPose(x=0.04, y=-0.02, z=0.01, roll=0.1, pitch=-0.1, yaw=0.2)
    s = scale(p, 0.5)
    assert math.isclose(s.x, 0.02)
    assert math.isclose(s.y, -0.01)
    assert math.isclose(s.z, 0.005)
    assert math.isclose(s.roll, 0.05)
    assert math.isclose(s.pitch, -0.05)
    assert math.isclose(s.yaw, 0.1)


def test_clamp_caps_each_axis_symmetrically():
    limits = PoseLimits(x=0.05, y=0.05, z=0.04, roll=0.3, pitch=0.3, yaw=0.5)
    runaway = BodyPose(x=10.0, y=-10.0, z=1.0, roll=5.0, pitch=-5.0, yaw=2.0)
    c = clamp(runaway, limits)
    assert c == BodyPose(x=0.05, y=-0.05, z=0.04, roll=0.3, pitch=-0.3, yaw=0.5)


def test_clamp_passes_through_in_envelope_values():
    limits = PoseLimits()
    inside = BodyPose(x=0.01, y=-0.02, z=0.005, roll=0.05, pitch=-0.05, yaw=0.1)
    assert clamp(inside, limits) == inside
