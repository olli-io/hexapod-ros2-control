import math

from hexa_posture import (
    IDENTITY,
    BodyPose,
    PoseLimits,
    add,
    clamp,
    compose_layered,
    lerp,
    scale,
)


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


def test_lerp_endpoints_and_midpoint():
    a = BodyPose(x=0.01, y=-0.02, z=0.005, roll=0.05, pitch=-0.05, yaw=0.1)
    b = BodyPose(x=0.04, y=-0.01, z=0.03, roll=0.2, pitch=-0.1, yaw=0.3)
    assert lerp(a, b, 0.0) == a  # t=0 returns a exactly
    end = lerp(a, b, 1.0)
    assert math.isclose(end.x, b.x)
    assert math.isclose(end.yaw, b.yaw)
    mid = lerp(a, b, 0.5)
    assert math.isclose(mid.x, 0.025)
    assert math.isclose(mid.y, -0.015)
    assert math.isclose(mid.z, 0.0175)
    assert math.isclose(mid.roll, 0.125)
    assert math.isclose(mid.pitch, -0.075)
    assert math.isclose(mid.yaw, 0.2)


def test_compose_layered_keeps_animation_symmetric():
    # A dialed-in posture beyond the reserved headroom must NOT clip the
    # animation asymmetrically — the pre-fix clamp(add(user, anim)) bug.
    limits = PoseLimits()  # x = 0.05
    reserve = PoseLimits(x=0.02)  # only x matters here
    user = BodyPose(x=0.045)  # beyond user_env (0.03)
    xp = compose_layered(user, BodyPose(x=0.02), limits, reserve).x
    xn = compose_layered(user, BodyPose(x=-0.02), limits, reserve).x
    # Baseline is the user clamped to user_env (0.03); the animation swings
    # a full symmetric +/-0.02 about it and stays inside the 0.05 envelope.
    assert math.isclose(xp, 0.05)
    assert math.isclose(xn, 0.01)
    baseline = 0.5 * (xp + xn)
    assert math.isclose(xp - baseline, baseline - xn)
    assert xp <= limits.x + 1e-12


def test_compose_layered_reserve_exceeding_limit_floors_user_envelope():
    limits = PoseLimits()  # x = 0.05
    reserve = PoseLimits(x=0.08)  # > limit -> user_env.x = 0
    out = compose_layered(BodyPose(x=0.04), BodyPose(x=0.01), limits, reserve)
    assert math.isclose(out.x, 0.01)  # user contributes 0; only the animation
