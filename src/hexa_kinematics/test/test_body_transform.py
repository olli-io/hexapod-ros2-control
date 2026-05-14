import math

from hexa_kinematics import (
    LegSpec,
    body_to_leg,
    forward_kinematics,
    inverse_kinematics,
    leg_to_body,
)


def _close(a, b, tol=1e-12):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def _leg(mount_xyz=(0.0, 0.0, 0.0), mount_yaw=0.0):
    return LegSpec(
        mount_xyz=mount_xyz,
        mount_yaw=mount_yaw,
        coxa_len=0.05,
        femur_len=0.08,
        tibia_len=0.12,
    )


def test_round_trip_recovers_body_point():
    leg = _leg(mount_xyz=(0.1, 0.05, 0.02), mount_yaw=math.radians(30))
    p_body = (0.2, 0.1, -0.05)
    assert _close(leg_to_body(body_to_leg(p_body, leg), leg), p_body)


def test_mount_position_maps_to_leg_origin():
    leg = _leg(mount_xyz=(0.1, 0.05, 0.02), mount_yaw=math.radians(45))
    assert _close(body_to_leg(leg.mount_xyz, leg), (0.0, 0.0, 0.0))


def test_yaw_rotates_xy_into_leg_frame():
    # Leg mounted at the origin with mount_yaw = 90° — its +x axis aligns
    # with body +y. So a body point at (0, 1, 0) is at (1, 0, 0) in the
    # leg frame, and (1, 0, 0) is at (0, -1, 0).
    leg = _leg(mount_yaw=math.pi / 2)
    assert _close(body_to_leg((0.0, 1.0, 0.0), leg), (1.0, 0.0, 0.0))
    assert _close(body_to_leg((1.0, 0.0, 0.0), leg), (0.0, -1.0, 0.0))


def test_z_is_unchanged_by_yaw():
    leg = _leg(mount_yaw=0.5)
    assert body_to_leg((0.0, 0.0, -0.07), leg)[2] == -0.07


def test_body_target_round_trip_through_ik_and_fk():
    # End-to-end stack: body → leg → IK → FK → leg → body. Catches sign-flip
    # bugs that the per-module tests can't see in isolation.
    leg = _leg(mount_xyz=(0.10, 0.06, 0.0), mount_yaw=math.radians(-30))
    p_body = (0.18, 0.02, -0.08)

    p_leg_in = body_to_leg(p_body, leg)
    angles = inverse_kinematics(p_leg_in, leg)
    p_leg_out = forward_kinematics(angles, leg)
    p_body_out = leg_to_body(p_leg_out, leg)

    assert _close(p_leg_out, p_leg_in, tol=1e-9)
    assert _close(p_body_out, p_body, tol=1e-9)
