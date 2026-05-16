import math

from hexa_kinematics import (
    IDENTITY_BODY_POSE,
    BodyPose,
    apply_body_pose,
)


def _close(a, b, tol=1e-12):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_identity_pose_is_passthrough():
    assert _close(apply_body_pose((0.1, -0.2, 0.05), IDENTITY_BODY_POSE), (0.1, -0.2, 0.05))


def test_translation_subtracts_from_target():
    # Body moves +x by 0.05 → a foot at (0.20, 0, -0.10) in the nominal
    # frame appears 0.05 closer in body +x to the body.
    pose = BodyPose(x=0.05)
    assert _close(apply_body_pose((0.20, 0.0, -0.10), pose), (0.15, 0.0, -0.10))


def test_yaw_rotates_xy_oppositely():
    # Body yaws +90°; a foot ahead of the body (body +x) in the nominal
    # frame now appears on the body's right (offset body −y).
    pose = BodyPose(yaw=math.pi / 2)
    assert _close(apply_body_pose((1.0, 0.0, 0.0), pose), (0.0, -1.0, 0.0))


def test_pitch_rotates_xz_oppositely():
    pose = BodyPose(pitch=math.pi / 2)
    # Positive pitch is a right-hand rotation about body +y (left). In
    # REP-103 (FLU) that tilts the body nose-down: a point directly
    # ahead in the nominal frame (body +x) ends up above the tilted
    # body (offset body +z).
    assert _close(apply_body_pose((1.0, 0.0, 0.0), pose), (0.0, 0.0, 1.0))


def test_roll_rotates_yz_oppositely():
    pose = BodyPose(roll=math.pi / 2)
    # Body rolls right (about body +x) by 90°; a point to the body's
    # left (body +y) in the nominal frame appears below the body.
    assert _close(apply_body_pose((0.0, 1.0, 0.0), pose), (0.0, 0.0, -1.0))


def test_pure_translation_preserves_relative_geometry():
    # Two foot targets shifted by the same body translation keep their
    # relative offset — only the absolute position changes.
    pose = BodyPose(x=0.03, y=-0.01, z=0.02)
    a = apply_body_pose((0.10, 0.05, -0.08), pose)
    b = apply_body_pose((0.20, 0.05, -0.08), pose)
    diff = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    assert _close(diff, (0.10, 0.0, 0.0))


def test_round_trip_compose_with_inverse_pose_recovers_target():
    # Applying pose P then pose (-P) should recover the original target,
    # but only when rotations are small enough that intrinsic XYZ commutes
    # to first order. Use a tiny pose to make the round-trip exact-ish.
    pose = BodyPose(x=0.01, y=-0.02, z=0.005, roll=0.02, pitch=-0.01, yaw=0.03)
    inv = BodyPose(x=-pose.x, y=-pose.y, z=-pose.z, roll=-pose.roll, pitch=-pose.pitch, yaw=-pose.yaw)
    p = (0.18, 0.04, -0.09)
    # Note: this is a sanity check on direction, not a strict inverse —
    # full SE(3) inverse is non-trivial. Tolerance accommodates the
    # rotation-order asymmetry at finite angles.
    out = apply_body_pose(apply_body_pose(p, pose), inv)
    assert _close(out, p, tol=1e-3)
