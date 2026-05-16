"""Internal pose types for the posture controller.

The ROS interface is ``hexa_interfaces/msg/BodyPose``; this module
provides a ROS-free analogue so the library code (animations, clamps,
composition) stays importable without ``rclpy`` and is unit-testable in
isolation. The ``posture_node`` is the only place that converts between
the ROS msg and these dataclasses.
"""

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class BodyPose:
    """6-DOF body pose offset from nominal — translations in metres,
    rotations in radians. REP-103 body frame (+x fwd, +y left, +z up),
    intrinsic XYZ rotation order. Mirrors ``BodyPose.msg`` field-for-field.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


IDENTITY = BodyPose()


def add(a: BodyPose, b: BodyPose) -> BodyPose:
    """Component-wise sum.

    Valid only for small offsets: full SE(3) composition doesn't commute,
    and intrinsic-XYZ Euler angles don't add. The posture stack stays in
    the small-angle regime (animation amplitudes are centimetres and
    single-digit degrees), so additive composition is good enough and
    keeps the math trivial. Document each call site if you find one
    that violates this assumption.
    """
    return BodyPose(
        x=a.x + b.x,
        y=a.y + b.y,
        z=a.z + b.z,
        roll=a.roll + b.roll,
        pitch=a.pitch + b.pitch,
        yaw=a.yaw + b.yaw,
    )


def scale(p: BodyPose, k: float) -> BodyPose:
    return BodyPose(
        x=p.x * k,
        y=p.y * k,
        z=p.z * k,
        roll=p.roll * k,
        pitch=p.pitch * k,
        yaw=p.yaw * k,
    )


@dataclass(frozen=True)
class PoseLimits:
    """Per-axis symmetric clamp envelope for the final pose target.

    A blunt first cut at safety: the real reachable envelope depends on
    leg geometry and current foot positions, so a proper clamp belongs
    in the IK node (or a shared geometry helper). This static envelope
    is a cheap upstream guard against runaway animation/teleop inputs.
    Replace with a geometry-aware version once foot positions are
    available to the posture node.
    """

    x: float = 0.05  # m
    y: float = 0.05  # m
    z: float = 0.04  # m, max body lift/drop
    roll: float = 0.30  # rad (~17°)
    pitch: float = 0.30  # rad
    yaw: float = 0.50  # rad (~29°)


def clamp(pose: BodyPose, limits: PoseLimits) -> BodyPose:
    def _c(v: float, lo_hi: float) -> float:
        return max(-lo_hi, min(lo_hi, v))

    return replace(
        pose,
        x=_c(pose.x, limits.x),
        y=_c(pose.y, limits.y),
        z=_c(pose.z, limits.z),
        roll=_c(pose.roll, limits.roll),
        pitch=_c(pose.pitch, limits.pitch),
        yaw=_c(pose.yaw, limits.yaw),
    )
