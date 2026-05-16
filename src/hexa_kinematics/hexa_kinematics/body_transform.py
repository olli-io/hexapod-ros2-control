"""Body frame ↔ leg coxa-mount frame transforms, and body-pose composition.

Two layers of body-frame manipulation live here:

- ``body_to_leg`` / ``leg_to_body`` map between the (nominal) body frame
  and a single leg's coxa-mount frame. Geometry only — no notion of body
  pose offset.
- ``BodyPose`` + ``apply_body_pose`` represent a 6-DOF offset of the
  body from its nominal pose, and re-express a point given in the
  nominal frame as seen from the offset body frame. Used by the IK node
  to support both pose mode (feet grounded, gait idle) and body
  animation during gait, without leaking body-pose state into the gait
  strategies.

Mirrors ``hexa_interfaces/msg/BodyPose.msg``. The rotation convention
is intrinsic XYZ (roll about body +x, then pitch about body +y, then
yaw about body +z).
"""

import math
from dataclasses import dataclass

from .leg_geometry import LegSpec, Point3


def body_to_leg(p_body: Point3, leg: LegSpec) -> Point3:
    """Map a point from the body frame into the leg's coxa-mount frame."""
    mx, my, mz = leg.mount_xyz
    dx, dy, dz = p_body[0] - mx, p_body[1] - my, p_body[2] - mz
    c, s = math.cos(leg.mount_yaw), math.sin(leg.mount_yaw)
    return (c * dx + s * dy, -s * dx + c * dy, dz)


def leg_to_body(p_leg: Point3, leg: LegSpec) -> Point3:
    """Map a point from the leg's coxa-mount frame back into the body frame."""
    c, s = math.cos(leg.mount_yaw), math.sin(leg.mount_yaw)
    x = c * p_leg[0] - s * p_leg[1]
    y = s * p_leg[0] + c * p_leg[1]
    mx, my, mz = leg.mount_xyz
    return (x + mx, y + my, p_leg[2] + mz)


@dataclass(frozen=True)
class BodyPose:
    """6-DOF offset of the body from its nominal walking pose.

    Mirrors ``hexa_interfaces/msg/BodyPose.msg``. Library-side type so
    pure-Python code (kinematics, tests) stays importable without
    ``rclpy``.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


IDENTITY_BODY_POSE = BodyPose()


def apply_body_pose(p_nominal: Point3, pose: BodyPose) -> Point3:
    """Re-express a foot target given in the nominal body frame as it
    appears in the body frame offset by ``pose``.

    The body has translated by ``(x, y, z)`` and rotated by intrinsic
    XYZ ``(roll, pitch, yaw)`` from its nominal pose. A target held
    fixed in the nominal frame appears in the offset body frame as::

        p_offset = R(pose)^T · (p_nominal − t(pose))

    Pure function; no state. Same operation serves both pose mode
    (held foot positions, gait idle) and gait-active body animation
    (gait-emitted foot trajectories composed with sway/lean/bob).
    """
    dx = p_nominal[0] - pose.x
    dy = p_nominal[1] - pose.y
    dz = p_nominal[2] - pose.z

    cr, sr = math.cos(pose.roll), math.sin(pose.roll)
    cp, sp = math.cos(pose.pitch), math.sin(pose.pitch)
    cy, sy = math.cos(pose.yaw), math.sin(pose.yaw)

    # R(pose) = Rz(yaw) · Ry(pitch) · Rx(roll); apply R^T = Rx(-roll)·Ry(-pitch)·Rz(-yaw).
    # Rz(-yaw):
    x1 = cy * dx + sy * dy
    y1 = -sy * dx + cy * dy
    z1 = dz
    # Ry(-pitch):
    x2 = cp * x1 - sp * z1
    y2 = y1
    z2 = sp * x1 + cp * z1
    # Rx(-roll):
    x3 = x2
    y3 = cr * y2 + sr * z2
    z3 = -sr * y2 + cr * z2
    return (x3, y3, z3)
