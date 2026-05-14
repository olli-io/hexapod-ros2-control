"""Body frame ↔ leg coxa-mount frame transforms."""

import math

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
