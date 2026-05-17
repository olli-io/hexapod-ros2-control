"""Per-joint servo configuration and default standing pose.

Loads the two YAMLs that live in ``hexa_description/config/``:

- ``geometry.yaml`` — under ``joints:``, per-joint-type servo center
  (URDF angle at the servo's physical zero) plus absolute lower / upper
  travel limits, all in intuitive per-joint degrees.
- ``standing_pose.yaml`` — per-joint-type default at-rest angle.

Both files express angles in **degrees**, in each joint's intuitive
sense. This module is the single source of truth for converting those
intuitive degrees into the IK-convention radians used by
``hexa_kinematics`` (see ``leg_geometry.py``). The same arithmetic is
inlined inside ``hexapod.urdf.xacro`` so the URDF stays a pure
mathematical presentation of the hexapod (joint zero = legs splayed
horizontally) without having to import any python.

Joint-type → IK-radian conversions:

- ``coxa``  — ``theta_coxa  =  radians(deg)``.
- ``femur`` — ``theta_femur = -radians(above_horizontal_deg)``; IK
  treats positive femur as tilting the foot toward ``-z``.
- ``tibia`` — ``theta_tibia =  pi - radians(interior_deg)``; matches
  the ``th_t = pi - gamma`` derivation in ``leg_ik.inverse_kinematics``.

Sign-aware swap: femur and tibia conversions are monotonically
decreasing, so an intuitive ``upper_limit_deg`` maps to a smaller
URDF-rad value than ``lower_limit_deg``. The loader reconciles this
with ``min/max`` after conversion.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from .leg_geometry import JointAngles


# Per-joint-type intuitive-center field name inside the YAML.
_CENTER_FIELD: dict[str, str] = {
    "coxa": "deg",
    "femur": "above_horizontal_deg",
    "tibia": "interior_deg",
}


@dataclass(frozen=True)
class JointLimits:
    """Servo configuration for one joint type, in IK-convention radians."""

    center: float    # rad — URDF angle at the servo's physical zero
    lower: float     # rad — URDF lower bound (always <= upper)
    upper: float     # rad — URDF upper bound
    effort: float    # Nm
    velocity: float  # rad/s


def _to_urdf_rad(joint_type: str, deg: float) -> float:
    """Convert an intuitive per-joint degree value to URDF-convention radians."""
    if joint_type == "coxa":
        return math.radians(deg)
    if joint_type == "femur":
        return -math.radians(deg)
    if joint_type == "tibia":
        return math.pi - math.radians(deg)
    raise ValueError(f"unknown joint type: {joint_type!r}")


def load_joint_limits(geometry_path: str | Path) -> dict[str, JointLimits]:
    """Parse ``geometry.yaml``'s ``joints:`` block into ``{joint_type: JointLimits}``.

    ``joint_type`` is one of ``"coxa"``, ``"femur"``, ``"tibia"``; the
    returned ``center``, ``lower``, and ``upper`` are in IK-convention
    radians, with ``lower <= center <= upper`` (the deg→rad sign flips
    on femur and tibia are absorbed by a ``min/max`` reconciliation).
    """
    with open(geometry_path) as f:
        raw = yaml.safe_load(f)
    joints = raw["joints"]
    out: dict[str, JointLimits] = {}
    for joint_type in ("coxa", "femur", "tibia"):
        cfg = joints[joint_type]
        center_deg = float(cfg[_CENTER_FIELD[joint_type]])
        lower_deg = float(cfg["lower_limit_deg"])
        upper_deg = float(cfg["upper_limit_deg"])

        center = _to_urdf_rad(joint_type, center_deg)
        a = _to_urdf_rad(joint_type, lower_deg)
        b = _to_urdf_rad(joint_type, upper_deg)
        lower, upper = (a, b) if a <= b else (b, a)

        if not (lower <= center <= upper):
            raise ValueError(
                f"{joint_type} servo center {center_deg:.2f}° lies outside "
                f"limit window [{lower_deg:.2f}°, {upper_deg:.2f}°]"
            )

        out[joint_type] = JointLimits(
            center=center,
            lower=lower,
            upper=upper,
            effort=float(cfg["effort"]),
            velocity=float(cfg["velocity"]),
        )
    return out


def load_standing_pose(
    standing_pose_path: str | Path,
    geometry_path: str | Path,
) -> JointAngles:
    """Parse ``standing_pose.yaml`` into ``(theta_coxa, theta_femur, theta_tibia)``.

    Angles are in IK-convention radians. Each joint's standing angle is
    validated against ``geometry.yaml``'s ``[lower, upper]`` window; a
    value outside that window raises ``ValueError`` so an inconsistent
    edit fails fast at startup instead of silently clipping inside the
    URDF.
    """
    with open(standing_pose_path) as f:
        raw = yaml.safe_load(f)
    limits = load_joint_limits(geometry_path)

    angles: dict[str, float] = {}
    for joint_type in ("coxa", "femur", "tibia"):
        cfg = raw[joint_type]
        theta = _to_urdf_rad(joint_type, float(cfg[_CENTER_FIELD[joint_type]]))
        lim = limits[joint_type]
        if not (lim.lower <= theta <= lim.upper):
            raise ValueError(
                f"standing pose {joint_type} angle {math.degrees(theta):.2f}° "
                f"lies outside servo range "
                f"[{math.degrees(lim.lower):.2f}°, {math.degrees(lim.upper):.2f}°]"
            )
        angles[joint_type] = theta

    return (angles["coxa"], angles["femur"], angles["tibia"])
