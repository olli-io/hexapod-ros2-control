"""LegSpec loader.

Expands ``hexa_description``'s ``geometry.yaml`` (which defines only the
two reference mounts ``l_front`` and ``l_middle``, with ``yaw_deg`` in
degrees) into the full six-leg dict by the URDF symmetry rules:

- rear mirrors front about the body y-axis: ``x → -x``, ``yaw → pi - yaw``.
- right mirrors left  about the body x-axis: ``y → -y``, ``yaw → -yaw``.

These match the ``mount_leg`` macro in ``hexapod.urdf.xacro``: the YAML
stays the single source of truth, and the URDF and this loader expand
it the same way.

The IK library's joint-zero convention (horizontal femur, tibia colinear
with femur) coincides with the URDF's joint-zero (each leg link extends
along its parent's +x at joint zero), so IK angles can be sent straight
to ros2_control without per-joint offsets. The physical servo zero is
the per-joint ``center`` returned by ``joint_config.load_joint_limits``
(loaded from the ``joints:`` block of ``geometry.yaml``); when the
real-hardware bridge lands it should subtract that center from the IK
angle before commanding each servo.
"""

import math
from pathlib import Path

import yaml

from .leg_geometry import LegSpec


LEG_NAMES: tuple[str, ...] = (
    "l_front",
    "l_middle",
    "l_rear",
    "r_front",
    "r_middle",
    "r_rear",
)


def load_leg_specs(geometry_yaml_path: str | Path) -> dict[str, LegSpec]:
    """Parse ``geometry.yaml`` and return one ``LegSpec`` per leg, by name.

    Segment lengths come from ``leg.*``; mount positions come from
    ``mounts.l_front`` / ``mounts.l_middle`` and are mirrored for the
    rear and right-side legs.
    """
    with open(geometry_yaml_path) as f:
        cfg = yaml.safe_load(f)
    leg_cfg = cfg["leg"]
    mounts = cfg["mounts"]

    coxa_len = float(leg_cfg["coxa_length"])
    femur_len = float(leg_cfg["femur_length"])
    tibia_len = float(leg_cfg["tibia_length"])

    front = mounts["l_front"]
    middle = mounts["l_middle"]

    out: dict[str, LegSpec] = {}
    for side in ("l", "r"):
        for name in ("front", "middle", "rear"):
            ref = middle if name == "middle" else front
            ref_yaw = math.radians(ref["yaw_deg"])
            x_fr = -ref["x"] if name == "rear" else ref["x"]
            yaw_fr = math.pi - ref_yaw if name == "rear" else ref_yaw
            mx = x_fr
            my = -ref["y"] if side == "r" else ref["y"]
            myaw = -yaw_fr if side == "r" else yaw_fr
            out[f"{side}_{name}"] = LegSpec(
                mount_xyz=(mx, my, 0.0),
                mount_yaw=myaw,
                coxa_len=coxa_len,
                femur_len=femur_len,
                tibia_len=tibia_len,
            )
    return out
