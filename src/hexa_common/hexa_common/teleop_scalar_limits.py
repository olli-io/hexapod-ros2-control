"""Pure-Python loader for the posture-mode scalar limits.

How far a stick may pose the body in posture mode. The values live in
hexa_description's ``tuning.yaml`` (``teleop_node`` block), next to the
posture envelope they have to stay inside — not in either front end's own
config, so the gamepad teleop, the web teleop and the Pico firmware's baked
joy mapping all pose the body over the same range. Bindings, deadband and the
height rate are per-device and stay in ``teleop_joy.yaml`` / ``webteleop.yaml``.

Kept rclpy-free so both teleop nodes can read the authoritative values at
startup without a ROS context.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class PostureScalarLimits:
    """The posture-mode limits, angles already in radians.

    Field names match ``hexa_teleop.joy_mapping.PostureConfig``'s, so a caller
    builds one by splatting this in alongside the per-device fields.
    """

    x_max: float
    y_max: float
    roll_max: float
    pitch_max: float
    yaw_max: float
    yaw_tau: float
    revert_tau: float
    wiggle_pivot_forward_m: float


# Each limit and the ``posture_node`` envelope key it must stay inside, as
# (YAML key, field, envelope key).
_ENVELOPE_KEYS: tuple[tuple[str, str, str], ...] = (
    ("x_max", "x_max", "pose_limit_x"),
    ("y_max", "y_max", "pose_limit_y"),
    ("roll_max_deg", "roll_max", "pose_limit_roll"),
    ("pitch_max_deg", "pitch_max", "pose_limit_pitch"),
    ("yaw_max_deg", "yaw_max", "pose_limit_yaw"),
)


def load_posture_scalar_limits(posture_yaml: str | Path) -> PostureScalarLimits:
    """Return ``teleop_node.ros__parameters.posture`` as radians-normalised limits.

    Raises if any axis reaches past the ``posture_node`` envelope in the same
    file: the stick would bank travel the posture stack clamps away, so the
    last part of its throw would move nothing.
    """
    path = Path(posture_yaml)
    with path.open() as f:
        raw = yaml.safe_load(f)
    posture = raw["teleop_node"]["ros__parameters"]["posture"]

    limits = PostureScalarLimits(
        x_max=float(posture["x_max"]),
        y_max=float(posture["y_max"]),
        roll_max=math.radians(float(posture["roll_max_deg"])),
        pitch_max=math.radians(float(posture["pitch_max_deg"])),
        yaw_max=math.radians(float(posture["yaw_max_deg"])),
        yaw_tau=float(posture["yaw_tau_s"]),
        revert_tau=float(posture["revert_tau_s"]),
        wiggle_pivot_forward_m=float(posture["wiggle_pivot_forward_m"]),
    )

    envelope = raw["posture_node"]["ros__parameters"]
    for yaml_key, field, envelope_key in _ENVELOPE_KEYS:
        value = getattr(limits, field)
        cap = float(envelope[envelope_key])
        if value > cap:
            raise ValueError(
                f"teleop_node posture.{yaml_key} = {posture[yaml_key]} "
                f"({value:.4g}) reaches past posture_node {envelope_key} = "
                f"{cap} in {path}; the stick would bank travel the posture "
                f"stack clamps away"
            )
    return limits
