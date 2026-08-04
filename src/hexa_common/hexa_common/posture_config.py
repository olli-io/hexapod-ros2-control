"""Pure-Python loader for the posture node's animation-mode list.

Kept rclpy-free so upstream packages (e.g. ``hexa_teleop``) can read the
authoritative values at startup without dragging in a ROS context.
"""

from __future__ import annotations

from pathlib import Path

import yaml


def load_animation_mode_animations(posture_yaml: str | Path) -> tuple[str, ...]:
    """Return ``posture_node.ros__parameters.animation_mode_animations``.

    This is the single source of truth for the ordered set of names
    teleop can publish on ``/animation/mode``; the teleop cycler walks
    through this list directly so adding an animation here exposes it
    on the joystick without any teleop-side edit.
    """
    path = Path(posture_yaml)
    with path.open() as f:
        raw = yaml.safe_load(f)
    names = raw["posture_node"]["ros__parameters"]["animation_mode_animations"]
    if not names:
        raise ValueError(
            f"animation_mode_animations in {path} must list at least one name"
        )
    return tuple(str(n) for n in names)


def load_body_height_offsets(
    gait_yaml: str | Path, posture_yaml: str | Path
) -> tuple[float, float]:
    """Return ``(height_min, height_max)`` as pose offsets, in metres.

    ``posture_node.body_height_{min,max}_m`` are absolute belly clearance off
    the ground; ``/body/pose`` z is an offset from the resting stance. This
    converts between the two by subtracting ``gait_node.default_standing_pose
    .body_height`` — the same subtraction ``PostureController``'s constructor
    does on the C++ side, so teleop saturates its height integrator at exactly
    the point the posture stack starts clamping.

    Teleop declares no height limits of its own; this is the single source.
    """
    with Path(gait_yaml).open() as f:
        gait_raw = yaml.safe_load(f)
    with Path(posture_yaml).open() as f:
        posture_raw = yaml.safe_load(f)

    gait_params = gait_raw["gait_node"]["ros__parameters"]
    nominal = float(gait_params["default_standing_pose"]["body_height"])
    pn = posture_raw["posture_node"]["ros__parameters"]
    height_max = float(pn["body_height_max_m"])
    height_min = float(pn["body_height_min_m"])
    if not height_min < nominal < height_max:
        raise ValueError(
            f"body_height_min_m = {height_min} m / body_height_max_m = "
            f"{height_max} m must bracket default_standing_pose.body_height = "
            f"{nominal} m"
        )
    return height_min - nominal, height_max - nominal
