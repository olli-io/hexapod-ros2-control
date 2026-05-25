"""Pure-Python loaders for the posture node's YAML config.

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
