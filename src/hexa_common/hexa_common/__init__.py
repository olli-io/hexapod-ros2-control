"""Shared pure-Python contracts and config loaders for the hexapod stack.

A leaf library: no ``rclpy``, no numpy, no hexapod node-package imports.
The input/control layers (``hexa_teleop``, ``hexa_webteleop``,
``hexa_control``) and the gait/posture engines all depend on this package
instead of importing each other, keeping the node-package graph one-way.
"""

from .gait_catalog import GAIT_DESCRIPTORS, GaitDescriptor
from .limits import (
    VelocityCaps,
    load_velocity_caps,
    outer_stance_radius,
    scale_to_envelope,
    standing_stance_xy,
    unit_stance_xy,
)
from .posture_config import (
    load_animation_mode_animations,
    load_body_height_offsets,
)
from .teleop_scalar_limits import (
    PostureScalarLimits,
    load_posture_scalar_limits,
)

__all__ = [
    "GAIT_DESCRIPTORS",
    "GaitDescriptor",
    "PostureScalarLimits",
    "VelocityCaps",
    "load_animation_mode_animations",
    "load_body_height_offsets",
    "load_posture_scalar_limits",
    "load_velocity_caps",
    "outer_stance_radius",
    "scale_to_envelope",
    "standing_stance_xy",
    "unit_stance_xy",
]
