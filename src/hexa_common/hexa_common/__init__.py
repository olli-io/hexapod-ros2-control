"""Shared pure-Python contracts and config loaders for the hexapod stack.

A leaf library: no ``rclpy``, no numpy, no hexapod node-package imports.
The input/control layers (``hexa_teleop``, ``hexa_webteleop``,
``hexa_control``) and the gait/posture engines all depend on this package
instead of importing each other, keeping the node-package graph one-way.
"""

from .gait_catalog import GAIT_DESCRIPTORS, GaitDescriptor
from .limits import VelocityCaps, load_velocity_caps, scale_to_envelope
from .overlay import deep_merge, load_tuning_block
from .posture_config import load_animation_mode_animations

__all__ = [
    "GAIT_DESCRIPTORS",
    "GaitDescriptor",
    "VelocityCaps",
    "deep_merge",
    "load_animation_mode_animations",
    "load_tuning_block",
    "load_velocity_caps",
    "scale_to_envelope",
]
