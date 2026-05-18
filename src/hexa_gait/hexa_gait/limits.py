"""Velocity caps derived from gait.yaml — single source of truth.

Downstream nodes (``hexa_teleop`` for stick scaling, ``hexa_control`` for
``/cmd_vel`` clamping) used to keep hand-aligned copies of the linear /
angular maxima. They drift in practice and hide the relationship to the
gait's actual physical reach. This module replaces those copies.

Linear cap is **derived** from existing engine parameters:

    linear_max = stride_length / (min_cycle_time * duty_factor)

That is exactly the per-leg velocity ceiling the gait saturates at:
beyond it the engine pins ``cycle_time`` at ``min_cycle_time`` and the
per-leg stride magnitude is capped at ``stride_length``. Anything teleop
or ``/cmd_vel`` publishes above this value would just be silently
clipped by the engine, so we make the ceiling explicit at the input
boundary instead.

Angular cap is taken as the raw ``angular_z_max`` from ``gait.yaml``. We
keep it explicit (not geometry-derived) because angular feel is harder
to predict from leg radii alone — the gait's geometric ceiling
(``linear_max / r_outer``) is typically much higher than what feels
comfortable to drive, and the right value depends on intent rather than
joint reach.

``scale_to_envelope`` jointly scales ``(v_x, v_y, omega_z)`` so the
implied per-leg planar speed never exceeds ``linear_max``. This is the
right place for that scaling — at the ``/cmd_vel`` boundary — because
the engine's per-leg stride clamp would otherwise distort the
translation:yaw ratio (the outer legs clip while inner legs don't,
which silently eats yaw at full forward + full yaw).

Pure-python; ``rclpy``-free so the helper is unit-testable standalone
and importable from both teleop and control without ROS overhead.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml


@dataclass(frozen=True)
class VelocityCaps:
    linear_max: float    # m/s, isotropic for x and y
    angular_max: float   # rad/s


def load_velocity_caps(gait_yaml: str | Path) -> VelocityCaps:
    path = Path(gait_yaml)
    with path.open() as f:
        raw = yaml.safe_load(f)

    stride_length = float(raw["stride_length"])
    min_cycle_time = float(raw["min_cycle_time"])
    duty_factor = float(raw["duty_factor"])
    angular_max = float(raw["angular_z_max"])

    linear_max = stride_length / (min_cycle_time * duty_factor)
    return VelocityCaps(linear_max=linear_max, angular_max=angular_max)


def scale_to_envelope(
    v_x: float,
    v_y: float,
    omega_z: float,
    leg_mounts: Mapping[str, tuple[float, float, float]],
    caps: VelocityCaps,
) -> tuple[float, float, float]:
    """Clamp ``omega_z`` and joint-scale the velocity triple to fit the gait envelope.

    Two-step shaping applied at the ``/cmd_vel`` boundary:

    1. ``omega_z`` is clamped to ``[-caps.angular_max, +caps.angular_max]``
       — an explicit feel knob, not derived from geometry.
    2. The implied per-leg planar speed is computed across all six
       legs as ``|(v_x - omega_z·r_y, v_y + omega_z·r_x)|``. If the
       maximum exceeds ``caps.linear_max``, all three components are
       scaled by the same ratio.

    Scaling all three together preserves the commanded direction
    (translation:yaw ratio), so the gait can realise the requested
    motion at the reduced speed instead of letting the engine's
    per-leg stride clamp clip the outer legs disproportionately.

    ``leg_mounts`` maps leg name to ``(r_x, r_y, r_z)`` mount position
    in the body frame; the ``r_z`` component is ignored.
    """
    if omega_z > caps.angular_max:
        omega_z = caps.angular_max
    elif omega_z < -caps.angular_max:
        omega_z = -caps.angular_max

    max_leg_v = 0.0
    for r_x, r_y, _ in leg_mounts.values():
        v_leg_x = v_x - omega_z * r_y
        v_leg_y = v_y + omega_z * r_x
        v_leg = math.hypot(v_leg_x, v_leg_y)
        if v_leg > max_leg_v:
            max_leg_v = v_leg

    if max_leg_v > caps.linear_max:
        scale = caps.linear_max / max_leg_v
        v_x *= scale
        v_y *= scale
        omega_z *= scale

    return v_x, v_y, omega_z
