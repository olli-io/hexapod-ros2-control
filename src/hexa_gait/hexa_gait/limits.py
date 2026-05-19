"""Velocity caps derived from gait.yaml — single source of truth.

Downstream nodes (``hexa_teleop`` for stick scaling, ``hexa_control`` for
``/cmd_vel`` clamping) used to keep hand-aligned copies of the linear /
angular maxima. They drift in practice and hide the relationship to the
gait's actual physical reach. This module replaces those copies.

Linear cap is **per-gait** because the per-leg velocity ceiling depends
on the active strategy's duty factor (β):

    linear_max(gait) = stride_length * (1 − β_gait) / (min_swing_time × β_gait)

That is exactly the per-leg velocity ceiling the gait saturates at:
beyond it the engine pins ``cycle_time`` at ``min_swing_time / (1 − β)``
and the per-leg stride magnitude is capped at ``stride_length``.
Tripod (β=0.5) sits at the high end of this curve; ripple (β=2/3) is
slower; wave (β=5/6) is slowest. Clamping ``/cmd_vel`` at the active
gait's cap keeps the engine's stance integration bounded — anything
higher would push the engagement controller's stance feet past PEP and
hit joint limits.

Angular cap is taken as the raw ``angular_z_max`` from ``gait.yaml``. We
keep it explicit (not geometry-derived) because angular feel is harder
to predict from leg radii alone — the gait's geometric ceiling
(``linear_max / r_outer``) is typically much higher than what feels
comfortable to drive, and the right value depends on intent rather than
joint reach. Same cap across gaits today; if a slower gait needed a
lower angular ceiling, that would land here as a per-gait knob too.

``scale_to_envelope`` cuts ``(v_x, v_y, omega_z)`` so the implied
per-leg planar speed never exceeds ``linear_max``. This is the right
place for that scaling — at the ``/cmd_vel`` boundary — because the
engine's per-leg stride clamp would otherwise distort the command (the
outer legs clip while inner legs don't, which silently eats yaw at full
forward + full yaw).

The cut between translation and yaw is split in ratio
``yaw_bias : (1 − yaw_bias)`` — translation absorbs the larger
fraction, so at the extremes the resulting motion keeps more of the
commanded yaw. ``yaw_bias = 0.5`` recovers the unbiased (uniform)
behaviour where both components scale by the same factor. The trade is
explicit: the commanded translation:yaw ratio is not preserved when
the cut kicks in.

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
    """Per-gait linear caps, a shared angular cap, and the yaw bias.

    Callers look up by gait name via ``linear_max(name)``; the dict is
    keyed by the same strings the gait registry uses ("tripod",
    "ripple", "wave"). Unknown names raise ``KeyError`` deliberately —
    a typo at the control layer should fail fast rather than silently
    fall back to the wrong cap. ``yaw_bias`` is shared across gaits and
    feeds ``scale_to_envelope``.
    """

    linear_max_by_gait: Mapping[str, float]
    angular_max: float
    yaw_bias: float

    def linear_max(self, gait: str) -> float:
        return self.linear_max_by_gait[gait]


def load_velocity_caps(gait_yaml: str | Path) -> VelocityCaps:
    path = Path(gait_yaml)
    with path.open() as f:
        raw = yaml.safe_load(f)

    stride_length = float(raw["stride_length"])
    min_swing_time = float(raw["min_swing_time"])
    angular_max = float(raw["angular_z_max"])
    yaw_bias = float(raw["yaw_bias"])

    linear_max_by_gait: dict[str, float] = {}
    for name, body in raw["gaits"].items():
        duty = float(body["duty_factor"])
        linear_max_by_gait[str(name)] = (
            stride_length * (1.0 - duty) / (min_swing_time * duty)
        )
    return VelocityCaps(
        linear_max_by_gait=linear_max_by_gait,
        angular_max=angular_max,
        yaw_bias=yaw_bias,
    )


def scale_to_envelope(
    v_x: float,
    v_y: float,
    omega_z: float,
    leg_mounts: Mapping[str, tuple[float, float, float]],
    linear_max: float,
    angular_max: float,
    yaw_bias: float,
) -> tuple[float, float, float]:
    """Clamp ``omega_z`` and cut the velocity triple to fit the gait envelope.

    Applied at the ``/cmd_vel`` boundary in three steps:

    1. ``omega_z`` is clamped to ``[-angular_max, +angular_max]`` — an
       explicit feel knob, not derived from geometry.
    2. The implied per-leg planar speed is computed across all six
       legs as ``|(v_x - omega_z·r_y, v_y + omega_z·r_x)|``. If the
       maximum is at or under ``linear_max`` the inputs pass through.
    3. Otherwise the reduction is split in ratio
       ``yaw_bias : (1 − yaw_bias)`` between translation and yaw —
       translation absorbs the larger share, so the resulting motion
       keeps more of the commanded yaw at the extremes.

    Concretely, parametrise the cut as ``s_v = 1 − ρ·t`` (applied to
    ``v_x``, ``v_y``) and ``s_w = 1 − t`` (applied to ``omega_z``) with
    ``ρ = yaw_bias / (1 − yaw_bias)``. The per-leg constraint
    ``|(s_v·v_x − s_w·omega_z·r_y, s_v·v_y + s_w·omega_z·r_x)| ≤
    linear_max`` becomes a quadratic in ``t`` per leg; the smallest
    positive root is the minimum cut that leg needs, and the binding
    cut is the maximum across legs. ``yaw_bias = 0.5`` makes ``ρ = 1``
    and recovers uniform scaling (direction preserved). Values above
    0.5 favour yaw; ``yaw_bias → 1`` approaches pure yaw priority.

    If the biased cut would drive ``s_v`` negative (the bias asks
    translation to scale past zero), translation is pinned at zero and
    ``omega_z`` is scaled alone to fit the per-leg cap. This only
    happens when the angular contribution at full ``angular_max``
    already exceeds the active gait's ``linear_max`` — which can occur
    for slow gaits whose ``linear_max`` is small relative to
    ``angular_max · r_outer``.

    ``leg_mounts`` maps leg name to ``(r_x, r_y, r_z)`` mount position
    in the body frame; ``r_z`` is ignored. ``linear_max`` /
    ``angular_max`` / ``yaw_bias`` are passed in as scalars so the
    caller can look them up per active gait via ``VelocityCaps``.
    """
    if omega_z > angular_max:
        omega_z = angular_max
    elif omega_z < -angular_max:
        omega_z = -angular_max

    cap_sq = linear_max * linear_max
    max_leg_v_sq = 0.0
    for r_x, r_y, _ in leg_mounts.values():
        vlx = v_x - omega_z * r_y
        vly = v_y + omega_z * r_x
        v_sq = vlx * vlx + vly * vly
        if v_sq > max_leg_v_sq:
            max_leg_v_sq = v_sq

    if max_leg_v_sq <= cap_sq:
        return v_x, v_y, omega_z

    rho = yaw_bias / (1.0 - yaw_bias)

    t_required = 0.0
    feasible = True
    for r_x, r_y, _ in leg_mounts.values():
        a0 = v_x - omega_z * r_y
        a1 = rho * v_x - omega_z * r_y
        b0 = v_y + omega_z * r_x
        b1 = rho * v_y + omega_z * r_x
        c = a0 * a0 + b0 * b0 - cap_sq
        if c <= 0.0:
            continue
        a = a1 * a1 + b1 * b1
        if a <= 0.0:
            feasible = False
            break
        b = -2.0 * (a0 * a1 + b0 * b1)
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            feasible = False
            break
        t_leg = (-b - math.sqrt(disc)) / (2.0 * a)
        if t_leg > t_required:
            t_required = t_leg

    if not feasible or rho * t_required >= 1.0:
        max_r = 0.0
        for r_x, r_y, _ in leg_mounts.values():
            r = math.hypot(r_x, r_y)
            if r > max_r:
                max_r = r
        omega_v_outer = abs(omega_z) * max_r
        if omega_v_outer > linear_max:
            return 0.0, 0.0, omega_z * (linear_max / omega_v_outer)
        return 0.0, 0.0, omega_z

    s_v = 1.0 - rho * t_required
    s_w = 1.0 - t_required
    return v_x * s_v, v_y * s_v, omega_z * s_w
