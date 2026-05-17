"""Gait strategy interface and shared swing-arc helper.

A ``Strategy`` is a pure function ``(phase, stride_params, leg) -> foot_target``.
It carries no state, performs no I/O, and reads no clocks. The engine
owns the phase clock and per-leg transition state; strategies only see
what they need to evaluate a single tick.

``swing_arc`` packages the two quartic-Bezier swing curves from
``trajectory`` into a single ``phase_in_swing`` -> ``foot_target``
helper, reused by both the normal swing-phase evaluation and the
``TransitionController.RECENTER`` ladder. Keeping the curve evaluation
here avoids duplicating the C++-derived trajectory logic.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..clock import PhaseOffsets
from ..trajectory import (
    generate_primary_swing_control_nodes,
    generate_secondary_swing_control_nodes,
    quartic_bezier,
)


__all__ = [
    "LegContext",
    "Strategy",
    "StrideParams",
    "identity_y_sign",
    "swing_arc",
]

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class LegContext:
    """Geometric description of one leg as the engine sees it.

    All fields are body-frame quantities except ``mount_yaw`` (the
    rotation that aligns the body frame with the leg's coxa-mount frame).
    ``nominal_stance`` is the foot position when ``cmd_vel`` is zero —
    the visual standing pose.
    """

    name: str
    mount_xyz: Vec3
    mount_yaw: float
    nominal_stance: Vec3


@dataclass(frozen=True)
class StrideParams:
    """Per-tick stride description for one leg.

    ``stride_vector`` is the body-frame displacement the foot must
    cover during one full stance phase — i.e. the foot moves from
    AEP at touchdown to PEP at lift-off along this vector. It is
    computed by the engine each tick from the commanded body velocity
    and the hip's position (yaw contributes a tangential component).
    """

    stride_vector: Vec3
    cycle_time: float
    duty_factor: float
    swing_clearance: float
    swing_width: float
    controller_dt: float


class Strategy(Protocol):
    """A gait strategy maps (phase, stride, leg) to a body-frame foot target."""

    phase_offsets: PhaseOffsets
    duty_factor: float

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> Vec3: ...


def identity_y_sign(nominal_stance: Vec3) -> int:
    """Return +1 if the nominal foot sits at positive y, else -1.

    Used by the swing-arc lateral arch (``swing_width``) to mirror the
    sideways shift across the body. Flat-tripod default has
    ``swing_width = 0`` so the sign is irrelevant.
    """
    return 1 if nominal_stance[1] > 0.0 else -1


def swing_arc(
    phase_in_swing: float,
    swing_origin: tuple[float, float, float],
    target: tuple[float, float, float],
    swing_clearance: float,
    swing_width: float,
    identity_y_sign: int,
    swing_time: float,
    controller_dt: float,
    swing_origin_velocity: tuple[float, float, float] | None = None,
) -> tuple[float, float, float]:
    """Evaluate the two-curve swing trajectory at ``phase_in_swing in [0, 1)``.

    Two primary/secondary quartic Beziers each cover one half of swing.
    The C1 lift-off velocity defaults to the analytical continuation of
    a constant-velocity stance, ``-stride / swing_time``, where stride
    is ``target - swing_origin``. Pass ``swing_origin_velocity=(0,0,0)``
    for a rest-to-rest move (RECENTER from the transition controller).

    ``swing_delta_t = controller_dt / swing_time`` is the Bezier-parameter
    step per controller tick. Stance and swing share the same magnitude
    here because we ramp into and out of the swing curves over the
    swing's own duration — the analytical join math from Syropod is
    expressed in these terms.
    """
    o = np.array(swing_origin, dtype=np.float64)
    t = np.array(target, dtype=np.float64)
    stride = t - o

    if swing_origin_velocity is None:
        velocity_in = -stride / swing_time
    else:
        velocity_in = np.array(swing_origin_velocity, dtype=np.float64)

    swing_delta_t = controller_dt / swing_time
    stance_delta_t = swing_delta_t  # rest-to-rest symmetric join

    primary = generate_primary_swing_control_nodes(
        swing_origin=o,
        swing_origin_velocity=velocity_in,
        target=t,
        swing_clearance=swing_clearance,
        swing_width=swing_width,
        identity_y_sign=identity_y_sign,
        controller_dt=controller_dt,
        swing_delta_t=swing_delta_t,
    )
    secondary = generate_secondary_swing_control_nodes(
        swing_1_nodes=primary,
        target=t,
        stride_vector=stride,
        controller_dt=controller_dt,
        swing_delta_t=swing_delta_t,
        stance_delta_t=stance_delta_t,
    )

    if phase_in_swing < 0.5:
        local = phase_in_swing / 0.5
        point = quartic_bezier(primary, local)
    else:
        local = (phase_in_swing - 0.5) / 0.5
        point = quartic_bezier(secondary, local)
    return (float(point[0]), float(point[1]), float(point[2]))
