"""Horizontal body roll — sways the body in y while yawing about
the vertical axis, phase-locked to the tripod gait. The whole
motion lives in the horizontal (transverse) plane, hence
"horizontal".

Mirror of ``VerticalBodyRoll`` in the horizontal plane: y replaces
x and yaw replaces pitch. Same tripod-only / walking gate
semantics.

Math (with ``φ = master_phase`` in ``[0, 1)``):

* ``y    = -y_amplitude * cos(2π φ)`` — cycle-rate cosine, one
  lateral sway per gait cycle (trough at φ = 0, peak at φ = 0.5).
* ``yaw  =  yaw_amplitude * sin(2π (φ + yaw_phase_offset))`` —
  cycle-rate sine. Convention: +yaw about +z rotates the body
  counter-clockwise viewed from above.

Pure function — reads only the context, returns a ``BodyPose``.
"""

from dataclasses import dataclass
import math

from ..pose import IDENTITY, BodyPose
from .base import AnimationContext


@dataclass(frozen=True)
class HorizontalBodyRoll:
    y_amplitude: float = 0.02
    """Half-range of the lateral sway (m). Body y oscillates in
    ``[-y_amplitude, +y_amplitude]``; the trough lands on each
    tripod touchdown."""

    yaw_amplitude: float = 0.1745
    """Half-range of the yaw oscillation (rad). Default ≈ 10°.
    Composes additively with the user pose, so stay well inside
    ``PoseLimits.yaw`` (0.50 rad)."""

    yaw_phase_offset: float = 0.0
    """Phase shift of the yaw sine, in cycles."""

    def __call__(self, ctx: AnimationContext) -> BodyPose:
        if not ctx.walking or ctx.master_phase is None:
            return IDENTITY
        if ctx.gait_name != "tripod":
            return IDENTITY
        phi = ctx.master_phase
        y = -self.y_amplitude * math.cos(2.0 * math.pi * phi)
        yaw = self.yaw_amplitude * math.sin(
            2.0 * math.pi * (phi + self.yaw_phase_offset)
        )
        return BodyPose(y=y, yaw=yaw)
