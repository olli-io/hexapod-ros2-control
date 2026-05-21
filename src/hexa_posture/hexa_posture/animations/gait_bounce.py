"""Gait-synced vertical bounce — lifts the body in +z to match the
swing arc in flight, so the chassis travels with the feet rather
than rocking against them.

**Tripod-only.** Gated on ``ctx.gait_name == "tripod"``; returns
IDENTITY under any other gait. The bounce is only meaningful with
the half/half stance pattern: tripod's two non-overlapping triads
give a clean two-bounce-per-cycle envelope whose troughs land on
full hexapodal support. Tetrapod and wave bounce more times per
cycle on smaller support polygons (one swing leg at a time), and
the overlapping gaits (ripple, surf) keep the chassis high during
two-leg-airborne windows — both modes drove the body unstable in
testing.

Body is at rest (z = 0) when no foot is lifted (all legs at or near
AEP / touchdown); body is at peak (``z = arc_height``) when the
swinging triad is at its swing apex. Linear in ``ctx.swing_lift_z``,
the max foot lift above the stance polygon emitted by the posture
node from ``/legs/targets``.

Stacks cleanly on top of ``GaitSway``: sway handles XY centroid
tracking, bounce handles Z. Together they cancel both the rocking
torque and the implicit vertical impulse the gait imposes.

Pure function: reads ``ctx.swing_lift_z`` and ``ctx.gait_name`` and
emits a vertical ``BodyPose(z=...)`` scaled to ``arc_height``. Off
when ``walking`` is False, the signal hasn't been observed yet, or
the active gait isn't tripod.
"""

from dataclasses import dataclass

from ..pose import IDENTITY, BodyPose
from .base import AnimationContext


@dataclass(frozen=True)
class GaitBounce:
    arc_height: float = 0.02
    """Max body lift (m) at swing apex. Body Z spans
    ``[0, arc_height]`` over the gait cycle. Tune conservatively —
    the lift composes additively with the user pose and any other
    Z-axis animation (e.g. ``Breathing``)."""

    step_height_ref: float = 0.06
    """Reference foot swing apex (m) used to normalise the lift
    signal. Defaults to ``hexa_gait/config/gait.yaml``'s
    ``step_height``. If the gait engine's ``step_height`` is
    retuned, mirror the change here so ``arc_height`` continues to
    represent the actual peak body lift in metres.

    A future iteration can move this to the wire (gait params on a
    topic) so the two stay in sync automatically without crossing
    the posture/gait import boundary."""

    def __call__(self, ctx: AnimationContext) -> BodyPose:
        if not ctx.walking or ctx.swing_lift_z is None:
            return IDENTITY
        if ctx.gait_name != "tripod":
            return IDENTITY
        if self.arc_height == 0.0 or self.step_height_ref <= 0.0:
            return IDENTITY
        ratio = ctx.swing_lift_z / self.step_height_ref
        if ratio < 0.0:
            ratio = 0.0
        elif ratio > 1.0:
            ratio = 1.0
        return BodyPose(z=self.arc_height * ratio)
