"""Gait-sensitive sway — translates the body in XY to track the live
support-polygon centroid, suppressing the rocking mode that
four-foot stance polygons (tetrapod, crawl, surf) exhibit.

Three-foot (tripod) and five-foot (ripple) polygon centroids land near
the body origin, so the animation naturally self-attenuates for those
gaits — no per-gait gating needed.

Pure function: reads ``ctx.support_centroid_xy`` (already low-pass
filtered by the posture node) and emits a planar ``BodyPose(x, y)``
scaled by ``gain · strength``. Off when ``walking`` is False or the
centroid hasn't been observed yet.

Depends only on ``/legs/targets``, not on ``hexa_gait`` — the
posture chain does not import the gait chain.
"""

from dataclasses import dataclass

from ..pose import IDENTITY, BodyPose
from .base import AnimationContext


@dataclass(frozen=True)
class GaitSway:
    gain: float = 1.0
    """Feedforward gain on the centroid. 1.0 makes the body track the
    polygon centroid one-for-one — the physical sweet spot for
    cancelling the rocking-axis torque."""

    strength: float = 0.5
    """User-facing attenuator in [0, 1]. Multiplies the gain output
    so the sway can be toned down (or off) without changing the
    physical gain. 0.0 disables the animation; 1.0 applies full
    feedforward."""

    def __call__(self, ctx: AnimationContext) -> BodyPose:
        if not ctx.walking or ctx.support_centroid_xy is None:
            return IDENTITY
        k = self.gain * self.strength
        if k == 0.0:
            return IDENTITY
        cx, cy = ctx.support_centroid_xy
        return BodyPose(x=k * cx, y=k * cy)
