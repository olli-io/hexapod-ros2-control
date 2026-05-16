"""Identity animation — emits a zero pose offset.

Useful as a default in a ``Stack`` and as a no-op placeholder while
other animations are being developed.
"""

from dataclasses import dataclass

from ..pose import IDENTITY, BodyPose
from .base import AnimationContext


@dataclass(frozen=True)
class Still:
    def __call__(self, ctx: AnimationContext) -> BodyPose:
        return IDENTITY
