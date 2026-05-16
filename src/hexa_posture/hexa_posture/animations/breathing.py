"""Idle breathing — a slow vertical bob when the robot is standing still.

Off during walking (the ``walking`` flag gates it). A simple sinusoid
on ``ctx.t``; no gait phase needed, no state of its own.
"""

import math
from dataclasses import dataclass

from ..pose import IDENTITY, BodyPose
from .base import AnimationContext


@dataclass(frozen=True)
class Breathing:
    amplitude: float = 0.005  # m, peak-to-zero height offset
    period: float = 4.0  # s, one full breath cycle

    def __call__(self, ctx: AnimationContext) -> BodyPose:
        if ctx.walking:
            return IDENTITY
        omega = 2.0 * math.pi / self.period
        z = self.amplitude * math.sin(omega * ctx.t)
        return BodyPose(z=z)
