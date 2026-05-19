"""Ripple gait: metachronal sequence, ``duty_factor = 2/3``.

Two legs in swing at a time (one per side, half a cycle apart). The
support polygon always has four feet on the ground, so the gait is
markedly more stable than tripod but still keeps a moderate top speed.
Phase offsets follow the same Wilson posterior → anterior sequence as
wave; the two gaits differ only in their duty factor.
"""

from __future__ import annotations

from ._common import METACHRONAL_OFFSETS, phased_foot_target
from .base import LegContext, StrideParams


__all__ = ["Ripple"]


class Ripple:
    phase_offsets = METACHRONAL_OFFSETS
    duty_factor = 2.0 / 3.0

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> tuple[float, float, float]:
        return phased_foot_target(phase, stride, leg)
