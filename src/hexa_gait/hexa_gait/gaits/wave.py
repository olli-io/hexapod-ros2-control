"""Wave gait: metachronal sequence, ``duty_factor = 5/6``.

One leg in swing at a time. Five feet on the ground throughout — the
maximum-stability gait, at the cost of a six-fold reduction in top
speed versus tripod. Phase offsets follow the same Wilson posterior →
anterior sequence as ripple; the two gaits differ only in their duty
factor.
"""

from __future__ import annotations

from ._common import METACHRONAL_OFFSETS, phased_foot_target
from .base import LegContext, StrideParams


__all__ = ["Wave"]


class Wave:
    phase_offsets = METACHRONAL_OFFSETS
    duty_factor = 5.0 / 6.0

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> tuple[float, float, float]:
        return phased_foot_target(phase, stride, leg)
