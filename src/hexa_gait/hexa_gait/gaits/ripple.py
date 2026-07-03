"""Ripple gait: metachronal sequence, ``duty_factor = 5/6``.

One leg in swing at a time. Five feet on the ground throughout — the
maximum-stability gait, at the cost of a six-fold reduction in top
speed versus tripod. Phase offsets follow the same Wilson posterior →
anterior sequence as crawl; the two gaits differ only in their duty
factor.
"""

from __future__ import annotations

from hexa_common.gait_catalog import GAIT_DESCRIPTORS

from ._common import METACHRONAL_OFFSETS, phased_foot_target
from .base import LegContext, StrideParams


__all__ = ["Ripple"]


class Ripple:
    phase_offsets = METACHRONAL_OFFSETS
    duty_factor = GAIT_DESCRIPTORS["ripple"].duty_factor
    unstable = GAIT_DESCRIPTORS["ripple"].unstable

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> tuple[float, float, float]:
        return phased_foot_target(phase, stride, leg)
