"""Surf gait: metachronal sequence, ``duty_factor = 7/12``.

Sits between tripod (β = 1/2, three legs airborne) and tetrapod /
ripple (β = 2/3, two legs airborne): a metachronal swing window of
5/12 against six legs offset by 1/6 means 2.5 legs are airborne on
average — the swing windows of consecutive legs overlap most of the
cycle, with one extra leg briefly joining the swing pair on each
hand-off. Top speed lands between tripod's and ripple's; stability
margin likewise.

Phase offsets follow the same Wilson posterior → anterior sequence as
ripple and wave; the three gaits differ only in their duty factor.
"""

from __future__ import annotations

from ._common import METACHRONAL_OFFSETS, phased_foot_target
from .base import LegContext, StrideParams


__all__ = ["Surf"]


class Surf:
    phase_offsets = METACHRONAL_OFFSETS
    duty_factor = 7.0 / 12.0

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> tuple[float, float, float]:
        return phased_foot_target(phase, stride, leg)
