"""Tripod gait: alternating 3+3, ``duty_factor = 0.5``.

The six legs split into two tripods that alternate between swing and
stance. Tripod A (``l_front``, ``r_middle``, ``l_rear``) lifts off at
``master = 0.0``; tripod B (``r_front``, ``l_middle``, ``r_rear``) lifts
off at ``master = 0.5``. With a 50% duty factor the support polygon
always has exactly three feet on the ground.
"""

from __future__ import annotations

from ..clock import PhaseOffsets
from ._common import phased_foot_target
from .base import LegContext, StrideParams


__all__ = ["TRIPOD_OFFSETS", "Tripod"]


TRIPOD_OFFSETS = PhaseOffsets(
    offsets={
        "l_front": 0.0,
        "r_middle": 0.0,
        "l_rear": 0.0,
        "r_front": 0.5,
        "l_middle": 0.5,
        "r_rear": 0.5,
    }
)


class Tripod:
    phase_offsets = TRIPOD_OFFSETS
    duty_factor = 0.5
    unstable = False

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> tuple[float, float, float]:
        return phased_foot_target(phase, stride, leg)
