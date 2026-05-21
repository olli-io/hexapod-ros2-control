"""Tetrapod gait: 3 pairs of 2, ``duty_factor = 2/3``.

Three pairs of legs swing together at offsets 0, 1/3, 2/3 — Wilson's
Type II tetrapod arrangement. Each pair is diagonally arranged across
the body so the four supporting feet always bracket the centre of mass:

- ``master = 0``     — ``l_front`` + ``r_middle``  swing together.
- ``master = 1/3``   — ``r_front`` + ``l_rear``    swing together.
- ``master = 2/3``   — ``l_middle`` + ``r_rear``   swing together.

Shares the duty factor with ripple (β = 2/3, 4 feet on ground at all
times) but differs in the *which two* — ripple stages its two swings
half a cycle apart on opposite body sides, while tetrapod lifts a
diagonal pair together. The resulting support polygon swap is more
abrupt than ripple but the static-stability margin is comparable.
"""

from __future__ import annotations

from ..clock import PhaseOffsets
from ._common import phased_foot_target
from .base import LegContext, StrideParams


__all__ = ["TETRAPOD_OFFSETS", "Tetrapod"]


TETRAPOD_OFFSETS = PhaseOffsets(
    offsets={
        "l_front": 0.0,
        "r_middle": 0.0,
        "r_front": 1.0 / 3.0,
        "l_rear": 1.0 / 3.0,
        "l_middle": 2.0 / 3.0,
        "r_rear": 2.0 / 3.0,
    }
)


class Tetrapod:
    phase_offsets = TETRAPOD_OFFSETS
    duty_factor = 2.0 / 3.0

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> tuple[float, float, float]:
        return phased_foot_target(phase, stride, leg)
