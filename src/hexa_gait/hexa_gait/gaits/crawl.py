"""Crawl gait: metachronal sequence, ``duty_factor = 2/3``.

Two legs in swing at a time (one per side, half a cycle apart). The
support polygon always has four feet on the ground, so the gait is
markedly more stable than tripod but still keeps a moderate top speed.
Phase offsets follow the same Wilson posterior → anterior sequence as
ripple; the two gaits differ only in their duty factor.

Crawl's touchdowns coincide with the next leg's lift-off (the swing
window is exactly two of the six evenly spaced lift-off slots), and
``1 − β`` is not exactly representable at β = 2/3. The engine's stance
test guards that seam with ``_STANCE_SEAM_EPSILON`` so a just-landed
foot is never misflagged airborne for a tick — see ``engine.py``.
"""

from __future__ import annotations

from ._common import METACHRONAL_OFFSETS, phased_foot_target
from .base import LegContext, StrideParams


__all__ = ["Crawl"]


class Crawl:
    phase_offsets = METACHRONAL_OFFSETS
    duty_factor = 2.0 / 3.0
    unstable = True

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> tuple[float, float, float]:
        return phased_foot_target(phase, stride, leg)
