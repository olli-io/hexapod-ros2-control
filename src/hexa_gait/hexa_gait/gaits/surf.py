"""Surf gait: tripod-grouped metachronal sequence, ``duty_factor = 5/8``.

Sits between tripod (β = 1/2, three legs airborne) and tetrapod /
crawl (β = 2/3, two legs airborne): six swing windows of 3/8 put
2.25 legs in the air on average, and top speed lands between tripod's
and crawl's (60 % of tripod's velocity cap; crawl is at 50 %).

Lift-off follows the same Wilson posterior → anterior cyclic order as
crawl and ripple (``r_rear → l_front → r_middle → l_rear → r_front →
l_middle``), but the timing is non-uniform. Evenly spread lift-offs
cannot be statically stable anywhere strictly between tripod and
tetrapod. The reason is a counting fact: with mean airborne
``6·(1 − β)`` greater than two whenever β < 2/3, there is always an
instant with three legs in the air, and the support triangle left by
an *evenly* spread airborne triple mixes the two natural tripods so
its long edge runs through the CoM (an even spread tips by ≈ 1 cm at
β = 5/8). Only at β = 2/3 does the airborne count fall to a flat two
and an even spread becomes stable — but that gait is crawl. So
between tripod and tetrapod surf must cluster its lift-offs by tripod:
``r_front``, ``l_middle``, ``r_rear`` lift at master 4/5, 9/10, 0 and
their mirrors ``l_front``, ``r_middle``, ``l_rear`` half a cycle later
at 3/10, 2/5, 1/2. The airborne set is then always a subset of the
active tripod, except at the group seams where at most one trailing
leg of the outgoing tripod overlaps the first leg of the incoming one
— a contralateral pair whose four grounded neighbours still bracket
the CoM. Surf is statically stable through the whole cycle, with a
worst-case margin better than tripod's at the same command fraction.

How loosely the lift-offs may cluster is what makes surf read as its
own gait rather than a jittery tripod, and it is bounded by β: the
within-tripod stagger may grow to about ``β − 1/2`` before a mixed
triple (two legs of one tripod plus one of the other) appears at the
seams and the margin collapses. At β = 5/8 that cliff is at 1/8.
The stagger here is 1/10 — about one engine tick below the cliff at
surf's cycle times, and 1.6× the spread the old β = 7/12 surf could
afford (its cliff was at 1/12). Smaller staggers are stable too, but
the lift-offs bunch toward tripod's simultaneous triple.
"""

from __future__ import annotations

from ..clock import PhaseOffsets
from ._common import phased_foot_target
from .base import LegContext, StrideParams


__all__ = ["SURF_OFFSETS", "Surf"]


# Offsets are the mirror of lift-off times — a leg with offset ``o``
# lifts off at ``master = (1 - o) mod 1`` (see the METACHRONAL_OFFSETS
# note in ``_common.py``).
SURF_OFFSETS = PhaseOffsets(
    offsets={
        "r_rear": 0.0,
        "l_middle": 1.0 / 10.0,
        "r_front": 2.0 / 10.0,
        "l_rear": 1.0 / 2.0,
        "r_middle": 1.0 / 2.0 + 1.0 / 10.0,
        "l_front": 1.0 / 2.0 + 2.0 / 10.0,
    }
)


class Surf:
    phase_offsets = SURF_OFFSETS
    duty_factor = 5.0 / 8.0

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> tuple[float, float, float]:
        return phased_foot_target(phase, stride, leg)
