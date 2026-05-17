"""Phase clock for the gait engine.

A ``GaitClock`` owns the engine's master phase in ``[0, 1)`` and projects
it through per-leg phase offsets. Strategies stay pure functions of
``phase``: the clock is the only place where time enters the gait chain.

``LEG_NAMES`` is re-exported here so callers that only need leg-name
iteration order do not have to import from ``hexa_kinematics``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from hexa_kinematics.leg_specs import LEG_NAMES


__all__ = ["LEG_NAMES", "GaitClock", "PhaseOffsets"]


@dataclass(frozen=True)
class PhaseOffsets:
    """Per-leg cycle start, relative to the master phase, in ``[0, 1)``."""

    offsets: Mapping[str, float]

    def __post_init__(self) -> None:
        missing = set(LEG_NAMES) - set(self.offsets)
        if missing:
            raise ValueError(f"PhaseOffsets missing legs: {sorted(missing)}")
        for name, value in self.offsets.items():
            if not (0.0 <= value < 1.0):
                raise ValueError(
                    f"PhaseOffsets[{name!r}] = {value} not in [0, 1)"
                )


class GaitClock:
    """Master phase clock with per-leg projections.

    ``advance(dt, cycle_time)`` integrates the master phase modulo one
    cycle. ``phases()`` returns each leg's projected phase as
    ``(master + offset) mod 1``.
    """

    def __init__(self, offsets: PhaseOffsets) -> None:
        self._offsets = offsets
        self._master = 0.0

    @property
    def master(self) -> float:
        return self._master

    def reset(self, master: float = 0.0) -> None:
        if not (0.0 <= master < 1.0):
            raise ValueError(f"master must be in [0, 1); got {master}")
        self._master = master

    def advance(self, dt: float, cycle_time: float) -> None:
        if cycle_time <= 0.0:
            raise ValueError(f"cycle_time must be positive; got {cycle_time}")
        self._master = (self._master + dt / cycle_time) % 1.0

    def phases(self) -> dict[str, float]:
        return {
            name: (self._master + self._offsets.offsets[name]) % 1.0
            for name in LEG_NAMES
        }
