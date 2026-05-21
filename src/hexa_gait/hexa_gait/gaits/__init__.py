from typing import Callable

from ._common import METACHRONAL_OFFSETS, phased_foot_target
from .base import LegContext, Strategy, StrideParams, swing_arc
from .ripple import Ripple
from .surf import Surf
from .tetrapod import TETRAPOD_OFFSETS, Tetrapod
from .tripod import TRIPOD_OFFSETS, Tripod
from .wave import Wave


__all__ = [
    "LegContext",
    "METACHRONAL_OFFSETS",
    "Ripple",
    "STRATEGIES",
    "Strategy",
    "StrideParams",
    "Surf",
    "TETRAPOD_OFFSETS",
    "TRIPOD_OFFSETS",
    "Tetrapod",
    "Tripod",
    "Wave",
    "phased_foot_target",
    "swing_arc",
]


# Strategy registry: name → zero-arg factory. The engine looks up by
# name (from /gait/params) when ``set_strategy`` is called. Adding a new
# gait is two lines: drop the strategy class in this package and add an
# entry here.
STRATEGIES: dict[str, Callable[[], Strategy]] = {
    "tripod": Tripod,
    "surf": Surf,
    "tetrapod": Tetrapod,
    "ripple": Ripple,
    "wave": Wave,
}
