"""Pure gait-descriptor catalog — the shared source of truth for gait metadata.

Downstream packages (``hexa_control`` for name validation, ``hexa_teleop`` /
``hexa_webteleop`` for the gait cycle and the unstable filter, and this
package's own ``limits.py`` for the per-gait velocity caps) need only three
facts about each gait: its **name**, its **duty factor** (β), and whether it
is flagged **unstable**. None of them need the numpy trajectory
implementations that live on the strategy classes in ``hexa_gait/gaits/``.

Keeping those three facts here — rclpy- and numpy-free — lets the input and
control layers read them without importing the gait engine (which would drag
in numpy and ``hexa_kinematics``). ``hexa_gait``'s strategy classes reference
this catalog so the values have a single Python source of truth; the C++
engine keeps its own copy in ``hexa_gait_cpp/gaits/registry.cpp`` (see the
cross-language unification note in the refactor plan).

``phase_offsets`` deliberately stay on the strategy classes — only the gait
engine consumes them, so they are not part of this shared contract.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GaitDescriptor:
    """Name + duty factor + stability flag for one gait strategy."""

    name: str
    duty_factor: float
    unstable: bool
    # Which legs the gait walks: "hexapod" (all six) or "quadruped" (the four
    # corners, middle pair parked). Here as well as on the C++ strategy class
    # because the swing phase margin — and so the derived velocity cap in
    # limits.py — is per leg set.
    leg_set: str = "hexapod"


# Registry: name → descriptor. Adding a gait means adding an entry here and
# dropping the matching strategy class in ``hexa_gait/gaits/``. Keyed by the
# same strings the engine looks up via ``/gait/params``.
GAIT_DESCRIPTORS: dict[str, GaitDescriptor] = {
    "tripod": GaitDescriptor("tripod", 0.5, False),
    "surf": GaitDescriptor("surf", 5.0 / 8.0, True),
    "tetrapod": GaitDescriptor("tetrapod", 2.0 / 3.0, False),
    "crawl": GaitDescriptor("crawl", 2.0 / 3.0, True),
    "ripple": GaitDescriptor("ripple", 5.0 / 6.0, False),
    # Quadruped leg set: the four corners creep one leg at a time while the
    # middle pair is parked. Deliberately absent from every gait_cycle — the
    # teleop select toggle is the only way in, and once there the operator
    # rotates among these two through quadruped_gait_cycle. They differ only in
    # footfall order (lateral sequence vs the perimeter one); neither is the
    # six-leg "tetrapod" above.
    "quad_walk": GaitDescriptor("quad_walk", 3.0 / 4.0, False, "quadruped"),
    "quad_canter": GaitDescriptor("quad_canter", 3.0 / 4.0, False, "quadruped"),
}
