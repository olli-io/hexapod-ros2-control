"""Animation strategy interface.

An animation is a pure function from ``AnimationContext`` to a
``BodyPose`` offset. State that needs to persist across calls (e.g. a
fade timer) lives in the animation instance itself, but the function
must not perform I/O, read clocks, or touch ROS. The posture node owns
the clock and feeds ``t`` in via the context.

A ``Stack`` is a thin composition primitive: it sums the offsets from
its child animations. Composition uses ``pose.add`` (component-wise),
which is only valid for small offsets — see the docstring on
``pose.add``.
"""

from dataclasses import dataclass
from typing import Protocol

from ..pose import IDENTITY, BodyPose, add


@dataclass(frozen=True)
class AnimationContext:
    """Read-only inputs an animation may consult.

    Extend this — don't bypass it — when an animation needs new state.
    Keeping the surface explicit makes it obvious what each animation
    depends on, which matters for testing and for understanding why a
    pose changed.
    """

    t: float
    """Monotonic time in seconds. Animations should treat this as the
    only clock source; passing it explicitly keeps them deterministic
    under test and lets the node run them faster than wall-time for
    previews."""

    walking: bool
    """True iff the latest /cmd_vel was non-zero. Used by animations
    that should only run in pose mode (e.g. idle breathing) or only
    during gait (e.g. sway)."""

    gait_phase: float | None = None
    """Reserved: gait phase in [0, 1) once it lands on /gait/state.
    The current `/gait/state` message carries only the engine state
    name; until phase is added to the wire this stays ``None`` and
    phase-locked animations should fall back to a free-running sine
    on ``t`` or skip themselves."""


class Animation(Protocol):
    def __call__(self, ctx: AnimationContext) -> BodyPose: ...


@dataclass(frozen=True)
class Stack:
    layers: tuple[Animation, ...]

    def __call__(self, ctx: AnimationContext) -> BodyPose:
        out = IDENTITY
        for layer in self.layers:
            out = add(out, layer(ctx))
        return out
