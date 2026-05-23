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

    master_phase: float | None = None
    """Master phase from the gait clock in [0, 1), sniffed from
    /legs/targets by the posture node. ``0`` is lift-off for the
    reference leg (phase_offset = 0). ``None`` until the first
    /legs/targets message has been seen. Phase-locked animations
    (e.g. ``VerticalBodyRoll``) gate themselves on this field."""

    gait_name: str | None = None
    """Active gait strategy name (e.g. ``"tripod"``, ``"ripple"``),
    sniffed from ``/gait/params`` by the posture node. ``None`` until
    a params message has been seen. Animations that are only safe
    under a specific gait (e.g. ``GaitBounce`` is too aggressive for
    overlapping gaits) gate themselves on this field."""

    support_centroid_xy: tuple[float, float] | None = None
    """Low-pass-filtered XY centroid of the current support polygon
    in the body frame (metres), derived from /legs/targets in the
    posture node. ``None`` until the first /legs/targets message is
    received. The filtering lives in the node so animations stay
    stateless; animations that don't need it ignore the field."""

    swing_lift_z: float | None = None
    """Max foot lift above ground (metres) across all legs in swing,
    derived from /legs/targets in the posture node. Computed as
    ``max(foot.z for swing legs) − mean(foot.z for stance legs)``,
    clamped ``≥ 0``. ``None`` until /legs/targets has been observed
    with a usable stance polygon.

    Drives the gait-synced vertical bounce. Naturally tracks the
    swinging foot closest to its arc apex: for overlapping gaits
    (ripple, surf) the max picks the higher of the two airborne
    legs, so the bounce follows the main wave and ignores the
    fractional ``half-phase`` leg lagging or leading behind it."""


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
