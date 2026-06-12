"""Face animation sequences for the ESP32 face.

Pure module: no rclpy, no I/O, no clocks. A face animation is a fixed,
looping sequence of timed steps — a gaze target, a blink trigger,
and/or an advance of the idle expression cycle. The display node owns
the clock: it tracks elapsed time since the animation started plus a
fired-step counter, and asks ``due_steps`` each tick which steps to
relay. The firmware still eases gaze and auto-blinks on top, so a
sparse step sequence reads as smooth motion.

Note: "face animation" is deliberately distinct from the posture
animation stack in ``hexa_posture`` (``/animation/mode``) — these only
drive the display.

- **breathing** — slow vertical gaze drift (up → center → down →
  center) while the display waits for the robot stack (servo UART,
  gait engine) to initialize.
- **idling** — look-around-and-blink cycle while the hexapod stands
  idle; the final blink of each cycle advances the configured idle
  expression cycle (blink-and-switch).
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .protocol import Gaze


@dataclass(frozen=True)
class FaceAnimationStep:
    at_s: float
    gaze: Gaze | None = None
    blink: bool = False
    advance_expression: bool = False


@dataclass(frozen=True)
class FaceAnimation:
    name: str
    period_s: float
    steps: tuple[FaceAnimationStep, ...] = field(default=())

    def __post_init__(self) -> None:
        if not self.steps:
            raise ValueError(f"{self.name}: animation needs at least one step")
        last = -1.0
        for step in self.steps:
            if step.at_s < last:
                raise ValueError(f"{self.name}: steps must be time-ordered")
            last = step.at_s
        if last >= self.period_s:
            raise ValueError(
                f"{self.name}: last step at {last}s is past the "
                f"{self.period_s}s period"
            )


def step_count_at(animation: FaceAnimation, elapsed_s: float) -> int:
    """Total steps due since the animation started, across cycles."""
    if elapsed_s < 0.0:
        return 0
    cycle, t_in = divmod(elapsed_s, animation.period_s)
    in_cycle = sum(1 for step in animation.steps if step.at_s <= t_in)
    return int(cycle) * len(animation.steps) + in_cycle


def due_steps(
    animation: FaceAnimation, elapsed_s: float, fired_count: int
) -> tuple[tuple[FaceAnimationStep, ...], int]:
    """Steps newly due at ``elapsed_s`` given ``fired_count`` already fired.

    Returns ``(steps, new_fired_count)``. After a stall longer than one
    period only the most recent cycle's worth of steps is replayed, so
    a hiccup cannot queue an unbounded blink burst.
    """
    target = step_count_at(animation, elapsed_s)
    n = len(animation.steps)
    start = max(fired_count, target - n)
    steps = tuple(animation.steps[k % n] for k in range(start, target))
    return steps, target


BREATHING = FaceAnimation(
    name="breathing",
    period_s=4.8,
    steps=(
        FaceAnimationStep(at_s=0.0, gaze=Gaze.UP),
        FaceAnimationStep(at_s=1.2, gaze=Gaze.CENTER),
        FaceAnimationStep(at_s=2.4, gaze=Gaze.DOWN),
        FaceAnimationStep(at_s=3.6, gaze=Gaze.CENTER),
    ),
)

# Mirrors the firmware test sequence: look left, blink, look right,
# look up, look down, recenter, then blink-and-switch to the next idle
# expression; the tail to the 3.04 s period lets the last blink play
# out. lookTo(x, y) maps as x=-1 → LEFT and y=-1 → UP (screen coords).
IDLING = FaceAnimation(
    name="idling",
    period_s=3.04,
    steps=(
        FaceAnimationStep(at_s=0.0, gaze=Gaze.LEFT),
        FaceAnimationStep(at_s=0.44, blink=True),
        FaceAnimationStep(at_s=0.8, gaze=Gaze.RIGHT),
        FaceAnimationStep(at_s=1.24, gaze=Gaze.UP),
        FaceAnimationStep(at_s=1.68, gaze=Gaze.DOWN),
        FaceAnimationStep(at_s=2.12, gaze=Gaze.CENTER),
        FaceAnimationStep(at_s=2.48, blink=True, advance_expression=True),
    ),
)

FACE_ANIMATIONS: dict[str, FaceAnimation] = {
    animation.name: animation for animation in (BREATHING, IDLING)
}
