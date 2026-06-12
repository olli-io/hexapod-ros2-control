import pytest

from hexa_display.face_animation import (
    BREATHING,
    FACE_ANIMATIONS,
    IDLING,
    FaceAnimation,
    FaceAnimationStep,
    due_steps,
    step_count_at,
)
from hexa_display.protocol import Gaze


def test_registry_names_match():
    assert set(FACE_ANIMATIONS) == {"breathing", "idling"}
    for name, animation in FACE_ANIMATIONS.items():
        assert animation.name == name


def test_breathing_is_a_slow_vertical_cycle():
    gazes = [step.gaze for step in BREATHING.steps]
    assert gazes == [Gaze.UP, Gaze.CENTER, Gaze.DOWN, Gaze.CENTER]
    assert not any(
        step.blink or step.advance_expression for step in BREATHING.steps
    )


def test_idling_mirrors_reference_sequence():
    # Timing and order of the firmware test snippet: look left, blink,
    # look right, up, down, recenter, blink-and-switch, 560 ms tail.
    expected = [
        (0.0, Gaze.LEFT, False, False),
        (0.44, None, True, False),
        (0.8, Gaze.RIGHT, False, False),
        (1.24, Gaze.UP, False, False),
        (1.68, Gaze.DOWN, False, False),
        (2.12, Gaze.CENTER, False, False),
        (2.48, None, True, True),
    ]
    got = [
        (step.at_s, step.gaze, step.blink, step.advance_expression)
        for step in IDLING.steps
    ]
    assert got == expected
    assert IDLING.period_s == 3.04


def test_step_count_at_boundaries():
    assert step_count_at(IDLING, -0.5) == 0
    assert step_count_at(IDLING, 0.0) == 1  # at_s == elapsed fires
    assert step_count_at(IDLING, 0.44) == 2
    assert step_count_at(IDLING, 3.0) == 7
    assert step_count_at(IDLING, IDLING.period_s) == 8  # next cycle starts


def test_due_steps_over_incremental_ticks():
    fired = 0
    seen = []
    t = 0.0
    while t < IDLING.period_s * 2:
        steps, fired = due_steps(IDLING, t, fired)
        seen.extend(steps)
        t += 0.1  # node tick
    assert len(seen) == 2 * len(IDLING.steps)
    assert seen[: len(IDLING.steps)] == list(IDLING.steps)
    assert seen[len(IDLING.steps) :] == list(IDLING.steps)


def test_due_steps_stall_replays_at_most_one_cycle():
    elapsed = IDLING.period_s * 10 + 1.0
    steps, fired = due_steps(IDLING, elapsed, 0)
    assert len(steps) == len(IDLING.steps)
    assert fired == step_count_at(IDLING, elapsed)


def test_animation_validation():
    with pytest.raises(ValueError):
        FaceAnimation(name="empty", period_s=1.0, steps=())
    with pytest.raises(ValueError):
        FaceAnimation(
            name="past-period",
            period_s=1.0,
            steps=(FaceAnimationStep(at_s=1.5, gaze=Gaze.UP),),
        )
    with pytest.raises(ValueError):
        FaceAnimation(
            name="unordered",
            period_s=1.0,
            steps=(
                FaceAnimationStep(at_s=0.5, gaze=Gaze.UP),
                FaceAnimationStep(at_s=0.2, gaze=Gaze.DOWN),
            ),
        )
