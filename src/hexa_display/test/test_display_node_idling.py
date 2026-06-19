"""Node-level timing for the idle look-around rest gap.

Exercises ``_run_face_animation`` directly on a bare ``DisplayNode``
(no rclpy / ROS graph): the burst plays once, then the eyes rest until
a random restart drawn from ``repeat_range_s`` before glancing again.
"""

import random

from hexa_display.display_node import DisplayNode
from hexa_display.face_animation import IDLING


class _Harness:
    """Bare DisplayNode counting the steps it would relay.

    Each IDLING step relays exactly one frame (a gaze or a blink), so
    the frame-write count equals the number of steps played.
    """

    def __init__(self, rng_value: float) -> None:
        self.node = DisplayNode.__new__(DisplayNode)
        self.node._rng = random.Random()
        self.node._rng.uniform = lambda lo, hi: rng_value  # type: ignore
        self.node._sent_gaze = None
        self.node._face_animation_fired = 0
        self.writes = 0

        def _write_frame(frame, now):
            self.writes += 1
            return True

        self.node._write_frame = _write_frame  # type: ignore

    def play(self, animation, span_s: float) -> int:
        self.node._start_face_animation_cycle(animation, 0.0)
        t = 0.0
        while t < span_s:
            self.node._run_face_animation(animation, t)
            t += 0.1
        return self.writes


def test_idling_rests_between_bursts():
    # With a 7 s repeat interval over 20 s the burst should fire 3 times
    # (t=0, 7, 14), not the ~6 it would if it looped every 3.04 s.
    steps = _Harness(rng_value=7.0).play(IDLING, span_s=20.0)
    assert steps == 3 * len(IDLING.steps)


def test_idling_first_burst_is_a_single_cycle():
    # Within one period only one cycle's worth of steps relay.
    steps = _Harness(rng_value=7.0).play(IDLING, span_s=IDLING.period_s)
    assert steps == len(IDLING.steps)


def test_idling_restart_uses_random_interval():
    # A longer interval yields fewer bursts over the same window.
    short = _Harness(rng_value=5.0).play(IDLING, span_s=30.0)
    long = _Harness(rng_value=10.0).play(IDLING, span_s=30.0)
    assert short > long
