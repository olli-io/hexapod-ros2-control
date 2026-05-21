"""Tests for the pure parts of joy_publisher.

Covers the ``js_event`` decoder and the ``_JsHandle.drain`` state
machine through a pair of file descriptors (no real device needed).
The reconnect / open paths are intentionally not unit-tested — they
hinge on the kernel reporting /dev/input/jsN behaviour, which is
better verified end-to-end in the dev container.
"""

import os
import struct

from unittest.mock import patch

from hexa_teleop.joy_publisher import (
    _AXIS_SCALE,
    _JS_EVENT_AXIS,
    _JS_EVENT_BUTTON,
    _JS_EVENT_INIT,
    _JsHandle,
    find_js_devices,
    parse_js_event,
)


def _make_event(value: int, ev_type: int, number: int) -> bytes:
    return struct.pack("<IhBB", 0, value, ev_type, number)


def test_parse_js_event_axis():
    buf = _make_event(16383, _JS_EVENT_AXIS, 2)
    _, value, ev_type, number = parse_js_event(buf)
    assert value == 16383
    assert ev_type == _JS_EVENT_AXIS
    assert number == 2


def _handle_from_events(events: list[bytes], n_axes: int, n_buttons: int) -> _JsHandle:
    """Build a _JsHandle reading from a pipe pre-loaded with ``events``."""
    read_fd, write_fd = os.pipe()
    os.set_blocking(read_fd, False)
    for ev in events:
        os.write(write_fd, ev)
    os.close(write_fd)
    return _JsHandle("test", read_fd, n_axes, n_buttons)


def test_drain_applies_axis_and_button_events():
    events = [
        _make_event(32767, _JS_EVENT_AXIS, 0),
        _make_event(-32767, _JS_EVENT_AXIS, 1),
        _make_event(1, _JS_EVENT_BUTTON, 3),
        # Init-flagged event is still applied (kernel sends these on
        # open to seed the initial axis values).
        _make_event(-16383, _JS_EVENT_AXIS | _JS_EVENT_INIT, 2),
    ]
    handle = _handle_from_events(events, n_axes=4, n_buttons=4)
    try:
        # Read returns True (EOF on pipe is treated as "device gone"
        # but here it isn't a real device — we don't assert the return.
        handle.drain()
        assert handle.axes[0] == 32767 * _AXIS_SCALE
        assert handle.axes[1] == -32767 * _AXIS_SCALE
        assert handle.axes[2] == -16383 * _AXIS_SCALE
        assert handle.buttons[3] == 1
    finally:
        handle.close()


def test_drain_ignores_out_of_range_indices():
    """A controller exposing fewer axes than we sized for is fine; one
    sending events for indices past the end must not raise."""
    events = [
        _make_event(32767, _JS_EVENT_AXIS, 9),  # past end of axes
        _make_event(1, _JS_EVENT_BUTTON, 9),  # past end of buttons
        _make_event(16383, _JS_EVENT_AXIS, 0),  # in range
    ]
    handle = _handle_from_events(events, n_axes=2, n_buttons=2)
    try:
        handle.drain()
        assert handle.axes[0] == 16383 * _AXIS_SCALE
        assert handle.axes[1] == 0.0
        assert handle.buttons == [0, 0]
    finally:
        handle.close()


def test_find_js_devices_sorts_numerically():
    """js2 must come before js10 — a lexicographic sort would invert
    them and pick the wrong device when both exist."""
    fake = ["/dev/input/js10", "/dev/input/js2", "/dev/input/js0"]
    with patch("hexa_teleop.joy_publisher.glob.glob", return_value=fake):
        assert find_js_devices() == [
            "/dev/input/js0",
            "/dev/input/js2",
            "/dev/input/js10",
        ]


def test_drain_reports_eof_as_device_lost():
    handle = _handle_from_events([], n_axes=2, n_buttons=2)
    try:
        # Pipe write end was closed in the helper, so the first read
        # returns 0 bytes → device-lost.
        assert handle.drain() is False
    finally:
        handle.close()
