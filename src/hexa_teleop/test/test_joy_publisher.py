"""Tests for the pure parts of joy_publisher.

Covers the ``js_event`` decoder and the ``_JsHandle.drain`` state
machine through a pair of file descriptors (no real device needed).
The reconnect / open paths are intentionally not unit-tested — they
hinge on the kernel reporting /dev/input/jsN behaviour, which is
better verified end-to-end in the sim container.
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
    event_device_for,
    find_js_devices,
    parse_js_event,
    trigger_axes,
)


def _make_event(value: int, ev_type: int, number: int) -> bytes:
    return struct.pack("<IhBB", 0, value, ev_type, number)


def test_parse_js_event_axis():
    buf = _make_event(16383, _JS_EVENT_AXIS, 2)
    _, value, ev_type, number = parse_js_event(buf)
    assert value == 16383
    assert ev_type == _JS_EVENT_AXIS
    assert number == 2


def _handle_from_events(
    events: list[bytes],
    n_axes: int,
    n_buttons: int,
    invert: set[int] | None = None,
) -> _JsHandle:
    """Build a _JsHandle reading from a pipe pre-loaded with ``events``."""
    read_fd, write_fd = os.pipe()
    os.set_blocking(read_fd, False)
    for ev in events:
        os.write(write_fd, ev)
    os.close(write_fd)
    return _JsHandle("test", read_fd, n_axes, n_buttons, invert)


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


def _fake_evdev(axes: dict[int, tuple[int, int]]):
    """Patch context emulating a device declaring ``{abs_code: (min, max)}``."""
    from hexa_teleop import joy_publisher as jp

    def fake_ioctl(fd, request, buf, mutate=False):
        if request == jp._EVIOCGBIT_ABS:
            for code in axes:
                buf[code // 8] |= 1 << (code % 8)
            return 0
        for code, (lo, hi) in axes.items():
            if request == jp._eviocgabs(code):
                buf[:] = struct.pack(jp._ABSINFO_FMT, lo, lo, hi, 0, 0, 0)
                return 0
        raise OSError("unexpected ioctl")

    return (
        patch.object(jp.glob, "glob", return_value=["/sys/class/input/js0/device/event5"]),
        patch.object(jp.fcntl, "ioctl", fake_ioctl),
        patch.object(jp.os, "open", return_value=999),
        patch.object(jp.os, "close", return_value=None),
    )


def test_trigger_axes_finds_triggers_but_not_unipolar_sticks():
    """Regression: an 8BitDo Pro 2 declares its STICKS as 0..65535 too,
    resting at the midpoint. Keying off the unipolar range alone would
    invert left/right and forward/back; only ABS_Z / ABS_RZ are
    triggers."""
    device = {
        0x00: (0, 65535),  # ABS_X   — stick, unipolar but centre-resting
        0x01: (0, 65535),  # ABS_Y
        0x02: (0, 1023),   # ABS_Z   — left trigger
        0x03: (0, 65535),  # ABS_RX
        0x04: (0, 65535),  # ABS_RY
        0x05: (0, 1023),   # ABS_RZ  — right trigger
        0x10: (-1, 1),     # ABS_HAT0X
        0x11: (-1, 1),     # ABS_HAT0Y
    }
    patches = _fake_evdev(device)
    for p in patches:
        p.start()
    try:
        # js indices, not ABS codes: ascending code order, so l2/r2 land
        # on 2 and 5 exactly as base.axes in teleop_joy.yaml declares.
        assert trigger_axes("/dev/input/js0") == {2, 5}
    finally:
        for p in patches:
            p.stop()


def test_trigger_axes_leaves_a_bipolar_abs_z_alone():
    """A device that reports ABS_Z as a signed axis is not a gamepad
    trigger resting at min — negating it would be wrong."""
    patches = _fake_evdev({0x00: (-32768, 32767), 0x02: (-32768, 32767)})
    for p in patches:
        p.start()
    try:
        assert trigger_axes("/dev/input/js0") == set()
    finally:
        for p in patches:
            p.stop()


def test_drain_negates_trigger_axes():
    """A joydev-scaled trigger rests at -1.0 and travels to +1.0; the
    published Joy must carry the opposite (rest +1.0), which is what
    teleop_joy.yaml's ``trigger_threshold`` reads as released/pressed."""
    events = [
        _make_event(-32767, _JS_EVENT_AXIS | _JS_EVENT_INIT, 2),  # at rest
        _make_event(32767, _JS_EVENT_AXIS, 5),  # fully squeezed
        _make_event(-32767, _JS_EVENT_AXIS, 1),  # bipolar stick, untouched
    ]
    handle = _handle_from_events(events, n_axes=8, n_buttons=8, invert={2, 5})
    try:
        handle.drain()
        assert handle.axes[2] == 32767 * _AXIS_SCALE  # released
        assert handle.axes[5] == -32767 * _AXIS_SCALE  # pressed
        assert handle.axes[1] == -32767 * _AXIS_SCALE  # stick unaffected
    finally:
        handle.close()


def test_trigger_axes_start_released_before_the_first_drain():
    """Init events only land on the first drain, and an unseeded 0.0
    would read as a held trigger — which silently cancels L1/R1 yaw."""
    handle = _handle_from_events([], n_axes=8, n_buttons=8, invert={2, 5})
    try:
        assert handle.axes[2] == 1.0
        assert handle.axes[5] == 1.0
        assert handle.axes[0] == 0.0
    finally:
        handle.close()


def test_trigger_axes_empty_without_an_evdev_node():
    """No sysfs sibling (or an unreadable one) must not break the pad —
    it just means no inversion, the pre-existing behaviour."""
    with patch("hexa_teleop.joy_publisher.glob.glob", return_value=[]):
        assert trigger_axes("/dev/input/js0") == set()


def test_event_device_for_maps_js_to_its_sibling():
    fake = ["/sys/class/input/js0/device/event5"]
    with patch("hexa_teleop.joy_publisher.glob.glob", return_value=fake):
        assert event_device_for("/dev/input/js0") == "/dev/input/event5"


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
