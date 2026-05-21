"""``/dev/input/jsN`` → ``sensor_msgs/Joy`` publisher with hot-plug.

Drop-in replacement for upstream ``joy_node``. The SDL2-backed
``joy_node`` nominally handles hot-plug via udev events, but those
events do not propagate reliably into the dev container, so a
disconnect leaves it holding a dead fd until restart. This node polls
``/dev/input/`` instead and re-opens the device whenever it returns,
so unplugging or rebooting the controller mid-session recovers without
restarting any ROS process.

Wire-compatible with ``joy_node`` for the controllers we care about:
publishes ``sensor_msgs/Joy`` on ``/joy`` with the axis/button layout
the Linux joystick API (``/dev/input/jsN``) exposes — which is what
``joy_node`` + SDL2 forward verbatim for Xbox-style pads (including the
8BitDo Pro 2 in X-input mode).

While the device is absent the node still publishes ``Joy`` at the
configured ``autorepeat_rate`` with empty axes and buttons. The
``joy_mapping`` reader bounds-checks every index, so an empty message
cleanly resolves to zero stick, no buttons — i.e. the safe idle state.
"""

from __future__ import annotations

import errno
import fcntl
import glob
import os
import struct
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


# Linux joystick API ioctls. _IOR('j', N, __u8): direction=READ (2),
# type='j' (0x6A), nr=0x11/0x12, size=1.
_JSIOCGAXES = 0x80016A11
_JSIOCGBUTTONS = 0x80016A12

# js_event layout from <linux/joystick.h>: u32 time_ms, s16 value,
# u8 type, u8 number — 8 bytes.
_JS_EVENT_FMT = "<IhBB"
_JS_EVENT_SIZE = struct.calcsize(_JS_EVENT_FMT)

_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80  # set on the synthetic events emitted on open

# Linux reports axis values as int16. Scaling to [-1, 1] divides by the
# positive max so a fully-pressed negative axis lands at ~-1.00003 in
# the worst case (-32768 / 32767); downstream consumers clamp.
_AXIS_SCALE = 1.0 / 32767.0


def parse_js_event(buf: bytes) -> tuple[int, int, int, int]:
    """Decode one 8-byte js_event. Returns ``(time_ms, value, type, number)``."""
    return struct.unpack(_JS_EVENT_FMT, buf)


def find_js_devices() -> list[str]:
    """Return all ``/dev/input/jsN`` paths, sorted by N.

    Linux assigns ``jsN`` numbers dynamically — a controller that came
    up as ``js0`` first plug-in might come back as ``js1`` after a
    replug if a different one took ``js0`` in between. So a fixed
    ``device_id`` is brittle; auto-discovery is the only thing that
    survives a reconnect.
    """
    paths = glob.glob("/dev/input/js*")
    def _num(p: str) -> int:
        try:
            return int(p.rsplit("js", 1)[-1])
        except ValueError:
            return 1_000_000  # sort unparseable names to the end
    return sorted(paths, key=_num)


class _JsHandle:
    """Open ``/dev/input/jsN`` fd plus the current axis / button state."""

    def __init__(self, path: str, fd: int, n_axes: int, n_buttons: int) -> None:
        self.path = path
        self.fd = fd
        self.axes = [0.0] * n_axes
        self.buttons = [0] * n_buttons

    @classmethod
    def open(cls, path: str) -> "_JsHandle":
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        try:
            buf = bytearray(1)
            fcntl.ioctl(fd, _JSIOCGAXES, buf, True)
            n_axes = buf[0]
            fcntl.ioctl(fd, _JSIOCGBUTTONS, buf, True)
            n_buttons = buf[0]
        except OSError:
            os.close(fd)
            raise
        return cls(path, fd, n_axes, n_buttons)

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass

    def drain(self) -> bool:
        """Consume all pending events, updating ``axes`` / ``buttons``.

        Returns False if the device has gone away (caller should close
        and watch for it to come back); True otherwise.
        """
        while True:
            try:
                chunk = os.read(self.fd, _JS_EVENT_SIZE)
            except BlockingIOError:
                return True
            except OSError as e:
                # ENODEV: device unplugged. EIO: read on a stale fd
                # after disconnect. EBADF: already closed elsewhere.
                if e.errno in (errno.ENODEV, errno.EIO, errno.EBADF):
                    return False
                raise
            if len(chunk) < _JS_EVENT_SIZE:
                # 0-byte read = EOF (device removed); short read is
                # unexpected on a char device but treat it the same.
                return False
            _, value, ev_type, number = parse_js_event(chunk)
            kind = ev_type & ~_JS_EVENT_INIT
            if kind == _JS_EVENT_AXIS and number < len(self.axes):
                self.axes[number] = value * _AXIS_SCALE
            elif kind == _JS_EVENT_BUTTON and number < len(self.buttons):
                self.buttons[number] = 1 if value else 0


class JoyPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("joy_node")
        # Empty string => auto-discover the first /dev/input/jsN.
        # Anything else is taken as the literal path to open. We
        # deliberately do NOT expose a numeric ``device_id``: Linux
        # renumbers jsN on replug, so a pinned number is a sharp edge
        # in exactly the use case this node was written for.
        self.declare_parameter("device_path", "")
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("autorepeat_rate", 50.0)
        self.declare_parameter("scan_period_s", 1.0)

        self._device_path = str(self.get_parameter("device_path").value)
        self._deadzone = float(self.get_parameter("deadzone").value)
        rate = float(self.get_parameter("autorepeat_rate").value)
        scan = float(self.get_parameter("scan_period_s").value)

        self._handle: _JsHandle | None = None
        # Edge-trigger logging so a missing device at startup logs
        # once and stays quiet until something changes.
        self._waiting_logged = False

        self._pub = self.create_publisher(Joy, "/joy", 10)
        self._publish_timer = self.create_timer(1.0 / rate, self._publish_tick)
        self._scan_timer = self.create_timer(scan, self._scan_tick)

        target = self._device_path or "/dev/input/js* (auto)"
        self.get_logger().info(
            f"watching {target} "
            f"(autorepeat={rate:.0f} Hz, deadzone={self._deadzone:.3f}, "
            f"scan={scan:.1f} s)"
        )
        # Attempt one open immediately so a controller already plugged
        # in at launch is live on the first publish tick.
        self._try_open()

    def _candidate_paths(self) -> list[str]:
        if self._device_path:
            return [self._device_path] if Path(self._device_path).exists() else []
        return find_js_devices()

    def _try_open(self) -> None:
        if self._handle is not None:
            return
        candidates = self._candidate_paths()
        if not candidates:
            if not self._waiting_logged:
                target = self._device_path or "any /dev/input/jsN"
                self.get_logger().warning(
                    f"{target} not present — waiting for controller"
                )
                self._waiting_logged = True
            return
        # Try each candidate; first one that opens cleanly wins. A
        # device may exist but be unreadable (permissions, exclusive
        # open by another process) — keep trying others instead of
        # giving up on the whole scan.
        last_err: OSError | None = None
        for path in candidates:
            try:
                handle = _JsHandle.open(path)
            except OSError as e:
                last_err = e
                continue
            self._handle = handle
            self._waiting_logged = False
            self.get_logger().info(
                f"opened {handle.path}: "
                f"{len(handle.axes)} axes, {len(handle.buttons)} buttons"
            )
            return
        if last_err is not None and not self._waiting_logged:
            self.get_logger().warning(
                f"found {candidates} but none opened: {last_err.strerror} "
                f"(errno={last_err.errno})"
            )
            self._waiting_logged = True

    def _drop_handle(self, reason: str) -> None:
        if self._handle is None:
            return
        path = self._handle.path
        self._handle.close()
        self._handle = None
        self.get_logger().warning(
            f"controller on {path} lost ({reason}); watching for reconnect"
        )

    def _scan_tick(self) -> None:
        if self._handle is None:
            self._try_open()

    def _publish_tick(self) -> None:
        if self._handle is not None and not self._handle.drain():
            self._drop_handle("read error / device removed")

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._handle is not None:
            msg.axes = [
                0.0 if abs(v) < self._deadzone else v
                for v in self._handle.axes
            ]
            msg.buttons = list(self._handle.buttons)
        # else: leave axes/buttons empty; joy_mapping bounds-checks
        # every index, so an empty Joy resolves to "all zero, no
        # buttons" — the safe idle state.
        self._pub.publish(msg)

    def shutdown(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
