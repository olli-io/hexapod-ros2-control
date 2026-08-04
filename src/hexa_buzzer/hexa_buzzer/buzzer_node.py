"""ROS glue for the buzzer: an event name on /buzzer/play becomes a sound.

The only rclpy module in the package; the work is all in `player`. The
subscription is `transient_local` because `up` is published from
hexa_hardware's activation, which can beat this node's subscription matching.
"""
from __future__ import annotations

import os
import queue
import threading

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from . import catalog, player, pwm, tunes

TOPIC = "/buzzer/play"

# Sentinel that retires the worker thread.
_STOP = None


def _config_dir() -> str:
    """The installed config/, or "" to let catalog search for it."""
    try:
        return os.path.join(get_package_share_directory("hexa_buzzer"), "config")
    except PackageNotFoundError:
        return ""


class BuzzerNode(Node):
    def __init__(self) -> None:
        super().__init__("buzzer_node")

        pwm_dev = self.declare_parameter("pwm_dev", pwm.DEFAULT_PWM_DEV).value
        channel = self.declare_parameter("channel", pwm.DEFAULT_CHANNEL).value
        wait_s = self.declare_parameter("wait_s", pwm.DEFAULT_WAIT_S).value

        self._pwm_dev = pwm_dev
        self._channel = channel
        self._wait_s = wait_s

        # Both tables come from the files, not from ROS parameters, so an event
        # resolves the same here as in the host units' player.
        self._tunes: dict[str, str] = {}
        self._events: dict[str, str] = {}
        try:
            config_dir = _config_dir()
            self._tunes = catalog.load_tunes(config_dir)
            self._events = catalog.load_events(config_dir)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f"no tunes to play — {exc}")

        # Probe up front, so a missing overlay or unmounted tree is reported
        # where somebody is reading logs rather than at the first fault.
        try:
            chip = pwm.find_chip(pwm_dev, wait_s)
        except pwm.BuzzerHardwareError as exc:
            # Not fatal: a robot with no buzzer must still bring the stack up.
            self.get_logger().error(
                f"buzzer disabled — {exc} The mount comes from "
                f"docker-compose.buzzer.yaml; see docs/robot-environment.md §15."
            )
            self._queue: queue.Queue | None = None
            self._worker: threading.Thread | None = None
            return

        self.get_logger().info(f"buzzer on {chip} channel {channel}")

        # A tune is seconds of sleeps and must never run on the executor. Depth
        # 1 because a backlog of stale beeps helps nobody.
        self._queue = queue.Queue(maxsize=1)
        self._worker = threading.Thread(target=self._play_loop, daemon=True)
        self._worker.start()

        self.create_subscription(
            String,
            TOPIC,
            self._on_tune,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def _on_tune(self, msg: String) -> None:
        name = msg.data.strip()
        if not name or self._queue is None:
            return
        try:
            notes = tunes.parse_rtttl(
                catalog.resolve(name, self._tunes, self._events)
            )
        except ValueError as exc:
            self.get_logger().warning(f"cannot play '{name}': {exc}")
            return
        try:
            self._queue.put_nowait((name, notes))
        except queue.Full:
            self.get_logger().debug(f"still playing — dropped '{name}'")

    def _play_loop(self) -> None:
        while True:
            item = self._queue.get()
            if item is _STOP:
                return
            name, notes = item
            try:
                player.play_notes(
                    notes,
                    pwm_dev=self._pwm_dev,
                    channel=self._channel,
                    wait_s=self._wait_s,
                )
            except pwm.BuzzerHardwareError as exc:
                self.get_logger().warning(f"cannot play '{name}': {exc}")

    def close(self) -> None:
        """Retire the worker so a shutdown does not race a tune in progress."""
        if self._queue is None or self._worker is None:
            return
        try:
            self._queue.put_nowait(_STOP)
        except queue.Full:
            pass
        self._worker.join(timeout=3.0)  # a stuck player must not wedge a restart


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BuzzerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
