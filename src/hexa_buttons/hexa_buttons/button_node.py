"""Front-panel buttons: two momentary switches on the Pi's GPIO header that put
diagnostic screens on the face's OLED.

  info button       press  -> pack percentage/voltage + the web teleop address
  bluetooth button  press  -> connected controller, or "no controllers"
                    hold   -> start a Bluetooth pairing scan (the face wears
                              its SCANNING spinners for as long as it runs)

A producer into the display, not part of it: hexa_display stays a pure sink of
robot state, and everything here reaches it over the topics it already listens
on — /display/text for the screens, /bluetooth/scanning for the spinners.
Nothing here imports hexa_display.

Bluetooth seam. This node owns the *session*: it publishes /bluetooth/scanning
true when the operator asks to pair and false when the scan is cancelled, times
out, or succeeds. The scanning utility (not yet written) is the *executor*: it
watches /bluetooth/scanning, runs the scan/pair for as long as it is true, and
reports the result by publishing the controller name on /bluetooth/status
(empty = nothing connected). Until it exists, the status topic simply has no
publisher, the bluetooth screen reads "No connected controllers", and a scan
runs out its scan_timeout_s.

Threading. gpiozero delivers edges on its own pin thread and holds on a second
one; rclpy runs the tick timer and both subscriptions on the executor thread.
The GPIO callbacks do exactly one thing — timestamp the event and put it on a
queue — and the tick drains that queue before advancing the clocks. So the
sequencer is touched by one thread only and needs no lock of its own, a press
at t=5.99 can never be applied after the timeout check at t=6.05, and a callback
firing during shutdown cannot reach a destroyed publisher. See _enqueue/_tick.
"""
from __future__ import annotations

import queue
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from .gpio_buttons import ButtonHardwareError, open_buttons
from .info_text import InfoConfig, screen_text
from .local_ip import local_ipv4
from .screen_logic import Event, Screen, ScreenConfig, ScreenSequencer

#: How often the battery/address screen is rebuilt while it is up. The pack
#: voltage and the DHCP lease both move on a human timescale; the tick runs
#: faster than either and must not re-render every time.
INFO_REFRESH_S = 1.0


def _read_only(description: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=description, read_only=True)


class ButtonNode(Node):
    def __init__(self) -> None:
        super().__init__("button_node")

        chip_path = self.declare_parameter(
            "gpio_chip", "/dev/gpiochip0", _read_only("GPIO character device")
        ).value
        info_line = self.declare_parameter(
            "info_line", 5, _read_only("BCM line of the battery/address button")
        ).value
        bluetooth_line = self.declare_parameter(
            "bluetooth_line", 6, _read_only("BCM line of the Bluetooth button")
        ).value
        active_low = self.declare_parameter(
            "active_low", True, _read_only("button shorts the line to ground")
        ).value
        bias_pull_up = self.declare_parameter(
            "bias_pull_up", True, _read_only("enable the SoC's internal pull-up")
        ).value
        tick_rate_hz = max(
            1.0,
            self.declare_parameter(
                "tick_rate_hz", 20.0, _read_only("housekeeping tick rate (Hz)")
            ).value,
        )
        debounce_s = self.declare_parameter("debounce_s", 0.03).value
        self._hold_s = self.declare_parameter("hold_s", 3.0).value

        self._sequencer = ScreenSequencer(
            ScreenConfig(
                hold_s=self._hold_s,
                screen_timeout_s=self.declare_parameter("screen_timeout_s", 6.0).value,
                scan_timeout_s=self.declare_parameter("scan_timeout_s", 30.0).value,
            )
        )
        self._info = InfoConfig(
            battery_empty_v=self.declare_parameter("battery_empty_v", 6.6).value,
            battery_full_v=self.declare_parameter("battery_full_v", 8.4).value,
            control_port=self.declare_parameter("control_port", 8080).value,
            arrow=self.declare_parameter("label_arrow", "->").value,
        )
        self._interfaces = self.declare_parameter(
            "ip_interfaces", ["wlan0", "eth0"]
        ).value
        battery_topic = self.declare_parameter(
            "battery_topic", "/hexa_hardware_aux/battery_state"
        ).value

        # Written by the GPIO threads, drained by the tick. SimpleQueue never
        # blocks its writer, so a stalled executor buffers events rather than
        # dropping them — and they replay with the timestamps they really had.
        self._events: queue.SimpleQueue = queue.SimpleQueue()

        self._buttons = None
        try:
            self._buttons = open_buttons(
                chip_path=chip_path,
                info_line=info_line,
                bluetooth_line=bluetooth_line,
                active_low=active_low,
                bias_pull_up=bias_pull_up,
                debounce_s=debounce_s,
                hold_s=self._hold_s,
                on_event=self._enqueue,
            )
        except ButtonHardwareError as exc:
            # Not fatal. The buttons are a convenience fitting: a robot with
            # none soldered on, or a dev box with no gpiochip, must still bring
            # the stack up. The node stays alive but inert.
            self.get_logger().error(
                f"buttons disabled — {exc}. Check the wiring, that the lines "
                f"({info_line}, {bluetooth_line}) are not claimed by another "
                f"driver, and that the container has access to {chip_path} "
                f"(group gpio)."
            )

        # transient_local, matching hexa_display's subscriptions: a display that
        # starts (or restarts) after a button press still gets the screen.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_text = self.create_publisher(String, "/display/text", latched)
        self._pub_scanning = self.create_publisher(Bool, "/bluetooth/scanning", latched)

        # Both subscriptions run on the executor thread, same as the tick — the
        # default executor is single-threaded, so these need no locking. Do not
        # swap in a MultiThreadedExecutor without revisiting that.
        self._voltage: float | None = None
        self._controller = ""
        self.create_subscription(
            BatteryState, battery_topic, self._on_battery, qos_profile_sensor_data
        )
        self.create_subscription(String, "/bluetooth/status", self._on_status, latched)

        self._published_text: str | None = None
        self._published_scanning: bool | None = None
        self._rendered_at = 0.0

        # Latch the resting state so the face has a definite answer before the
        # first press. The text topic is deliberately left alone until a button
        # is pressed — publishing an empty string at startup would stomp a
        # message somebody else latched there.
        self._publish_scanning(False)

        self.create_timer(1.0 / tick_rate_hz, self._tick)

        self.get_logger().info(
            f"buttons up: info on {chip_path} line {info_line}, bluetooth on "
            f"line {bluetooth_line} (hold {self._hold_s:.0f} s to pair) @ "
            f"{tick_rate_hz:.0f} Hz"
            + ("" if self._buttons else " — INERT, no GPIO")
        )

    def close(self) -> None:
        if self._buttons is not None:
            self._buttons.close()
            self._buttons = None

    # --- GPIO thread ------------------------------------------------------

    def _enqueue(self, event: Event) -> None:
        """Runs on a gpiozero callback thread.

        The timestamp is taken HERE, not at drain time, so a late drain still
        applies the event at the instant it actually happened. Nothing else is
        touched — no publisher, no node state, no sequencer — which is the whole
        safety argument for this design.
        """
        self._events.put((event, time.monotonic()))

    # --- executor thread --------------------------------------------------

    def _on_battery(self, msg: BatteryState) -> None:
        self._voltage = msg.voltage

    def _on_status(self, msg: String) -> None:
        self._controller = msg.data
        # A controller appearing while a scan is running means that scan
        # succeeded: drop the spinners and hand the panel back straight away
        # rather than making the operator wait out scan_timeout_s.
        if self._controller:
            self._sequencer.on_controller_seen(time.monotonic())
            if self._sequencer.changed:
                self.get_logger().info(
                    f"paired with '{self._controller}' — scan complete"
                )
                self._publish_scanning(False)
        # The bluetooth screen, if it happens to be up, now names someone else.
        self._render()

    def _tick(self) -> None:
        # Monotonic throughout, never the node clock: that is RCL_ROS_TIME, i.e.
        # system time, and a Pi has no RTC — an NTP step at boot would expire a
        # live screen or hang one for the size of the step.
        now = time.monotonic()

        changed = False
        while True:  # drain BEFORE the tick, so ordering is total
            try:
                event, timestamp = self._events.get_nowait()
            except queue.Empty:
                break
            self._sequencer.apply(event, timestamp)
            changed = changed or self._sequencer.changed

        self._sequencer.apply(Event.TICK, now)
        changed = changed or self._sequencer.changed

        if changed:
            self.get_logger().info(f"screen -> {self._sequencer.screen.value}")
            self._publish_scanning(self._sequencer.screen is Screen.SCANNING)
        # A live screen is rebuilt periodically so the pack percentage and the
        # address track reality while the operator is reading them.
        if changed or now - self._rendered_at >= INFO_REFRESH_S:
            self._rendered_at = now
            self._render()

    def _render(self) -> None:
        self._publish_text(
            screen_text(
                self._sequencer.screen,
                voltage=self._voltage,
                ip=local_ipv4(self._interfaces),
                controller=self._controller,
                config=self._info,
                hold_s=self._hold_s,
            )
        )

    def _publish_text(self, text: str) -> None:
        if text == self._published_text:
            return
        self._published_text = text
        self._pub_text.publish(String(data=text))

    def _publish_scanning(self, scanning: bool) -> None:
        if self._published_scanning == scanning:
            return
        self._published_scanning = scanning
        self._pub_scanning.publish(Bool(data=scanning))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ButtonNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
