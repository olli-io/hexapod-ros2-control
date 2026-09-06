"""Front-panel buttons: two momentary switches on the Pi's GPIO header that put
diagnostic screens on the face's OLED.

  info button       press  -> pack percentage/voltage + the web teleop address
                    hold   -> switch the Pi between wifi station and hotspot
                              mode (the face wears its spinners meanwhile)
  bluetooth button  press  -> connected controller, or "no controllers"
                    hold   -> start a Bluetooth pairing scan (the face wears
                              its SCANNING spinners for as long as it runs)

A producer into the display, not part of it: hexa_display stays a pure sink of
robot state, and everything here reaches it over the topics it already listens
on — /display/text for the screens, /bluetooth/scanning and /display/busy for
the spinners. Nothing here imports hexa_display.

Network seam. The stack runs in an unprivileged container with host networking,
no D-Bus socket and no NET_ADMIN, so it cannot run nmcli: the switch has to
happen on the host. This node asks for it the same way hexa_hardware asks for a
buzzer tune — write a word into the bind-mounted log volume and let a host
systemd .path unit relay it out (systemd/network-mode.sh). The host answers in a
second file and is the authority on which mode the radio is actually in; this
node only ever reports what it was told. See network_state for the wire format.

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
import secrets
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from . import network_spool
from .gpio_buttons import ButtonHardwareError, open_buttons
from .info_text import InfoConfig, screen_text
from .local_ip import local_ipv4
from .network_state import (
    REQUEST_TOGGLE,
    RESULT_ERROR,
    NetworkState,
    is_terminal,
    new_token,
    parse_state,
)
from .screen_logic import (
    Event,
    Screen,
    ScreenConfig,
    ScreenSequencer,
    wants_spinners,
)

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
                network_timeout_s=self.declare_parameter(
                    "network_timeout_s", 60.0
                ).value,
                network_info_timeout_s=self.declare_parameter(
                    "network_info_timeout_s", 20.0
                ).value,
            )
        )
        self._info = InfoConfig(
            battery_empty_v=self.declare_parameter("battery_empty_v", 6.6).value,
            battery_full_v=self.declare_parameter("battery_full_v", 8.4).value,
            control_port=self.declare_parameter("control_port", 8080).value,
            mdns_name=self.declare_parameter("mdns_name", "").value,
            arrow=self.declare_parameter("label_arrow", "->").value,
        )
        self._interfaces = self.declare_parameter(
            "ip_interfaces", ["wlan0", "eth0"]
        ).value
        battery_topic = self.declare_parameter(
            "battery_topic", "/hexa_hardware_aux/battery_state"
        ).value

        # Container-side paths into the bind-mounted log volume; the host sees
        # them under ~/hexa-robot/log/. Empty disables the whole gesture.
        self._network_enabled = self.declare_parameter(
            "network_toggle_enabled", True
        ).value
        self._network_spool = self.declare_parameter(
            "network_spool", "/workspace/log/network"
        ).value
        self._network_state_file = self.declare_parameter(
            "network_state_file", "/workspace/log/network.state"
        ).value
        # How long to wait for the host to acknowledge that it exists at all.
        # The helper writes result=switching before it starts the slow part, so
        # silence past this means the systemd units were never installed —
        # worth saying immediately rather than after network_timeout_s.
        self._network_ack_timeout_s = self.declare_parameter(
            "network_ack_timeout_s", 5.0
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
        # Separate from /bluetooth/scanning because it means something else —
        # the node is busy with a job the operator should wait out. hexa_display
        # ORs the two into the same spinners.
        self._pub_busy = self.create_publisher(Bool, "/display/busy", latched)

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
        self._published_busy: bool | None = None
        self._rendered_at = 0.0

        # Network-switch bookkeeping. The nonce is per-process, so a node that
        # restarts mid-switch cannot mistake the answer to its previous life's
        # request for an ack of a new one.
        self._network = NetworkState()
        self._network_nonce = secrets.token_hex(4)
        self._network_seq = 0
        self._network_token = ""
        self._network_requested_at = 0.0
        self._network_state_mtime = 0.0
        self._network_acked = False
        self._network_done = True  # nothing in flight
        # Seed from whatever the host last reported, ignoring the token: that is
        # how a container restart — or a reboot that came back in hotspot mode —
        # comes up rendering the truth instead of guessing station.
        self._poll_network_state(0.0)

        # Latch the resting state so the face has a definite answer before the
        # first press. The text topic is deliberately left alone until a button
        # is pressed — publishing an empty string at startup would stomp a
        # message somebody else latched there.
        self._publish_scanning(False)
        self._publish_busy(False)

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
            if event is Event.INFO_HOLD and not self._network_enabled:
                # Dropped here rather than in the GPIO callback: that thread is
                # allowed to touch the queue and nothing else.
                continue
            self._sequencer.apply(event, timestamp)
            changed = changed or self._sequencer.changed

        was_switching = self._sequencer.screen is Screen.NETWORK_SWITCHING
        self._sequencer.apply(Event.TICK, now)
        changed = changed or self._sequencer.changed

        if changed:
            self.get_logger().info(f"screen -> {self._sequencer.screen.value}")
            # Entering this screen is reachable only through an info hold, so
            # the transition IS the request — no extra flag on the sequencer.
            if self._sequencer.screen is Screen.NETWORK_SWITCHING:
                self._request_network_switch(now)

        switching = self._sequencer.screen is Screen.NETWORK_SWITCHING
        if was_switching and not switching and not self._network_done:
            # The sequencer's own backstop expired: the host took the request
            # and then went quiet. Say so, rather than showing a result screen
            # rendered from whatever the previous switch left behind.
            self._fail_network_switch(now, "timeout")
        elif switching:
            # Poll every tick while a switch is in flight so the panel turns
            # over the moment the host answers; otherwise ride the slow
            # refresh. It is a stat() and nothing is waiting on it.
            changed = self._poll_network_state(now) or changed
            changed = self._check_network_ack(now) or changed
        elif now - self._rendered_at >= INFO_REFRESH_S:
            changed = self._poll_network_state(now) or changed

        if changed:
            self._publish_scanning(self._sequencer.screen is Screen.SCANNING)
            self._publish_busy(wants_spinners(self._sequencer.screen))

        # A live screen is rebuilt periodically so the pack percentage and the
        # address track reality while the operator is reading them.
        if changed or now - self._rendered_at >= INFO_REFRESH_S:
            self._rendered_at = now
            self._render()

    # --- network mode -----------------------------------------------------

    def _request_network_switch(self, now: float) -> None:
        """Ask the host to toggle the radio. TOGGLE, not an explicit target:
        the host owns the radio and is therefore the only side that reliably
        knows which way round it currently is."""
        self._network_seq += 1
        self._network_token = new_token(self._network_nonce, self._network_seq)
        self._network_requested_at = now
        self._network_acked = False
        self._network_done = False
        if network_spool.request(self._network_spool, REQUEST_TOGGLE, self._network_token):
            self.get_logger().info(
                f"network switch requested ({self._network_token})"
            )
            return
        # No spool to write means no helper, and we know it now rather than in
        # five seconds' time.
        self.get_logger().error(
            f"cannot write the network spool '{self._network_spool}' — is the "
            f"log volume mounted, and has 'hexa robot install-network' been run?"
        )
        self._fail_network_switch(now, "no-helper")

    def _poll_network_state(self, now: float) -> bool:
        """Read the host's report if it has changed. True if the panel should
        be rebuilt."""
        mtime = network_spool.state_mtime(self._network_state_file)
        if mtime == self._network_state_mtime:
            return False
        self._network_state_mtime = mtime
        state = parse_state(network_spool.read_state(self._network_state_file))
        if state is None:
            return False
        # mode/ssid/psk are adopted whatever the token says: they are the host's
        # standing report of where the radio is, not an answer to this node.
        self._network = state
        if self._sequencer.screen is not Screen.NETWORK_SWITCHING:
            return True
        if state.token != self._network_token:
            return True  # somebody else's switch, or a stale file
        self._network_acked = True
        if not is_terminal(state.result):
            return True  # the host has it, and is still working
        self._network_done = True
        self.get_logger().info(
            f"network switch finished: mode={state.mode or '?'} "
            f"result={state.result}" + (f" reason={state.reason}" if state.reason else "")
        )
        self._sequencer.on_network_result(now)
        self._publish_scanning(False)
        self._publish_busy(wants_spinners(self._sequencer.screen))
        return True

    def _check_network_ack(self, now: float) -> bool:
        """Give up early if the host never even acknowledged the request.

        network-mode.sh writes result=switching before it touches the radio, so
        silence past network_ack_timeout_s means nothing is listening to the
        spool at all. Without this the panel would spin for the full
        network_timeout_s before admitting the units are not installed.
        """
        if self._network_acked:
            return False
        if now - self._network_requested_at < self._network_ack_timeout_s:
            return False
        self.get_logger().error(
            "no answer from the host network helper — run "
            "'hexa robot install-network' on the Pi"
        )
        self._fail_network_switch(now, "no-helper")
        return True

    def _fail_network_switch(self, now: float, reason: str) -> None:
        """End a switch the host is not going to finish, with a reason the
        panel can explain. Keeps whatever mode the host last reported — the
        radio has not moved."""
        self._network = NetworkState(
            mode=self._network.mode,
            token=self._network_token,
            result=RESULT_ERROR,
            reason=reason,
            ssid=self._network.ssid,
            psk=self._network.psk,
        )
        self._network_acked = True
        self._network_done = True
        self._sequencer.on_network_result(now)
        self._publish_scanning(False)
        self._publish_busy(wants_spinners(self._sequencer.screen))

    def _render(self) -> None:
        self._publish_text(
            screen_text(
                self._sequencer.screen,
                voltage=self._voltage,
                ip=local_ipv4(self._interfaces),
                controller=self._controller,
                config=self._info,
                hold_s=self._hold_s,
                network=self._network,
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

    def _publish_busy(self, busy: bool) -> None:
        if self._published_busy == busy:
            return
        self._published_busy = busy
        self._pub_busy.publish(Bool(data=busy))


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
