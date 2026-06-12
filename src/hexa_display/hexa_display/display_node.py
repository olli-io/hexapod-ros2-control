"""Display relay node.

Subscribes to the gait engine state (`/gait/state`), the latest body
velocity command (`/cmd_vel`), the user body pose (`/body/pose`), the
posture animation mode (`/animation/mode`), and the battery state.
On a fixed timer it runs the pure expression policy and relays
SET_EXPRESSION / SET_GAZE frames to the ESP32 face over the configured
transport (`serial` on the robot, `stub` in sim — the stub logs the
decoded frames instead).

Pure sink: nothing in the workspace subscribes to or imports this
package. Fire-and-forget TX — the firmware animates autonomously and
NACKs are only logged. The transport is retried in the background, so
the robot comes up (and stays up) faceless if the display is absent.

Face animations: while the policy selects one (breathing during
stack bringup, idling once the robot stands idle), this node runs its
clock and relays the due gaze/blink steps; the animation owns the
gaze until it ends.
"""

from dataclasses import replace

import rclpy
from geometry_msgs.msg import Twist
from hexa_interfaces.msg import BodyPose as BodyPoseMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from .expression_policy import (
    DEFAULT_EXPRESSION_MAP,
    IDLE_TARGET,
    BatteryMonitor,
    DisplayTarget,
    PolicyConfig,
    PolicyInputs,
    decide,
    select_face_animation,
)
from .face_animation import FACE_ANIMATIONS, IDLING, FaceAnimation, due_steps
from .protocol import Cmd, Expression, NackReason, decode_frames
from .protocol import set_expression as set_expression_frame
from .protocol import set_gaze as set_gaze_frame
from .protocol import trigger_blink as trigger_blink_frame
from .transport import (
    SerialTransport,
    StubTransport,
    Transport,
    TransportError,
)

WARN_THROTTLE_S = 10.0


def _parse_expression(name: str, param: str) -> Expression:
    key = name.strip().upper()
    if key not in Expression.__members__:
        valid = ", ".join(m.lower() for m in Expression.__members__)
        raise ValueError(f"{param}: unknown expression {name!r}; valid: {valid}")
    return Expression[key]


class DisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("display_node")

        self._gait_state: str | None = None
        self._cmd_vel = Twist()
        self._body_pose = BodyPoseMsg()
        self._animation_mode: str = ""
        self._battery_voltage: float | None = None
        self._last_target: DisplayTarget = IDLE_TARGET
        self._sent_expression: Expression | None = None
        self._sent_gaze = None
        self._rx_buf = b""
        self._last_refresh_t: float | None = None
        self._last_reconnect_t: float | None = None
        self._active_face_animation: FaceAnimation | None = None
        self._face_animation_start_t = 0.0
        self._face_animation_fired = 0
        self._pending_face_animation: str | None = None
        self._pending_face_animation_since = 0.0
        self._idling_expression_i = 0

        self.declare_parameter("transport", "serial")
        self.declare_parameter("serial_device", "/dev/serial0")
        self.declare_parameter("serial_baud", 921600)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("refresh_period_s", 5.0)
        for state, expression in DEFAULT_EXPRESSION_MAP.items():
            self.declare_parameter(
                f"expression_map.{state}", expression.name.lower()
            )
        self.declare_parameter("animation_expression", "woozy")
        self.declare_parameter(
            "battery_topic", "/hexa_hardware_aux/battery_state"
        )
        self.declare_parameter("battery_warning_expression", "sleepy")
        self.declare_parameter("battery_critical_expression", "dead")
        self.declare_parameter("battery_warning_v", 0.0)
        self.declare_parameter("battery_critical_v", 0.0)
        self.declare_parameter("battery_hysteresis_v", 0.3)
        self.declare_parameter("battery_hold_s", 3.0)
        self.declare_parameter("gaze_deadband", 0.15)
        self.declare_parameter("gaze_exit_ratio", 0.6)
        self.declare_parameter("gaze_wz_weight", 1.0)
        self.declare_parameter("gaze_vy_max", 0.1)
        self.declare_parameter("gaze_wz_max", 0.5)
        self.declare_parameter("pose_pitch_threshold_rad", 0.08)
        self.declare_parameter("pose_tilt_threshold_rad", 0.08)
        self.declare_parameter("idling_expressions", ["neutral", "happy"])
        self.declare_parameter("idling_start_delay_s", 4.0)

        def _str(name: str) -> str:
            return self.get_parameter(name).get_parameter_value().string_value

        def _dbl(name: str) -> float:
            return self.get_parameter(name).get_parameter_value().double_value

        # Fail fast on expression-name typos in the YAML.
        expression_map = {
            state: _parse_expression(
                _str(f"expression_map.{state}"), f"expression_map.{state}"
            )
            for state in DEFAULT_EXPRESSION_MAP
        }
        # [''] in the YAML disables the idle expression cycling.
        idling_expressions = tuple(
            _parse_expression(name, "idling_expressions")
            for name in self.get_parameter("idling_expressions")
            .get_parameter_value()
            .string_array_value
            if name.strip()
        )
        self._config = PolicyConfig(
            expression_map=expression_map,
            animation_expression=_parse_expression(
                _str("animation_expression"), "animation_expression"
            ),
            battery_warning_expression=_parse_expression(
                _str("battery_warning_expression"), "battery_warning_expression"
            ),
            battery_critical_expression=_parse_expression(
                _str("battery_critical_expression"), "battery_critical_expression"
            ),
            gaze_deadband=_dbl("gaze_deadband"),
            gaze_exit_ratio=_dbl("gaze_exit_ratio"),
            gaze_wz_weight=_dbl("gaze_wz_weight"),
            gaze_vy_max=_dbl("gaze_vy_max"),
            gaze_wz_max=_dbl("gaze_wz_max"),
            pose_pitch_threshold_rad=_dbl("pose_pitch_threshold_rad"),
            pose_tilt_threshold_rad=_dbl("pose_tilt_threshold_rad"),
            idling_expressions=idling_expressions,
            idling_start_delay_s=_dbl("idling_start_delay_s"),
        )
        self._battery_monitor = BatteryMonitor(
            warning_v=_dbl("battery_warning_v"),
            critical_v=_dbl("battery_critical_v"),
            hysteresis_v=_dbl("battery_hysteresis_v"),
            hold_s=_dbl("battery_hold_s"),
        )
        self._reconnect_period_s = _dbl("reconnect_period_s")
        self._refresh_period_s = _dbl("refresh_period_s")

        self._transport = self._make_transport(_str("transport"))
        try:
            self._transport.open()
        except TransportError as e:
            # The robot must come up faceless: keep retrying from the
            # tick timer instead of crashing.
            self.get_logger().warn(f"display transport unavailable: {e}")

        self._sub_gait_state = self.create_subscription(
            String, "/gait/state", self._on_gait_state, 10
        )
        self._sub_vel = self.create_subscription(
            Twist, "/cmd_vel", self._on_vel, 10
        )
        self._sub_pose = self.create_subscription(
            BodyPoseMsg, "/body/pose", self._on_pose, 10
        )
        # transient_local to match the teleop publisher, so a late
        # display start still sees the active animation mode.
        animation_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._sub_animation_mode = self.create_subscription(
            String, "/animation/mode", self._on_animation_mode, animation_qos
        )
        self._sub_battery = self.create_subscription(
            BatteryState,
            _str("battery_topic"),
            self._on_battery,
            qos_profile_sensor_data,
        )

        rate = _dbl("update_rate_hz")
        self._timer = self.create_timer(1.0 / rate, self._tick)

    def _make_transport(self, kind: str) -> Transport:
        if kind == "serial":
            device = (
                self.get_parameter("serial_device")
                .get_parameter_value()
                .string_value
            )
            baud = (
                self.get_parameter("serial_baud")
                .get_parameter_value()
                .integer_value
            )
            self.get_logger().info(f"display transport: serial {device} @ {baud}")
            return SerialTransport(device, baud)
        if kind == "stub":
            self.get_logger().info("display transport: stub (logging frames)")
            return StubTransport(
                log_fn=lambda msg: self.get_logger().info(f"display: {msg}")
            )
        raise ValueError(f"transport: unknown kind {kind!r} (serial | stub)")

    def _on_gait_state(self, msg: String) -> None:
        self._gait_state = msg.data

    def _on_vel(self, msg: Twist) -> None:
        self._cmd_vel = msg

    def _on_pose(self, msg: BodyPoseMsg) -> None:
        self._body_pose = msg

    def _on_animation_mode(self, msg: String) -> None:
        self._animation_mode = msg.data

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_voltage = msg.voltage

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _ensure_transport(self, now: float) -> bool:
        if self._transport.is_open:
            return True
        if (
            self._last_reconnect_t is not None
            and now - self._last_reconnect_t < self._reconnect_period_s
        ):
            return False
        self._last_reconnect_t = now
        try:
            self._transport.open()
        except TransportError as e:
            self.get_logger().warn(
                f"display transport reconnect failed: {e}",
                throttle_duration_sec=WARN_THROTTLE_S,
            )
            return False
        self.get_logger().info("display transport reconnected")
        # Push full state on reconnect.
        self._sent_expression = None
        self._sent_gaze = None
        return True

    def _write_frame(self, frame: bytes) -> bool:
        try:
            self._transport.write(frame)
            return True
        except TransportError as e:
            self.get_logger().warn(
                f"display write failed: {e}",
                throttle_duration_sec=WARN_THROTTLE_S,
            )
            self._sent_expression = None
            self._sent_gaze = None
            return False

    def _send_target(
        self, target: DisplayTarget, now: float, suppress_gaze: bool
    ) -> None:
        refresh = (
            self._last_refresh_t is None
            or now - self._last_refresh_t >= self._refresh_period_s
        )
        if refresh:
            self._last_refresh_t = now
        if refresh or target.expression != self._sent_expression:
            if self._write_frame(set_expression_frame(target.expression)):
                self._sent_expression = target.expression
        if suppress_gaze:
            # A face animation owns the gaze; its own steps resync a
            # rebooted face within one cycle.
            return
        if refresh or target.gaze != self._sent_gaze:
            if self._write_frame(set_gaze_frame(target.gaze)):
                self._sent_gaze = target.gaze

    def _update_face_animation(
        self, name: str | None, now: float
    ) -> FaceAnimation | None:
        if name is None:
            self._pending_face_animation = None
            self._active_face_animation = None
            return None
        if (
            self._active_face_animation is not None
            and self._active_face_animation.name == name
        ):
            return self._active_face_animation
        if self._pending_face_animation != name:
            self._pending_face_animation = name
            self._pending_face_animation_since = now
        delay = (
            self._config.idling_start_delay_s if name == IDLING.name else 0.0
        )
        if now - self._pending_face_animation_since < delay:
            self._active_face_animation = None
            return None
        self._active_face_animation = FACE_ANIMATIONS[name]
        self._face_animation_start_t = now
        self._face_animation_fired = 0
        return self._active_face_animation

    def _run_face_animation(self, animation: FaceAnimation, now: float) -> None:
        steps, self._face_animation_fired = due_steps(
            animation,
            now - self._face_animation_start_t,
            self._face_animation_fired,
        )
        cycle = self._config.idling_expressions
        for step in steps:
            if step.blink:
                self._write_frame(trigger_blink_frame())
            if step.advance_expression and cycle:
                # blink-and-switch: swap the expression mid-blink.
                self._idling_expression_i += 1
                expression = cycle[self._idling_expression_i % len(cycle)]
                if self._write_frame(set_expression_frame(expression)):
                    self._sent_expression = expression
            if step.gaze is not None:
                if self._write_frame(set_gaze_frame(step.gaze)):
                    self._sent_gaze = step.gaze

    def _drain_rx(self) -> None:
        try:
            data = self._transport.read()
        except TransportError:
            return  # write path already logs and schedules reconnect
        if not data:
            return
        frames, self._rx_buf = decode_frames(self._rx_buf + data)
        for frame in frames:
            if frame.cmd == Cmd.NACK:
                reason = "?"
                if len(frame.payload) == 1:
                    try:
                        reason = NackReason(frame.payload[0]).name
                    except ValueError:
                        reason = f"0x{frame.payload[0]:02X}"
                self.get_logger().warn(f"display NACK: {reason}")
            elif frame.cmd == Cmd.LOG:
                text = frame.payload.decode("utf-8", errors="replace")
                self.get_logger().warn(f"display log: {text}")
            else:
                self.get_logger().debug(
                    f"display rx: cmd=0x{frame.cmd:02X} "
                    f"payload={frame.payload.hex()}"
                )

    def _tick(self) -> None:
        now = self._now_s()
        battery_low, battery_critical = False, False
        if self._battery_voltage is not None:
            battery_low, battery_critical = self._battery_monitor.update(
                self._battery_voltage, now
            )
        inputs = PolicyInputs(
            gait_state=self._gait_state,
            vx=self._cmd_vel.linear.x,
            vy=self._cmd_vel.linear.y,
            wz=self._cmd_vel.angular.z,
            animation_mode=self._animation_mode,
            roll=self._body_pose.roll,
            pitch=self._body_pose.pitch,
            yaw=self._body_pose.yaw,
            battery_low=battery_low,
            battery_critical=battery_critical,
        )
        self._last_target = decide(inputs, self._config, self._last_target)
        animation = self._update_face_animation(
            select_face_animation(inputs, self._config), now
        )
        target = self._last_target
        cycle = self._config.idling_expressions
        if animation is not None and animation.name == IDLING.name and cycle:
            target = replace(
                target,
                expression=cycle[self._idling_expression_i % len(cycle)],
            )
        if not self._ensure_transport(now):
            return
        self._send_target(target, now, suppress_gaze=animation is not None)
        if animation is not None:
            self._run_face_animation(animation, now)
        self._drain_rx()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DisplayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
