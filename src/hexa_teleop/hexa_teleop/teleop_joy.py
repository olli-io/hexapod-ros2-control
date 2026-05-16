"""ROS glue for the joystick teleop.

Reads ``sensor_msgs/Joy`` from ``/joy``, maps it via the pure
``joy_mapping`` library, and publishes:

* ``/cmd_vel`` (``geometry_msgs/Twist``) — body velocity for the gait
  chain. ``hexa_gait`` is not yet implemented, but ``hexa_posture``
  listens to ``/cmd_vel`` to switch animation state between idle and
  walking, so the topic is meaningful today.
* ``/body/pose`` (``hexa_interfaces/BodyPose``) — body-translation
  offset for the posture chain.

Both topics publish on a fixed timer; the inactive channel is
zero-filled so consumers always see a coherent command.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from hexa_interfaces.msg import BodyPose as BodyPoseMsg
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .joy_mapping import GAIT, POSTURE, JoyConfig, JoyState, map_joy

PUBLISH_RATE_HZ = 50.0


def _load_config(path: Path) -> tuple[JoyConfig, str]:
    with path.open() as f:
        raw = yaml.safe_load(f)
    cfg = JoyConfig(
        axis_left_x=int(raw["axis"]["left_x"]),
        axis_left_y=int(raw["axis"]["left_y"]),
        axis_right_x=int(raw["axis"]["right_x"]),
        axis_right_y=int(raw["axis"]["right_y"]),
        mode_toggle_button=int(raw["mode_toggle_button"]),
        deadband=float(raw["deadband"]),
        gait_linear_x_max=float(raw["gait"]["linear_x_max"]),
        gait_linear_y_max=float(raw["gait"]["linear_y_max"]),
        gait_angular_z_max=float(raw["gait"]["angular_z_max"]),
        posture_x_max=float(raw["posture"]["x_max"]),
        posture_y_max=float(raw["posture"]["y_max"]),
    )
    initial_mode = str(raw.get("initial_mode", POSTURE))
    if initial_mode not in (POSTURE, GAIT):
        raise ValueError(
            f"initial_mode must be {POSTURE!r} or {GAIT!r}, got {initial_mode!r}"
        )
    return cfg, initial_mode


class TeleopJoyNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_joy")

        default_cfg_path = (
            Path(get_package_share_directory("hexa_teleop"))
            / "config"
            / "teleop_joy.yaml"
        )
        self.declare_parameter("config_file", str(default_cfg_path))
        cfg_path = Path(
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        self._cfg, initial_mode = _load_config(cfg_path)
        self._state = JoyState(mode=initial_mode, prev_toggle=False)

        self.get_logger().info(f"loaded teleop config from {cfg_path}")
        self.get_logger().info(f"mode={self._state.mode}")

        self._latest_axes: tuple[float, ...] = ()
        self._latest_buttons: tuple[int, ...] = ()

        self._sub_joy = self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self._pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_body_pose = self.create_publisher(BodyPoseMsg, "/body/pose", 10)

        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

    def _on_joy(self, msg: Joy) -> None:
        self._latest_axes = tuple(msg.axes)
        self._latest_buttons = tuple(msg.buttons)

    def _tick(self) -> None:
        out = map_joy(
            self._latest_axes, self._latest_buttons, self._cfg, self._state
        )
        if out.mode_changed:
            self.get_logger().info(f"mode={self._state.mode}")

        stamp = self.get_clock().now().to_msg()

        twist = Twist()
        twist.linear.x = out.linear_x
        twist.linear.y = out.linear_y
        twist.angular.z = out.angular_z
        self._pub_cmd_vel.publish(twist)

        pose = BodyPoseMsg()
        pose.header.stamp = stamp
        pose.x = out.pose_x
        pose.y = out.pose_y
        self._pub_body_pose.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopJoyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
