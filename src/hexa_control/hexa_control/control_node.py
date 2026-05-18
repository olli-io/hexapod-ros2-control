"""Velocity shaping pass-through.

Subscribes to ``/cmd_vel`` (``geometry_msgs/Twist``) from teleop / nav,
clamps the linear and angular components against the YAML-configured
speed caps, and republishes the result as ``GaitParams`` on
``/gait/params`` at 50 Hz. The gait/duty/step knobs come from
``config/control.yaml``; v1 ships a single ``"tripod"`` gait, so
``gait_name`` is hard-coded by config.

Intentionally thin in v1: no deadband (teleop already applies one), no
acceleration limits, no gait-selection logic. The gait engine has its
own fallback values; the wire values from this node win whenever it is
running.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from hexa_interfaces.msg import GaitParams
from rclpy.node import Node


PUBLISH_RATE_HZ = 50.0


@dataclass(frozen=True)
class ControlConfig:
    gait_name: str
    cycle_time: float
    duty_factor: float
    step_height: float
    linear_x_max: float
    linear_y_max: float
    angular_z_max: float


def _load_config(path: Path) -> ControlConfig:
    with path.open() as f:
        raw = yaml.safe_load(f)
    return ControlConfig(
        gait_name=str(raw["gait_name"]),
        cycle_time=float(raw["cycle_time"]),
        duty_factor=float(raw["duty_factor"]),
        step_height=float(raw["step_height"]),
        linear_x_max=float(raw["linear_x_max"]),
        linear_y_max=float(raw["linear_y_max"]),
        angular_z_max=float(raw["angular_z_max"]),
    )


def _clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        share = Path(get_package_share_directory("hexa_control")) / "config"
        self._cfg = _load_config(share / "control.yaml")
        self._latest: Twist = Twist()  # zero-initialized

        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_vel, 10)
        self._pub = self.create_publisher(GaitParams, "/gait/params", 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self.get_logger().info(
            f"control_node up: gait={self._cfg.gait_name}, "
            f"linear=({self._cfg.linear_x_max:.2f}, {self._cfg.linear_y_max:.2f}) m/s, "
            f"angular={self._cfg.angular_z_max:.2f} rad/s"
        )

    def _on_vel(self, msg: Twist) -> None:
        self._latest = msg

    def _tick(self) -> None:
        out = GaitParams()
        out.header.stamp = self.get_clock().now().to_msg()
        out.gait_name = self._cfg.gait_name
        out.linear_x = _clamp(self._latest.linear.x, self._cfg.linear_x_max)
        out.linear_y = _clamp(self._latest.linear.y, self._cfg.linear_y_max)
        out.angular_z = _clamp(self._latest.angular.z, self._cfg.angular_z_max)
        out.cycle_time = self._cfg.cycle_time
        out.duty_factor = self._cfg.duty_factor
        out.step_height = self._cfg.step_height
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
