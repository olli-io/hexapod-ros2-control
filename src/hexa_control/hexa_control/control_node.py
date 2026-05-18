"""Velocity shaping pass-through.

Subscribes to ``/cmd_vel`` (``geometry_msgs/Twist``) from teleop / nav,
shapes the linear and angular components to fit the gait's velocity
envelope (loaded from ``hexa_gait/config/gait.yaml`` — single source of
truth — via ``hexa_gait.load_velocity_caps`` and
``hexa_gait.scale_to_envelope``), and republishes the result as
``GaitParams`` on ``/gait/params`` at 50 Hz. v1 ships a single
``"tripod"`` gait, so ``gait_name`` is hard-coded by config.

Shaping uses a joint scale rather than per-axis clamps. ``omega_z`` is
clamped to ``angular_max`` first; then if the implied per-leg planar
speed (computed from the leg mounts loaded via
``hexa_kinematics.load_leg_specs``) exceeds ``linear_max``, all three
velocity components are scaled by the same factor. This preserves the
commanded translation:yaw ratio, so a stick-fully-forward + stick-yaw
command still turns at the right relative rate instead of being eaten
by the engine's per-leg stride clamp.

The walk-cycle knobs (cycle_time, duty_factor, step_height, stride_length)
live in ``hexa_gait/config/gait.yaml`` and are not on the wire —
cycle_time is derived inside the gait engine each tick from the commanded
velocity and the configured stride_length.

Intentionally thin in v1: no deadband (teleop already applies one), no
acceleration limits, no gait-selection logic.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from hexa_gait import load_velocity_caps, scale_to_envelope
from hexa_interfaces.msg import GaitParams
from hexa_kinematics.leg_specs import load_leg_specs
from rclpy.node import Node


PUBLISH_RATE_HZ = 50.0


@dataclass(frozen=True)
class ControlConfig:
    gait_name: str


def _load_config(path: Path) -> ControlConfig:
    with path.open() as f:
        raw = yaml.safe_load(f)
    return ControlConfig(
        gait_name=str(raw["gait_name"]),
    )


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        share = Path(get_package_share_directory("hexa_control")) / "config"
        gait_yaml = (
            Path(get_package_share_directory("hexa_gait")) / "config" / "gait.yaml"
        )
        geometry_yaml = (
            Path(get_package_share_directory("hexa_description"))
            / "config"
            / "geometry.yaml"
        )
        self._cfg = _load_config(share / "control.yaml")
        self._caps = load_velocity_caps(gait_yaml)
        self._leg_mounts = {
            name: spec.mount_xyz for name, spec in load_leg_specs(geometry_yaml).items()
        }
        self._latest: Twist = Twist()  # zero-initialized

        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_vel, 10)
        self._pub = self.create_publisher(GaitParams, "/gait/params", 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self.get_logger().info(
            f"control_node up: gait={self._cfg.gait_name}, "
            f"caps from {gait_yaml}: "
            f"linear_max={self._caps.linear_max:.2f} m/s, "
            f"angular_z_max={self._caps.angular_max:.2f} rad/s"
        )

    def _on_vel(self, msg: Twist) -> None:
        self._latest = msg

    def _tick(self) -> None:
        v_x, v_y, omega_z = scale_to_envelope(
            self._latest.linear.x,
            self._latest.linear.y,
            self._latest.angular.z,
            self._leg_mounts,
            self._caps,
        )
        out = GaitParams()
        out.header.stamp = self.get_clock().now().to_msg()
        out.gait_name = self._cfg.gait_name
        out.linear_x = v_x
        out.linear_y = v_y
        out.angular_z = omega_z
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
