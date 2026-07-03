"""Velocity shaping pass-through.

Subscribes to ``/cmd_vel``, runs it through ``scale_to_envelope`` and
the ``BodyVelocityLimiter`` rate-cap slew, and republishes as
``GaitParams`` on ``/gait/params`` at 200 Hz. ``/cmd_gait`` multiplexes
the active gait name (validated against the gait catalog); on every
gait switch the limiter's ``accel_linear`` is recomputed from
``linear_max(gait) / vmax_ramp_time_linear`` so the ramp time stays
constant across gaits despite the per-gait velocity ceiling.
The limiter resets to zero on edges leaving the walking set
(``{engaging, gait}``) so each STAND → ENGAGING starts clean.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from hexa_common import (
    load_velocity_caps,
    scale_to_envelope,
)
from hexa_common.gait_catalog import GAIT_DESCRIPTORS
from hexa_interfaces.msg import GaitParams
from hexa_kinematics.leg_specs import load_leg_specs
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from hexa_control.body_velocity_limiter import BodyVelocityLimiter


PUBLISH_RATE_HZ = 200.0

# Engine states in which cmd_vel is actively driving the body.
_WALKING_STATES: frozenset[str] = frozenset({"engaging", "gait"})


@dataclass(frozen=True)
class ControlConfig:
    default_gait: str
    vmax_ramp_time_linear: float
    vmax_ramp_time_angular: float
    snap_tol_linear: float
    snap_tol_angular: float


def _read_control_config(node: Node) -> ControlConfig:
    """Declare and read ``control_node``'s ros params into a ``ControlConfig``.

    Defaults mirror hexa_description's ``config/tuning.yaml`` (the
    ``control_node`` block); the launch files pass that file so the YAML is
    authoritative in the normal composed run, while a bare ``ros2 run`` still
    starts on the built-in defaults.
    """
    node.declare_parameter("default_gait", "tripod")
    node.declare_parameter("vmax_ramp_time_linear", 0.8)
    node.declare_parameter("vmax_ramp_time_angular", 1.0)
    node.declare_parameter("snap_tol_linear", 1.0e-4)
    node.declare_parameter("snap_tol_angular", 1.0e-4)

    name = str(node.get_parameter("default_gait").value)
    if name not in GAIT_DESCRIPTORS:
        raise ValueError(
            f"default_gait={name!r} not in the gait catalog "
            f"({sorted(GAIT_DESCRIPTORS)})"
        )
    vmax_ramp_time_linear = float(node.get_parameter("vmax_ramp_time_linear").value)
    vmax_ramp_time_angular = float(node.get_parameter("vmax_ramp_time_angular").value)
    if vmax_ramp_time_linear <= 0.0:
        raise ValueError(
            f"vmax_ramp_time_linear must be positive, got {vmax_ramp_time_linear}"
        )
    if vmax_ramp_time_angular <= 0.0:
        raise ValueError(
            f"vmax_ramp_time_angular must be positive, got {vmax_ramp_time_angular}"
        )
    snap_tol_linear = float(node.get_parameter("snap_tol_linear").value)
    snap_tol_angular = float(node.get_parameter("snap_tol_angular").value)
    return ControlConfig(
        default_gait=name,
        vmax_ramp_time_linear=vmax_ramp_time_linear,
        vmax_ramp_time_angular=vmax_ramp_time_angular,
        snap_tol_linear=snap_tol_linear,
        snap_tol_angular=snap_tol_angular,
    )


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        # Velocity caps derive from the gait_node block of hexa_description's
        # tuning.yaml — the single source of truth the gait node also reads.
        tuning_yaml = (
            Path(get_package_share_directory("hexa_description"))
            / "config"
            / "tuning.yaml"
        )
        geometry_yaml = (
            Path(get_package_share_directory("hexa_description"))
            / "config"
            / "geometry.yaml"
        )
        self._cfg = _read_control_config(self)
        self._caps = load_velocity_caps(tuning_yaml)
        self._leg_mounts = {
            name: spec.mount_xyz for name, spec in load_leg_specs(geometry_yaml).items()
        }
        self._latest: Twist = Twist()  # zero-initialized
        self._active_gait: str = self._cfg.default_gait
        self._limiter = BodyVelocityLimiter(
            accel_linear=self._accel_linear_for(self._active_gait),
            accel_angular=self._accel_angular(),
            snap_tol_linear=self._cfg.snap_tol_linear,
            snap_tol_angular=self._cfg.snap_tol_angular,
        )
        self._engine_state: str = ""
        self._dt = 1.0 / PUBLISH_RATE_HZ

        # Transient-local so a late subscriber catches the last name.
        gait_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_vel, 10)
        self._sub_gait = self.create_subscription(
            String, "/cmd_gait", self._on_gait, gait_qos
        )
        self._sub_state = self.create_subscription(
            String, "/gait/state", self._on_state, 10
        )
        self._pub = self.create_publisher(GaitParams, "/gait/params", 10)
        self._timer = self.create_timer(self._dt, self._tick)

        cap_summary = ", ".join(
            f"{n}={v:.2f}" for n, v in sorted(self._caps.linear_max_by_gait.items())
        )
        bias_summary = ", ".join(
            f"{n}={v:.2f}" for n, v in sorted(self._caps.yaw_bias_by_gait.items())
        )
        self.get_logger().info(
            f"control_node up: default_gait={self._cfg.default_gait}, "
            f"caps from {tuning_yaml}: "
            f"linear_max=({cap_summary}) m/s, "
            f"angular_z_max={self._caps.angular_max:.2f} rad/s, "
            f"yaw_bias=({bias_summary}), "
            f"vmax_ramp_time_linear={self._cfg.vmax_ramp_time_linear:.2f} s, "
            f"vmax_ramp_time_angular={self._cfg.vmax_ramp_time_angular:.2f} s, "
            f"accel_linear[{self._active_gait}]="
            f"{self._limiter.accel_linear:.3f} m/s^2, "
            f"accel_angular={self._limiter.accel_angular:.3f} rad/s^2"
        )

    def _accel_linear_for(self, gait: str) -> float:
        return self._caps.linear_max(gait) / self._cfg.vmax_ramp_time_linear

    def _accel_angular(self) -> float:
        return self._caps.angular_max / self._cfg.vmax_ramp_time_angular

    def _on_vel(self, msg: Twist) -> None:
        self._latest = msg

    def _on_gait(self, msg: String) -> None:
        name = msg.data
        if name not in GAIT_DESCRIPTORS:
            self.get_logger().warn(
                f"/cmd_gait={name!r} is not a known gait "
                f"({sorted(GAIT_DESCRIPTORS)}); dropping"
            )
            return
        if name == self._active_gait:
            return
        self._active_gait = name
        new_accel = self._accel_linear_for(name)
        self._limiter.accel_linear = new_accel
        self.get_logger().info(
            f"/cmd_gait switching active gait to {name!r} "
            f"(accel_linear={new_accel:.3f} m/s^2)"
        )

    def _on_state(self, msg: String) -> None:
        new_state = msg.data
        if new_state == self._engine_state:
            return
        was_walking = self._engine_state in _WALKING_STATES
        now_walking = new_state in _WALKING_STATES
        if was_walking and not now_walking:
            self._limiter.reset((0.0, 0.0, 0.0))
        self._engine_state = new_state

    def _tick(self) -> None:
        v_x, v_y, omega_z = scale_to_envelope(
            self._latest.linear.x,
            self._latest.linear.y,
            self._latest.angular.z,
            self._leg_mounts,
            self._caps.linear_max(self._active_gait),
            self._caps.angular_max,
            self._caps.yaw_bias(self._active_gait),
        )
        v_x, v_y, omega_z = self._limiter.step((v_x, v_y, omega_z), self._dt)
        out = GaitParams()
        out.header.stamp = self.get_clock().now().to_msg()
        out.gait_name = self._active_gait
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
