"""Velocity shaping pass-through.

Subscribes to ``/cmd_vel``, runs it through ``scale_to_envelope`` and
the ``BodyVelocityLimiter`` rate-cap slew, and republishes as
``GaitParams`` on ``/gait/params`` at 50 Hz. ``/cmd_gait`` multiplexes
the active gait name (validated against tripod/ripple/wave); on every
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
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from hexa_gait import load_velocity_caps, scale_to_envelope
from hexa_gait.gaits import STRATEGIES
from hexa_interfaces.msg import GaitParams
from hexa_kinematics.leg_specs import load_leg_specs
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from hexa_control.body_velocity_limiter import BodyVelocityLimiter


PUBLISH_RATE_HZ = 50.0

# Engine states in which cmd_vel is actively driving the body.
_WALKING_STATES: frozenset[str] = frozenset({"engaging", "gait"})


@dataclass(frozen=True)
class ControlConfig:
    default_gait: str
    vmax_ramp_time_linear: float
    vmax_ramp_time_angular: float
    snap_tol_linear: float
    snap_tol_angular: float


def _load_config(path: Path) -> ControlConfig:
    with path.open() as f:
        raw = yaml.safe_load(f)
    name = str(raw["default_gait"])
    if name not in STRATEGIES:
        raise ValueError(
            f"default_gait={name!r} not in STRATEGIES "
            f"({sorted(STRATEGIES)})"
        )
    vmax_ramp_time_linear = float(raw["vmax_ramp_time_linear"])
    vmax_ramp_time_angular = float(raw["vmax_ramp_time_angular"])
    if vmax_ramp_time_linear <= 0.0:
        raise ValueError(
            f"vmax_ramp_time_linear must be positive, got {vmax_ramp_time_linear}"
        )
    if vmax_ramp_time_angular <= 0.0:
        raise ValueError(
            f"vmax_ramp_time_angular must be positive, got {vmax_ramp_time_angular}"
        )
    snap_tol_linear = float(raw.get("snap_tol_linear", 1.0e-4))
    snap_tol_angular = float(raw.get("snap_tol_angular", 1.0e-4))
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
            f"caps from {gait_yaml}: "
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
        if name not in STRATEGIES:
            self.get_logger().warn(
                f"/cmd_gait={name!r} is not a known strategy "
                f"({sorted(STRATEGIES)}); dropping"
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
