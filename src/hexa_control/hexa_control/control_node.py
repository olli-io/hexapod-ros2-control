"""Velocity shaping pass-through.

Subscribes to ``/cmd_vel`` (``geometry_msgs/Twist``) from teleop / nav,
shapes the linear and angular components to fit the gait's velocity
envelope (loaded from ``hexa_gait/config/gait.yaml`` — single source of
truth — via ``hexa_gait.load_velocity_caps`` and
``hexa_gait.scale_to_envelope``), applies a body-velocity rate limit
(``BodyVelocityLimiter``), and republishes the result as ``GaitParams``
on ``/gait/params`` at 50 Hz.

Also subscribes to ``/cmd_gait`` (``std_msgs/String``, ``transient_local``
durability so a late-starting control node still receives the latest
selection) and multiplexes the chosen gait onto
``GaitParams.gait_name``. Validates against the known strategy set
(tripod / ripple / wave); unknown names are warned and dropped.

Shaping cuts the velocity triple in one pass. ``omega_z`` is clamped to
``angular_max`` first; then if the implied per-leg planar speed
(computed from the leg mounts loaded via
``hexa_kinematics.load_leg_specs``) exceeds ``linear_max``, the cut is
split between translation and yaw in ratio ``yaw_bias : (1 − yaw_bias)``
— translation absorbs the larger fraction, so a stick-fully-forward +
stick-yaw command keeps more of the commanded yaw at the extremes
instead of being eaten by the engine's per-leg stride clamp. The
trade-off vs uniform scaling is that the commanded translation:yaw
ratio is not preserved when the cut kicks in.

The rate limiter runs *after* ``scale_to_envelope`` and bounds the
per-tick change in ``(v_x, v_y, omega_z)``. Without it, releasing one
axis (e.g. yaw) makes ``scale_to_envelope`` stop suppressing the
others, and the published velocity steps in a single tick. The limiter
absorbs that step into a ``max_*_accel``-bounded ramp. It also smooths
gait-cap changes on ``/cmd_gait`` and Nav2 stops. Subscribed to
``/gait/state``; resets to zero on every transition out of the active
walking set ({``engaging``, ``gait``}) so a fresh ``STAND → ENGAGING``
cycle always starts the limiter from zero (no fight with the
engagement controller's smoothstep). The limiter is not a hard safety
limit: during ramp-down it allows brief excursions outside the static
``scale_to_envelope`` window — see the package README.

The walk-cycle knobs (cycle_time, duty_factor, step_height, stride_length)
live in ``hexa_gait/config/gait.yaml`` and are not on the wire —
cycle_time is derived inside the gait engine each tick from the commanded
velocity and the configured stride_length.
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

# Engine states (published as strings on /gait/state) during which the
# engine is actively driving the body from cmd_vel. Outside this set
# the limiter resets to zero so a future STAND→ENGAGING begins fresh.
_WALKING_STATES: frozenset[str] = frozenset({"engaging", "gait"})


@dataclass(frozen=True)
class ControlConfig:
    default_gait: str
    max_linear_accel: float
    max_angular_accel: float


def _load_config(path: Path) -> ControlConfig:
    with path.open() as f:
        raw = yaml.safe_load(f)
    name = str(raw["default_gait"])
    if name not in STRATEGIES:
        raise ValueError(
            f"default_gait={name!r} not in STRATEGIES "
            f"({sorted(STRATEGIES)})"
        )
    max_linear_accel = float(raw["max_linear_accel"])
    max_angular_accel = float(raw["max_angular_accel"])
    return ControlConfig(
        default_gait=name,
        max_linear_accel=max_linear_accel,
        max_angular_accel=max_angular_accel,
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
            max_linear_accel=self._cfg.max_linear_accel,
            max_angular_accel=self._cfg.max_angular_accel,
        )
        self._engine_state: str = ""
        self._dt = 1.0 / PUBLISH_RATE_HZ

        # Transient-local on both sides so a late subscriber catches
        # the last published name without the publisher needing to keep
        # re-sending it. Single-slot history is enough — the value
        # changes only on a user press, never at high rate.
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
        self.get_logger().info(
            f"control_node up: default_gait={self._cfg.default_gait}, "
            f"caps from {gait_yaml}: "
            f"linear_max=({cap_summary}) m/s, "
            f"angular_z_max={self._caps.angular_max:.2f} rad/s, "
            f"yaw_bias={self._caps.yaw_bias:.2f}, "
            f"max_linear_accel={self._cfg.max_linear_accel:.2f} m/s^2, "
            f"max_angular_accel={self._cfg.max_angular_accel:.2f} rad/s^2"
        )

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
        self.get_logger().info(f"/cmd_gait switching active gait to {name!r}")
        self._active_gait = name

    def _on_state(self, msg: String) -> None:
        new_state = msg.data
        if new_state == self._engine_state:
            return
        # Edge: leaving the walking set. Drop the limiter to zero so the
        # next STAND → ENGAGING begins from a known-zero state and
        # cannot race the engagement controller's smoothstep.
        was_walking = self._engine_state in _WALKING_STATES
        now_walking = new_state in _WALKING_STATES
        if was_walking and not now_walking:
            self._limiter.reset((0.0, 0.0, 0.0))
        self._engine_state = new_state

    def _tick(self) -> None:
        # Per-tick cap lookup: the active gait can change between ticks
        # when /cmd_gait arrives, and each gait has its own linear cap
        # (slower gaits would otherwise push the engagement controller's
        # stance integration past PEP and hit joint limits — the cap is
        # the gait's actual per-leg velocity ceiling).
        v_x, v_y, omega_z = scale_to_envelope(
            self._latest.linear.x,
            self._latest.linear.y,
            self._latest.angular.z,
            self._leg_mounts,
            self._caps.linear_max(self._active_gait),
            self._caps.angular_max,
            self._caps.yaw_bias,
        )
        # Apply body-velocity rate limit after the envelope shaper so
        # the limiter smooths envelope-unwind jumps (the whole reason
        # it exists). The limiter is stateful across ticks.
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
