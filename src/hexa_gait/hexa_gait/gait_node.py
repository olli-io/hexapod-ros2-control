"""Gait engine ROS node.

Subscribes to ``/gait/params`` (last-write-wins, no queue replay) and
publishes ``/legs/targets`` at 50 Hz. Builds an ``Engine`` + ``Tripod``
at init using the YAML in ``hexa_description`` (single source of truth
for body geometry and standing pose) and this package's ``config/gait.yaml``
(engine-internal knobs + cold-start fallbacks).

The node is intentionally thin: all gait logic lives in the pure-python
``Engine``. Tests live alongside the engine modules; this file owns
only the ROS plumbing.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from hexa_interfaces.msg import GaitParams, LegState, LegTargets
from rclpy.node import Node

from .clock import LEG_NAMES
from .engine import Engine, EngineConfig, build_leg_contexts, nominal_stance_from_yaml
from .gaits.tripod import Tripod


PUBLISH_RATE_HZ = 50.0


def _load_engine_config(path: Path) -> EngineConfig:
    with path.open() as f:
        raw = yaml.safe_load(f)
    return EngineConfig(
        cycle_time=float(raw["cycle_time"]),
        duty_factor=float(raw["duty_factor"]),
        step_height=float(raw["step_height"]),
        swing_width=float(raw["swing_width"]),
        controller_dt=float(raw["controller_dt"]),
        force_touchdown_speed=float(raw["force_touchdown_speed"]),
        recenter_swing_time=float(raw["recenter_swing_time"]),
        recenter_order=tuple(raw["recenter_order"]),
        cmd_zero_tol=float(raw["cmd_zero_tol"]),
    )


class GaitNode(Node):
    def __init__(self) -> None:
        super().__init__("gait_node")

        gait_share = Path(get_package_share_directory("hexa_gait")) / "config"
        desc_share = Path(get_package_share_directory("hexa_description")) / "config"

        self._cfg = _load_engine_config(gait_share / "gait.yaml")
        nominal = nominal_stance_from_yaml(
            desc_share / "geometry.yaml", desc_share / "standing_pose.yaml"
        )
        leg_contexts = build_leg_contexts(
            desc_share / "geometry.yaml", desc_share / "standing_pose.yaml"
        )
        self._engine = Engine(
            config=self._cfg,
            strategy=Tripod(),
            nominal_stance=nominal,
            leg_contexts=leg_contexts,
        )

        # Latest GaitParams, falling back to engine defaults until the
        # first message arrives.
        self._gait_name: str = "tripod"
        self._linear_x: float = 0.0
        self._linear_y: float = 0.0
        self._angular_z: float = 0.0
        self._cycle_time: float = self._cfg.cycle_time
        self._duty_factor: float = self._cfg.duty_factor
        self._step_height: float = self._cfg.step_height

        self._sub_params = self.create_subscription(
            GaitParams, "/gait/params", self._on_params, 10
        )
        self._pub_targets = self.create_publisher(LegTargets, "/legs/targets", 10)

        self._last_tick_ns: int | None = None
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self.get_logger().info(
            f"gait_node up: strategy=tripod, fallback cycle_time={self._cfg.cycle_time:.2f}s, "
            f"duty_factor={self._cfg.duty_factor:.2f}, step_height={self._cfg.step_height:.3f} m"
        )

    def _on_params(self, msg: GaitParams) -> None:
        if msg.gait_name and msg.gait_name != "tripod":
            # v1 only ships tripod; ignore but log so future gait
            # selection lands cleanly.
            self.get_logger().warn(
                f"GaitParams.gait_name={msg.gait_name!r} is unsupported; staying on tripod"
            )
        self._gait_name = "tripod"
        self._linear_x = float(msg.linear_x)
        self._linear_y = float(msg.linear_y)
        self._angular_z = float(msg.angular_z)
        # Reject non-positive durations / out-of-range duty factor; keep
        # the previous value rather than crash the timer.
        if msg.cycle_time > 0.0:
            self._cycle_time = float(msg.cycle_time)
        if 0.0 < msg.duty_factor < 1.0:
            self._duty_factor = float(msg.duty_factor)
        if msg.step_height >= 0.0:
            self._step_height = float(msg.step_height)

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_tick_ns is None:
            dt = 1.0 / PUBLISH_RATE_HZ
        else:
            dt = (now_ns - self._last_tick_ns) * 1e-9
            if dt <= 0.0:
                # /clock can rewind during sim resets; skip this tick.
                self._last_tick_ns = now_ns
                return
        self._last_tick_ns = now_ns

        out = self._engine.update(
            dt=dt,
            v_body_xy=(self._linear_x, self._linear_y),
            omega_z=self._angular_z,
            cycle_time=self._cycle_time,
            duty_factor=self._duty_factor,
            step_height=self._step_height,
        )

        msg = LegTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        leg_states: list[LegState] = []
        for name in LEG_NAMES:
            leg = out[name]
            state = LegState()
            state.leg_name = name
            state.foot_target = Point(
                x=leg.foot_target[0],
                y=leg.foot_target[1],
                z=leg.foot_target[2],
            )
            state.phase = leg.phase
            state.stance = leg.stance
            leg_states.append(state)
        msg.legs = leg_states
        self._pub_targets.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
