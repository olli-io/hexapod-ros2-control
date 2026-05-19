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
from hexa_interfaces.msg import BodyPose as BodyPoseMsg
from hexa_interfaces.msg import GaitParams, LegState, LegTargets
from hexa_kinematics.leg_specs import load_leg_specs
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .clock import LEG_NAMES
from .engine import (
    Engine,
    EngineConfig,
    build_leg_contexts,
    initial_stance_from_yaml,
    nominal_stance_from_yaml,
    reseat_geometry_from_yaml,
)
from .gaits.tripod import Tripod


PUBLISH_RATE_HZ = 50.0


def _load_engine_config(path: Path) -> EngineConfig:
    with path.open() as f:
        raw = yaml.safe_load(f)
    init_cfg = raw["initialize"]
    reseat_cfg = raw["reseat"]
    return EngineConfig(
        stride_length=float(raw["stride_length"]),
        min_cycle_time=float(raw["min_cycle_time"]),
        max_cycle_time=float(raw["max_cycle_time"]),
        duty_factor=float(raw["duty_factor"]),
        step_height=float(raw["step_height"]),
        swing_width=float(raw["swing_width"]),
        controller_dt=float(raw["controller_dt"]),
        recenter_swing_time=float(raw["recenter_swing_time"]),
        cmd_zero_tol=float(raw["cmd_zero_tol"]),
        forced_touchdown_delay=float(raw["forced_touchdown_delay"]),
        touchdown_settle_time=float(raw["touchdown_settle_time"]),
        init_pair_swing_time=float(init_cfg["pair_swing_time"]),
        init_lift_body_time=float(init_cfg["lift_body_time"]),
        init_swing_clearance=float(init_cfg["swing_clearance"]),
        init_place_feet_clearance=float(init_cfg["place_feet_clearance"]),
        reseat_settle_delay=float(reseat_cfg["settle_delay"]),
        reseat_height_change_threshold=float(reseat_cfg["height_change_threshold"]),
        reseat_pair_swing_time=float(reseat_cfg["pair_swing_time"]),
        reseat_swing_clearance=float(reseat_cfg["swing_clearance"]),
    )


def _load_coxa_to_bottom(geometry_path: Path) -> float:
    with geometry_path.open() as f:
        raw = yaml.safe_load(f)
    return float(raw["body"]["coxa_to_bottom"])


class GaitNode(Node):
    def __init__(self) -> None:
        super().__init__("gait_node")

        gait_share = Path(get_package_share_directory("hexa_gait")) / "config"
        desc_share = Path(get_package_share_directory("hexa_description")) / "config"

        self._cfg = _load_engine_config(gait_share / "gait.yaml")
        nominal = nominal_stance_from_yaml(
            desc_share / "geometry.yaml", desc_share / "standing_pose.yaml"
        )
        initial = initial_stance_from_yaml(desc_share / "geometry.yaml")
        coxa_to_bottom = _load_coxa_to_bottom(desc_share / "geometry.yaml")
        leg_contexts = build_leg_contexts(
            desc_share / "geometry.yaml", desc_share / "standing_pose.yaml"
        )
        leg_specs = load_leg_specs(desc_share / "geometry.yaml")
        reseat_geometry = reseat_geometry_from_yaml(
            desc_share / "geometry.yaml", desc_share / "standing_pose.yaml"
        )
        self._engine = Engine(
            config=self._cfg,
            strategy=Tripod(),
            nominal_stance=nominal,
            initial_stance=initial,
            coxa_to_bottom=coxa_to_bottom,
            leg_contexts=leg_contexts,
            leg_specs=leg_specs,
            reseat_geometry=reseat_geometry,
        )

        # Latest GaitParams. Walk-cycle knobs are no longer on the wire;
        # only the commanded velocity arrives via /gait/params.
        self._gait_name: str = "tripod"
        self._linear_x: float = 0.0
        self._linear_y: float = 0.0
        self._angular_z: float = 0.0

        self._sub_params = self.create_subscription(
            GaitParams, "/gait/params", self._on_params, 10
        )
        # Operator-gated FOLDED ↔ STAND toggle. The engine starts in
        # FOLDED and emits the initial_pose foot positions until this
        # message arrives. Once standing, a second press triggers the
        # symmetric warm-shutdown (STAND → FOLDING → FOLDED). The start
        # button on the joystick is the canonical source, but anything
        # publishing here counts. Stray presses outside FOLDED / STAND
        # are no-ops.
        self._sub_init = self.create_subscription(
            Empty, "/gait/initialize", self._on_init, 10
        )
        # Sniff /body/pose for the height axis only. The other fields
        # belong to the kinematics / posture chain, not the gait. The
        # engine watches the height for a settle window and runs its
        # reseat ladder when the user lets the chassis come to rest
        # at a new height — see Engine.set_target_height.
        self._sub_body_pose = self.create_subscription(
            BodyPoseMsg, "/body/pose", self._on_body_pose, 10
        )
        self._pub_targets = self.create_publisher(LegTargets, "/legs/targets", 10)
        # State broadcast for the posture chain: `hexa_posture` gates body
        # pose application on this so the user can't push the chassis
        # around while the legs are folded or mid-INITIALIZE / mid-FOLD.
        # Published every tick so late subscribers converge within one
        # 50 Hz interval — cheaper than configuring transient_local QoS
        # on both ends.
        self._pub_state = self.create_publisher(String, "/gait/state", 10)

        self._last_tick_ns: int | None = None
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self.get_logger().info(
            f"gait_node up: strategy=tripod, stride_length={self._cfg.stride_length:.3f} m, "
            f"cycle_time in [{self._cfg.min_cycle_time:.2f}, {self._cfg.max_cycle_time:.2f}] s, "
            f"duty_factor={self._cfg.duty_factor:.2f}, step_height={self._cfg.step_height:.3f} m"
        )

    def _on_init(self, _msg: Empty) -> None:
        if self._engine.start_initialize():
            self.get_logger().info("start-button trigger received: FOLDED → INITIALIZE")
            return
        # Anywhere other than FOLDED: queue a fold request. The engine
        # consumes the flag the next time it sits in STAND at zero
        # height, so a press while RESEATING / STAND folds cleanly
        # once the legs are settled. A stray press during GAIT /
        # STOPPING is harmless — the request stays latched until the
        # engine returns to STAND.
        if self._engine.request_fold():
            self.get_logger().info(
                f"start-button trigger received: fold requested (engine in {self._engine.state.name})"
            )
            return
        self.get_logger().info(
            f"start-button trigger ignored: engine in state {self._engine.state.name}"
        )

    def _on_body_pose(self, msg: BodyPoseMsg) -> None:
        self._engine.set_target_height(float(msg.z))

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
        self._pub_state.publish(String(data=self._engine.state.value))


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
