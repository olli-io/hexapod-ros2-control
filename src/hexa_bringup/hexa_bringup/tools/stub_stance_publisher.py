"""Throw-away stand-in for hexa_gait while the gait engine is still WIP.

Emits the same six nominal-stance foot targets every tick on
``/legs/targets``: phase frozen at 0, all legs in stance. The goal is
to let the IK loop (ik_node → controller → Gazebo) be exercised
end-to-end before the real gait engine lands.

Delete this node — and its launch reference — once hexa_gait's STAND
state can publish the same shape on ``/legs/targets``.
"""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from hexa_interfaces.msg import LegState, LegTargets
from rclpy.node import Node

# Reuse the shared YAML expander so the stub, the IK node, and the URDF
# all derive the six legs from geometry.yaml the same way.
from hexa_kinematics.leg_specs import LEG_NAMES, load_leg_specs


# Foot reach radially outward from the coxa pivot (m) and standoff below
# the body (m). Tuned for the default URDF (coxa 0.04, femur 0.07,
# tibia 0.10, limits ±60°/±60°/±90°): puts the femur near horizontal and
# bends the tibia ~75° — comfortably inside every joint limit.
NOMINAL_RADIAL = 0.13
NOMINAL_Z = -0.10

PUBLISH_RATE_HZ = 50.0


class StubStancePublisher(Node):
    def __init__(self) -> None:
        super().__init__("stub_stance_publisher")

        geometry_yaml = (
            Path(get_package_share_directory("hexa_description"))
            / "config"
            / "geometry.yaml"
        )
        legs = load_leg_specs(geometry_yaml)

        # Pre-compute one body-frame foot target per leg. The leg's local
        # +x rotates into body frame by mount_yaw; the radial standoff
        # walks the foot out from the coxa pivot along that direction.
        self._foot_targets: dict[str, tuple[float, float, float]] = {}
        for name in LEG_NAMES:
            spec = legs[name]
            fx = spec.mount_xyz[0] + NOMINAL_RADIAL * math.cos(spec.mount_yaw)
            fy = spec.mount_xyz[1] + NOMINAL_RADIAL * math.sin(spec.mount_yaw)
            fz = spec.mount_xyz[2] + NOMINAL_Z
            self._foot_targets[name] = (fx, fy, fz)

        self._pub = self.create_publisher(LegTargets, "/legs/targets", 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.get_logger().info(
            "publishing frozen stance on /legs/targets — temporary stand-in for hexa_gait"
        )

    def _tick(self) -> None:
        msg = LegTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        legs: list[LegState] = []
        for name in LEG_NAMES:
            fx, fy, fz = self._foot_targets[name]
            state = LegState()
            state.leg_name = name
            state.foot_target = Point(x=fx, y=fy, z=fz)
            state.phase = 0.0
            state.stance = True
            legs.append(state)
        msg.legs = legs
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StubStancePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
