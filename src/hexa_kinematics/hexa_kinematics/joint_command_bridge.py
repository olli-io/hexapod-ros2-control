"""Adapter: ``sensor_msgs/JointState`` → ``std_msgs/Float64MultiArray``.

The canonical kinematics output is ``/joint_commands`` (``JointState``) —
semantic, self-describing, joint names included. ros2_control position
group controllers (``joint_group_position_controller`` and friends) take
``Float64MultiArray`` in a fixed joint order set by the controller's
YAML config. This bridge sits between the two: it indexes each joint's
latest position by name and emits the fixed-order array.

Holding latest-per-joint values means a partial ``JointState`` update
(e.g. one leg) doesn't zero unrelated joints. Joint order and topics are
parameterised so the same bridge serves sim and any future real-robot
controller that takes ``Float64MultiArray``.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


DEFAULT_JOINT_ORDER: tuple[str, ...] = (
    "l_front_coxa_joint", "l_front_femur_joint", "l_front_tibia_joint",
    "l_middle_coxa_joint", "l_middle_femur_joint", "l_middle_tibia_joint",
    "l_rear_coxa_joint", "l_rear_femur_joint", "l_rear_tibia_joint",
    "r_front_coxa_joint", "r_front_femur_joint", "r_front_tibia_joint",
    "r_middle_coxa_joint", "r_middle_femur_joint", "r_middle_tibia_joint",
    "r_rear_coxa_joint", "r_rear_femur_joint", "r_rear_tibia_joint",
)


class JointCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__("joint_command_bridge")

        self.declare_parameter("input_topic", "/joint_commands")
        self.declare_parameter(
            "output_topic", "/joint_group_position_controller/commands"
        )
        self.declare_parameter("joint_order", list(DEFAULT_JOINT_ORDER))

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._joint_order: tuple[str, ...] = tuple(
            self.get_parameter("joint_order").value
        )
        self._positions: dict[str, float] = {j: 0.0 for j in self._joint_order}

        self._sub = self.create_subscription(JointState, in_topic, self._on_joints, 10)
        self._pub = self.create_publisher(Float64MultiArray, out_topic, 10)

        self.get_logger().info(
            f"bridging {in_topic} (JointState) → {out_topic} "
            f"(Float64MultiArray) for {len(self._joint_order)} joints"
        )

    def _on_joints(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._positions:
                self._positions[name] = pos
        out = Float64MultiArray()
        out.data = [self._positions[j] for j in self._joint_order]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointCommandBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
