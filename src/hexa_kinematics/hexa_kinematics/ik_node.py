"""ROS glue for the hexa_kinematics IK library.

Sits at the convergence of the gait chain (foot targets in the nominal
body frame, published on ``/legs/targets``) and the body-pose chain
(``BodyPose`` offset published on ``/body/pose_target`` by ``hexa_posture``).

Per ``/legs/targets`` tick, for each leg:

1. Apply ``apply_body_pose`` to the foot target — re-express it in the
   pose-offset body frame.
2. ``body_to_leg`` into the leg's coxa-mount frame.
3. ``inverse_kinematics`` to recover (coxa, femur, tibia) joint angles.

Emit an 18-entry ``sensor_msgs/JointState`` on ``/joint_commands`` in
the URDF joint order (see ``JOINT_ORDER`` below).
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from hexa_interfaces.msg import BodyPose as BodyPoseMsg
from hexa_interfaces.msg import LegTargets
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .body_transform import (
    IDENTITY_BODY_POSE,
    BodyPose,
    apply_body_pose,
    body_to_leg,
)
from .leg_ik import UnreachableTarget, inverse_kinematics
from .leg_specs import LEG_NAMES, load_leg_specs

SEGMENTS: tuple[str, ...] = ("coxa", "femur", "tibia")

JOINT_ORDER: tuple[str, ...] = tuple(
    f"{leg}_{seg}_joint" for leg in LEG_NAMES for seg in SEGMENTS
)


def _msg_to_pose(m: BodyPoseMsg) -> BodyPose:
    return BodyPose(x=m.x, y=m.y, z=m.z, roll=m.roll, pitch=m.pitch, yaw=m.yaw)


class IKNode(Node):
    def __init__(self) -> None:
        super().__init__("ik_node")

        geometry_yaml = (
            Path(get_package_share_directory("hexa_description"))
            / "config"
            / "geometry.yaml"
        )
        self._legs = load_leg_specs(geometry_yaml)
        missing = set(LEG_NAMES) - set(self._legs)
        if missing:
            raise RuntimeError(f"geometry.yaml is missing legs: {sorted(missing)}")
        self.get_logger().info(
            f"loaded {len(self._legs)} leg specs from {geometry_yaml}"
        )

        # Latest /body/pose_target. Default to identity so we can produce
        # joint commands the moment /legs/targets starts arriving — even
        # if hexa_posture hasn't published yet.
        self._body_pose: BodyPose = IDENTITY_BODY_POSE

        # Hold the last successful joint angles so a transient
        # UnreachableTarget on one leg doesn't zero unrelated joints
        # (or this leg's own joints).
        self._last_angles: dict[str, float] = {j: 0.0 for j in JOINT_ORDER}

        self._sub_pose = self.create_subscription(
            BodyPoseMsg, "/body/pose_target", self._on_body_pose, 10
        )
        self._sub_legs = self.create_subscription(
            LegTargets, "/legs/targets", self._on_legs, 10
        )
        self._pub_joints = self.create_publisher(JointState, "/joint_commands", 10)

    def _on_body_pose(self, msg: BodyPoseMsg) -> None:
        self._body_pose = _msg_to_pose(msg)

    def _on_legs(self, msg: LegTargets) -> None:
        angles = dict(self._last_angles)
        for leg_state in msg.legs:
            spec = self._legs.get(leg_state.leg_name)
            if spec is None:
                self.get_logger().warn(
                    f"unknown leg_name {leg_state.leg_name!r} — skipping"
                )
                continue
            target_body = (
                leg_state.foot_target.x,
                leg_state.foot_target.y,
                leg_state.foot_target.z,
            )
            target_offset = apply_body_pose(target_body, self._body_pose)
            target_leg = body_to_leg(target_offset, spec)
            try:
                th_c, th_f, th_t = inverse_kinematics(target_leg, spec)
            except UnreachableTarget as exc:
                self.get_logger().warn(
                    f"{leg_state.leg_name}: IK unreachable ({exc}); holding last angles"
                )
                continue
            angles[f"{leg_state.leg_name}_coxa_joint"] = th_c
            angles[f"{leg_state.leg_name}_femur_joint"] = th_f
            angles[f"{leg_state.leg_name}_tibia_joint"] = th_t

        self._last_angles = angles

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(JOINT_ORDER)
        out.position = [angles[j] for j in JOINT_ORDER]
        self._pub_joints.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
