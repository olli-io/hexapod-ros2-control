"""Posture controller node.

Subscribes to the user pose (`/body/pose`) and the latest body velocity
command (`/cmd_vel`). On a fixed timer, runs the animation stack with
the current context, sums in the user pose, clamps to the static safety
envelope, and publishes the result on `/body/pose_target` for the IK
node to consume.

The animation stack is hard-coded for now (Still + Breathing). Once
config-driven gait loading lands, this should move to a YAML-defined
list of animation specs.
"""

import rclpy
from geometry_msgs.msg import Twist
from hexa_interfaces.msg import BodyPose as BodyPoseMsg
from rclpy.node import Node

from .animations import AnimationContext, Breathing, Stack, Still
from .pose import IDENTITY, BodyPose, PoseLimits, add, clamp

PUBLISH_RATE_HZ = 50.0
CMD_VEL_ZERO_TOL = 1e-4


def _twist_is_zero(t: Twist) -> bool:
    return (
        abs(t.linear.x) < CMD_VEL_ZERO_TOL
        and abs(t.linear.y) < CMD_VEL_ZERO_TOL
        and abs(t.linear.z) < CMD_VEL_ZERO_TOL
        and abs(t.angular.x) < CMD_VEL_ZERO_TOL
        and abs(t.angular.y) < CMD_VEL_ZERO_TOL
        and abs(t.angular.z) < CMD_VEL_ZERO_TOL
    )


def _msg_to_pose(m: BodyPoseMsg) -> BodyPose:
    return BodyPose(x=m.x, y=m.y, z=m.z, roll=m.roll, pitch=m.pitch, yaw=m.yaw)


def _pose_to_msg(p: BodyPose, now_msg) -> BodyPoseMsg:
    out = BodyPoseMsg()
    out.header.stamp = now_msg
    # frame_id intentionally left blank: the pose is an offset in the
    # body frame, not a transform into a named TF frame. Setting a
    # frame_id would invite the wrong consumer assumption.
    out.x = p.x
    out.y = p.y
    out.z = p.z
    out.roll = p.roll
    out.pitch = p.pitch
    out.yaw = p.yaw
    return out


class PostureNode(Node):
    def __init__(self) -> None:
        super().__init__("posture_node")

        self._user_pose: BodyPose = IDENTITY
        self._walking: bool = False

        self._animations = Stack(layers=(Still(), Breathing()))
        self._limits = PoseLimits()

        self._sub_pose = self.create_subscription(
            BodyPoseMsg, "/body/pose", self._on_pose, 10
        )
        self._sub_vel = self.create_subscription(
            Twist, "/cmd_vel", self._on_vel, 10
        )
        self._pub_target = self.create_publisher(BodyPoseMsg, "/body/pose_target", 10)

        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

    def _on_pose(self, msg: BodyPoseMsg) -> None:
        self._user_pose = _msg_to_pose(msg)

    def _on_vel(self, msg: Twist) -> None:
        self._walking = not _twist_is_zero(msg)

    def _tick(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        ctx = AnimationContext(t=t, walking=self._walking)
        animated = self._animations(ctx)
        target = clamp(add(self._user_pose, animated), self._limits)
        self._pub_target.publish(_pose_to_msg(target, now.to_msg()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PostureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
