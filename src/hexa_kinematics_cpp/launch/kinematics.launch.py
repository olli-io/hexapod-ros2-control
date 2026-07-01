"""Standalone bench launcher for the C++ kinematics nodes.

Brings up the IK node and the joint-command bridge in isolation. Production sim
composition lives in hexa_bringup; this file is for running the nodes on a
bench. Mirrors how hexa_gait_cpp/launch/gait.launch.py exposes its node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="hexa_kinematics_cpp",
                executable="ik_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="hexa_kinematics_cpp",
                executable="joint_command_bridge",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
