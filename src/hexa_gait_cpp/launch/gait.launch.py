"""Standalone bench launcher for the C++ gait_node.

Mirrors hexa_gait/launch/gait.launch.py. Production sim composition lives in
hexa_bringup; this file is for running the node in isolation on a bench.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="hexa_gait_cpp",
                executable="gait_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
