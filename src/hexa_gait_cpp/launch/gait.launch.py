"""Standalone bench launcher for the C++ gait_node.

Mirrors hexa_gait/launch/gait.launch.py. Production sim composition lives in
hexa_bringup; this file is for running the node in isolation on a bench.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gait_config = PathJoinSubstitution([
        FindPackageShare("hexa_description"), "config", "tuning.yaml",
    ])
    return LaunchDescription(
        [
            Node(
                package="hexa_gait_cpp",
                executable="gait_node",
                output="screen",
                parameters=[{"use_sim_time": False}, gait_config],
            ),
        ]
    )
