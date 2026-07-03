"""Standalone bench launcher for the C++ posture_node.

Mirrors hexa_gait_cpp/launch/gait.launch.py. Production sim composition lives in
hexa_bringup (which still launches the Python posture_node until the cutover);
this file is for running the C++ node in isolation on a bench, loading
hexa_description's tuning.yaml so the animation stack matches production.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("hexa_description"), "config", "tuning.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="hexa_posture_cpp",
                executable="posture_node",
                output="screen",
                parameters=[config, {"use_sim_time": False}],
            ),
        ]
    )
