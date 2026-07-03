"""Launch ``gait_node`` standalone.

Sim composition (``hexa_bringup/launch/sim.launch.py``) starts
``gait_node`` directly with its own ``use_sim_time: True`` parameter
block, bypassing this launch file. This launcher is for bench / unit
work where the engine runs on the host clock.

Run with::

    ros2 launch hexa_gait gait.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gait_config = PathJoinSubstitution([
        FindPackageShare("hexa_description"), "config", "tuning.yaml",
    ])
    gait_node = Node(
        package="hexa_gait",
        executable="gait_node",
        output="screen",
        parameters=[{"use_sim_time": False}, gait_config],
    )
    return LaunchDescription([gait_node])
