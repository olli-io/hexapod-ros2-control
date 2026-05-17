"""Launch ``gait_node`` standalone.

Sim composition (``hexa_bringup/launch/sim.launch.py``) starts
``gait_node`` directly with its own ``use_sim_time: True`` parameter
block, bypassing this launch file. This launcher is for bench / unit
work where the engine runs on the host clock.

Run with::

    ros2 launch hexa_gait gait.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gait_node = Node(
        package="hexa_gait",
        executable="gait_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    return LaunchDescription([gait_node])
