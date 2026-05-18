"""Launch ``control_node`` standalone.

Sim composition (``hexa_bringup/launch/sim.launch.py``) starts
``control_node`` directly with its own ``use_sim_time: True`` parameter
block, bypassing this launch file.

Run with::

    ros2 launch hexa_control control.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    control_node = Node(
        package="hexa_control",
        executable="control_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    return LaunchDescription([control_node])
