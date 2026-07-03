"""Launch ``control_node`` standalone.

Sim composition (``hexa_bringup/launch/sim.launch.py``) starts
``control_node`` directly with its own param wiring (hexa_description's
tuning.yaml), bypassing this launch file.

Run with::

    ros2 launch hexa_control control.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_config = PathJoinSubstitution([
        FindPackageShare("hexa_description"), "config", "tuning.yaml",
    ])
    control_node = Node(
        package="hexa_control",
        executable="control_node",
        output="screen",
        parameters=[{"use_sim_time": False}, control_config],
    )
    return LaunchDescription([control_node])
