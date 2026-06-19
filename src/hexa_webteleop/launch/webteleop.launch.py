"""Launch the web-app teleop node.

Brings up ``webteleop_node``, which hosts an HTTP + WebSocket server
serving the phone/tablet webapp and publishing ``/cmd_vel``,
``/body/pose``, ``/cmd_gait``, ``/animation/mode``, ``/gait/initialize``
— the same topics the gamepad teleop publishes. Coexistence with the
gamepad is mediated by ``/teleop/owner``.

Pass ``config_file:=/path/to/file.yaml`` to override the default config.

Run with::

    ros2 launch hexa_webteleop webteleop.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_cfg = PathJoinSubstitution([
        FindPackageShare("hexa_webteleop"), "config", "webteleop.yaml",
    ])

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_cfg,
        description="Path to the webteleop YAML config.",
    )

    webteleop_node = Node(
        package="hexa_webteleop",
        executable="webteleop_node",
        name="web_teleop",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
        }],
    )

    return LaunchDescription([
        config_file_arg,
        webteleop_node,
    ])
