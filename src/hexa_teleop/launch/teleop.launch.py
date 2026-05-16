"""Launch the 8BitDo Pro 2 joystick teleop.

Brings up the standard ``joy_node`` (publishing ``sensor_msgs/Joy`` on
``/joy``) and ``teleop_joy``, which reads ``/joy`` and publishes
``/cmd_vel`` + ``/body/pose``.

Pass ``joy_config_file:=/path/to/file.yaml`` to override the default
config installed alongside this package.

Run with::

    ros2 launch hexa_teleop teleop.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_cfg = PathJoinSubstitution([
        FindPackageShare("hexa_teleop"), "config", "teleop_joy.yaml",
    ])

    joy_config_file_arg = DeclareLaunchArgument(
        "joy_config_file",
        default_value=default_cfg,
        description="Path to the teleop_joy YAML config.",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            # /dev/input/jsN selector — bump if multiple joysticks are
            # attached. 0 is the default but stated here for clarity.
            "device_id": 0,
            # Deadband is applied in teleop_joy so the YAML stays the
            # single source of truth; tell joy_node to pass through raw.
            "deadzone": 0.0,
            # Resend the last Joy at this rate so teleop_joy keeps
            # producing fresh /cmd_vel + /body/pose even when the
            # sticks are idle.
            "autorepeat_rate": 50.0,
        }],
    )

    teleop_node = Node(
        package="hexa_teleop",
        executable="teleop_joy",
        name="teleop_joy",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("joy_config_file"),
        }],
    )

    return LaunchDescription([
        joy_config_file_arg,
        joy_node,
        teleop_node,
    ])
