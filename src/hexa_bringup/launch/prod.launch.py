"""Production bringup — the CMD of the `hexa-prod` container.

Composes the real-robot stack with the safety gate engaged and brings up
joystick teleop in the same process tree, so a freshly-started container
is one `hexa --prod engage` away from being drivable.

  1. robot.launch.py with ``engage_on_start:=false`` — the hardware
     component stops at `inactive`, the servo-rail relay stays open, no
     controllers are spawned.
  2. teleop.launch.py — joy_publisher + teleop_joy publishing /cmd_vel
     and /body/pose. Safe to run cold: with no controllers loaded, the
     commands have no consumer.

Run with::

    ros2 launch hexa_bringup prod.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hexa_bringup"), "launch", "robot.launch.py",
            ])
        ),
        launch_arguments={
            "engage_on_start": "false",
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hexa_teleop"), "launch", "teleop.launch.py",
            ])
        ),
    )

    return LaunchDescription([robot, teleop])
