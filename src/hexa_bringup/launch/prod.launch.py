"""Production entrypoint (CMD of the `hexa-prod` container).

Wraps the robot stack with production policy: it composes robot.launch.py —
the reusable robot — with the gamepad and web teleop input sources, which
robot.launch.py deliberately omits. Boots cold (``engage_on_start:=false``,
relay open) so the container is one `hexa --prod engage` away from drivable.

  1. robot.launch.py (engage_on_start:=false) — the robot, brought up cold.
  2. teleop.launch.py — gamepad → /cmd_vel + /body/pose.
  3. webteleop.launch.py — web UI → /cmd_vel, on port 8080. Coexists with the
     gamepad via /teleop/owner arbitration (gamepad owns by default).

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

    webteleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hexa_webteleop"), "launch", "webteleop.launch.py",
            ])
        ),
    )

    return LaunchDescription([robot, teleop, webteleop])
