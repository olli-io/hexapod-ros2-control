"""Robot bringup entrypoint (CMD of the `hexa-robot` container).

Wraps the robot stack with production policy: it composes robot.launch.py —
the reusable robot — with the gamepad and web teleop input sources, which
robot.launch.py deliberately omits. robot.launch.py energizes on launch
(hardware active + controllers spawned), so the container comes up drivable;
the servo rail relay still waits for a stand command.

  1. robot.launch.py — the robot, brought up energized.
  2. teleop.launch.py — gamepad → /cmd_vel + /body/pose.
  3. webteleop.launch.py — web UI → /cmd_vel, on port 8080. Coexists with the
     gamepad via /teleop/owner arbitration (gamepad owns by default).

    ros2 launch hexa_bringup bringup.launch.py
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
