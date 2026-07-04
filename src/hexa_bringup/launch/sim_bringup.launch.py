"""Sim bringup entrypoint (CMD of the `hexa-sim` container).

The sim twin of bringup.launch.py: it composes sim.launch.py — Gazebo plus the
kinematics/gait/posture/control node chain — with the gamepad and web teleop
input sources, so the whole desktop sim stack runs as the container's single
PID 1 process (docker-native lifecycle: `docker compose up -d` / `logs` / `down`).

  1. sim.launch.py — Gazebo + the node chain. Which port of the
     kinematics/gait/posture nodes it runs (C++ by default) is set by the
     hexa_launch block of ros2_controllers.yaml, via its node_implementation arg.
  2. teleop.launch.py — gamepad -> /cmd_vel + /body/pose.
  3. webteleop.launch.py — web UI -> /cmd_vel, on port 8080. Coexists with the
     gamepad via /teleop/owner arbitration (gamepad owns by default).

No /clock wait is needed: the sim-time nodes declare use_sim_time and queue
until /clock arrives, and the controllers-after-model-spawn ordering is already
handled by OnProcessExit handlers inside hexa_simulation/sim.launch.py, so all
three includes can start concurrently.

    ros2 launch hexa_bringup sim_bringup.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hexa_bringup"), "launch", "sim.launch.py",
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

    return LaunchDescription([sim, teleop, webteleop])
