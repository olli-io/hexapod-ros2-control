"""Top-level sim bringup.

Composes the existing ``hexa_simulation`` launch (Gazebo + ros2_control
+ joint_group_position_controller) with the hexapod kinematics chain:

- ``ik_node`` (hexa_kinematics) — composes ``/body/pose_target`` with
  ``/legs/targets`` and publishes ``/joint_commands`` (JointState).
- ``joint_command_bridge`` (hexa_kinematics) — translates
  ``/joint_commands`` to ``/joint_group_position_controller/commands``
  (Float64MultiArray) for the sim controller.
- ``posture_node`` (hexa_posture) — turns ``/cmd_vel`` + ``/body/pose``
  into ``/body/pose_target``. Launched here with the animation stack
  trimmed to ``["still"]`` so the breathing bob doesn't mask
  gait-induced body motion while locomotion is being tuned.
- ``control_node`` (hexa_control) — clamps ``/cmd_vel`` and republishes
  the result as ``GaitParams`` on ``/gait/params`` at 50 Hz.
- ``gait_node`` (hexa_gait) — runs the tripod gait engine, publishing
  per-leg foot targets on ``/legs/targets`` at 50 Hz.

Run with::

    ros2 launch hexa_bringup sim.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hexa_simulation"), "launch", "sim.launch.py",
            ])
        ),
    )

    common_params = [{"use_sim_time": True}]

    ik_node = Node(
        package="hexa_kinematics",
        executable="ik_node",
        output="screen",
        parameters=common_params,
    )

    joint_command_bridge = Node(
        package="hexa_kinematics",
        executable="joint_command_bridge",
        output="screen",
        parameters=common_params,
    )

    posture_config = PathJoinSubstitution([
        FindPackageShare("hexa_posture"), "config", "posture.yaml",
    ])
    posture_node = Node(
        package="hexa_posture",
        executable="posture_node",
        output="screen",
        parameters=common_params + [posture_config],
    )

    control_node = Node(
        package="hexa_control",
        executable="control_node",
        output="screen",
        parameters=common_params,
    )

    gait_node = Node(
        package="hexa_gait",
        executable="gait_node",
        output="screen",
        parameters=common_params,
    )

    return LaunchDescription([
        sim,
        ik_node,
        joint_command_bridge,
        posture_node,
        control_node,
        gait_node,
    ])
