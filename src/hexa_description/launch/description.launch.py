"""Publishes the hexapod URDF on /robot_description via robot_state_publisher.

Downstream packages (gait, kinematics, simulation, bringup) consume the URDF
from this topic rather than re-parsing the xacro themselves.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim = LaunchConfiguration("use_sim")

    xacro_path = PathJoinSubstitution([
        FindPackageShare("hexa_description"), "urdf", "hexapod.urdf.xacro",
    ])

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ",
            xacro_path, " ",
            "use_sim:=", use_sim,
        ]),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation clock from /clock.",
        ),
        DeclareLaunchArgument(
            "use_sim", default_value="false",
            description="Include hexapod.gazebo.xacro overlay (materials, friction).",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
    ])
