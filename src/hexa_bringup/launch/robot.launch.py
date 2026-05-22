"""Real-robot bringup.

Brings up, in order:
  1. robot_state_publisher (via hexa_description/launch/description.launch.py)
     with use_sim:=false so the URDF tags out the hexa_hardware
     SystemInterface plugin instead of gz_ros2_control.
  2. ros2_control_node — the standalone controller manager. Loads
     ros2_controllers.yaml from this package.
  3. joint_state_broadcaster spawner, then joint_group_position_controller
     spawner, chained on OnProcessExit so the controllers come up only
     after the manager is alive.
  4. The kinematics / gait / posture chain (ik_node, joint_command_bridge,
     posture_node, control_node, gait_node), identical to sim.launch.py
     except use_sim_time is false.

Run with::

    ros2 launch hexa_bringup robot.launch.py
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_hexa_bringup = FindPackageShare("hexa_bringup")
    pkg_hexa_description = FindPackageShare("hexa_description")
    pkg_hexa_posture = FindPackageShare("hexa_posture")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_hexa_description, "launch", "description.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim": "false",
            "use_sim_time": "false",
        }.items(),
    )

    # The controller manager needs robot_description in its own parameters,
    # not just on the topic; re-expand xacro here for that.
    xacro_path = PathJoinSubstitution([
        pkg_hexa_description, "urdf", "hexapod.urdf.xacro",
    ])
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                xacro_path, " ",
                "use_sim:=false",
            ]),
            value_type=str,
        ),
    }

    controllers_yaml = PathJoinSubstitution([
        pkg_hexa_bringup, "config", "ros2_controllers.yaml",
    ])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_group_position_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    posture_config = PathJoinSubstitution([
        pkg_hexa_posture, "config", "posture.yaml",
    ])
    posture_node = Node(
        package="hexa_posture",
        executable="posture_node",
        output="screen",
        parameters=[posture_config],
    )

    ik_node = Node(
        package="hexa_kinematics", executable="ik_node", output="screen",
    )
    joint_command_bridge = Node(
        package="hexa_kinematics", executable="joint_command_bridge", output="screen",
    )
    control_node = Node(
        package="hexa_control", executable="control_node", output="screen",
    )
    gait_node = Node(
        package="hexa_gait", executable="gait_node", output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="ros2 logging level for the kinematics/gait nodes.",
        ),
        description,
        controller_manager,
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_manager,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[position_controller_spawner],
            )
        ),
        ik_node,
        joint_command_bridge,
        posture_node,
        control_node,
        gait_node,
    ])
