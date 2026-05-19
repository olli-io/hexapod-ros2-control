"""Launches the hexapod in Gazebo Harmonic with ros2_control.

Brings up, in order:
  1. gz_sim with the SDF world (default: empty.sdf in this package).
  2. robot_state_publisher (via hexa_description/launch/description.launch.py)
     with use_sim:=true and use_sim_time:=true. This publishes the URDF on
     /robot_description, including the gz_ros2_control plugin tag.
  3. A /clock parameter_bridge so any ROS node using sim time gets it.
  4. ros_gz_sim create, which reads /robot_description and spawns the model.
  5. joint_state_broadcaster, then joint_group_position_controller. Chained
     via OnProcessExit so the controllers come up only after the model has
     been spawned (and gz_ros2_control has therefore started its
     controller_manager).
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_hexa_simulation = FindPackageShare("hexa_simulation")
    pkg_hexa_description = FindPackageShare("hexa_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    # Spawn z defaults to the chassis's coxa-to-bottom half-thickness so
    # the hexapod lands belly-flush on the ground at the folded
    # initial_pose. Resolved eagerly (not via Substitution) because
    # DeclareLaunchArgument needs a concrete string for its default and
    # for its -h help text. Override on the CLI when debugging.
    _geom_path = Path(get_package_share_directory("hexa_description")) / "config" / "geometry.yaml"
    _geom = yaml.safe_load(_geom_path.read_text())
    default_spawn_z = str(_geom["body"]["coxa_to_bottom"])

    default_world = PathJoinSubstitution(
        [pkg_hexa_simulation, "worlds", "empty.sdf"]
    )
    world = LaunchConfiguration("world")
    spawn_z = LaunchConfiguration("spawn_z")
    headless = LaunchConfiguration("headless")

    # gz_sim. `-r` starts unpaused, `-v 3` is info-level verbosity.
    # `-s` (added when headless:=true) runs the physics server with no GUI;
    # useful for CI smoke tests and any environment without a display.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [world, " -r -v 3 ",
                        PythonExpression(["'-s' if '", headless, "' == 'true' else ''"])],
        }.items(),
    )

    # robot_state_publisher with the sim overlay enabled.
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_hexa_description, "launch", "description.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim": "true",
            "use_sim_time": "true",
        }.items(),
    )

    # Bridge gz /clock -> ROS /clock so use_sim_time=true is honoured.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Spawn the model from /robot_description.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "hexapod",
            "-z", spawn_z,
        ],
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value=default_world,
            description="SDF world to load in gz_sim.",
        ),
        DeclareLaunchArgument(
            "spawn_z", default_value=default_spawn_z,
            description="Initial z-height (m) at which the model is spawned. "
                        "Defaults to body.coxa_to_bottom from "
                        "hexa_description/config/geometry.yaml so the model "
                        "spawns belly-flush at the folded initial_pose.",
        ),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run gz_sim in server-only mode (no GUI).",
        ),
        gz_sim,
        description,
        clock_bridge,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[position_controller_spawner],
            )
        ),
    ])
