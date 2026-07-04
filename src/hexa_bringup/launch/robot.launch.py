"""Real-robot bringup: controller manager, consolidated locomotion node, display.

    ros2 launch hexa_bringup robot.launch.py
    ros2 launch hexa_bringup robot.launch.py engage_on_start:=false
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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


# Must match the <ros2_control> block name in hexa_description's URDF xacro.
HARDWARE_COMPONENT_NAME = "HexaSystem"


def _display_params() -> tuple[dict, bool]:
    """hexa_display's params and the `enabled` gate."""
    path = os.path.join(
        get_package_share_directory("hexa_display"), "config", "display.yaml"
    )
    with open(path) as f:
        params = yaml.safe_load(f)["display_node"]["ros__parameters"]
    return params, bool(params.pop("enabled", True))


def _bringup(context, *args, **kwargs):
    pkg_hexa_bringup = FindPackageShare("hexa_bringup")
    pkg_hexa_description = FindPackageShare("hexa_description")

    engage_on_start = LaunchConfiguration("engage_on_start").perform(context)
    engage = engage_on_start.lower() in ("1", "true", "yes")

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
    # not just on the topic; re-expand xacro here.
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

    cm_parameters = [robot_description, controllers_yaml]

    # Cold-start: bring the hardware to `inactive` only. The relay stays open
    # until `hexa robot up` energizes the component.
    if not engage:
        cm_parameters.append({
            "hardware_components_initial_state": {
                "unconfigured": [],
                "inactive": [HARDWARE_COMPONENT_NAME],
            },
        })

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=cm_parameters,
        output="screen",
    )

    # The consolidated locomotion node runs the whole velocity -> gait -> posture
    # -> compose/IK pipeline in one 200 Hz tick (mirroring the Pi Pico firmware's
    # single loop). It loads geometry+tuning from hexa_description's YAML at
    # startup (PipelineConfig), so it needs no params file; it folds ik_node +
    # joint_command_bridge and re-publishes /gait/state for the face.
    locomotion_node = Node(
        package="hexa_locomotion", executable="locomotion_node", output="screen",
    )

    actions = [description, controller_manager, locomotion_node]

    # Face: one node maps robot state to an expression/gaze policy and
    # rasterizes the eyes on the SH1122 OLED (spidev + GPIO).
    display_params, display_enabled = _display_params()
    if display_enabled:
        actions.append(Node(
            package="hexa_display",
            executable="display_node",
            output="screen",
            parameters=[display_params],
        ))

    if engage:
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
        actions += [
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
        ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="ros2 logging level for the locomotion node.",
        ),
        DeclareLaunchArgument(
            "engage_on_start", default_value="true",
            description=(
                "If true, activate the hardware and spawn controllers at "
                "launch. If false, boot cold (inactive, relay open, no "
                "controllers); `hexa robot up` energizes it live."
            ),
        ),
        OpaqueFunction(function=_bringup),
    ])
