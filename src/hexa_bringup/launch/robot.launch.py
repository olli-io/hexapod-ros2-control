"""Real-robot bringup: controller manager, consolidated locomotion node, display.

    ros2 launch hexa_bringup robot.launch.py

The robot energizes on launch: the hardware comes up `active` and both controllers
(joint_state_broadcaster + joint_group_position_controller) are spawned. This never
powers the servo rail on its own — the relay closes only once the robot stands
(gamepad Start or /gait/initialize) — so an unattended auto-restart comes back
energized but stationary. `hexa robot down` safe-stops (unload + deactivate).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
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

    # Energize on launch: bring the hardware straight to `active`. The servo rail
    # relay stays open until the robot stands, so this is safe unattended. Only
    # the `active` key is emitted — an empty `unconfigured: []` would normalize to
    # an empty tuple, which launch_ros rejects as a parameter value.
    cm_parameters = [
        robot_description,
        controllers_yaml,
        {
            "hardware_components_initial_state": {
                "active": [HARDWARE_COMPONENT_NAME],
            },
        },
    ]

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

    # Spawn both controllers. The spawner blocks until controller_manager's
    # services answer, so these are launched directly (no OnProcessExit gating on
    # the long-lived controller_manager, which never exits).
    for controller in ("joint_state_broadcaster", "joint_group_position_controller"):
        actions.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
            output="screen",
        ))

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

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="ros2 logging level for the locomotion node.",
        ),
        OpaqueFunction(function=_bringup),
    ])
