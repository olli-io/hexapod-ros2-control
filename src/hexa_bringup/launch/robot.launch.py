"""Real-robot bringup.

Brings up, in order:
  1. robot_state_publisher (via hexa_description/launch/description.launch.py)
     with use_sim:=false so the URDF tags out the hexa_hardware
     SystemInterface plugin instead of gz_ros2_control.
  2. ros2_control_node — the standalone controller manager. Loads
     ros2_controllers.yaml from this package.
  3. joint_state_broadcaster spawner, then joint_group_position_controller
     spawner, chained on OnProcessExit so the controllers come up only
     after the manager is alive. **Skipped when ``engage_on_start:=false``** —
     in that mode the hardware boots in `inactive` and an external operator
     promotes it (and spawns controllers) via `hexa --prod engage`.
  4. The kinematics / gait / posture chain (ik_node, joint_command_bridge,
     posture_node, control_node, gait_node), identical to sim.launch.py
     except use_sim_time is false.
  5. display_node (hexa_display) with the serial transport — relays
     expression/gaze to the ESP32 face over /dev/serial0. Comes up
     faceless (retrying in the background) if the display is absent.
     Skipped entirely when ``enabled: false`` in hexa_display's
     display.yaml.

Run with::

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


# Name of the <ros2_control> block declared in hexa_description's URDF
# xacro. controller_manager keys its hardware_components_initial_state map
# off this exact string.
HARDWARE_COMPONENT_NAME = "HexaSystem"


def _display_params(transport: str) -> tuple[dict, bool]:
    """hexa_display's display.yaml params and its `enabled` flag.

    Returned as a plain dict (with the launch-appropriate transport
    forced) rather than a params-file path: the YAML scopes its entries
    under the exact node name, and exact-name entries outrank the
    wildcard `/**` file launch_ros generates for dict overrides — so a
    `{"transport": ...}` dict after the file would silently lose.
    """
    path = os.path.join(
        get_package_share_directory("hexa_display"), "config", "display.yaml"
    )
    with open(path) as f:
        params = yaml.safe_load(f)["display_node"]["ros__parameters"]
    params["transport"] = transport
    return params, bool(params.pop("enabled", True))


def _bringup(context, *args, **kwargs):
    pkg_hexa_bringup = FindPackageShare("hexa_bringup")
    pkg_hexa_description = FindPackageShare("hexa_description")
    pkg_hexa_posture = FindPackageShare("hexa_posture")

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

    cm_parameters = [robot_description, controllers_yaml]

    # Cold-start mode: tell the controller manager to bring the hardware
    # component up to `inactive` only, not `active`. The plugin's
    # `on_activate` (which drives the servo-rail relay high) does not fire
    # until `hexa --prod engage` transitions the component to active.
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

    actions = [
        description,
        controller_manager,
        ik_node,
        joint_command_bridge,
        posture_node,
        control_node,
        gait_node,
    ]

    display_params, display_enabled = _display_params(transport="serial")
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
            description="ros2 logging level for the kinematics/gait nodes.",
        ),
        DeclareLaunchArgument(
            "engage_on_start", default_value="true",
            description=(
                "If true (default), the hardware component is brought to "
                "`active` at launch and controllers are spawned — the relay "
                "energises and the robot is immediately drivable. Set to "
                "false on the real robot to boot cold: the component stops "
                "at `inactive`, the relay stays open, and the controllers "
                "are not spawned. `hexa --prod engage` flips it live."
            ),
        ),
        OpaqueFunction(function=_bringup),
    ])
