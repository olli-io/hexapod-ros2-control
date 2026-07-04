"""Real-robot bringup: controller manager, kinematics/gait/posture chain, display.

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


def _node_implementation() -> str:
    """Default node-implementation selector, read from hexa_bringup's
    ros2_controllers.yaml (the `hexa_launch` block). "cpp" (default) selects
    the C++ ports of kinematics/gait/posture; "python" the ament_python
    originals. Used as the default of the `node_implementation` launch arg.
    """
    path = os.path.join(
        get_package_share_directory("hexa_bringup"), "config", "ros2_controllers.yaml"
    )
    with open(path) as f:
        cfg = yaml.safe_load(f)
    impl = cfg.get("hexa_launch", {}).get("ros__parameters", {}).get(
        "node_implementation", "cpp"
    )
    return str(impl).lower()


def _bringup(context, *args, **kwargs):
    pkg_hexa_bringup = FindPackageShare("hexa_bringup")
    pkg_hexa_description = FindPackageShare("hexa_description")

    engage_on_start = LaunchConfiguration("engage_on_start").perform(context)
    engage = engage_on_start.lower() in ("1", "true", "yes")

    # Select the C++ or Python port of each subsystem. Default is "cpp" (from
    # the `hexa_launch` block of ros2_controllers.yaml): the *_cpp ports run and
    # the Python nodes are NOT started. `node_implementation:=python` flips the
    # whole chain back. The ports are drop-in: same node names, topics, message
    # types, and params. hexa_control has no C++ port, so control_node below is
    # always the Python node.
    use_cpp = LaunchConfiguration("node_implementation").perform(context).lower() == "cpp"
    kinematics_pkg = "hexa_kinematics_cpp" if use_cpp else "hexa_kinematics"
    gait_pkg = "hexa_gait_cpp" if use_cpp else "hexa_gait"
    posture_pkg = "hexa_posture_cpp" if use_cpp else "hexa_posture"

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

    # Every subsystem reads its knobs from hexa_description's tuning.yaml —
    # the single source of truth. It is a standard params file keyed by node
    # name (gait_node / control_node / posture_node), so the one file serves
    # all three nodes; each picks up only its own block.
    tuning_config = PathJoinSubstitution([
        pkg_hexa_description, "config", "tuning.yaml",
    ])

    posture_node = Node(
        package=posture_pkg,
        executable="posture_node",
        output="screen",
        parameters=[tuning_config],
    )

    ik_node = Node(
        package=kinematics_pkg, executable="ik_node", output="screen",
    )
    joint_command_bridge = Node(
        package=kinematics_pkg, executable="joint_command_bridge", output="screen",
    )
    control_node = Node(
        package="hexa_control", executable="control_node", output="screen",
        parameters=[tuning_config],
    )
    gait_node = Node(
        package=gait_pkg, executable="gait_node", output="screen",
        parameters=[tuning_config],
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
            description="ros2 logging level for the kinematics/gait nodes.",
        ),
        DeclareLaunchArgument(
            "engage_on_start", default_value="true",
            description=(
                "If true, activate the hardware and spawn controllers at "
                "launch. If false, boot cold (inactive, relay open, no "
                "controllers); `hexa robot up` energizes it live."
            ),
        ),
        # Which port of the kinematics/gait/posture nodes to launch. Default
        # ("cpp") comes from the `hexa_launch` block of ros2_controllers.yaml;
        # `node_implementation:=python` runs the ament_python originals instead.
        DeclareLaunchArgument(
            "node_implementation",
            default_value=_node_implementation(),
            description="Which implementation of the kinematics/gait/posture "
                        "nodes to launch: 'cpp' (the hexa_*_cpp ports; Python "
                        "nodes not started) or 'python' (the ament_python "
                        "originals).",
        ),
        OpaqueFunction(function=_bringup),
    ])
