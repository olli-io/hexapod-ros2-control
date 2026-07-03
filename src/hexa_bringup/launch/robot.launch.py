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


def _display_params(transport: str) -> tuple[dict, bool]:
    """hexa_display's display.yaml params (transport forced) and `enabled` flag.

    Returned as a dict, not a file path: exact-name YAML entries would
    outrank a transport override layered on top of the params file.
    """
    path = os.path.join(
        get_package_share_directory("hexa_display"), "config", "display.yaml"
    )
    with open(path) as f:
        params = yaml.safe_load(f)["display_node"]["ros__parameters"]
    params["transport"] = transport
    return params, bool(params.pop("enabled", True))


def _tuning_block(domain: str) -> dict:
    """The ``domain`` block of hexa_description's tuning overlay, or ``{}``.

    Layered on top of a node's base params file (later param sources win)
    so an uncommented knob in config/tuning.yaml overrides it. A missing
    file / block / key yields ``{}`` (no override).
    """
    path = os.path.join(
        get_package_share_directory("hexa_description"), "config", "tuning.yaml"
    )
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}
    block = data.get(domain, {})
    return block if isinstance(block, dict) else {}


def _bringup(context, *args, **kwargs):
    pkg_hexa_bringup = FindPackageShare("hexa_bringup")
    pkg_hexa_description = FindPackageShare("hexa_description")

    engage_on_start = LaunchConfiguration("engage_on_start").perform(context)
    engage = engage_on_start.lower() in ("1", "true", "yes")

    # Select the Python or C++ port of each subsystem. Default keeps the Python
    # nodes; set the arg true to run the ament_cmake ports (built side-by-side).
    # The ports are drop-in: same node names, topics, message types, and params.
    use_cpp_kinematics = LaunchConfiguration("use_cpp_kinematics").perform(context)
    use_cpp_gait = LaunchConfiguration("use_cpp_gait").perform(context)
    use_cpp_posture = LaunchConfiguration("use_cpp_posture").perform(context)
    kinematics_pkg = (
        "hexa_kinematics_cpp"
        if use_cpp_kinematics.lower() in ("1", "true", "yes")
        else "hexa_kinematics"
    )
    gait_pkg = (
        "hexa_gait_cpp"
        if use_cpp_gait.lower() in ("1", "true", "yes")
        else "hexa_gait"
    )
    posture_pkg = (
        "hexa_posture_cpp"
        if use_cpp_posture.lower() in ("1", "true", "yes")
        else "hexa_posture"
    )
    pkg_hexa_posture = FindPackageShare(posture_pkg)

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

    posture_config = PathJoinSubstitution([
        pkg_hexa_posture, "config", "posture.yaml",
    ])
    # Layer the tuning overlay's posture block on top of posture.yaml (last
    # param source wins). Appended only when non-empty so the base file is
    # untouched in the normal override-nothing case.
    posture_params = [posture_config]
    posture_overlay = _tuning_block("posture")
    if posture_overlay:
        posture_params.append(posture_overlay)
    posture_node = Node(
        package=posture_pkg,
        executable="posture_node",
        output="screen",
        parameters=posture_params,
    )

    ik_node = Node(
        package=kinematics_pkg, executable="ik_node", output="screen",
    )
    joint_command_bridge = Node(
        package=kinematics_pkg, executable="joint_command_bridge", output="screen",
    )
    control_node = Node(
        package="hexa_control", executable="control_node", output="screen",
    )
    gait_node = Node(
        package=gait_pkg, executable="gait_node", output="screen",
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
                "If true, activate the hardware and spawn controllers at "
                "launch. If false, boot cold (inactive, relay open, no "
                "controllers); `hexa robot up` energizes it live."
            ),
        ),
        # Defaults honour the HEXA_CPP env var (set by `hexa sim up --cpp`), so the
        # whole chain flips to the C++ ports without per-command args. An
        # explicit `use_cpp_*:=...` on the command line still overrides.
        DeclareLaunchArgument(
            "use_cpp_kinematics",
            default_value=os.environ.get("HEXA_CPP", "false"),
            description="Run the C++ hexa_kinematics_cpp nodes instead of the "
                        "Python hexa_kinematics nodes.",
        ),
        DeclareLaunchArgument(
            "use_cpp_gait",
            default_value=os.environ.get("HEXA_CPP", "false"),
            description="Run the C++ hexa_gait_cpp gait_node instead of the "
                        "Python hexa_gait gait_node.",
        ),
        DeclareLaunchArgument(
            "use_cpp_posture",
            default_value=os.environ.get("HEXA_CPP", "false"),
            description="Run the C++ hexa_posture_cpp posture_node instead of "
                        "the Python hexa_posture posture_node.",
        ),
        OpaqueFunction(function=_bringup),
    ])
