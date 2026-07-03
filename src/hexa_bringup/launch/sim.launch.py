"""Top-level sim bringup: hexa_simulation (Gazebo) + the kinematics/gait/posture chain.

    ros2 launch hexa_bringup sim.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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


def _gait_params() -> dict:
    """gait_node's knobs from config/gait.yaml with the tuning overlay merged.

    Returned as a single dict, not the file path plus a dict overlay: an
    exact-name YAML params file (``gait_node:``) outranks a dict layered on
    top of it, so the overlay would silently lose (same reasoning as
    ``_display_params``). Merging here keeps a single param source where the
    overlay reliably wins. The ``gait`` overlay block is flat scalars only, so
    a shallow update suffices — the nested initialize:/reseat: maps are never
    overridden. gait.yaml is byte-identical across the Python and C++ packages,
    so hexa_gait's copy is authoritative for either node.
    """
    path = os.path.join(
        get_package_share_directory("hexa_gait"), "config", "gait.yaml"
    )
    with open(path) as f:
        params = yaml.safe_load(f)["gait_node"]["ros__parameters"]
    params.update(_tuning_block("gait"))
    return params


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hexa_simulation"), "launch", "sim.launch.py",
            ])
        ),
    )

    common_params = [{"use_sim_time": True}]

    # Select the Python or C++ port of each subsystem. Default keeps the Python
    # nodes; set the arg true to run the ament_cmake ports (built side-by-side).
    # The ports are drop-in: same node names, topics, message types, and params.
    use_cpp_kinematics = LaunchConfiguration("use_cpp_kinematics")
    use_cpp_gait = LaunchConfiguration("use_cpp_gait")
    use_cpp_posture = LaunchConfiguration("use_cpp_posture")
    kinematics_pkg = PythonExpression(
        ["'hexa_kinematics_cpp' if '", use_cpp_kinematics,
         "'.lower() in ('true', '1') else 'hexa_kinematics'"]
    )
    gait_pkg = PythonExpression(
        ["'hexa_gait_cpp' if '", use_cpp_gait,
         "'.lower() in ('true', '1') else 'hexa_gait'"]
    )
    posture_pkg = PythonExpression(
        ["'hexa_posture_cpp' if '", use_cpp_posture,
         "'.lower() in ('true', '1') else 'hexa_posture'"]
    )

    ik_node = Node(
        package=kinematics_pkg,
        executable="ik_node",
        output="screen",
        parameters=common_params,
    )

    joint_command_bridge = Node(
        package=kinematics_pkg,
        executable="joint_command_bridge",
        output="screen",
        parameters=common_params,
    )

    posture_config = PathJoinSubstitution([
        FindPackageShare(posture_pkg), "config", "posture.yaml",
    ])
    # Layer the tuning overlay's posture block on top of posture.yaml (last
    # param source wins). Appended only when non-empty so the base file is
    # untouched in the normal override-nothing case.
    posture_params = common_params + [posture_config]
    posture_overlay = _tuning_block("posture")
    if posture_overlay:
        posture_params.append(posture_overlay)
    posture_node = Node(
        package=posture_pkg,
        executable="posture_node",
        output="screen",
        parameters=posture_params,
    )

    control_config = PathJoinSubstitution([
        FindPackageShare("hexa_control"), "config", "control.yaml",
    ])
    # Layer the tuning overlay's control block on top of control.yaml (last
    # param source wins). Appended only when non-empty.
    control_params = common_params + [control_config]
    control_overlay = _tuning_block("control")
    if control_overlay:
        control_params.append(control_overlay)
    control_node = Node(
        package="hexa_control",
        executable="control_node",
        output="screen",
        parameters=control_params,
    )

    # gait_node's knobs are passed as one merged dict (see _gait_params) so the
    # tuning overlay reliably wins over the base gait.yaml values.
    gait_node = Node(
        package=gait_pkg,
        executable="gait_node",
        output="screen",
        parameters=common_params + [_gait_params()],
    )

    actions = [
        # Defaults honour the HEXA_CPP env var (set by `hexa sim up --cpp`), so the
        # whole sim stack flips to the C++ ports without per-command args. An
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
        sim,
        ik_node,
        joint_command_bridge,
        posture_node,
        control_node,
        gait_node,
    ]

    display_params, display_enabled = _display_params(transport="stub")
    if display_enabled:
        actions.append(Node(
            package="hexa_display",
            executable="display_node",
            output="screen",
            parameters=common_params + [display_params],
        ))

    return LaunchDescription(actions)
