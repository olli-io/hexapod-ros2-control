"""Top-level sim bringup: hexa_simulation (Gazebo) + the kinematics/gait/posture chain.

    ros2 launch hexa_bringup sim.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _display_params() -> tuple[dict, bool]:
    """hexa_display's params and the `enabled` gate.

    Returned as a dict so callers can layer overrides (e.g. use_sim_time,
    headless) without an exact-name YAML file outranking them.
    """
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


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hexa_simulation"), "launch", "sim.launch.py",
            ])
        ),
    )

    common_params = [{"use_sim_time": True}]

    # Select the C++ or Python port of each subsystem. Default is "cpp" (from
    # the `hexa_launch` block of ros2_controllers.yaml): the *_cpp ports run and
    # the Python nodes are NOT started. `node_implementation:=python` flips the
    # whole chain back. The ports are drop-in: same node names, topics, message
    # types, and params. hexa_control has no C++ port, so control_node below is
    # always the Python node.
    impl = LaunchConfiguration("node_implementation")
    kinematics_pkg = PythonExpression(
        ["'hexa_kinematics_cpp' if '", impl,
         "'.lower() == 'cpp' else 'hexa_kinematics'"]
    )
    gait_pkg = PythonExpression(
        ["'hexa_gait_cpp' if '", impl, "'.lower() == 'cpp' else 'hexa_gait'"]
    )
    posture_pkg = PythonExpression(
        ["'hexa_posture_cpp' if '", impl, "'.lower() == 'cpp' else 'hexa_posture'"]
    )

    # `consolidated` runs the single hexa_locomotion node (the shared control
    # brain, /cmd_vel-native) INSTEAD of the control/gait/posture/kinematics
    # chain. It folds ik_node + joint_command_bridge (publishes joint commands
    # directly) and re-publishes /gait/state for the face. The two conditions
    # below are mutually exclusive, so exactly one of the two node sets launches.
    consolidated = PythonExpression(
        ["'", impl, "'.lower() == 'consolidated'"]
    )

    ik_node = Node(
        package=kinematics_pkg,
        executable="ik_node",
        output="screen",
        parameters=common_params,
        condition=UnlessCondition(consolidated),
    )

    joint_command_bridge = Node(
        package=kinematics_pkg,
        executable="joint_command_bridge",
        output="screen",
        parameters=common_params,
        condition=UnlessCondition(consolidated),
    )

    locomotion_node = Node(
        package="hexa_locomotion",
        executable="locomotion_node",
        output="screen",
        parameters=common_params,
        condition=IfCondition(consolidated),
    )

    # Every subsystem reads its knobs from hexa_description's tuning.yaml —
    # the single source of truth. It is a standard params file keyed by node
    # name (gait_node / control_node / posture_node), so the one file serves
    # all three nodes; each picks up only its own block.
    tuning_config = PathJoinSubstitution([
        FindPackageShare("hexa_description"), "config", "tuning.yaml",
    ])

    posture_node = Node(
        package=posture_pkg,
        executable="posture_node",
        output="screen",
        parameters=common_params + [tuning_config],
        condition=UnlessCondition(consolidated),
    )

    control_node = Node(
        package="hexa_control",
        executable="control_node",
        output="screen",
        parameters=common_params + [tuning_config],
        condition=UnlessCondition(consolidated),
    )

    gait_node = Node(
        package=gait_pkg,
        executable="gait_node",
        output="screen",
        parameters=common_params + [tuning_config],
        condition=UnlessCondition(consolidated),
    )

    actions = [
        # Which port of the kinematics/gait/posture nodes to launch. Default
        # ("cpp") comes from the `hexa_launch` block of ros2_controllers.yaml;
        # `node_implementation:=python` runs the ament_python originals instead.
        DeclareLaunchArgument(
            "node_implementation",
            default_value=_node_implementation(),
            description="Which implementation to launch: 'cpp' (the hexa_*_cpp "
                        "ports; Python nodes not started), 'python' (the "
                        "ament_python originals), or 'consolidated' (the single "
                        "hexa_locomotion node replacing the whole chain).",
        ),
        sim,
        ik_node,
        joint_command_bridge,
        locomotion_node,
        posture_node,
        control_node,
        gait_node,
    ]

    # Face: one node maps robot state to an expression/gaze policy and
    # rasterizes the eyes. No OLED in the sim container, so it runs headless
    # (full pipeline, no SPI/GPIO).
    display_params, display_enabled = _display_params()
    if display_enabled:
        display_params["headless"] = True
        actions.append(Node(
            package="hexa_display",
            executable="display_node",
            output="screen",
            parameters=common_params + [display_params],
        ))

    return LaunchDescription(actions)
