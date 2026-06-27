"""Top-level sim bringup: hexa_simulation (Gazebo) + the kinematics/gait/posture chain.

    ros2 launch hexa_bringup sim.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hexa_simulation"), "launch", "sim.launch.py",
            ])
        ),
    )

    common_params = [{"use_sim_time": True}]

    ik_node = Node(
        package="hexa_kinematics",
        executable="ik_node",
        output="screen",
        parameters=common_params,
    )

    joint_command_bridge = Node(
        package="hexa_kinematics",
        executable="joint_command_bridge",
        output="screen",
        parameters=common_params,
    )

    posture_config = PathJoinSubstitution([
        FindPackageShare("hexa_posture"), "config", "posture.yaml",
    ])
    posture_node = Node(
        package="hexa_posture",
        executable="posture_node",
        output="screen",
        parameters=common_params + [posture_config],
    )

    control_node = Node(
        package="hexa_control",
        executable="control_node",
        output="screen",
        parameters=common_params,
    )

    gait_node = Node(
        package="hexa_gait",
        executable="gait_node",
        output="screen",
        parameters=common_params,
    )

    actions = [
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
