"""Top-level sim bringup: hexa_simulation (Gazebo) + the consolidated locomotion node.

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


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hexa_simulation"), "launch", "sim.launch.py",
            ])
        ),
    )

    common_params = [{"use_sim_time": True}]

    # The consolidated locomotion node runs the whole velocity -> gait -> posture
    # -> compose/IK pipeline in one 200 Hz tick (mirroring the Pi Pico firmware's
    # single loop). It folds ik_node + joint_command_bridge (publishes joint
    # commands directly) and re-publishes /gait/state for the face. Geometry +
    # tuning are loaded from hexa_description's YAML at startup (PipelineConfig),
    # so runtime tuning is preserved without a params file.
    locomotion_node = Node(
        package="hexa_locomotion",
        executable="locomotion_node",
        output="screen",
        parameters=common_params,
    )

    actions = [sim, locomotion_node]

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
