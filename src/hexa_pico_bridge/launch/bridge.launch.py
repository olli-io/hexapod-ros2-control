"""Gazebo-in-the-loop firmware bridge (plan part 10, Tier 3).

Brings up the firmware brain driving the simulated hexapod:

  - the Gazebo sim (hexa_simulation/sim.launch.py) — the robot model + the
    gz_ros2_control joint_group_position_controller,
  - the joy publisher (hexa_teleop/joy_publisher) — sensor_msgs/Joy on /joy in
    the exact layout the firmware's bt_teleop emits,
  - firmware_bridge_node — runs hexa::pipeline::Pipeline at 50 Hz and publishes
    joint commands to the controller.

Same teleop input should walk the Gazebo hexapod identically to the ROS2 node
chain (ik_node + gait_node + posture_node) in the same world (sim-first).

Launch args:
  sim      (default true)  — include the Gazebo sim launch. Set false to run the
                             bridge against an already-running sim (e.g. `hexa sim up`).
  joy      (default true)  — start the joy publisher. Set false to feed /joy
                             yourself (a host joystick, ros2 topic pub, a bag).
  headless (default false) — forwarded to the sim launch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    sim = LaunchConfiguration("sim")
    joy = LaunchConfiguration("joy")
    headless = LaunchConfiguration("headless")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("hexa_simulation"), "launch", "sim.launch.py"]
            )
        ),
        launch_arguments={"headless": headless}.items(),
        condition=IfCondition(sim),
    )

    joy_node = Node(
        package="hexa_teleop",
        executable="joy_publisher",
        name="joy_node",
        output="screen",
        condition=IfCondition(joy),
    )

    bridge_node = Node(
        package="hexa_pico_bridge",
        executable="firmware_bridge_node",
        name="firmware_bridge",
        output="screen",
        # Pace the pipeline off sim time so it stays in lockstep with the
        # gz_ros2_control controller_manager (also use_sim_time). On a wall clock
        # the firmware brain drifts against Gazebo's real-time factor and the
        # synchronized body motions (stand/sit ramp, breathing) stutter.
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="true"),
            DeclareLaunchArgument("joy", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            sim_launch,
            joy_node,
            bridge_node,
        ]
    )
