"""Launch joystick teleop (X-input controller).

Brings up our ``joy_publisher`` (publishing ``sensor_msgs/Joy`` on
``/joy``) and ``teleop_joy``, which reads ``/joy`` and publishes
``/cmd_vel`` + ``/body/pose``. ``joy_publisher`` replaces upstream
``joy_node`` so the controller can be unplugged / replugged mid-session
and recovers without restarting any ROS process.

Pass ``joy_config_file:=/path/to/file.yaml`` to override the default
config installed alongside this package.

Run with::

    ros2 launch hexa_teleop teleop.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_cfg = PathJoinSubstitution([
        FindPackageShare("hexa_teleop"), "config", "teleop_joy.yaml",
    ])

    joy_config_file_arg = DeclareLaunchArgument(
        "joy_config_file",
        default_value=default_cfg,
        description="Path to the teleop_joy YAML config.",
    )

    joy_node = Node(
        package="hexa_teleop",
        executable="joy_publisher",
        name="joy_node",
        output="screen",
        parameters=[{
            # Empty => auto-discover the first /dev/input/jsN. Linux
            # renumbers jsN on replug (a controller that was js0 can
            # come back as js1), so pinning a number is brittle.
            # Override with a literal path (e.g. "/dev/input/js2") if
            # multiple controllers are attached and one must be picked
            # deterministically.
            "device_path": "",
            # Small driver-level deadzone to nuke stick noise at the
            # source — keeps released-stick axes pinned to exact zero so
            # downstream cmd_vel doesn't flicker around the gait engine's
            # cmd_zero_tol. The larger shaping deadband still lives in
            # teleop_joy / the YAML.
            "deadzone": 0.05,
            # Resend the last Joy at this rate so teleop_joy keeps
            # producing fresh /cmd_vel + /body/pose even when the
            # sticks are idle.
            "autorepeat_rate": 50.0,
            # Poll /dev/input/jsN at this period while the controller
            # is unplugged. 1 s is the longest a user should wait
            # between replugging and the node picking the device up.
            "scan_period_s": 1.0,
        }],
    )

    teleop_node = Node(
        package="hexa_teleop",
        executable="teleop_joy",
        name="teleop_joy",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("joy_config_file"),
        }],
    )

    return LaunchDescription([
        joy_config_file_arg,
        joy_node,
        teleop_node,
    ])
