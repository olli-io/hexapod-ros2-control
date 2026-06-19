"""ROS glue for the joystick teleop.

Reads ``sensor_msgs/Joy`` from ``/joy``, maps it via the pure
``joy_mapping`` library, and publishes ``/cmd_vel`` (body velocity for
the gait chain) and ``/body/pose`` (body-pose offset for the posture
chain) on a fixed timer. Also publishes ``/cmd_gait``,
``/animation/mode``, and ``/gait/initialize`` on the appropriate user
inputs. The inactive channel of cmd_vel / body/pose is zero-filled so
consumers always see a coherent command.
"""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from hexa_interfaces.msg import BodyPose as BodyPoseMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String

from hexa_gait import VelocityCaps, load_velocity_caps
from hexa_gait.gaits import STRATEGIES
from hexa_posture import load_animation_mode_animations

from .joy_mapping import (
    ANIMATION,
    AXIS_CLASS_FUNCTIONS,
    BASE_FUNCTIONS,
    BUTTON_CLASS_FUNCTIONS,
    GAIT,
    POSTURE,
    BaseConfig,
    JoyConfig,
    JoyState,
    ModeConfig,
    PostureConfig,
    cross_section_function_check,
    map_joy,
    resolve_gait_cycle,
    validate_bindings,
)
from .teleop_arbitration import GAMEPAD, ArbitrationState, on_owner_msg, should_publish

PUBLISH_RATE_HZ = 50.0
TICK_DT_S = 1.0 / PUBLISH_RATE_HZ

# Engine states in which a gait switch may be published. STAND swaps
# immediately; the others latch a pending change that the engine
# commits via its pause-and-reseat sequence. The gait is locked during
# engaging / resuming, and a switch is meaningless during initialize /
# folding / folded. The empty pre-first-publish state stays refused
# for free.
_GAIT_SWITCH_STATES: frozenset[str] = frozenset(
    {"stand", "gait", "pausing", "paused", "reseating"}
)


def _parse_base(raw: dict, /) -> BaseConfig:
    base_raw = raw["base"]
    button_index = {str(k): int(v) for k, v in base_raw["buttons"].items()}
    axis_index = {str(k): int(v) for k, v in base_raw["axes"].items()}
    axis_sign = {
        str(k): float(v) for k, v in base_raw.get("axis_signs", {}).items()
    }
    bindings = {str(k): str(v) for k, v in base_raw["bindings"].items()}
    validate_bindings(
        "base",
        bindings,
        base_buttons=set(button_index),
        base_axes=set(axis_index),
        allowed_functions=BASE_FUNCTIONS,
    )
    return BaseConfig(
        deadband=float(base_raw["deadband"]),
        trigger_threshold=float(base_raw["trigger_threshold"]),
        button_index=button_index,
        axis_index=axis_index,
        axis_sign=axis_sign,
        bindings=bindings,
    )


def _parse_mode_bindings(
    section: str, raw_section: dict, base: BaseConfig
) -> dict[str, str]:
    bindings = {str(k): str(v) for k, v in raw_section["bindings"].items()}
    validate_bindings(
        section,
        bindings,
        base_buttons=set(base.button_index),
        base_axes=set(base.axis_index),
        allowed_functions=BUTTON_CLASS_FUNCTIONS | AXIS_CLASS_FUNCTIONS,
    )
    return bindings


def _load_config(
    path: Path, gait_yaml: Path, posture_yaml: Path
) -> tuple[JoyConfig, str, str, VelocityCaps, bool]:
    with path.open() as f:
        raw = yaml.safe_load(f)
    caps = load_velocity_caps(gait_yaml)
    animation_list = load_animation_mode_animations(posture_yaml)

    gait_cycle_raw = tuple(str(n) for n in raw["gait_cycle"])
    allow_unstable = bool(raw.get("allow_unstable_gaits", False))
    unstable_gaits = frozenset(
        name for name, factory in STRATEGIES.items() if factory().unstable
    )
    gait_cycle = resolve_gait_cycle(
        gait_cycle_raw, set(STRATEGIES), unstable_gaits, allow_unstable
    )
    default_gait = str(raw["default_gait"])
    if default_gait not in gait_cycle:
        detail = (
            "is excluded by allow_unstable_gaits: false"
            if default_gait in gait_cycle_raw
            else f"must be in gait_cycle={list(gait_cycle_raw)}"
        )
        raise ValueError(f"default_gait={default_gait!r} {detail}")

    base = _parse_base(raw)
    gait_bindings = _parse_mode_bindings("gait", raw["gait"], base)
    posture_raw = raw["posture"]
    posture_bindings = _parse_mode_bindings("posture", posture_raw, base)
    animation_bindings = _parse_mode_bindings(
        "animation", raw["animation"], base
    )
    cross_section_function_check({
        "gait": gait_bindings,
        "posture": posture_bindings,
        "animation": animation_bindings,
    })

    height = posture_raw["height"]
    posture_cfg = PostureConfig(
        bindings=posture_bindings,
        x_max=float(posture_raw["x_max"]),
        y_max=float(posture_raw["y_max"]),
        roll_max=math.radians(float(posture_raw["roll_max_deg"])),
        pitch_max=math.radians(float(posture_raw["pitch_max_deg"])),
        yaw_max=math.radians(float(posture_raw["yaw_max_deg"])),
        yaw_tau=float(posture_raw["yaw_tau_s"]),
        revert_tau=float(posture_raw["revert_tau_s"]),
        wiggle_pivot_forward_m=float(posture_raw["wiggle_pivot_forward_m"]),
        height_max=float(height["max_m"]),
        height_min=float(height["min_m"]),
        height_rate=float(height["rate_m_per_s"]),
    )

    cfg = JoyConfig(
        base=base,
        gait=ModeConfig(bindings=gait_bindings),
        posture=posture_cfg,
        animation=ModeConfig(bindings=animation_bindings),
        gait_cycle=gait_cycle,
        # Seed with the default gait's cap; the node swaps this in via
        # dataclasses.replace whenever a /cmd_gait publish lands.
        gait_linear_max=caps.linear_max(default_gait),
        gait_angular_z_max=caps.angular_max,
        animation_list=animation_list,
    )

    initial_mode = str(raw.get("initial_mode", POSTURE))
    if initial_mode not in (POSTURE, GAIT, ANIMATION):
        raise ValueError(
            f"initial_mode must be one of "
            f"{POSTURE!r}, {GAIT!r}, {ANIMATION!r}; got {initial_mode!r}"
        )
    arbitration_raw = raw.get("arbitration", {})
    arbitration_enabled = bool(arbitration_raw.get("enabled", True))
    return cfg, initial_mode, default_gait, caps, arbitration_enabled


class TeleopJoyNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_joy")

        default_cfg_path = (
            Path(get_package_share_directory("hexa_teleop"))
            / "config"
            / "teleop_joy.yaml"
        )
        gait_yaml_path = (
            Path(get_package_share_directory("hexa_gait"))
            / "config"
            / "gait.yaml"
        )
        posture_yaml_path = (
            Path(get_package_share_directory("hexa_posture"))
            / "config"
            / "posture.yaml"
        )
        self.declare_parameter("config_file", str(default_cfg_path))
        cfg_path = Path(
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        self._cfg, initial_mode, default_gait, self._caps, self._arbitration_enabled = _load_config(
            cfg_path, gait_yaml_path, posture_yaml_path
        )
        self._state = JoyState(
            mode=initial_mode,
            prev_gait_mode=False,
            prev_posture_mode=False,
            current_gait_idx=self._cfg.gait_cycle.index(default_gait),
        )
        # Most-recently-published-and-accepted gait. Stick scaling cap
        # in ``self._cfg.gait_linear_max`` is rebuilt on every change
        # so stick range tracks the gait's true capacity.
        self._active_gait: str = default_gait
        # Cached /gait/state for synchronous reads inside _tick. Empty
        # until gait_node publishes — refuse to switch in that window.
        self._latest_gait_state: str = ""

        self.get_logger().info(f"loaded teleop config from {cfg_path}")
        self.get_logger().info(
            f"gait rotation: {list(self._cfg.gait_cycle)}"
        )
        cap_summary = ", ".join(
            f"{n}={v:.2f}" for n, v in sorted(self._caps.linear_max_by_gait.items())
        )
        self.get_logger().info(
            f"velocity caps from {gait_yaml_path}: "
            f"linear_max=({cap_summary}) m/s, "
            f"angular_z_max={self._cfg.gait_angular_z_max:.2f} rad/s, "
            f"active gait={self._active_gait!r}"
        )
        self.get_logger().info(f"mode={self._state.mode}")

        self._latest_axes: tuple[float, ...] = ()
        self._latest_buttons: tuple[int, ...] = ()
        # Diagnostic state: one-shot length log + per-button edge log so
        # users can see which physical index a press actually fires at.
        self._joy_shape_logged = False
        self._last_buttons_for_log: tuple[int, ...] = ()

        self._sub_joy = self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self._sub_gait_state = self.create_subscription(
            String, "/gait/state", self._on_gait_state, 10
        )
        # Arbitration: when web teleop is running, /teleop/owner carries
        # the current owner ("gamepad" default, "web" when the webapp has
        # claimed control). TRANSIENT_LOCAL so a late-joining gamepad node
        # gets the last owner value. Dormant means we skip publishing but
        # still run map_joy to keep prev_* edge trackers fresh.
        self._arbitration = ArbitrationState()
        self._was_dormant = False
        owner_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sub_owner = self.create_subscription(
            String, "/teleop/owner", self._on_owner, owner_qos
        )
        self._pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_body_pose = self.create_publisher(BodyPoseMsg, "/body/pose", 10)
        # One-shot trigger on rising-edge of the init binding.
        # hexa_gait routes this to start_initialize (FOLDED → STAND) or
        # start_fold (STAND → FOLDED); a stray press elsewhere is a no-op.
        self._pub_init = self.create_publisher(Empty, "/gait/initialize", 10)
        # transient_local so a late-starting control node still picks
        # up the latest gait selection; depth 1 because the value
        # changes only on a user press.
        gait_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_cmd_gait = self.create_publisher(String, "/cmd_gait", gait_qos)
        # Animation-mode selection (``""`` = default stack, otherwise
        # the name of the selected animation). transient_local so a
        # late-starting posture node still sees the current selection.
        animation_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._pub_animation_mode = self.create_publisher(
            String, "/animation/mode", animation_qos
        )

        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

    def _on_joy(self, msg: Joy) -> None:
        self._latest_axes = tuple(msg.axes)
        self._latest_buttons = tuple(msg.buttons)

    def _on_gait_state(self, msg: String) -> None:
        self._latest_gait_state = msg.data

    def _on_owner(self, msg: String) -> None:
        prev = self._arbitration.owner
        on_owner_msg(self._arbitration, msg.data)
        if self._arbitration.owner != prev:
            self.get_logger().info(
                f"/teleop/owner: {prev!r} -> {self._arbitration.owner!r}"
            )

    def _tick(self) -> None:
        out = map_joy(
            self._latest_axes,
            self._latest_buttons,
            self._cfg,
            self._state,
            TICK_DT_S,
        )
        if out.mode_changed:
            self.get_logger().info(f"mode={self._state.mode}")
        # Arbitration: map_joy always runs (keeps prev_* edge trackers
        # fresh so no spurious edges on resume), but all publishes are
        # gated on ownership. When web owns, gamepad goes dormant.
        if self._arbitration_enabled and not should_publish(
            self._arbitration, GAMEPAD
        ):
            if not self._was_dormant:
                self._was_dormant = True
                self.get_logger().info("web teleop owns /cmd_vel — gamepad dormant")
            return
        if self._was_dormant:
            self._was_dormant = False
            self.get_logger().info("gamepad regained /cmd_vel ownership")
        if out.init_request:
            self.get_logger().info("start button pressed — publishing /gait/initialize")
            self._pub_init.publish(Empty())
        if out.animation_name is not None:
            self.get_logger().info(
                f"publishing /animation/mode={out.animation_name!r}"
            )
            self._pub_animation_mode.publish(String(data=out.animation_name))
        if out.gait_select is not None:
            # Gate on the engine states that accept a switch so a stale
            # request never sits on the wire. The JoyState index has
            # already advanced — the next press resumes from that slot
            # regardless.
            if self._latest_gait_state in _GAIT_SWITCH_STATES:
                self.get_logger().info(f"switching gait to {out.gait_select!r}")
                self._pub_cmd_gait.publish(String(data=out.gait_select))
                # Update the active cap so the next stick read scales
                # to the new gait's per-leg velocity ceiling. During a
                # mid-walk switch the cap leads the engine for the
                # length of its pause-and-reseat sequence — harmless,
                # the engine clamps stride internally.
                self._active_gait = out.gait_select
                new_cap = self._caps.linear_max(self._active_gait)
                self._cfg = dataclasses.replace(self._cfg, gait_linear_max=new_cap)
                self.get_logger().info(
                    f"stick linear_max={new_cap:.3f} m/s for gait "
                    f"{self._active_gait!r}"
                )
            else:
                self.get_logger().info(
                    f"gait switch to {out.gait_select!r} dropped — "
                    f"engine in {self._latest_gait_state!r} (gait locked)"
                )

        stamp = self.get_clock().now().to_msg()

        twist = Twist()
        twist.linear.x = out.linear_x
        twist.linear.y = out.linear_y
        twist.angular.z = out.angular_z
        self._pub_cmd_vel.publish(twist)

        pose = BodyPoseMsg()
        pose.header.stamp = stamp
        pose.x = out.pose_x
        pose.y = out.pose_y
        pose.z = out.pose_z
        pose.yaw = out.pose_yaw
        pose.roll = out.pose_roll
        pose.pitch = out.pose_pitch
        self._pub_body_pose.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopJoyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
