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

from hexa_common import (
    VelocityCaps,
    load_animation_mode_animations,
    load_body_height_offsets,
    load_posture_scalar_limits,
    load_velocity_caps,
    unit_stance_xy,
)
from hexa_common.gait_catalog import GAIT_DESCRIPTORS

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

# Matches the gamepad's polling rate (8BitDo and most modern pads: 250 Hz).
PUBLISH_RATE_HZ = 250.0
TICK_DT_S = 1.0 / PUBLISH_RATE_HZ

# Engine states in which a gait switch may be published. "folded" and
# "fault" swap the strategy the next stand comes up on, leg set included;
# "stand" swaps immediately; the others latch a pending change that the
# engine commits once it has settled back to a stand. The gait is locked
# during engaging, and a switch is meaningless mid-ladder (initialize /
# folding). The empty pre-first-publish state stays refused for free.
_GAIT_SWITCH_STATES: frozenset[str] = frozenset(
    {"folded", "fault", "stand", "gait", "settling", "reseating"}
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
    path: Path, gait_yaml: Path, posture_yaml: Path, geometry_yaml: Path
) -> tuple[JoyConfig, str, str, VelocityCaps, bool]:
    with path.open() as f:
        raw = yaml.safe_load(f)
    caps = load_velocity_caps(gait_yaml, geometry_yaml)
    animation_list = load_animation_mode_animations(posture_yaml)
    # Shared with the web teleop and the firmware, so tuning.yaml owns them.
    posture_limits = load_posture_scalar_limits(posture_yaml)
    # Not a teleop knob: the posture stack's own body-height envelope, as pose
    # offsets. Teleop uses it only to saturate the height integrator.
    height_min, height_max = load_body_height_offsets(gait_yaml, posture_yaml)

    gait_cycle_raw = tuple(str(n) for n in raw["gait_cycle"])
    allow_unstable = bool(raw.get("allow_unstable_gaits", False))
    unstable_gaits = frozenset(
        name for name, descriptor in GAIT_DESCRIPTORS.items() if descriptor.unstable
    )
    # The two rotations are partitioned by leg set: each one's validator is
    # handed the other set as foreign, so a quadruped gait in gait_cycle (or a
    # six-leg gait in the quadruped rotation) is a load-time error rather than a
    # cycler press the engine silently refuses.
    quadruped_gaits = frozenset(
        name
        for name, descriptor in GAIT_DESCRIPTORS.items()
        if descriptor.leg_set == "quadruped"
    )
    hexapod_gaits = frozenset(GAIT_DESCRIPTORS) - quadruped_gaits
    gait_cycle = resolve_gait_cycle(
        gait_cycle_raw,
        set(GAIT_DESCRIPTORS),
        unstable_gaits,
        allow_unstable,
        foreign_gaits=quadruped_gaits,
    )
    default_gait = str(raw["default_gait"])
    if default_gait not in gait_cycle:
        detail = (
            "is excluded by allow_unstable_gaits: false"
            if default_gait in gait_cycle_raw
            else f"must be in gait_cycle={list(gait_cycle_raw)}"
        )
        raise ValueError(f"default_gait={default_gait!r} {detail}")

    quad_cycle_raw = tuple(str(n) for n in raw["quadruped_gait_cycle"])
    quadruped_gait_cycle = resolve_gait_cycle(
        quad_cycle_raw,
        set(GAIT_DESCRIPTORS),
        unstable_gaits,
        allow_unstable,
        foreign_gaits=hexapod_gaits,
        key="quadruped_gait_cycle",
    )
    default_quadruped_gait = str(raw["default_quadruped_gait"])
    if default_quadruped_gait not in quadruped_gait_cycle:
        detail = (
            "is excluded by allow_unstable_gaits: false"
            if default_quadruped_gait in quad_cycle_raw
            else f"must be in quadruped_gait_cycle={list(quad_cycle_raw)}"
        )
        raise ValueError(
            f"default_quadruped_gait={default_quadruped_gait!r} {detail}"
        )

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
        # PostureScalarLimits carries PostureConfig's own field names.
        **dataclasses.asdict(posture_limits),
        height_max=height_max,
        height_min=height_min,
        height_rate=float(height["rate_m_per_s"]),
    )

    cfg = JoyConfig(
        base=base,
        gait=ModeConfig(bindings=gait_bindings),
        posture=posture_cfg,
        animation=ModeConfig(bindings=animation_bindings),
        gait_cycle=gait_cycle,
        quadruped_gait_cycle=quadruped_gait_cycle,
        default_quadruped_gait=default_quadruped_gait,
        # Seed with the default gait's caps; the node swaps these in via
        # dataclasses.replace whenever a /cmd_gait publish lands. Both are
        # per-gait — the angular cap is the linear one over the stance radius.
        gait_linear_max=caps.linear_max(default_gait),
        gait_angular_z_max=caps.angular_max(default_gait),
        stance_unit=unit_stance_xy(geometry_yaml, gait_yaml),
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
        # Velocity caps (gait_node block) and the animation-mode list
        # (posture_node block) both come from hexa_description's tuning.yaml —
        # the single source of truth the gait/posture nodes also read.
        description_config = (
            Path(get_package_share_directory("hexa_description")) / "config"
        )
        tuning_yaml_path = description_config / "tuning.yaml"
        # The angular stick cap is the linear one over the outermost foot's
        # standing radius, so the caps need the leg mounts as well.
        geometry_yaml_path = description_config / "geometry.yaml"
        self.declare_parameter("config_file", str(default_cfg_path))
        cfg_path = Path(
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        self._cfg, initial_mode, default_gait, self._caps, self._arbitration_enabled = _load_config(
            cfg_path, tuning_yaml_path, tuning_yaml_path, geometry_yaml_path
        )
        self._state = JoyState(
            mode=initial_mode,
            prev_gait_mode=False,
            prev_posture_mode=False,
            current_gait_idx=self._cfg.gait_cycle.index(default_gait),
            current_quadruped_gait_idx=self._cfg.quadruped_gait_cycle.index(
                self._cfg.default_quadruped_gait
            ),
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
            f"gait rotation: {list(self._cfg.gait_cycle)}; "
            f"quadruped rotation: {list(self._cfg.quadruped_gait_cycle)}"
        )
        cap_summary = ", ".join(
            f"{n}={v:.2f}" for n, v in sorted(self._caps.linear_max_by_gait.items())
        )
        angular_summary = ", ".join(
            f"{n}={v:.2f}" for n, v in sorted(self._caps.angular_max_by_gait.items())
        )
        self.get_logger().info(
            f"velocity caps from {tuning_yaml_path}: "
            f"linear_max=({cap_summary}) m/s, "
            f"angular_max=({angular_summary}) rad/s, "
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
        # TRANSIENT_LOCAL to match hexa_locomotion's latched publisher:
        # /gait/state publishes on change only, so a late-joining teleop
        # would otherwise wait for the next state change to learn the state.
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sub_gait_state = self.create_subscription(
            String, "/gait/state", self._on_gait_state, state_qos
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
        # One-shot trigger on a rising edge of either init binding (start
        # for six legs, select for four). hexa_locomotion routes it to
        # start_initialize (FOLDED → STAND, on the leg set the latched
        # /cmd_gait asks for) or to a fold request; a stray press
        # mid-ladder is a no-op.
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
                # Update the active caps so the next stick read scales
                # to the new gait's per-leg velocity ceiling. During a
                # mid-walk switch the cap leads the engine for as long
                # as it takes to settle — harmless, the engine clamps
                # stride internally.
                self._active_gait = out.gait_select
                new_linear = self._caps.linear_max(self._active_gait)
                new_angular = self._caps.angular_max(self._active_gait)
                self._cfg = dataclasses.replace(
                    self._cfg,
                    gait_linear_max=new_linear,
                    gait_angular_z_max=new_angular,
                )
                self.get_logger().info(
                    f"stick linear_max={new_linear:.3f} m/s, "
                    f"angular_max={new_angular:.3f} rad/s for gait "
                    f"{self._active_gait!r}"
                )
            else:
                self.get_logger().info(
                    f"gait switch to {out.gait_select!r} dropped — "
                    f"engine in {self._latest_gait_state!r} (gait locked)"
                )
        # After the gait publish, not before: an init request carries its leg
        # set as the gait it publishes, and hexa_locomotion reads the leg set
        # off the strategy that is applied by the time it starts the ladder.
        if out.init_request:
            which = "select" if out.init_quadruped else "start"
            self.get_logger().info(
                f"{which} button pressed — publishing /gait/initialize"
            )
            self._pub_init.publish(Empty())

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
