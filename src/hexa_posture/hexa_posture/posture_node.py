"""Posture controller node.

Subscribes to the user pose (`/body/pose`), the latest body velocity
command (`/cmd_vel`), and the gait engine state (`/gait/state`). On a
fixed timer, runs the animation stack with the current context, sums
in the user pose, clamps to the static safety envelope, and publishes
the result on `/body/pose_target` for the IK node to consume.

The gait state gates the whole stack: posture is only meaningful when
the legs are at (or transitioning around) the nominal stance
footprint — `stand`, `engaging`, `gait`, the pause trio (`pausing`,
`paused`, `resuming`), and `reseating`. In `folded`, `initialize`, or
`folding` the foot targets come from a separate ladder and composing
a body-pose offset onto them would be nonsense; the node emits
IDENTITY in those states regardless of user input.

The animation stack is built from the ``enabled_animations`` parameter
(string list of layer names, in order). Default is the standard
``("still", "breathing")``; the sim bringup launches with
``["still"]`` while locomotion is being tuned so the bob doesn't mask
gait-induced body motion.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from hexa_interfaces.msg import BodyPose as BodyPoseMsg
from hexa_interfaces.msg import GaitParams, LegTargets
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .animations import (
    Animation,
    AnimationContext,
    BodyRoll3D,
    Breathing,
    GaitBounce,
    GaitSway,
    HorizontalBodyRoll,
    Stack,
    Still,
    VerticalBodyRoll,
)
from .pose import IDENTITY, BodyPose, PoseLimits, add, clamp

PUBLISH_RATE_HZ = 50.0
CMD_VEL_ZERO_TOL = 1e-4

# Gait engine states in which body posture is meaningful. In the other
# states (`folded`, `initialize`, `folding`) the legs are not at the
# nominal standing footprint, so composing a body-pose offset would
# push the IK against an unrelated foot configuration — at best
# nonsense, at worst unsafe. Posture publishes IDENTITY in those
# states regardless of user input.
#
# `reseating` and the pause trio (`pausing`, `paused`, `resuming`) are
# included so the persistent body pose (especially height) continues
# to be applied while the gait engine soft-releases or walks the feet
# to a new nominal stance — pause should affect only the legs, not
# snap the body back to IDENTITY.
POSTURE_ACTIVE_STATES: frozenset[str] = frozenset(
    {"stand", "engaging", "gait", "pausing", "paused", "resuming", "reseating"}
)

DEFAULT_ANIMATIONS: tuple[str, ...] = ("still", "breathing")
# Each entry is the key the teleop layer publishes on /animation/mode. A
# comma-separated entry composes multiple animations into a single stack
# (e.g. ``"vertical_body_roll,gait_bounce"`` plays both at once).
DEFAULT_ANIMATION_MODE_ANIMATIONS: tuple[str, ...] = (
    "vertical_body_roll",
    "horizontal_body_roll",
    "body_roll_3d",
)
# Minimum stance legs needed to define a meaningful support polygon.
# During swing transitions the count can momentarily dip; we hold the
# previous centroid rather than emit a noisy value.
MIN_STANCE_FOR_CENTROID = 3
_ANIMATION_FACTORIES: dict[str, type[Animation]] = {
    "still": Still,
    "breathing": Breathing,
    "gait_sway": GaitSway,
    "gait_bounce": GaitBounce,
    "vertical_body_roll": VerticalBodyRoll,
    "horizontal_body_roll": HorizontalBodyRoll,
    "body_roll_3d": BodyRoll3D,
}


def _build_animation_stack(
    names: list[str],
    *,
    overrides: dict[str, Animation] | None = None,
) -> Stack:
    unknown = [n for n in names if n not in _ANIMATION_FACTORIES]
    if unknown:
        raise ValueError(
            f"unknown animation(s) {unknown!r}; "
            f"available: {sorted(_ANIMATION_FACTORIES)}"
        )
    overrides = overrides or {}
    layers = tuple(
        overrides[n] if n in overrides else _ANIMATION_FACTORIES[n]()
        for n in names
    )
    return Stack(layers=layers)


def _twist_is_zero(t: Twist) -> bool:
    return (
        abs(t.linear.x) < CMD_VEL_ZERO_TOL
        and abs(t.linear.y) < CMD_VEL_ZERO_TOL
        and abs(t.linear.z) < CMD_VEL_ZERO_TOL
        and abs(t.angular.x) < CMD_VEL_ZERO_TOL
        and abs(t.angular.y) < CMD_VEL_ZERO_TOL
        and abs(t.angular.z) < CMD_VEL_ZERO_TOL
    )


def _msg_to_pose(m: BodyPoseMsg) -> BodyPose:
    return BodyPose(x=m.x, y=m.y, z=m.z, roll=m.roll, pitch=m.pitch, yaw=m.yaw)


def _lpf_step_xy(
    prev: tuple[float, float] | None,
    raw: tuple[float, float] | None,
    tau: float,
    dt: float,
) -> tuple[float, float] | None:
    """One first-order low-pass step on an XY signal.

    Pulled out as a free function so the filter math is unit-testable
    without spinning a ROS node. ``prev=None`` seeds the filter from
    ``raw`` (no startup transient from (0, 0)); ``raw=None`` holds the
    previous output (mid-swing-transition behaviour).
    """
    if raw is None:
        return prev
    if prev is None:
        return raw
    denom = tau + dt
    alpha = dt / denom if denom > 0.0 else 1.0
    px, py = prev
    rx, ry = raw
    return (px + alpha * (rx - px), py + alpha * (ry - py))


def _lpf_step_scalar(
    prev: float | None,
    raw: float | None,
    tau: float,
    dt: float,
) -> float | None:
    """One first-order low-pass step on a scalar signal.

    Same seeding and hold-on-None semantics as ``_lpf_step_xy`` —
    ``prev=None`` adopts ``raw`` so there's no startup ramp from 0,
    ``raw=None`` holds the previous value across degenerate frames.
    """
    if raw is None:
        return prev
    if prev is None:
        return raw
    denom = tau + dt
    alpha = dt / denom if denom > 0.0 else 1.0
    return prev + alpha * (raw - prev)


def _stance_centroid_xy(msg: LegTargets) -> tuple[float, float] | None:
    """Mean of foot_target.{x,y} over legs flagged stance=True.

    Returns ``None`` when fewer than ``MIN_STANCE_FOR_CENTROID`` legs
    are in stance — the polygon is degenerate during swing transitions
    and a noisy centroid would re-excite the rocking mode we are trying
    to suppress.
    """
    xs: list[float] = []
    ys: list[float] = []
    for leg in msg.legs:
        if leg.stance:
            xs.append(leg.foot_target.x)
            ys.append(leg.foot_target.y)
    if len(xs) < MIN_STANCE_FOR_CENTROID:
        return None
    n = float(len(xs))
    return (sum(xs) / n, sum(ys) / n)


def _max_swing_lift_z(msg: LegTargets) -> float | None:
    """Max foot lift (m) above the stance polygon's mean Z.

    Computed as ``max(foot.z for swing legs) − mean(foot.z for stance
    legs)``, clamped ``≥ 0``. Returns ``None`` when the stance polygon
    is degenerate (``< MIN_STANCE_FOR_CENTROID`` legs in stance) so the
    caller can hold the previous value rather than emit a noisy zero
    mid-transition.

    When no leg is in swing the result is ``0.0`` — semantically
    distinct from ``None`` (the signal is observed and quiet, not
    missing). ``GaitBounce`` treats that as the body's resting
    altitude.
    """
    stance_zs: list[float] = []
    swing_zs: list[float] = []
    for leg in msg.legs:
        if leg.stance:
            stance_zs.append(leg.foot_target.z)
        else:
            swing_zs.append(leg.foot_target.z)
    if len(stance_zs) < MIN_STANCE_FOR_CENTROID:
        return None
    if not swing_zs:
        return 0.0
    ground = sum(stance_zs) / len(stance_zs)
    lift = max(swing_zs) - ground
    return lift if lift > 0.0 else 0.0


def _pose_to_msg(p: BodyPose, now_msg) -> BodyPoseMsg:
    out = BodyPoseMsg()
    out.header.stamp = now_msg
    # frame_id intentionally left blank: the pose is an offset in the
    # body frame, not a transform into a named TF frame. Setting a
    # frame_id would invite the wrong consumer assumption.
    out.x = p.x
    out.y = p.y
    out.z = p.z
    out.roll = p.roll
    out.pitch = p.pitch
    out.yaw = p.yaw
    return out


class PostureNode(Node):
    def __init__(self) -> None:
        super().__init__("posture_node")

        self._user_pose: BodyPose = IDENTITY
        self._walking: bool = False
        # Cold-start default: until /gait/state arrives the engine could
        # still be in FOLDED, so play it safe and emit IDENTITY.
        self._gait_state: str | None = None
        # Active gait strategy name from /gait/params. Held None until
        # the first params message lands; animations that gate on a
        # specific gait (e.g. GaitBounce → tripod-only) treat that as
        # "unknown" and stay silent.
        self._gait_name: str | None = None
        # Filtered support-polygon centroid in body-frame XY. Held at
        # None until the first /legs/targets arrives so GaitSway (and
        # any other consumer) can tell "no data yet" from "centred".
        self._support_centroid_xy: tuple[float, float] | None = None
        self._latest_raw_centroid: tuple[float, float] | None = None
        # Filtered max foot-lift signal (m). None until /legs/targets
        # is seen with a usable stance polygon; held through
        # degenerate transitions so GaitBounce sees a continuous
        # signal. Filtering smooths the slope kink at swing-leg
        # handover in overlapping gaits (ripple, surf) where the
        # max-across-legs switches between two arcs.
        self._swing_lift_z: float | None = None
        self._latest_raw_swing_lift: float | None = None
        # Master gait phase in [0, 1), sniffed from /legs/targets.
        # Held None until the first targets message arrives so
        # phase-locked animations can stay silent on cold start.
        self._master_phase: float | None = None
        self._last_tick_ns: int | None = None

        self.declare_parameter("enabled_animations", list(DEFAULT_ANIMATIONS))
        self.declare_parameter(
            "animation_mode_animations",
            list(DEFAULT_ANIMATION_MODE_ANIMATIONS),
        )
        self.declare_parameter("gait_sway_gain", 1.0)
        self.declare_parameter("gait_sway_strength", 0.5)
        self.declare_parameter("gait_bounce_arc_height", 0.02)
        self.declare_parameter("gait_bounce_step_height_ref", 0.06)
        self.declare_parameter("vertical_body_roll_z_amplitude", 0.02)
        self.declare_parameter("vertical_body_roll_pitch_amplitude_deg", 10.0)
        self.declare_parameter("vertical_body_roll_phase_offset", 0.0)
        self.declare_parameter("horizontal_body_roll_y_amplitude", 0.02)
        self.declare_parameter("horizontal_body_roll_yaw_amplitude_deg", 10.0)
        self.declare_parameter("horizontal_body_roll_phase_offset", 0.0)
        self.declare_parameter("body_roll_3d_z_amplitude", 0.02)
        self.declare_parameter("body_roll_3d_pitch_amplitude_deg", 10.0)
        self.declare_parameter("body_roll_3d_y_amplitude", 0.02)
        self.declare_parameter("body_roll_3d_yaw_amplitude_deg", 10.0)
        self.declare_parameter("body_roll_3d_horizontal_phase_offset", 0.25)
        self.declare_parameter("body_roll_3d_pitch_phase_offset", 0.0)
        self.declare_parameter("body_roll_3d_yaw_phase_offset", 0.0)
        self.declare_parameter("support_centroid_tau", 0.1)
        self.declare_parameter("swing_lift_tau", 0.04)
        enabled = list(
            self.get_parameter("enabled_animations")
            .get_parameter_value()
            .string_array_value
        ) or list(DEFAULT_ANIMATIONS)
        animation_mode_names = list(
            self.get_parameter("animation_mode_animations")
            .get_parameter_value()
            .string_array_value
        ) or list(DEFAULT_ANIMATION_MODE_ANIMATIONS)
        gait_sway_gain = (
            self.get_parameter("gait_sway_gain").get_parameter_value().double_value
        )
        gait_sway_strength = (
            self.get_parameter("gait_sway_strength")
            .get_parameter_value()
            .double_value
        )
        gait_bounce_arc_height = (
            self.get_parameter("gait_bounce_arc_height")
            .get_parameter_value()
            .double_value
        )
        gait_bounce_step_height_ref = (
            self.get_parameter("gait_bounce_step_height_ref")
            .get_parameter_value()
            .double_value
        )
        vbr_z = (
            self.get_parameter("vertical_body_roll_z_amplitude")
            .get_parameter_value()
            .double_value
        )
        vbr_pitch_rad = math.radians(
            self.get_parameter("vertical_body_roll_pitch_amplitude_deg")
            .get_parameter_value()
            .double_value
        )
        vbr_phase = (
            self.get_parameter("vertical_body_roll_phase_offset")
            .get_parameter_value()
            .double_value
        )
        hbr_y = (
            self.get_parameter("horizontal_body_roll_y_amplitude")
            .get_parameter_value()
            .double_value
        )
        hbr_yaw_rad = math.radians(
            self.get_parameter("horizontal_body_roll_yaw_amplitude_deg")
            .get_parameter_value()
            .double_value
        )
        hbr_phase = (
            self.get_parameter("horizontal_body_roll_phase_offset")
            .get_parameter_value()
            .double_value
        )
        br3d_z = (
            self.get_parameter("body_roll_3d_z_amplitude")
            .get_parameter_value()
            .double_value
        )
        br3d_pitch_rad = math.radians(
            self.get_parameter("body_roll_3d_pitch_amplitude_deg")
            .get_parameter_value()
            .double_value
        )
        br3d_y = (
            self.get_parameter("body_roll_3d_y_amplitude")
            .get_parameter_value()
            .double_value
        )
        br3d_yaw_rad = math.radians(
            self.get_parameter("body_roll_3d_yaw_amplitude_deg")
            .get_parameter_value()
            .double_value
        )
        br3d_h_phase = (
            self.get_parameter("body_roll_3d_horizontal_phase_offset")
            .get_parameter_value()
            .double_value
        )
        br3d_pitch_phase = (
            self.get_parameter("body_roll_3d_pitch_phase_offset")
            .get_parameter_value()
            .double_value
        )
        br3d_yaw_phase = (
            self.get_parameter("body_roll_3d_yaw_phase_offset")
            .get_parameter_value()
            .double_value
        )
        self._centroid_tau = (
            self.get_parameter("support_centroid_tau")
            .get_parameter_value()
            .double_value
        )
        self._swing_lift_tau = (
            self.get_parameter("swing_lift_tau")
            .get_parameter_value()
            .double_value
        )
        overrides: dict[str, Animation] = {
            "gait_sway": GaitSway(
                gain=gait_sway_gain, strength=gait_sway_strength
            ),
            "gait_bounce": GaitBounce(
                arc_height=gait_bounce_arc_height,
                step_height_ref=gait_bounce_step_height_ref,
            ),
            "vertical_body_roll": VerticalBodyRoll(
                z_amplitude=vbr_z,
                pitch_amplitude=vbr_pitch_rad,
                pitch_phase_offset=vbr_phase,
            ),
            "horizontal_body_roll": HorizontalBodyRoll(
                y_amplitude=hbr_y,
                yaw_amplitude=hbr_yaw_rad,
                yaw_phase_offset=hbr_phase,
            ),
            "body_roll_3d": BodyRoll3D(
                z_amplitude=br3d_z,
                pitch_amplitude=br3d_pitch_rad,
                y_amplitude=br3d_y,
                yaw_amplitude=br3d_yaw_rad,
                horizontal_phase_offset=br3d_h_phase,
                pitch_phase_offset=br3d_pitch_phase,
                yaw_phase_offset=br3d_yaw_phase,
            ),
        }
        self._default_stack = _build_animation_stack(enabled, overrides=overrides)
        # Per-animation stacks for ANIMATION mode: each entry yields a
        # dedicated stack containing only ``still`` + the named
        # animation(s), so gait_sway/gait_bounce do not bleed in while
        # the user is demoing a body animation. Comma-separated entries
        # compose multiple animations into one stack.
        self._animation_stacks: dict[str, Stack] = {
            name: _build_animation_stack(
                ["still", *(n.strip() for n in name.split(","))],
                overrides=overrides,
            )
            for name in animation_mode_names
        }
        # Active animation-mode selection. Empty string means the
        # default stack is in use; otherwise the value names an entry in
        # ``_animation_stacks``.
        self._animation_mode: str = ""
        self.get_logger().info(f"animations enabled: {enabled}")
        self.get_logger().info(
            f"animation-mode animations available: {animation_mode_names}"
        )
        self._limits = PoseLimits()

        self._sub_pose = self.create_subscription(
            BodyPoseMsg, "/body/pose", self._on_pose, 10
        )
        self._sub_vel = self.create_subscription(
            Twist, "/cmd_vel", self._on_vel, 10
        )
        self._sub_gait_state = self.create_subscription(
            String, "/gait/state", self._on_gait_state, 10
        )
        self._sub_gait_params = self.create_subscription(
            GaitParams, "/gait/params", self._on_gait_params, 10
        )
        self._sub_targets = self.create_subscription(
            LegTargets, "/legs/targets", self._on_leg_targets, 10
        )
        # transient_local so the posture node always picks up the latest
        # animation-mode selection from teleop even if it starts after
        # the user has already picked one.
        animation_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._sub_animation_mode = self.create_subscription(
            String, "/animation/mode", self._on_animation_mode, animation_qos
        )
        self._pub_target = self.create_publisher(BodyPoseMsg, "/body/pose_target", 10)

        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

    def _on_pose(self, msg: BodyPoseMsg) -> None:
        self._user_pose = _msg_to_pose(msg)

    def _on_vel(self, msg: Twist) -> None:
        self._walking = not _twist_is_zero(msg)

    def _on_gait_state(self, msg: String) -> None:
        self._gait_state = msg.data

    def _on_animation_mode(self, msg: String) -> None:
        new_mode = msg.data
        if new_mode and new_mode not in self._animation_stacks:
            self.get_logger().warn(
                f"unknown animation mode {new_mode!r}; "
                f"available: {sorted(self._animation_stacks)}"
            )
            return
        if new_mode == self._animation_mode:
            return
        self._animation_mode = new_mode
        if new_mode:
            self.get_logger().info(f"animation mode active: {new_mode}")
        else:
            self.get_logger().info("animation mode cleared")

    def _on_gait_params(self, msg: GaitParams) -> None:
        # Only the name matters here — velocity fields belong to the
        # gait chain. Empty string means "unset"; keep the previous
        # value so a stray default-constructed message doesn't blank
        # the gate.
        if msg.gait_name:
            self._gait_name = msg.gait_name

    def _on_leg_targets(self, msg: LegTargets) -> None:
        raw = _stance_centroid_xy(msg)
        if raw is not None:
            self._latest_raw_centroid = raw
        lift = _max_swing_lift_z(msg)
        if lift is not None:
            self._latest_raw_swing_lift = lift
        self._master_phase = float(msg.master_phase) % 1.0

    def _step_filters(self, dt: float) -> None:
        """Advance the first-order low-passes toward their latest raw
        samples. Called from ``_tick`` so the time step is driven by
        the node's actual cadence, not the /legs/targets rate. Both
        filters hold the previous value when the underlying frame is
        degenerate (None), so consumers never see a transient zero
        mid-handover."""
        self._support_centroid_xy = _lpf_step_xy(
            self._support_centroid_xy,
            self._latest_raw_centroid,
            self._centroid_tau,
            dt,
        )
        self._swing_lift_z = _lpf_step_scalar(
            self._swing_lift_z,
            self._latest_raw_swing_lift,
            self._swing_lift_tau,
            dt,
        )

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self._last_tick_ns is None:
            dt = 1.0 / PUBLISH_RATE_HZ
        else:
            dt = max((now_ns - self._last_tick_ns) * 1e-9, 0.0)
        self._last_tick_ns = now_ns
        self._step_filters(dt)

        if self._gait_state not in POSTURE_ACTIVE_STATES:
            # Engine is FOLDED / INITIALIZE / FOLDING (or no state seen
            # yet): the legs aren't at nominal stance, so applying any
            # body-pose offset would compose against the wrong foot
            # configuration. Hold IDENTITY until the engine reports a
            # state in which posture is meaningful.
            self._pub_target.publish(_pose_to_msg(IDENTITY, now.to_msg()))
            return
        t = now_ns * 1e-9
        ctx = AnimationContext(
            t=t,
            walking=self._walking,
            gait_name=self._gait_name,
            support_centroid_xy=self._support_centroid_xy,
            swing_lift_z=self._swing_lift_z,
            master_phase=self._master_phase,
        )
        stack = (
            self._animation_stacks[self._animation_mode]
            if self._animation_mode
            else self._default_stack
        )
        animated = stack(ctx)
        target = clamp(add(self._user_pose, animated), self._limits)
        self._pub_target.publish(_pose_to_msg(target, now.to_msg()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PostureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
