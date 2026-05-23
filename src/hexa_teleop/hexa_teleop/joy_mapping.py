"""Pure mapping from a sensor_msgs/Joy snapshot to high-level commands.

The teleop node is the ROS glue around this module. The functions here
take plain sequences of axes and buttons (no rclpy types) so the logic
is unit-testable without spinning a ROS context.

The config is keybind-driven: each YAML section enumerates a controller's
physical keys (``a``, ``l1``, ``dpad_up``, ``left_stick_x``, …) and
assigns each one a *function* name (``gait_mode``, ``yaw_left``,
``tilt_roll``, …) or the empty string for unbound. The mapping code
asks "which key is bound to function X?" via ``BaseConfig`` /
``ModeConfig`` lookups; the controller-specific button / axis indices
and sign conventions live in ``BaseConfig`` so a different controller
needs only the ``base.buttons`` / ``base.axes`` / ``base.axis_signs``
block edited.

Mode model:

  * ``posture`` — right stick translates the body in the x-y plane
    (``pose_x`` / ``pose_y``), the left stick tilts the body toward
    the direction it is pushed (``tilt_pitch`` forward / ``tilt_roll``
    sideways), and the L1/R1 shoulder buttons yaw the body about +z
    (``yaw_left`` / ``yaw_right``). All three inputs apply together.
    ``/cmd_vel`` is zero so ``hexa_posture`` stays in pose mode.
  * ``gait`` — left stick X is spin rate (``drive_yaw``), right stick
    is linear velocity (``drive_x`` / ``drive_y``). ``/body/pose`` is
    zero so any standing translation/yaw decays back to the nominal
    stance.
  * ``animation`` — joysticks behave as in GAIT (drive /cmd_vel) but
    the posture stack is swapped for a phase-locked body animation.
    Discrete animation selectors are bound to D-pad directions by
    default (``animation_vertical_body_roll`` /
    ``animation_horizontal_body_roll`` / ``animation_body_roll_3d``).
    Tripod is forced on entry; gait-cycling and height integration are
    suppressed in this mode. Entered/exited via the ``animation_mode``
    binding (B button by default).

Mode selection uses dedicated base bindings: a rising edge on the key
bound to ``gait_mode`` selects GAIT; same for ``posture_mode``;
``animation_mode`` toggles between GAIT and ANIMATION. Pressing the
button for the mode that is already active is a no-op (``mode_changed``
stays false). Holding does not re-fire.

Axis sign convention follows REP-103 body frame: stick forward / left /
dpad-up is +1 after applying ``base.axis_signs``. Per-axis signs let a
driver that reports the opposite be normalised in YAML.

The yaw shoulder buttons are binary, so a press would snap the body
to its limit. The yaw output goes through a first-order low-pass:
``alpha = 1 - exp(-dt / posture.yaw_tau)``. The state lives on
``JoyState.yaw_current`` so it persists across calls.

L2/R2 (or whatever ``wiggle_left`` / ``wiggle_right`` is bound to)
trigger a "wiggle": shared yaw target with L1/R1 plus a body
translation that holds a configurable point a set distance forward
of body centre stationary. Translation magnitude per tick:

    pose_x_wiggle = wiggle_amount * px * (1 - cos(yaw_current))
    pose_y_wiggle = -wiggle_amount * px * sin(yaw_current)

The wiggle binding is polymorphic: if it resolves to an analog axis
(an Xbox-style trigger in ``base.axes``), the axis is thresholded
against ``base.trigger_threshold``; if it resolves to a button, the
button state is used directly. Same dispatch for every button-class
function — bind ``wiggle_left`` to a face button on a controller
without triggers and it just works.

**D-pad as virtual buttons**: ``dpad_up`` / ``dpad_down`` /
``dpad_left`` / ``dpad_right`` are exposed as bindable keys. Their
state is derived from the bound ``dpad_x`` / ``dpad_y`` axis (sign
normalised), thresholded at ±0.5. So binding ``dpad_up: height_up``
and ``dpad_down: height_down`` integrates body height while the D-pad
is held — and binding the same functions to ``l1`` / ``l2`` lets the
shoulder buttons do it instead.

A rising-edge press of the **record** binding (Select) in posture
mode folds the current live posture input into a persistent baseline
on ``JoyState`` (the six ``recorded_*`` fields), then zeros the
integrated ``height_current`` and eased ``yaw_current`` so the live
state can't double-count on the next tick. The output for each
posture axis is ``clamp(recorded + live, ±axis_max)``, so re-pushing
a stick that's already at its limit has no further effect. The
baseline bleeds through into gait mode like the D-pad height does
(the robot walks at the recorded posture).

The **init** binding (Start) extends today's two-press semantics over
the recorded baseline as well: if any posture state is non-default,
the first press arms a smooth revert (``state.reverting``) instead of
snapping to zero. Each subsequent tick decays ``height_current`` and
the six ``recorded_*`` toward zero with the ``posture.revert_tau``
time constant; ``yaw_current`` rides the existing yaw low-pass back
to zero on its own. ``init_request`` is suppressed during the revert;
the next press at the now-default state fires init as usual. A
``record`` press mid-revert cancels the revert.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Sequence

POSTURE = "posture"
GAIT = "gait"
ANIMATION = "animation"

# Function namespace. The loader validates every YAML binding value
# against these sets; runtime helpers trust well-formed configs.
BASE_FUNCTIONS: frozenset[str] = frozenset({
    "gait_mode",
    "posture_mode",
    "animation_mode",
    "init",
    "record",
})
BUTTON_CLASS_FUNCTIONS: frozenset[str] = frozenset({
    "yaw_left",
    "yaw_right",
    "wiggle_left",
    "wiggle_right",
    "height_up",
    "height_down",
    "gait_prev",
    "gait_next",
    "animation_vertical_body_roll",
    "animation_horizontal_body_roll",
    "animation_body_roll_3d",
})
AXIS_CLASS_FUNCTIONS: frozenset[str] = frozenset({
    "drive_x",
    "drive_y",
    "drive_yaw",
    "pose_x",
    "pose_y",
    "tilt_roll",
    "tilt_pitch",
})
ALL_FUNCTIONS: frozenset[str] = (
    BASE_FUNCTIONS | BUTTON_CLASS_FUNCTIONS | AXIS_CLASS_FUNCTIONS
)

# Animation selectors fire a /animation/mode publish on rising edge.
ANIMATION_SELECT_FUNCTIONS: tuple[str, ...] = (
    "animation_vertical_body_roll",
    "animation_horizontal_body_roll",
    "animation_body_roll_3d",
)
_ANIMATION_FUNCTION_TO_NAME: dict[str, str] = {
    "animation_vertical_body_roll": "vertical_body_roll",
    "animation_horizontal_body_roll": "horizontal_body_roll",
    "animation_body_roll_3d": "body_roll_3d",
}

# Virtual D-pad direction keys. Maps the bindable key name to the
# physical axis name in ``base.axes`` and the sign (after sign
# normalisation) that counts as "pressed" for that direction.
DPAD_DIRECTIONS: dict[str, tuple[str, int]] = {
    "dpad_up": ("dpad_y", +1),
    "dpad_down": ("dpad_y", -1),
    "dpad_left": ("dpad_x", -1),
    "dpad_right": ("dpad_x", +1),
}


@dataclass(frozen=True)
class BaseConfig:
    deadband: float
    trigger_threshold: float
    # Controller hardware layout: physical key name -> Joy.{buttons,axes}
    # index. Edit these blocks to support a different controller.
    button_index: Mapping[str, int]
    axis_index: Mapping[str, int]
    # Per-axis sign so a driver that reports the opposite direction can
    # be normalised to "+x forward, +y left, dpad-up = +1". Missing
    # entries default to +1.0.
    axis_sign: Mapping[str, float]
    # Mode-agnostic key bindings (mode-select buttons, init, record).
    # key name -> function name (or "" for unbound).
    bindings: Mapping[str, str]


@dataclass(frozen=True)
class ModeConfig:
    """Per-mode bindings: physical key name -> function name."""

    bindings: Mapping[str, str]


@dataclass(frozen=True)
class PostureConfig:
    """Posture-mode bindings + the scalar limits the mode needs."""

    bindings: Mapping[str, str]
    x_max: float
    y_max: float
    roll_max: float
    pitch_max: float
    yaw_max: float
    yaw_tau: float
    revert_tau: float
    wiggle_pivot_forward_m: float
    height_max: float
    height_min: float
    height_rate: float


@dataclass(frozen=True)
class JoyConfig:
    base: BaseConfig
    gait: ModeConfig
    posture: PostureConfig
    animation: ModeConfig
    # Ordered list of gait names the cycler walks through. Index
    # ``current_gait_idx`` on ``JoyState`` tracks the user's selection.
    gait_cycle: tuple[str, ...]
    # Per-gait stick scaling. Updated at runtime from /cmd_gait via
    # ``dataclasses.replace`` whenever the active gait changes.
    gait_linear_max: float
    gait_angular_z_max: float


@dataclass
class JoyState:
    mode: str = POSTURE
    prev_gait_mode: bool = False
    prev_posture_mode: bool = False
    prev_animation_mode: bool = False
    prev_init: bool = False
    prev_record: bool = False
    yaw_current: float = 0.0
    wiggle_amount: float = 0.0
    # Persistent body-height offset, driven by ``height_up`` /
    # ``height_down`` in POSTURE mode. Unlike every other posture axis
    # this value survives release and a mode toggle into GAIT (the
    # robot walks at the lifted/lowered posture).
    height_current: float = 0.0
    # Persistent posture baseline captured by a rising-edge ``record``
    # press. Each component is bounded by its ``posture.*_max`` at
    # record time. Bleeds through into GAIT mode. Reset by ``init``
    # when any of it is non-default.
    recorded_x: float = 0.0
    recorded_y: float = 0.0
    recorded_z: float = 0.0
    recorded_roll: float = 0.0
    recorded_pitch: float = 0.0
    recorded_yaw: float = 0.0
    # True while an ``init``-triggered revert to default posture is in
    # progress.
    reverting: bool = False
    # Rising-edge trackers for the gait cycler. Per-function so the
    # cycler works whether bound to a D-pad axis or to two separate
    # buttons.
    prev_gait_prev: bool = False
    prev_gait_next: bool = False
    # The ROS layer seeds ``current_gait_idx`` from the control-node
    # default at startup and on every accepted publish.
    current_gait_idx: int = 0
    # Per-function rising-edge tracker for animation selectors.
    prev_animation_select: dict[str, bool] = field(default_factory=dict)
    # Active animation-mode selection. ``""`` when ANIMATION mode is
    # not in effect; otherwise the name of the selected animation.
    animation_name: str = ""


@dataclass(frozen=True)
class JoyOutput:
    linear_x: float
    linear_y: float
    angular_z: float
    pose_x: float
    pose_y: float
    pose_z: float
    pose_yaw: float
    pose_roll: float
    pose_pitch: float
    mode_changed: bool
    init_request: bool
    # Populated with the freshly-cycled gait name on a ``gait_prev`` /
    # ``gait_next`` rising edge; ``None`` on every other tick. The
    # mapping does NOT gate on engine state (POSTURE/GAIT,
    # STAND/walking) — that lives in the ROS layer.
    gait_select: str | None = None
    # Populated with the desired ``/animation/mode`` value on the tick
    # the selection changes; ``None`` on every other tick.
    animation_name: str | None = None


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return value


def _mode_cfg(cfg: JoyConfig, mode: str) -> ModeConfig | PostureConfig:
    if mode == GAIT:
        return cfg.gait
    if mode == POSTURE:
        return cfg.posture
    if mode == ANIMATION:
        return cfg.animation
    # Fallback to gait — should not happen since the loader validates
    # ``initial_mode`` and the mapping only ever assigns to known
    # constants.
    return cfg.gait


def _resolve_function_key(
    function: str,
    base: BaseConfig,
    mode_cfg: ModeConfig | PostureConfig,
) -> str | None:
    """Return the physical key bound to ``function``, or ``None`` if unbound.

    Search order is mode-cfg first, then base. The loader allows the
    same function to appear in multiple sections only when every
    binding resolves to the same key, so the order here doesn't change
    the result for well-formed configs.
    """
    for key, fn in mode_cfg.bindings.items():
        if fn == function:
            return key
    for key, fn in base.bindings.items():
        if fn == function:
            return key
    return None


def _read_button_idx(buttons: Sequence[int], idx: int) -> bool:
    if idx < 0 or idx >= len(buttons):
        return False
    return bool(buttons[idx])


def _read_axis_idx(axes: Sequence[float], idx: int) -> float:
    if idx < 0 or idx >= len(axes):
        return 0.0
    return float(axes[idx])


def _dpad_pressed(
    virtual_key: str,
    base: BaseConfig,
    axes: Sequence[float],
) -> bool:
    """Return True if the D-pad direction ``virtual_key`` is held.

    Reads the bound ``dpad_x`` / ``dpad_y`` axis from ``base.axes``,
    applies its sign, and thresholds at ±0.5 so a bouncy axis can't
    double-count.
    """
    axis_name, side = DPAD_DIRECTIONS[virtual_key]
    if axis_name not in base.axis_index:
        return False
    sign = base.axis_sign.get(axis_name, 1.0)
    value = sign * _read_axis_idx(axes, base.axis_index[axis_name])
    if side > 0:
        return value > 0.5
    return value < -0.5


def button_pressed_for(
    function: str,
    base: BaseConfig,
    mode_cfg: ModeConfig | PostureConfig,
    buttons: Sequence[int],
    axes: Sequence[float],
) -> bool:
    """Press-state of the key bound to ``function``.

    Polymorphic across binding kinds:
      * physical button in ``base.buttons`` — direct read.
      * virtual D-pad direction (``dpad_up`` / ``dpad_down`` / …) —
        derived from the bound axis with sign normalisation.
      * analog axis (e.g. an Xbox-style trigger in ``base.axes``) —
        thresholded against ``base.trigger_threshold`` with the
        joy_node convention (released = +1.0, pressed = -1.0; so
        ``value < threshold`` reads as pressed).

    Returns False if unbound or the key is unknown.
    """
    key = _resolve_function_key(function, base, mode_cfg)
    if key is None:
        return False
    if key in base.button_index:
        return _read_button_idx(buttons, base.button_index[key])
    if key in DPAD_DIRECTIONS:
        return _dpad_pressed(key, base, axes)
    if key in base.axis_index:
        idx = base.axis_index[key]
        if idx < 0 or idx >= len(axes):
            # Out-of-range trigger reads as "released" (joy_node
            # convention: released = +1.0).
            return False
        return float(axes[idx]) < base.trigger_threshold
    return False


def axis_value_for(
    function: str,
    base: BaseConfig,
    mode_cfg: ModeConfig | PostureConfig,
    axes: Sequence[float],
) -> float:
    """Sign-normalised, deadband-applied value of the axis bound to ``function``.

    Returns 0.0 if unbound, bound to a non-axis key, or the index is
    out of range.
    """
    key = _resolve_function_key(function, base, mode_cfg)
    if key is None or key not in base.axis_index:
        return 0.0
    idx = base.axis_index[key]
    raw = _read_axis_idx(axes, idx)
    sign = base.axis_sign.get(key, 1.0)
    return apply_deadband(sign * raw, base.deadband)


def _clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def validate_bindings(
    section: str,
    bindings: Mapping[str, str],
    base_buttons: set[str],
    base_axes: set[str],
    allowed_functions: frozenset[str],
) -> None:
    """Validate one section's bindings dict.

    Checks:
      * every key is a known physical key (in base.buttons / base.axes)
        or a virtual D-pad direction;
      * every value is "" or a function name in ``allowed_functions``;
      * axis-class functions are bound only to stick axes (not buttons
        or D-pad directions);
      * button-class functions are bound to buttons, D-pad directions,
        or analog triggers (``l2`` / ``r2``) — not stick axes;
      * no function is bound to two different keys inside the section.

    Raises ``ValueError`` on the first violation.
    """
    known_keys = base_buttons | base_axes | set(DPAD_DIRECTIONS)
    seen_fn: dict[str, str] = {}
    for key, fn in bindings.items():
        if key not in known_keys:
            raise ValueError(
                f"{section}.bindings: unknown key {key!r} "
                f"(not in base.buttons, base.axes, or dpad directions)"
            )
        if fn == "":
            continue
        if fn not in allowed_functions:
            raise ValueError(
                f"{section}.bindings.{key}: unknown function {fn!r}"
            )
        if fn in AXIS_CLASS_FUNCTIONS:
            if key not in base_axes:
                raise ValueError(
                    f"{section}.bindings.{key}: axis-class function {fn!r} "
                    f"bound to non-axis key {key!r}"
                )
            if key in DPAD_DIRECTIONS:
                raise ValueError(
                    f"{section}.bindings.{key}: axis-class function {fn!r} "
                    f"cannot be bound to a D-pad direction"
                )
        elif fn in BUTTON_CLASS_FUNCTIONS or fn in BASE_FUNCTIONS:
            if key in base_axes and key not in {"l2", "r2"}:
                # Stick axes can't be button-class. Triggers (l2/r2)
                # are analog axes used as binary — that's the
                # explicit polymorphism `button_pressed_for` handles.
                raise ValueError(
                    f"{section}.bindings.{key}: button-class function {fn!r} "
                    f"bound to stick axis {key!r}"
                )
        if fn in seen_fn:
            raise ValueError(
                f"{section}.bindings: function {fn!r} bound to both "
                f"{seen_fn[fn]!r} and {key!r}"
            )
        seen_fn[fn] = key


def cross_section_function_check(
    sections: Mapping[str, Mapping[str, str]],
) -> None:
    """Ensure a function used in multiple sections resolves to the same key.

    Identical duplicates (e.g. ``dpad_left: gait_prev`` in both
    ``gait.bindings`` and ``posture.bindings``) are allowed; conflicting
    bindings (same function bound to different keys across sections)
    raise.
    """
    fn_to_keys: dict[str, dict[str, str]] = {}
    for section_name, bindings in sections.items():
        for key, fn in bindings.items():
            if not fn:
                continue
            fn_to_keys.setdefault(fn, {})[section_name] = key
    for fn, by_section in fn_to_keys.items():
        keys = set(by_section.values())
        if len(keys) > 1:
            details = ", ".join(
                f"{sec}={k!r}" for sec, k in sorted(by_section.items())
            )
            raise ValueError(
                f"function {fn!r} bound to different keys across sections: "
                f"{details}"
            )


def map_joy(
    axes: Sequence[float],
    buttons: Sequence[int],
    cfg: JoyConfig,
    state: JoyState,
    dt: float,
) -> JoyOutput:
    base = cfg.base
    mode_cfg = _mode_cfg(cfg, state.mode)

    # Mode buttons: rising edge on the key bound to gait_mode selects
    # GAIT; rising edge on posture_mode selects POSTURE; rising edge
    # on animation_mode toggles GAIT ↔ ANIMATION. Pressing the button
    # for the already-active mode is a no-op. Held buttons don't
    # repeat. If multiple edges land on the same tick, posture wins
    # (safer fallback).
    gait_pressed = button_pressed_for("gait_mode", base, mode_cfg, buttons, axes)
    posture_pressed = button_pressed_for(
        "posture_mode", base, mode_cfg, buttons, axes
    )
    animation_pressed = button_pressed_for(
        "animation_mode", base, mode_cfg, buttons, axes
    )
    gait_edge = gait_pressed and not state.prev_gait_mode
    posture_edge = posture_pressed and not state.prev_posture_mode
    animation_edge = animation_pressed and not state.prev_animation_mode
    state.prev_gait_mode = gait_pressed
    state.prev_posture_mode = posture_pressed
    state.prev_animation_mode = animation_pressed
    mode_changed = False
    prev_mode = state.mode
    if posture_edge and state.mode != POSTURE:
        state.mode = POSTURE
        mode_changed = True
    elif gait_edge and state.mode != GAIT:
        state.mode = GAIT
        mode_changed = True
    elif animation_edge:
        # Rising-edge toggle between GAIT and ANIMATION. From POSTURE,
        # animation_mode hops directly into ANIMATION.
        state.mode = GAIT if state.mode == ANIMATION else ANIMATION
        mode_changed = True

    # If the mode changed, refresh the mode-cfg view so this tick's
    # remaining reads use the new mode's bindings.
    if mode_changed:
        mode_cfg = _mode_cfg(cfg, state.mode)

    # Side effects of leaving / entering ANIMATION mode.
    animation_name_out: str | None = None
    forced_gait: str | None = None
    if prev_mode == ANIMATION and state.mode != ANIMATION:
        # Leaving ANIMATION: tell posture to restore the default stack.
        state.animation_name = ""
        animation_name_out = ""
    elif prev_mode != ANIMATION and state.mode == ANIMATION:
        # Entering ANIMATION: force tripod (the new animations are
        # tripod-only) and start with vertical_body_roll as the
        # default so the body is visibly animated immediately.
        state.animation_name = "vertical_body_roll"
        animation_name_out = "vertical_body_roll"
        forced_gait = "tripod"
        if cfg.gait_cycle and "tripod" in cfg.gait_cycle:
            state.current_gait_idx = cfg.gait_cycle.index("tripod")

    # Init button: one-shot rising-edge trigger with two-press
    # semantics when the chassis is in a non-default posture.
    init_pressed = button_pressed_for("init", base, mode_cfg, buttons, axes)
    init_edge = init_pressed and not state.prev_init
    state.prev_init = init_pressed
    init_request = False
    if init_edge:
        # Tolerance well below the integration step so a stale tiny
        # value doesn't trap the user in a "revert-then-revert" loop.
        posture_modified = (
            abs(state.height_current) > 1e-4
            or abs(state.yaw_current) > 1e-4
            or abs(state.recorded_x) > 1e-4
            or abs(state.recorded_y) > 1e-4
            or abs(state.recorded_z) > 1e-4
            or abs(state.recorded_roll) > 1e-4
            or abs(state.recorded_pitch) > 1e-4
            or abs(state.recorded_yaw) > 1e-4
        )
        if posture_modified:
            state.reverting = True
        else:
            init_request = True

    # Revert decay: while ``state.reverting`` is set, ease the
    # persistent baseline toward zero with ``posture.revert_tau``.
    if state.reverting:
        decay = math.exp(-dt / cfg.posture.revert_tau)
        state.height_current *= decay
        state.recorded_x *= decay
        state.recorded_y *= decay
        state.recorded_z *= decay
        state.recorded_roll *= decay
        state.recorded_pitch *= decay
        state.recorded_yaw *= decay
        if (
            abs(state.height_current) <= 1e-4
            and abs(state.yaw_current) <= 1e-4
            and abs(state.recorded_x) <= 1e-4
            and abs(state.recorded_y) <= 1e-4
            and abs(state.recorded_z) <= 1e-4
            and abs(state.recorded_roll) <= 1e-4
            and abs(state.recorded_pitch) <= 1e-4
            and abs(state.recorded_yaw) <= 1e-4
        ):
            state.height_current = 0.0
            state.recorded_x = 0.0
            state.recorded_y = 0.0
            state.recorded_z = 0.0
            state.recorded_roll = 0.0
            state.recorded_pitch = 0.0
            state.recorded_yaw = 0.0
            state.reverting = False

    # Record button: rising-edge press. Applied after live posture is
    # computed (see below) so the snapshot includes this tick's input.
    record_pressed = button_pressed_for("record", base, mode_cfg, buttons, axes)
    record_edge = record_pressed and not state.prev_record
    state.prev_record = record_pressed

    # Posture-mode stick reads. ``axis_value_for`` applies the bound
    # axis's sign and deadband, so by the time these locals are
    # populated, "stick forward / left → positive" is already in
    # effect (assuming the YAML's ``axis_signs`` match the controller).
    posture_cfg = cfg.posture
    lx = axis_value_for("tilt_roll", base, posture_cfg, axes)
    ly = axis_value_for("tilt_pitch", base, posture_cfg, axes)
    rx = axis_value_for("pose_y", base, posture_cfg, axes)
    ry = axis_value_for("pose_x", base, posture_cfg, axes)

    # Body height: ``height_up`` / ``height_down`` are button-class.
    # Integrate (up - down) * rate * dt in POSTURE. Held both ⇒ no net
    # change. Works equally well bound to D-pad up/down or to face
    # buttons or to L1/R1.
    height_up = button_pressed_for(
        "height_up", base, posture_cfg, buttons, axes
    )
    height_down = button_pressed_for(
        "height_down", base, posture_cfg, buttons, axes
    )
    if state.mode == POSTURE:
        net = (1.0 if height_up else 0.0) - (1.0 if height_down else 0.0)
        state.height_current += net * posture_cfg.height_rate * dt
        if state.height_current > posture_cfg.height_max:
            state.height_current = posture_cfg.height_max
        elif state.height_current < posture_cfg.height_min:
            state.height_current = posture_cfg.height_min

    # Animation selectors: each animation-* function fires a publish
    # on its own rising edge. Bound to D-pad directions by default,
    # but any button-class binding works.
    animation_cfg = cfg.animation
    if state.mode == ANIMATION:
        for fn in ANIMATION_SELECT_FUNCTIONS:
            pressed = button_pressed_for(
                fn, base, animation_cfg, buttons, axes
            )
            prev = state.prev_animation_select.get(fn, False)
            if pressed and not prev:
                anim_name = _ANIMATION_FUNCTION_TO_NAME[fn]
                if state.animation_name != anim_name:
                    state.animation_name = anim_name
                    animation_name_out = anim_name
            state.prev_animation_select[fn] = pressed
    else:
        # Keep the prev map fresh in non-ANIMATION modes too so a
        # button that's still held when ANIMATION is entered doesn't
        # spuriously rising-edge on the entry tick. Evaluate against
        # the animation bindings (the bindings that will be in effect
        # if/when ANIMATION is entered) so prev tracks the same
        # interpretation as the in-mode read.
        for fn in ANIMATION_SELECT_FUNCTIONS:
            state.prev_animation_select[fn] = button_pressed_for(
                fn, base, animation_cfg, buttons, axes
            )

    # Gait cycler: ``gait_prev`` / ``gait_next`` rising edges. Cycling
    # is mode-agnostic for the resolution itself but suppressed in
    # ANIMATION (tripod was forced on entry).
    gait_select: str | None = forced_gait
    prev_pressed = button_pressed_for("gait_prev", base, mode_cfg, buttons, axes)
    next_pressed = button_pressed_for("gait_next", base, mode_cfg, buttons, axes)
    if cfg.gait_cycle and state.mode != ANIMATION:
        delta = 0
        if next_pressed and not state.prev_gait_next:
            delta += 1
        if prev_pressed and not state.prev_gait_prev:
            delta -= 1
        if delta != 0:
            state.current_gait_idx = (
                state.current_gait_idx + delta
            ) % len(cfg.gait_cycle)
            gait_select = cfg.gait_cycle[state.current_gait_idx]
    state.prev_gait_prev = prev_pressed
    state.prev_gait_next = next_pressed

    # Yaw + wiggle: same shared yaw target so L1 + L2 doesn't double
    # the yaw — L2 only adds the wiggle translation on top.
    yaw_btn_left = button_pressed_for("yaw_left", base, mode_cfg, buttons, axes)
    yaw_btn_right = button_pressed_for(
        "yaw_right", base, mode_cfg, buttons, axes
    )
    wiggle_left = button_pressed_for(
        "wiggle_left", base, mode_cfg, buttons, axes
    )
    wiggle_right = button_pressed_for(
        "wiggle_right", base, mode_cfg, buttons, axes
    )
    push_left = yaw_btn_left or wiggle_left
    push_right = yaw_btn_right or wiggle_right
    if state.mode == POSTURE and push_left != push_right:
        yaw_target = (
            posture_cfg.yaw_max if push_left else -posture_cfg.yaw_max
        )
    else:
        # No active input, both sides cancelled, or non-POSTURE mode —
        # ease back to zero so the offset bleeds off smoothly.
        yaw_target = 0.0

    wiggle_target = (
        1.0
        if state.mode == POSTURE and (wiggle_left or wiggle_right)
        else 0.0
    )
    alpha = 1.0 - math.exp(-dt / posture_cfg.yaw_tau)
    state.yaw_current += (yaw_target - state.yaw_current) * alpha
    state.wiggle_amount += (wiggle_target - state.wiggle_amount) * alpha

    # Wiggle translation: rotation about a pivot at (+px, 0) in the
    # body frame is equivalent to (rotate about body centre) +
    # (translate by px*(1-cos θ), -px*sin θ).
    px = posture_cfg.wiggle_pivot_forward_m
    wx = state.wiggle_amount * px * (1.0 - math.cos(state.yaw_current))
    wy = -state.wiggle_amount * px * math.sin(state.yaw_current)

    # Apply the deferred record press now that every live posture
    # component is up to date.
    if record_edge and state.mode == POSTURE:
        # A new baseline trumps any in-flight revert.
        state.reverting = False
        state.recorded_x = _clip(
            state.recorded_x + ry * posture_cfg.x_max + wx,
            -posture_cfg.x_max,
            posture_cfg.x_max,
        )
        state.recorded_y = _clip(
            state.recorded_y + rx * posture_cfg.y_max + wy,
            -posture_cfg.y_max,
            posture_cfg.y_max,
        )
        state.recorded_z = _clip(
            state.recorded_z + state.height_current,
            posture_cfg.height_min,
            posture_cfg.height_max,
        )
        state.recorded_roll = _clip(
            state.recorded_roll + (-lx) * posture_cfg.roll_max,
            -posture_cfg.roll_max,
            posture_cfg.roll_max,
        )
        state.recorded_pitch = _clip(
            state.recorded_pitch + ly * posture_cfg.pitch_max,
            -posture_cfg.pitch_max,
            posture_cfg.pitch_max,
        )
        state.recorded_yaw = _clip(
            state.recorded_yaw + state.yaw_current,
            -posture_cfg.yaw_max,
            posture_cfg.yaw_max,
        )
        state.height_current = 0.0
        state.yaw_current = 0.0

    if state.mode == POSTURE:
        # Tilt sign: stick-forward (ly > 0) → +pitch about +y (front
        # dips). stick-left (lx > 0) → -roll about +x (left side dips).
        return JoyOutput(
            linear_x=0.0,
            linear_y=0.0,
            angular_z=0.0,
            pose_x=_clip(
                state.recorded_x + ry * posture_cfg.x_max + wx,
                -posture_cfg.x_max,
                posture_cfg.x_max,
            ),
            pose_y=_clip(
                state.recorded_y + rx * posture_cfg.y_max + wy,
                -posture_cfg.y_max,
                posture_cfg.y_max,
            ),
            pose_z=_clip(
                state.recorded_z + state.height_current,
                posture_cfg.height_min,
                posture_cfg.height_max,
            ),
            pose_yaw=_clip(
                state.recorded_yaw + state.yaw_current,
                -posture_cfg.yaw_max,
                posture_cfg.yaw_max,
            ),
            pose_roll=_clip(
                state.recorded_roll + (-lx) * posture_cfg.roll_max,
                -posture_cfg.roll_max,
                posture_cfg.roll_max,
            ),
            pose_pitch=_clip(
                state.recorded_pitch + ly * posture_cfg.pitch_max,
                -posture_cfg.pitch_max,
                posture_cfg.pitch_max,
            ),
            mode_changed=mode_changed,
            init_request=init_request,
            gait_select=gait_select,
            animation_name=animation_name_out,
        )
    # GAIT or ANIMATION mode: sticks drive linear/angular velocity;
    # recorded posture baseline bleeds through on every posture axis
    # so the robot walks at the recorded posture.
    drive_x = axis_value_for("drive_x", base, mode_cfg, axes)
    drive_y = axis_value_for("drive_y", base, mode_cfg, axes)
    drive_yaw = axis_value_for("drive_yaw", base, mode_cfg, axes)
    return JoyOutput(
        linear_x=drive_x * cfg.gait_linear_max,
        linear_y=drive_y * cfg.gait_linear_max,
        angular_z=drive_yaw * cfg.gait_angular_z_max,
        pose_x=state.recorded_x,
        pose_y=state.recorded_y,
        pose_z=_clip(
            state.recorded_z + state.height_current,
            posture_cfg.height_min,
            posture_cfg.height_max,
        ),
        pose_yaw=state.recorded_yaw,
        pose_roll=state.recorded_roll,
        pose_pitch=state.recorded_pitch,
        mode_changed=mode_changed,
        init_request=init_request,
        gait_select=gait_select,
        animation_name=animation_name_out,
    )
