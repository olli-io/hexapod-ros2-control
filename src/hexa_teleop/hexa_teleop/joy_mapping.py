"""Pure mapping from a sensor_msgs/Joy snapshot to high-level commands.

Takes plain sequences of axes and buttons (no rclpy types) so the
logic is unit-testable without spinning a ROS context. The ROS glue
lives in ``teleop_joy.py``; user-facing behavior is documented in the
package README.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Collection, Mapping, Sequence

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
    "animation_prev",
    "animation_next",
    # Stand up in quadruped mode — the select half of the init button.
    # Bound only in the GAIT section, which is what confines it to gait
    # mode: elsewhere the same key keeps its base binding.
    "quadruped_mode",
})
AXIS_CLASS_FUNCTIONS: frozenset[str] = frozenset({
    "drive_x",
    # Second source for the same quantity, summed with ``drive_x``. Binding it
    # to the yaw stick's free axis turns that stick into an arcade drive
    # (throttle + steer under one thumb) without taking anything away from the
    # translation stick.
    "drive_x_aux",
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
    # index. Edit these blocks to support a different controller. All four
    # default to empty because they describe a *physical* device: an input
    # source that names its functions outright (the webapp) has no layout to
    # declare and leaves them alone.
    button_index: Mapping[str, int] = field(default_factory=dict)
    axis_index: Mapping[str, int] = field(default_factory=dict)
    # Per-axis sign so a driver that reports the opposite direction can
    # be normalised to "+x forward, +y left, dpad-up = +1". Missing
    # entries default to +1.0.
    axis_sign: Mapping[str, float] = field(default_factory=dict)
    # Mode-agnostic key bindings (mode-select buttons, init, record).
    # key name -> function name (or "" for unbound).
    bindings: Mapping[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class ModeConfig:
    """Per-mode bindings: physical key name -> function name.

    Empty for an input source that names its functions outright, which has
    no keys to bind — see ``BaseConfig``.
    """

    bindings: Mapping[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class PostureConfig:
    """Posture-mode bindings + the scalar limits the mode needs.

    ``height_max`` / ``height_min`` are pose offsets derived from the posture
    stack's own envelope (``tuning.yaml`` ``body_height_{max,min}_m``, absolute
    belly clearance) — see ``hexa_common.load_body_height_offsets``. They are
    not a teleop-owned limit; they exist here only so the height integrator
    saturates where the pipeline clamps, instead of banking travel that will
    never be honoured.
    """

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
    # Last so it can default to empty, as on ``ModeConfig``. Every caller
    # passes the scalars by keyword, so the order is nobody's business.
    bindings: Mapping[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class JoyConfig:
    base: BaseConfig
    gait: ModeConfig
    posture: PostureConfig
    animation: ModeConfig
    # Ordered list of gait names the cycler walks through, already
    # filtered by ``allow_unstable_gaits`` at load time. Index
    # ``current_gait_idx`` on ``JoyState`` tracks the user's selection.
    gait_cycle: tuple[str, ...]
    # The rotation walked while standing on four legs, and the entry the
    # select init stands up on. Disjoint from ``gait_cycle`` by load-time
    # validation, so neither cycler can land on the other leg set.
    quadruped_gait_cycle: tuple[str, ...]
    default_quadruped_gait: str
    # Per-gait stick scaling. Updated at runtime from /cmd_gait via
    # ``dataclasses.replace`` whenever the active gait changes.
    gait_linear_max: float
    gait_angular_z_max: float
    # Nominal standing stance, normalised by the outermost foot's radius —
    # the unitless lever arms ``fit_drive_to_envelope`` needs. Gait-agnostic:
    # every gait's angular cap is its linear cap over that same radius, so the
    # units cancel and one table serves them all.
    stance_unit: tuple[tuple[float, float], ...]
    # Ordered list of animation names the ANIMATION-mode cycler walks
    # through. Entry into ANIMATION snaps to index 0; subsequent
    # ``animation_prev`` / ``animation_next`` presses step the index.
    animation_list: tuple[str, ...]


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
    # ``height_down`` in any mode. Unlike every other posture axis
    # this value survives release and a mode toggle (the robot walks
    # at the lifted/lowered posture).
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
    # Rising-edge trackers for the animation cycler.
    prev_animation_prev: bool = False
    prev_animation_next: bool = False
    # Index into ``cfg.animation_list`` for the active selection. Reset
    # to 0 every time ANIMATION mode is entered.
    current_animation_idx: int = 0
    # Active animation-mode selection. ``""`` when ANIMATION mode is
    # not in effect; otherwise the name of the selected animation.
    animation_name: str = ""
    # Quadruped mode: the middle pair folded, the four corners creeping.
    # A property of the selected gait on the wire, so this flag only
    # tracks which of the two init buttons was pressed last.
    # The two rotations keep separate slots, so cycling on four legs
    # never overwrites the six-leg gait the operator was on (and back).
    quadruped: bool = False
    prev_quad_init: bool = False
    # Index into ``cfg.quadruped_gait_cycle``. Reset to
    # ``default_quadruped_gait`` on every accepted quad init.
    current_quadruped_gait_idx: int = 0


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
    # Start or select: stand up from the belly, else fold. The ROS layer
    # is what knows which — the mapping cannot see the engine.
    init_request: bool
    # Populated with the freshly-cycled gait name on a ``gait_prev`` /
    # ``gait_next`` rising edge; ``None`` on every other tick. The
    # mapping does NOT gate on engine state (POSTURE/GAIT,
    # STAND/walking) — that lives in the ROS layer.
    gait_select: str | None = None
    # Populated with the desired ``/animation/mode`` value on the tick
    # the selection changes; ``None`` on every other tick.
    animation_name: str | None = None
    # The init request asked for the quadruped leg set (select rather
    # than start). It rides ``gait_select`` on the wire; this is the same
    # fact for a caller that wants to log or gate on it without matching
    # gait names.
    init_quadruped: bool = False


@dataclass(frozen=True)
class FunctionInput:
    """What the operator is asking for, named by function rather than key.

    The seam between an input device and the state machine. ``pressed`` holds
    the button-class and base functions held this tick; ``axes`` maps each
    axis-class function to its sign-normalised value in [-1, 1], deadband not
    yet applied (``map_functions`` owns that, so every device gets one rule).
    A function absent from ``axes`` reads as 0.0.
    """

    pressed: frozenset[str]
    axes: Mapping[str, float]


# Resolves the operator's input against one config section (GAIT / POSTURE /
# ANIMATION). A section rather than a bare snapshot because the same key means
# different functions per mode, and the state machine deliberately reads three
# things against a fixed section — see ``map_functions``.
FunctionSource = Callable[[str], FunctionInput]


EMPTY_INPUT = FunctionInput(pressed=frozenset(), axes={})


def _axis(inp: FunctionInput, function: str, deadband: float) -> float:
    return apply_deadband(inp.axes.get(function, 0.0), deadband)


def apply_deadband(value: float, deadband: float) -> float:
    """Zero the deadband, then rescale the survivors back onto [0, 1].

    Scaled-radial rather than a hard cut: without the rescale the output
    jumps to ``deadband`` (a tenth of the cap on the gamepad) the instant
    the stick clears centre. The endpoint is preserved — full deflection
    still maps to exactly 1.0 — so this composes with any downstream
    shaping.
    """
    magnitude = abs(value)
    if magnitude < deadband:
        return 0.0
    span = 1.0 - deadband
    if span <= 0.0:
        return value
    return math.copysign((magnitude - deadband) / span, value)


def fit_drive_to_envelope(
    drive_x: float,
    drive_y: float,
    drive_yaw: float,
    stance_unit: Sequence[Sequence[float]],
) -> tuple[float, float, float]:
    """Map stick deflection onto the reachable velocity envelope.

    The three drive axes come in as unitless stick values in ``[-1, 1]``,
    where 1 means "this axis' cap". Commanding several at once asks for
    more than the legs can lay down: a full diagonal wants sqrt(2) times
    the linear cap, and full translation plus full yaw wants twice it. The
    engine already refuses such a triple (``hexa_common.scale_to_envelope``),
    but silently — the operator just finds the top of the stick's travel
    dead, and the direction they asked for quietly rotated.

    So shape it here instead. The commanded direction is held fixed and the
    deflection along it is mapped linearly onto ``0 -> envelope boundary``:

    * ``M`` — deflection, as the inf-norm of the triple. The inf-norm (not
      the 2-norm) is what makes the *corners* of the physical stick gate
      reachable, so no travel is wasted.
    * ``peak`` — the fastest foot the triple implies, in units of the linear
      cap. Same per-leg planar speed the engine bounds,
      ``|(v_x - w*r_y, v_y + w*r_x)|``, but unitless: dividing the stance by
      the outer radius cancels the cap ratio, since every gait's angular cap
      is its linear cap over exactly that radius.
    * ``s = M / peak`` — puts full deflection exactly on the boundary.

    Single-axis commands come out untouched (``peak == M``), so this only
    ever bites on combinations. Scaling all three by one factor keeps the
    commanded direction exact, unlike the engine's ``yaw_bias`` split — the
    bias only shapes autonomous ``/cmd_vel`` sources now, since the triple
    this returns is inside the envelope by construction and passes through.

    The ``min(1, ...)`` is a guard, not a normal path: it costs nothing and
    saves the caller from having to prove ``peak >= M`` for an arbitrary
    stance.
    """
    deflection = max(abs(drive_x), abs(drive_y), abs(drive_yaw))
    if deflection <= 0.0:
        return 0.0, 0.0, 0.0

    peak = 0.0
    for r_x, r_y in (r[:2] for r in stance_unit):
        foot = math.hypot(drive_x - drive_yaw * r_y, drive_y + drive_yaw * r_x)
        if foot > peak:
            peak = foot
    if peak <= 0.0:
        return 0.0, 0.0, 0.0

    scale = min(1.0, deflection / peak)
    return drive_x * scale, drive_y * scale, drive_yaw * scale


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


def _button_pressed(
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


def _axis_signed(
    function: str,
    base: BaseConfig,
    mode_cfg: ModeConfig | PostureConfig,
    axes: Sequence[float],
) -> float:
    """Sign-normalised value of the axis bound to ``function``.

    Deadband is NOT applied here: it is the same rule for every input
    device, so ``map_functions`` applies it to whatever a ``FunctionSource``
    hands over. Returns 0.0 if unbound, bound to a non-axis key, or the
    index is out of range.
    """
    key = _resolve_function_key(function, base, mode_cfg)
    if key is None or key not in base.axis_index:
        return 0.0
    idx = base.axis_index[key]
    raw = _read_axis_idx(axes, idx)
    sign = base.axis_sign.get(key, 1.0)
    return sign * raw


def resolve_functions(
    axes: Sequence[float],
    buttons: Sequence[int],
    cfg: JoyConfig,
    section: str,
) -> FunctionInput:
    """Read a ``sensor_msgs/Joy`` snapshot as functions, per ``section``.

    The only place a Joy index is read. A device that reports indices — the
    gamepad — comes through here; one whose operator names the function
    outright builds a ``FunctionInput`` without it.
    """
    base = cfg.base
    mode_cfg = _mode_cfg(cfg, section)
    return FunctionInput(
        pressed=frozenset(
            fn
            for fn in BASE_FUNCTIONS | BUTTON_CLASS_FUNCTIONS
            if _button_pressed(fn, base, mode_cfg, buttons, axes)
        ),
        axes={
            fn: _axis_signed(fn, base, mode_cfg, axes)
            for fn in AXIS_CLASS_FUNCTIONS
        },
    )


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


def resolve_gait_cycle(
    raw_cycle: Sequence[str],
    known_gaits: Collection[str],
    unstable_gaits: Collection[str],
    allow_unstable: bool,
    foreign_gaits: Collection[str] = (),
    key: str = "gait_cycle",
) -> tuple[str, ...]:
    """Validate one gait rotation and apply the ``allow_unstable_gaits`` filter.

    Every name must be in ``known_gaits`` and none may be in
    ``foreign_gaits`` — the gaits of the *other* leg set, which is what
    keeps the six-leg and four-leg rotations from crossing. With
    ``allow_unstable`` False, names in ``unstable_gaits`` are dropped
    (order preserved); an all-unstable cycle raises rather than silently
    disabling the cycler. The caller passes the gait-knowledge sets so
    this stays a pure validator like ``validate_bindings``.
    """
    unknown = [n for n in raw_cycle if n not in known_gaits]
    if unknown:
        raise ValueError(
            f"{key}: unknown gait(s) {unknown} "
            f"(known: {sorted(known_gaits)})"
        )
    foreign = [n for n in raw_cycle if n in foreign_gaits]
    if foreign:
        raise ValueError(
            f"{key}: {foreign} walk the other leg set — the six-leg and "
            f"four-leg rotations stay disjoint"
        )
    if allow_unstable:
        return tuple(raw_cycle)
    filtered = tuple(n for n in raw_cycle if n not in unstable_gaits)
    if raw_cycle and not filtered:
        raise ValueError(
            f"{key}: every entry in {list(raw_cycle)} is unstable "
            f"and allow_unstable_gaits is false — nothing left to cycle"
        )
    return filtered


def map_functions(
    source: FunctionSource,
    cfg: JoyConfig,
    state: JoyState,
    dt: float,
) -> JoyOutput:
    """The whole teleop state machine, over functions rather than keys.

    ``source(section)`` answers "which functions are held, and what do the
    axis-class ones read, resolved against ``section``'s bindings". A device
    with a physical layout builds one with ``resolve_functions``; a device
    whose operator presses the function itself — the webapp — builds one
    directly and never owns an index.

    Most reads go through ``active``, the input resolved against the mode in
    force. Three do not, and say so where they happen: ``quadruped_mode``
    against GAIT, the posture sticks against POSTURE, the animation cycler
    against ANIMATION.
    """
    deadband = cfg.base.deadband
    active = source(state.mode)

    # Mode buttons: rising edge on the key bound to gait_mode selects
    # GAIT; rising edge on posture_mode selects POSTURE; rising edge
    # on animation_mode toggles GAIT ↔ ANIMATION. Pressing the button
    # for the already-active mode is a no-op. Held buttons don't
    # repeat. If multiple edges land on the same tick, posture wins
    # (safer fallback).
    gait_pressed = "gait_mode" in active.pressed
    posture_pressed = "posture_mode" in active.pressed
    animation_pressed = "animation_mode" in active.pressed
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
    elif animation_edge and not state.quadruped:
        # Rising-edge toggle between GAIT and ANIMATION. From POSTURE,
        # animation_mode hops directly into ANIMATION. Inert in
        # quadruped mode: every animation is written for six legs, and
        # the four-corner stance is cut for the support shift rather
        # than for a body the operator swings around on it. Gait and
        # posture stay available; the way out is the fold that got in.
        state.mode = GAIT if state.mode == ANIMATION else ANIMATION
        mode_changed = True

    # If the mode changed, re-resolve so this tick's remaining reads see
    # the new mode's functions.
    if mode_changed:
        active = source(state.mode)

    # Side effects of leaving / entering ANIMATION mode.
    animation_name_out: str | None = None
    forced_gait: str | None = None
    if prev_mode == ANIMATION and state.mode != ANIMATION:
        # Leaving ANIMATION: tell posture to restore the default stack.
        state.animation_name = ""
        animation_name_out = ""
    elif prev_mode != ANIMATION and state.mode == ANIMATION:
        # Entering ANIMATION: force tripod (animations are tripod-only)
        # and snap to the first entry in ``animation_list`` so the
        # body is visibly animated immediately.
        if cfg.animation_list:
            state.current_animation_idx = 0
            state.animation_name = cfg.animation_list[0]
            animation_name_out = cfg.animation_list[0]
        forced_gait = "tripod"
        if cfg.gait_cycle and "tripod" in cfg.gait_cycle:
            state.current_gait_idx = cfg.gait_cycle.index("tripod")

    # The two init buttons, with the two-press revert they share when
    # the chassis is in a non-default posture. Start asks for the
    # six-leg stand, select for the four-corner one; off the belly
    # either is a fold, which the ROS layer resolves because the
    # mapping cannot see the engine.
    init_pressed = "init" in active.pressed
    init_edge = init_pressed and not state.prev_init
    state.prev_init = init_pressed
    # Resolved against the GAIT section in EVERY mode, not the active
    # one: the key is bound only there, so reading the active mode's
    # table would leave ``prev_quad_init`` False while the button was
    # held in posture mode and fire a spurious edge the instant gait
    # mode was entered.
    quad_pressed = "quadruped_mode" in source(GAIT).pressed
    # Confined to GAIT mode, which is where the key is bound:
    # everywhere else ``select`` keeps its base binding (``record``),
    # and firing both off one press would record a posture on the way to
    # standing up. The teleop boots into gait mode, so the cold start is
    # covered. The tracker still advances in the other modes, so
    # returning to gait with the button held fires nothing.
    quad_edge = (
        quad_pressed and not state.prev_quad_init and state.mode == GAIT
    )
    state.prev_quad_init = quad_pressed
    init_request = False
    init_quadruped = False
    if init_edge or quad_edge:
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
            # Start wins a tie: six legs is the stand to land on when
            # both edges arrive in one tick.
            init_quadruped = quad_edge and not init_edge
            state.quadruped = init_quadruped
            # The leg set rides the gait, so every init edge republishes
            # the gait that walks the set it asked for. Idempotent when
            # nothing changed, and it is what keeps the engine's strategy
            # and this flag from drifting apart when the request lands as
            # a fold instead of a stand.
            if init_quadruped:
                forced_gait = cfg.default_quadruped_gait
                if forced_gait in cfg.quadruped_gait_cycle:
                    state.current_quadruped_gait_idx = (
                        cfg.quadruped_gait_cycle.index(forced_gait)
                    )
            elif cfg.gait_cycle:
                forced_gait = cfg.gait_cycle[
                    state.current_gait_idx % len(cfg.gait_cycle)
                ]

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
    record_pressed = "record" in active.pressed
    record_edge = record_pressed and not state.prev_record
    state.prev_record = record_pressed

    # Posture-mode stick reads, always against the POSTURE section. The
    # source has already applied the sign, ``_axis`` the deadband, so by
    # the time these locals are populated "stick forward / left → positive"
    # is in effect.
    posture_cfg = cfg.posture
    sticks = source(POSTURE)
    lx = _axis(sticks, "tilt_roll", deadband)
    ly = _axis(sticks, "tilt_pitch", deadband)
    rx = _axis(sticks, "pose_y", deadband)
    ry = _axis(sticks, "pose_x", deadband)

    # Body height: ``height_up`` / ``height_down`` are button-class.
    # Integrate (up - down) * rate * dt in any mode. Held both ⇒ no net
    # change. Bindings resolve against the active mode's config, so the
    # function must be bound in each mode's section to be reachable
    # there. The scalar limits / rate are always the canonical
    # ``posture`` values. Works equally well bound to D-pad up/down or
    # to face buttons or to L1/R1.
    height_up = "height_up" in active.pressed
    height_down = "height_down" in active.pressed
    net = (1.0 if height_up else 0.0) - (1.0 if height_down else 0.0)
    state.height_current += net * posture_cfg.height_rate * dt
    if state.height_current > posture_cfg.height_max:
        state.height_current = posture_cfg.height_max
    elif state.height_current < posture_cfg.height_min:
        state.height_current = posture_cfg.height_min

    # Animation cycler: ``animation_prev`` / ``animation_next`` rising
    # edges step through ``cfg.animation_list``. Active only in
    # ANIMATION mode; prev-state is still refreshed in other modes so
    # a button still held when ANIMATION is entered doesn't spuriously
    # rising-edge on the entry tick.
    anim = source(ANIMATION)
    anim_prev_pressed = "animation_prev" in anim.pressed
    anim_next_pressed = "animation_next" in anim.pressed
    if state.mode == ANIMATION and cfg.animation_list:
        delta = 0
        if anim_next_pressed and not state.prev_animation_next:
            delta += 1
        if anim_prev_pressed and not state.prev_animation_prev:
            delta -= 1
        if delta != 0:
            state.current_animation_idx = (
                state.current_animation_idx + delta
            ) % len(cfg.animation_list)
            new_name = cfg.animation_list[state.current_animation_idx]
            if state.animation_name != new_name:
                state.animation_name = new_name
                animation_name_out = new_name
    state.prev_animation_prev = anim_prev_pressed
    state.prev_animation_next = anim_next_pressed

    # Gait cycler: ``gait_prev`` / ``gait_next`` rising edges. Cycling
    # is mode-agnostic for the resolution itself but suppressed in
    # ANIMATION (tripod was forced on entry). Which rotation it walks
    # follows the leg set the robot is standing on: the two are disjoint,
    # so the cycler can never ask for a gait the engine would refuse.
    gait_select: str | None = forced_gait
    cycle = cfg.quadruped_gait_cycle if state.quadruped else cfg.gait_cycle
    prev_pressed = "gait_prev" in active.pressed
    next_pressed = "gait_next" in active.pressed
    if cycle and state.mode != ANIMATION:
        delta = 0
        if next_pressed and not state.prev_gait_next:
            delta += 1
        if prev_pressed and not state.prev_gait_prev:
            delta -= 1
        if delta != 0:
            if state.quadruped:
                state.current_quadruped_gait_idx = (
                    state.current_quadruped_gait_idx + delta
                ) % len(cycle)
                gait_select = cycle[state.current_quadruped_gait_idx]
            else:
                state.current_gait_idx = (
                    state.current_gait_idx + delta
                ) % len(cycle)
                gait_select = cycle[state.current_gait_idx]
    state.prev_gait_prev = prev_pressed
    state.prev_gait_next = next_pressed

    # Yaw + wiggle: same shared yaw target so L1 + L2 doesn't double
    # the yaw — L2 only adds the wiggle translation on top. The yaw
    # buttons are live in every mode, like height: the offset rides
    # ``yaw_current``, which the gait/animation return below adds to the
    # pose it publishes, so the body can be held turned while walking.
    # Bindings still resolve against the active mode's config, so a mode
    # that leaves them unbound has no yaw. Wiggle is live in GAIT and
    # POSTURE the same way — its translation rides ``wiggle_amount`` and
    # both returns add it — but not in ANIMATION, where the animation is
    # driving the body and an offset held underneath it fights it.
    wiggle_live = state.mode != ANIMATION
    yaw_btn_left = "yaw_left" in active.pressed
    yaw_btn_right = "yaw_right" in active.pressed
    wiggle_left = wiggle_live and "wiggle_left" in active.pressed
    wiggle_right = wiggle_live and "wiggle_right" in active.pressed
    push_left = yaw_btn_left or wiggle_left
    push_right = yaw_btn_right or wiggle_right
    if push_left != push_right:
        yaw_target = (
            posture_cfg.yaw_max if push_left else -posture_cfg.yaw_max
        )
    else:
        # No active input or both sides cancelled — ease back to zero so
        # the offset bleeds off smoothly.
        yaw_target = 0.0

    wiggle_target = 1.0 if (wiggle_left or wiggle_right) else 0.0
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
    # component is up to date. Inert in quadruped mode: the live
    # posture is welcome on four feet — posture mode is not walking —
    # but a RECORDED one bleeds through into gait mode, where it would
    # spend the same x-y envelope the support shift needs to carry the
    # body into the next support triangle, and that margin is
    # millimetres. Height is exempt by construction: it rides
    # ``height_current``, not the record.
    if record_edge and state.mode == POSTURE and not state.quadruped:
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
            init_quadruped=init_quadruped,
        )
    # GAIT or ANIMATION mode: sticks drive linear/angular velocity;
    # recorded posture baseline bleeds through on every posture axis
    # so the robot walks at the recorded posture. Height, yaw and the
    # wiggle translation are the live offsets that ride along: all are
    # held functions the operator can work while walking, so each is
    # added to its baseline here exactly as the posture return adds it.
    # The x-y pair carries the wiggle alone — the sticks are driving
    # here, so there is no live pose trim to add.
    # Forward has two sources so the yaw stick can drive arcade-style. They
    # add; the clip keeps the sum a deflection, which is what the envelope fit
    # below is defined on. The other two are clipped for the same reason —
    # joy_publisher scales raw int16 without clamping, so a fully-pressed axis
    # reads a hair past -1.
    drive_x = _clip(
        _axis(active, "drive_x", deadband)
        + _axis(active, "drive_x_aux", deadband),
        -1.0,
        1.0,
    )
    drive_y = _clip(_axis(active, "drive_y", deadband), -1.0, 1.0)
    drive_yaw = _clip(_axis(active, "drive_yaw", deadband), -1.0, 1.0)
    drive_x, drive_y, drive_yaw = fit_drive_to_envelope(
        drive_x, drive_y, drive_yaw, cfg.stance_unit
    )
    return JoyOutput(
        linear_x=drive_x * cfg.gait_linear_max,
        linear_y=drive_y * cfg.gait_linear_max,
        angular_z=drive_yaw * cfg.gait_angular_z_max,
        pose_x=_clip(
            state.recorded_x + wx,
            -posture_cfg.x_max,
            posture_cfg.x_max,
        ),
        pose_y=_clip(
            state.recorded_y + wy,
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
        pose_roll=state.recorded_roll,
        pose_pitch=state.recorded_pitch,
        mode_changed=mode_changed,
        init_request=init_request,
        gait_select=gait_select,
        animation_name=animation_name_out,
        init_quadruped=init_quadruped,
    )


def map_joy(
    axes: Sequence[float],
    buttons: Sequence[int],
    cfg: JoyConfig,
    state: JoyState,
    dt: float,
) -> JoyOutput:
    """``map_functions`` for a device that reports Joy indices.

    The snapshot is fixed for the tick, so each section is resolved at most
    once however many times the state machine asks for it.
    """
    resolved: dict[str, FunctionInput] = {}

    def source(section: str) -> FunctionInput:
        got = resolved.get(section)
        if got is None:
            got = resolved[section] = resolve_functions(
                axes, buttons, cfg, section
            )
        return got

    return map_functions(source, cfg, state, dt)
