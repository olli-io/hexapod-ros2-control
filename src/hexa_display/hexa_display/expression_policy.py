"""Expression + gaze decision policy for the ESP32 face.

Pure module: no rclpy, no I/O, no clocks. The node feeds a
``PolicyInputs`` snapshot into ``decide`` each tick and relays the
returned ``DisplayTarget`` over the transport.

Precedence (highest first):

1. battery critical → DEAD, gaze CENTER, unconditional.
2. battery warning → SLEEPY, but only when idle (cmd_vel ≈ 0, gait
   state in the idle set, no animation mode) — a warning must not
   mask the face mid-walk.
3. animation mode non-empty → the configured animation expression
   (default WOOZY).
4. gait-state map from YAML; unknown or unseen state → NEUTRAL.

Gaze: the vertical axis always follows body pitch (nose up → UP) —
driving forward or backward never moves the gaze. The horizontal axis
tracks cmd_vel during gait-active motion (left → LEFT per REP-103)
and body yaw/roll in pose mode; DEAD always forces CENTER. Axis
quantization uses enter/exit hysteresis so the gaze doesn't chatter
at the deadband edge.

Face animations: ``select_face_animation`` picks a looping face
animation — breathing while no gait state has been heard yet (the
robot stack is still initializing), idling once the hexapod stands
idle and level. Battery flags and an active posture animation mode
suppress both. The node owns the animation clock and relays the
steps; while one is active the animation owns the gaze.

The only stateful piece is ``BatteryMonitor`` (hold time + voltage
hysteresis), kept separate from ``decide`` and deterministic given a
(voltage, t) sequence.
"""

from __future__ import annotations

from dataclasses import dataclass

from .face_animation import BREATHING, IDLING
from .protocol import Expression, Gaze

CMD_VEL_ZERO_TOL = 1e-4

# Gait states in which the battery warning may take the face over —
# anything else counts as "busy" and keeps the normal expression.
IDLE_GAIT_STATES: frozenset[str] = frozenset({"folded", "stand", "paused"})

# Gait states in which the idling face animation may run. Narrower
# than IDLE_GAIT_STATES on purpose: folded and paused show SLEEPY and
# the eyes stay still.
IDLING_GAIT_STATES: frozenset[str] = frozenset({"stand"})

# Default gait-state → expression map; overridable per state from YAML.
DEFAULT_EXPRESSION_MAP: dict[str, Expression] = {
    "folded": Expression.SLEEPY,
    "initialize": Expression.NEUTRAL,
    "stand": Expression.NEUTRAL,
    "engaging": Expression.NEUTRAL,
    "gait": Expression.HAPPY,
    "pausing": Expression.NEUTRAL,
    "paused": Expression.SLEEPY,
    "resuming": Expression.NEUTRAL,
    "reseating": Expression.NEUTRAL,
    "folding": Expression.SLEEPY,
}

# (vertical, horizontal) sign pair → Gaze. Vertical +1 = UP, horizontal
# +1 = LEFT (REP-103 +y / positive wz are both leftward).
_GAZE_TABLE: dict[tuple[int, int], Gaze] = {
    (0, 0): Gaze.CENTER,
    (1, 0): Gaze.UP,
    (-1, 0): Gaze.DOWN,
    (0, 1): Gaze.LEFT,
    (0, -1): Gaze.RIGHT,
    (1, 1): Gaze.UP_LEFT,
    (1, -1): Gaze.UP_RIGHT,
    (-1, 1): Gaze.DOWN_LEFT,
    (-1, -1): Gaze.DOWN_RIGHT,
}


@dataclass(frozen=True)
class PolicyInputs:
    gait_state: str | None
    vx: float
    vy: float
    wz: float
    animation_mode: str
    roll: float
    pitch: float
    yaw: float
    battery_low: bool
    battery_critical: bool


@dataclass(frozen=True)
class PolicyConfig:
    expression_map: dict[str, Expression]
    animation_expression: Expression = Expression.WOOZY
    battery_warning_expression: Expression = Expression.SLEEPY
    battery_critical_expression: Expression = Expression.DEAD
    gaze_deadband: float = 0.15
    gaze_exit_ratio: float = 0.6
    gaze_wz_weight: float = 1.0
    gaze_vy_max: float = 0.1
    gaze_wz_max: float = 0.5
    pose_pitch_threshold_rad: float = 0.08
    pose_tilt_threshold_rad: float = 0.08
    # Expressions the idling blink-and-switch steps through; empty
    # disables the expression cycling (the gaze cycle still runs).
    idling_expressions: tuple[Expression, ...] = (
        Expression.NEUTRAL,
        Expression.HAPPY,
    )
    idling_start_delay_s: float = 4.0


@dataclass(frozen=True)
class DisplayTarget:
    expression: Expression
    gaze: Gaze


IDLE_TARGET = DisplayTarget(expression=Expression.NEUTRAL, gaze=Gaze.CENTER)


def quantize_axis(
    value: float, prev_sign: int, deadband: float, exit_ratio: float
) -> int:
    """Sign-quantize ``value`` to {-1, 0, +1} with hysteresis.

    Enters a direction at ``|value| >= deadband``; once entered, holds
    it until ``|value|`` drops below ``deadband * exit_ratio``. A sign
    flip past the full deadband switches directly without passing
    through 0.
    """
    if abs(value) >= deadband:
        return 1 if value > 0.0 else -1
    if prev_sign != 0 and abs(value) >= deadband * exit_ratio:
        if (value > 0.0) == (prev_sign > 0):
            return prev_sign
    return 0


def _cmd_vel_is_zero(inputs: PolicyInputs) -> bool:
    return (
        abs(inputs.vx) < CMD_VEL_ZERO_TOL
        and abs(inputs.vy) < CMD_VEL_ZERO_TOL
        and abs(inputs.wz) < CMD_VEL_ZERO_TOL
    )


def _is_idle(inputs: PolicyInputs) -> bool:
    return (
        _cmd_vel_is_zero(inputs)
        and inputs.gait_state in IDLE_GAIT_STATES
        and not inputs.animation_mode
    )


def _gaze_signs(target: DisplayTarget) -> tuple[int, int]:
    for signs, gaze in _GAZE_TABLE.items():
        if gaze == target.gaze:
            return signs
    return (0, 0)


def _norm(value: float, scale: float) -> float:
    return value / scale if scale > 0.0 else 0.0


def _decide_gaze(
    inputs: PolicyInputs, config: PolicyConfig, prev: DisplayTarget
) -> Gaze:
    prev_v, prev_h = _gaze_signs(prev)
    # Vertical gaze follows body pitch in both modes — walking forward
    # or backward must not move the gaze up or down. REP-103
    # right-handed angles: +pitch (about +y) tips the nose down → gaze
    # DOWN.
    sv = quantize_axis(
        -inputs.pitch, prev_v, config.pose_pitch_threshold_rad,
        config.gaze_exit_ratio,
    )
    if not _cmd_vel_is_zero(inputs):
        # gait-active: horizontal gaze leads the strafe/turn direction
        # (+vy and +wz are both leftward).
        horizontal = _norm(inputs.vy, config.gaze_vy_max) + (
            config.gaze_wz_weight * _norm(inputs.wz, config.gaze_wz_max)
        )
        sh = quantize_axis(
            horizontal, prev_h, config.gaze_deadband, config.gaze_exit_ratio
        )
        return _GAZE_TABLE[(sv, sh)]
    # pose mode: horizontal gaze follows body tilt. +yaw turns the
    # nose left → LEFT; +roll (about +x) leans the body right → RIGHT,
    # hence the minus sign in the blend.
    sh = quantize_axis(
        inputs.yaw - inputs.roll, prev_h, config.pose_tilt_threshold_rad,
        config.gaze_exit_ratio,
    )
    return _GAZE_TABLE[(sv, sh)]


def decide(
    inputs: PolicyInputs, config: PolicyConfig, prev: DisplayTarget
) -> DisplayTarget:
    if inputs.battery_critical:
        return DisplayTarget(
            expression=config.battery_critical_expression, gaze=Gaze.CENTER
        )
    gaze = _decide_gaze(inputs, config, prev)
    if inputs.battery_low and _is_idle(inputs):
        return DisplayTarget(
            expression=config.battery_warning_expression, gaze=gaze
        )
    if inputs.animation_mode:
        return DisplayTarget(expression=config.animation_expression, gaze=gaze)
    expression = config.expression_map.get(
        inputs.gait_state or "", Expression.NEUTRAL
    )
    return DisplayTarget(expression=expression, gaze=gaze)


def _pose_is_level(inputs: PolicyInputs, config: PolicyConfig) -> bool:
    # Same axes and thresholds as the pose-mode gaze, so idling yields
    # exactly where tilt-following gaze would take over.
    return (
        abs(inputs.pitch) < config.pose_pitch_threshold_rad
        and abs(inputs.yaw - inputs.roll) < config.pose_tilt_threshold_rad
    )


def select_face_animation(
    inputs: PolicyInputs, config: PolicyConfig
) -> str | None:
    """Pick the face animation for this tick, or None.

    Breathing runs while no gait state has been heard yet — the robot
    stack (servo UART, gait engine) is still initializing. Idling runs
    while the hexapod stands idle, level, and command-free. Battery
    warning/critical and an active posture animation mode suppress
    both; the node applies ``idling_start_delay_s`` before idling
    actually starts.
    """
    if inputs.battery_critical or inputs.battery_low:
        return None
    if inputs.animation_mode:
        return None
    if inputs.gait_state is None:
        return BREATHING.name
    if (
        inputs.gait_state in IDLING_GAIT_STATES
        and _cmd_vel_is_zero(inputs)
        and _pose_is_level(inputs, config)
    ):
        return IDLING.name
    return None


class BatteryMonitor:
    """Debounce raw battery voltage into (low, critical) flags.

    A threshold of 0.0 disables that flag entirely (shipped default —
    the voltage-divider scale is uncalibrated). A flag raises only
    after the voltage has stayed below the threshold for ``hold_s``
    seconds, and clears only once it rises above ``threshold +
    hysteresis_v`` (no hold on the way up: good news is immediate).
    """

    def __init__(
        self,
        warning_v: float = 0.0,
        critical_v: float = 0.0,
        hysteresis_v: float = 0.3,
        hold_s: float = 3.0,
    ) -> None:
        self._warning_v = warning_v
        self._critical_v = critical_v
        self._hysteresis_v = hysteresis_v
        self._hold_s = hold_s
        self._low = False
        self._critical = False
        self._below_warning_since: float | None = None
        self._below_critical_since: float | None = None

    def _step(
        self,
        voltage: float,
        t: float,
        threshold: float,
        active: bool,
        below_since: float | None,
    ) -> tuple[bool, float | None]:
        if threshold <= 0.0:
            return False, None
        if active:
            if voltage > threshold + self._hysteresis_v:
                return False, None
            return True, below_since
        if voltage < threshold:
            if below_since is None:
                below_since = t
            if t - below_since >= self._hold_s:
                return True, below_since
            return False, below_since
        return False, None

    def update(self, voltage: float, t: float) -> tuple[bool, bool]:
        self._low, self._below_warning_since = self._step(
            voltage, t, self._warning_v, self._low, self._below_warning_since
        )
        self._critical, self._below_critical_since = self._step(
            voltage, t, self._critical_v, self._critical,
            self._below_critical_since,
        )
        return self._low, self._critical
