"""Pure mapping from a sensor_msgs/Joy snapshot to high-level commands.

The teleop node is the ROS glue around this module. The functions here
take plain sequences of axes and buttons (no rclpy types) so the logic
is unit-testable without spinning a ROS context.

Mode model:
  * ``posture`` — right stick translates the body in the x-y plane,
    the left stick tilts the body toward the direction it is pushed
    (stick forward → pitch forward / front dips; stick left → roll
    left / left side dips), and the L1/R1 shoulder buttons yaw the
    body about +z (L1 = left/+yaw, R1 = right/-yaw). All three inputs
    apply together. ``/cmd_vel`` is zero so ``hexa_posture`` stays in
    pose mode.
  * ``gait`` — left stick X is spin rate, right stick is linear
    velocity of the body. ``/body/pose`` is zero so any standing
    translation/yaw decays back to the nominal stance.

Note: the left stick has different semantics across modes (tilt in
posture, yaw rate in gait). This is intentional — tilting while
walking isn't exposed at the teleop layer, and the walking yaw rate
needs a continuous axis rather than a button.

Mode selection uses two dedicated buttons: a rising edge on the
``gait_mode_button`` sets the mode to ``gait``; a rising edge on the
``posture_mode_button`` sets the mode to ``posture``. Pressing the
button for the mode that is already active is a no-op (``mode_changed``
stays false). Holding either button does not re-fire; the user must
release and press again.

Axis sign convention follows joy_node defaults: stick pushed
forward / left is positive — same as REP-103 body frame
(+x forward, +y left). Mapping is therefore unit-gain through the
sign; per-axis maxima come from YAML.

The yaw shoulder buttons are binary, so a press would snap the body
to its limit. To keep the motion bearable the yaw output goes through
a first-order low-pass: each tick eases ``yaw_current`` toward the
button-driven target by ``alpha = 1 - exp(-dt / posture_yaw_tau)``.
The state lives on ``JoyState.yaw_current`` so it persists across
calls. Target is 0 in gait mode, so the offset decays smoothly back
to zero on a mode flip.

L2/R2 trigger a "wiggle": they share the same yaw target as L1/R1
(so L1 + L2 does not double the yaw), and additionally translate the
body so a configurable point a set distance forward of body centre
holds still in the world. Visual effect: the rear of the hexapod
swings while the front stays planted. The wiggle scalar
(``JoyState.wiggle_amount``, also eased through the same low-pass)
goes from 0 to 1 while L2 or R2 reads pressed, which prevents the
translation from snapping when the wiggle is engaged or released
mid-yaw. Translation magnitude per tick:

    pose_x_wiggle = wiggle_amount * px * (1 - cos(yaw_current))
    pose_y_wiggle = -wiggle_amount * px * sin(yaw_current)

where ``px`` is ``posture_wiggle_pivot_forward_m``. Like the rest of
the posture-mode outputs, both terms are forced to zero in gait mode.

**D-pad X** rising edges cycle through the configured ``gait_cycle``.
D-right advances by +1, D-left by -1, modulo the list length. Cycling
is mode-agnostic (works in POSTURE and GAIT); the teleop ROS layer
filters the resulting ``gait_select`` against the engine's ``STAND``
state and only publishes when the swap is acceptable. The index in
``JoyState`` advances on every press regardless of whether the publish
landed, so the user can press past slots that were rejected mid-walk
and resume cycling from there once the engine returns to STAND.

A rising-edge press of the **record** button (Select) in posture mode
folds the current live posture input into a persistent baseline on
``JoyState`` (the six ``recorded_*`` fields), then zeros the integrated
``height_current`` and eased ``yaw_current`` so the live state can't
double-count on the next tick. The output for each posture axis is
``clamp(recorded + live, ±axis_max)``, so re-pushing a stick that's
already at its limit has no further effect (the user's "tilt full left,
record, tilt full left again" example). The baseline bleeds through
into gait mode like the D-pad height does — the robot walks at the
recorded posture. Select in gait mode is a no-op (but ``prev_record``
still updates so edge detection stays correct).

The **Start** button extends today's two-press semantics over the
recorded baseline as well: if any posture state is non-default
(``height_current``, ``yaw_current``, or any ``recorded_*`` outside a
small tolerance), the first press arms a smooth revert
(``state.reverting``) instead of snapping to zero. Each subsequent
tick decays ``height_current`` and the six ``recorded_*`` toward zero
with the ``posture_revert_tau`` time constant; the eased
``yaw_current`` rides the existing yaw low-pass back to zero on its
own. ``init_request`` is suppressed during the revert; the next
Start press at the now-default state fires init as usual. A Select
press mid-revert cancels the revert (the user is recording a fresh
baseline and that should not bleed away).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

POSTURE = "posture"
GAIT = "gait"


@dataclass(frozen=True)
class JoyConfig:
    axis_left_x: int
    axis_left_y: int
    axis_right_x: int
    axis_right_y: int
    axis_dpad_x: int
    axis_dpad_y: int
    dpad_up_sign: float
    dpad_right_sign: float
    gait_mode_button: int
    posture_mode_button: int
    init_button: int
    record_button: int
    yaw_left_button: int
    yaw_right_button: int
    wiggle_left_trigger_axis: int
    wiggle_right_trigger_axis: int
    wiggle_trigger_threshold: float
    deadband: float
    gait_linear_max: float
    gait_angular_z_max: float
    posture_x_max: float
    posture_y_max: float
    posture_roll_max: float
    posture_pitch_max: float
    posture_yaw_max: float
    posture_yaw_tau: float
    posture_revert_tau: float
    posture_wiggle_pivot_forward_m: float
    posture_height_max: float
    posture_height_min: float
    posture_height_rate: float
    # Ordered list of gait names the D-pad X cycler walks through. Index
    # ``current_gait_idx`` on ``JoyState`` tracks the user's selection;
    # D-right advances by +1, D-left by −1, modulo the list length. The
    # teleop ROS layer filters and publishes; the mapping itself is
    # mode-agnostic so the user can cycle in POSTURE or GAIT.
    gait_cycle: tuple[str, ...]


@dataclass
class JoyState:
    mode: str = POSTURE
    prev_gait_mode: bool = False
    prev_posture_mode: bool = False
    prev_init: bool = False
    prev_record: bool = False
    yaw_current: float = 0.0
    wiggle_amount: float = 0.0
    # Persistent body-height offset, driven by the D-pad in POSTURE
    # mode. Unlike every other posture axis this value survives D-pad
    # release and a mode toggle into GAIT (the robot walks at the
    # lifted/lowered posture). Reset to zero on a Start press while
    # non-zero — see ``map_joy`` for the two-press semantics.
    height_current: float = 0.0
    # Persistent posture baseline captured by a rising-edge Select
    # press. Each component is bounded by its ``posture_*_max`` from
    # ``JoyConfig`` at record time. Bleeds through into GAIT mode (the
    # robot walks at the recorded body offset). Reset to zero by the
    # Start button alongside ``height_current`` / ``yaw_current`` when
    # any of them is non-default — see ``map_joy``.
    recorded_x: float = 0.0
    recorded_y: float = 0.0
    recorded_z: float = 0.0
    recorded_roll: float = 0.0
    recorded_pitch: float = 0.0
    recorded_yaw: float = 0.0
    # True while a Start-triggered revert to default posture is in
    # progress. Each tick decays ``height_current`` and the six
    # ``recorded_*`` toward zero with ``posture_revert_tau``; cleared
    # when every persistent component is below the 1e-4 tolerance, or
    # immediately by a Select press (the user is recording a new
    # baseline, which trumps the revert).
    reverting: bool = False
    # D-pad X edge-detect state for the gait cycler. ``prev_dpad_x`` is
    # the sign of the last tick's D-pad X (rounded to {-1, 0, +1});
    # ``current_gait_idx`` is the user's current position in
    # ``cfg.gait_cycle``. The ROS layer seeds the index from the
    # control-node default at startup and on every accepted publish.
    prev_dpad_x: int = 0
    current_gait_idx: int = 0


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
    # Populated with the freshly-cycled gait name on a D-pad X rising
    # edge; ``None`` on every other tick. The mapping does NOT gate on
    # engine state (POSTURE/GAIT, STAND/walking) — that lives in the
    # ROS layer so the pure function stays I/O-free.
    gait_select: str | None = None


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return value


def _read_axis(axes: Sequence[float], idx: int, deadband: float) -> float:
    if idx < 0 or idx >= len(axes):
        return 0.0
    return apply_deadband(float(axes[idx]), deadband)


def _read_button(buttons: Sequence[int], idx: int) -> bool:
    if idx < 0 or idx >= len(buttons):
        return False
    return bool(buttons[idx])


def _read_trigger(axes: Sequence[float], idx: int) -> float:
    # Out-of-range trigger reads as "released" (joy_node convention:
    # released = +1.0 on Xbox-style triggers) so a short Joy message
    # never accidentally registers as a held trigger.
    if idx < 0 or idx >= len(axes):
        return 1.0
    return float(axes[idx])


def _clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def map_joy(
    axes: Sequence[float],
    buttons: Sequence[int],
    cfg: JoyConfig,
    state: JoyState,
    dt: float,
) -> JoyOutput:
    # Mode buttons: rising edge on gait_mode_button selects GAIT;
    # rising edge on posture_mode_button selects POSTURE. Pressing the
    # button for the already-active mode is a no-op. Held buttons don't
    # repeat — the user must release and press again. If both edges
    # land on the same tick, posture wins (safer fallback).
    gait_pressed = _read_button(buttons, cfg.gait_mode_button)
    posture_pressed = _read_button(buttons, cfg.posture_mode_button)
    gait_edge = gait_pressed and not state.prev_gait_mode
    posture_edge = posture_pressed and not state.prev_posture_mode
    state.prev_gait_mode = gait_pressed
    state.prev_posture_mode = posture_pressed
    mode_changed = False
    if posture_edge and state.mode != POSTURE:
        state.mode = POSTURE
        mode_changed = True
    elif gait_edge and state.mode != GAIT:
        state.mode = GAIT
        mode_changed = True

    # Start button: one-shot rising-edge trigger with two-press
    # semantics when the chassis is in a non-default posture.
    #
    #   * any of height_current / yaw_current / recorded_* outside a
    #     small tolerance — first press arms a SMOOTH revert (sets
    #     ``state.reverting``; the per-tick decay below pulls every
    #     persistent component toward zero with
    #     ``posture_revert_tau``) and SUPPRESSES init_request.
    #   * everything at default — fires init_request, which the engine
    #     routes to start_initialize() if FOLDED or to a deferred fold
    #     request if STAND.
    #
    # So a "non-default-then-fold" sequence is two presses: revert,
    # then (once the revert has settled) fold. Holding the button does
    # nothing extra; the user must release and press again. Pressing
    # Start again mid-revert is a no-op (posture is still non-default
    # so init stays suppressed and ``reverting`` is just re-armed).
    init_pressed = _read_button(buttons, cfg.init_button)
    init_edge = init_pressed and not state.prev_init
    state.prev_init = init_pressed
    init_request = False
    if init_edge:
        # Tolerance well below the integration step so a stale tiny
        # value from a very brief D-pad tap doesn't trap the user in
        # a perpetual "revert-then-revert" loop. 0.1 mm / 0.0001 rad is
        # far below the min increment per tick at the YAML defaults
        # (height rate=0.05 m/s * dt=0.02 s = 1 mm; one yaw easing
        # tick from zero is on the order of yaw_max * 0.18).
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
    # persistent baseline (height + recorded_*) toward zero with the
    # ``posture_revert_tau`` time constant. ``yaw_current`` is left to
    # the existing yaw_tau easing further down (which already pulls it
    # toward zero when no yaw button is held); we still check it in
    # the settle condition so the revert doesn't clear while a stale
    # eased yaw lingers. Runs in both modes so a revert armed in
    # POSTURE keeps running across a mid-revert toggle to GAIT.
    if state.reverting:
        decay = math.exp(-dt / cfg.posture_revert_tau)
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

    # Select button: rising-edge press in POSTURE mode folds the
    # current live posture input into ``state.recorded_*`` (clamped
    # per-axis), then zeros the integrated height and eased yaw so the
    # live state can't double-count on the next tick. Stick reads stay
    # live — the per-axis clamp on the final output handles the
    # "already at the limit" saturation. ``prev_record`` updates in
    # GAIT mode too so the edge detection stays correct across mode
    # toggles, but the recording itself only happens in POSTURE.
    record_pressed = _read_button(buttons, cfg.record_button)
    record_edge = record_pressed and not state.prev_record
    state.prev_record = record_pressed

    lx = _read_axis(axes, cfg.axis_left_x, cfg.deadband)
    ly = _read_axis(axes, cfg.axis_left_y, cfg.deadband)
    rx = _read_axis(axes, cfg.axis_right_x, cfg.deadband)
    ry = _read_axis(axes, cfg.axis_right_y, cfg.deadband)

    # D-pad Y: integrate body-height offset while held (POSTURE only).
    # No deadband — joy_node reports the D-pad as a clean ±1 / 0 axis.
    # In GAIT mode the integration is suppressed so the user can't
    # change the chassis height while walking; the already-integrated
    # height bleeds through unchanged into pose.z.
    if state.mode == POSTURE:
        dpad_y = 0.0
        if 0 <= cfg.axis_dpad_y < len(axes):
            dpad_y = float(axes[cfg.axis_dpad_y])
        state.height_current += (
            cfg.dpad_up_sign * dpad_y * cfg.posture_height_rate * dt
        )
        if state.height_current > cfg.posture_height_max:
            state.height_current = cfg.posture_height_max
        elif state.height_current < cfg.posture_height_min:
            state.height_current = cfg.posture_height_min

    # D-pad X: rising-edge cycle through ``cfg.gait_cycle``. Direction
    # is governed by ``dpad_right_sign`` so a flipped joy_node sign can
    # be normalised in YAML. Works regardless of POSTURE / GAIT mode;
    # the ROS layer gates the publish on the current engine state.
    gait_select: str | None = None
    if cfg.gait_cycle:
        dpad_x_raw = 0.0
        if 0 <= cfg.axis_dpad_x < len(axes):
            dpad_x_raw = float(axes[cfg.axis_dpad_x])
        # Normalize joy_node's ±1 / 0 axis to an integer sign, then
        # flip if the driver reports right as −1. Anything between is
        # treated as released so a bouncy axis can't double-cycle.
        if dpad_x_raw > 0.5:
            dpad_x = 1
        elif dpad_x_raw < -0.5:
            dpad_x = -1
        else:
            dpad_x = 0
        signed = int(cfg.dpad_right_sign) * dpad_x
        if signed != 0 and state.prev_dpad_x == 0:
            state.current_gait_idx = (
                state.current_gait_idx + signed
            ) % len(cfg.gait_cycle)
            gait_select = cfg.gait_cycle[state.current_gait_idx]
        state.prev_dpad_x = signed

    # L1/R1 (shoulder buttons) and L2/R2 (analog triggers thresholded
    # to on/off) share the same yaw target. L1 and L2 both push left;
    # R1 and R2 both push right; left side ∥ right side cancels to
    # zero. Result: L1+L2 doesn't double the yaw — L2 only adds the
    # wiggle translation on top.
    yaw_btn_left = _read_button(buttons, cfg.yaw_left_button)
    yaw_btn_right = _read_button(buttons, cfg.yaw_right_button)
    wiggle_left = (
        _read_trigger(axes, cfg.wiggle_left_trigger_axis)
        < cfg.wiggle_trigger_threshold
    )
    wiggle_right = (
        _read_trigger(axes, cfg.wiggle_right_trigger_axis)
        < cfg.wiggle_trigger_threshold
    )
    push_left = yaw_btn_left or wiggle_left
    push_right = yaw_btn_right or wiggle_right
    if state.mode == POSTURE and push_left != push_right:
        yaw_target = cfg.posture_yaw_max if push_left else -cfg.posture_yaw_max
    else:
        # No active input, both sides (cancel), or gait mode — ease
        # back to zero. Keeping the integration alive in gait mode
        # lets the offset bleed off smoothly so a return to posture
        # starts near zero rather than snapping back to a stale value.
        yaw_target = 0.0

    # Wiggle scalar: 1 while either trigger is pressed (and we're in
    # posture mode), 0 otherwise. Eased through the same low-pass as
    # yaw so the translation doesn't snap if the user engages L2 mid
    # L1-yaw, or releases L2 while still holding L1.
    wiggle_target = (
        1.0 if state.mode == POSTURE and (wiggle_left or wiggle_right) else 0.0
    )
    alpha = 1.0 - math.exp(-dt / cfg.posture_yaw_tau)
    state.yaw_current += (yaw_target - state.yaw_current) * alpha
    state.wiggle_amount += (wiggle_target - state.wiggle_amount) * alpha

    # Wiggle translation: rotation about a pivot at (+px, 0) in the
    # body frame is equivalent to (rotate about body centre) +
    # (translate by px*(1-cos θ), -px*sin θ). Scaled by the eased
    # wiggle scalar so the translation only appears when the user
    # actually wants the pivoting effect. Computed unconditionally
    # (gait mode forces wiggle_amount toward zero, so wx/wy decay to
    # zero there) so the record-fold below can read them too.
    px = cfg.posture_wiggle_pivot_forward_m
    wx = state.wiggle_amount * px * (1.0 - math.cos(state.yaw_current))
    wy = -state.wiggle_amount * px * math.sin(state.yaw_current)

    # Apply the deferred Select press now that every live posture
    # component is up to date. Fold the live values into the recorded
    # baseline (per-axis clamped at record time) and zero the
    # integrated/eased values so they can't double-count on the next
    # tick. Stick reads (lx/ly/rx/ry) and wiggle (wx/wy) are NOT
    # zeroed — they're re-read or re-eased on the next tick, and the
    # output clamp below catches the saturation.
    if record_edge and state.mode == POSTURE:
        # A new baseline is being captured — trumps any in-flight
        # revert (the user is explicitly setting a pose, not asking
        # for default).
        state.reverting = False
        state.recorded_x = _clip(
            state.recorded_x + ry * cfg.posture_x_max + wx,
            -cfg.posture_x_max,
            cfg.posture_x_max,
        )
        state.recorded_y = _clip(
            state.recorded_y + rx * cfg.posture_y_max + wy,
            -cfg.posture_y_max,
            cfg.posture_y_max,
        )
        state.recorded_z = _clip(
            state.recorded_z + state.height_current,
            cfg.posture_height_min,
            cfg.posture_height_max,
        )
        state.recorded_roll = _clip(
            state.recorded_roll + (-lx) * cfg.posture_roll_max,
            -cfg.posture_roll_max,
            cfg.posture_roll_max,
        )
        state.recorded_pitch = _clip(
            state.recorded_pitch + ly * cfg.posture_pitch_max,
            -cfg.posture_pitch_max,
            cfg.posture_pitch_max,
        )
        state.recorded_yaw = _clip(
            state.recorded_yaw + state.yaw_current,
            -cfg.posture_yaw_max,
            cfg.posture_yaw_max,
        )
        state.height_current = 0.0
        state.yaw_current = 0.0

    if state.mode == POSTURE:
        # Tilt sign: stick-forward (ly > 0) → +pitch about +y (front
        # dips). stick-left (lx > 0) → -roll about +x (left side dips,
        # which is CCW about +x viewed from behind).
        #
        # Each axis is the sum of the persistent baseline and the live
        # input, clamped to its YAML max. With a fully-saturated
        # baseline, additional stick input in the same direction has
        # no further effect (the user's "tilt left, record, tilt left
        # again" example); opposite-direction input unwinds the
        # baseline up to its own limit.
        return JoyOutput(
            linear_x=0.0,
            linear_y=0.0,
            angular_z=0.0,
            pose_x=_clip(
                state.recorded_x + ry * cfg.posture_x_max + wx,
                -cfg.posture_x_max,
                cfg.posture_x_max,
            ),
            pose_y=_clip(
                state.recorded_y + rx * cfg.posture_y_max + wy,
                -cfg.posture_y_max,
                cfg.posture_y_max,
            ),
            pose_z=_clip(
                state.recorded_z + state.height_current,
                cfg.posture_height_min,
                cfg.posture_height_max,
            ),
            pose_yaw=_clip(
                state.recorded_yaw + state.yaw_current,
                -cfg.posture_yaw_max,
                cfg.posture_yaw_max,
            ),
            pose_roll=_clip(
                state.recorded_roll + (-lx) * cfg.posture_roll_max,
                -cfg.posture_roll_max,
                cfg.posture_roll_max,
            ),
            pose_pitch=_clip(
                state.recorded_pitch + ly * cfg.posture_pitch_max,
                -cfg.posture_pitch_max,
                cfg.posture_pitch_max,
            ),
            mode_changed=mode_changed,
            init_request=init_request,
            gait_select=gait_select,
        )
    # GAIT mode: live posture input is suppressed (sticks drive
    # linear/angular velocity instead), but the recorded baseline
    # bleeds through on every posture axis so the robot walks at the
    # recorded posture. pose_z keeps today's "height_current bleeds
    # through" behavior on top of the recorded_z baseline.
    return JoyOutput(
        linear_x=ry * cfg.gait_linear_max,
        linear_y=rx * cfg.gait_linear_max,
        angular_z=lx * cfg.gait_angular_z_max,
        pose_x=state.recorded_x,
        pose_y=state.recorded_y,
        pose_z=_clip(
            state.recorded_z + state.height_current,
            cfg.posture_height_min,
            cfg.posture_height_max,
        ),
        pose_yaw=state.recorded_yaw,
        pose_roll=state.recorded_roll,
        pose_pitch=state.recorded_pitch,
        mode_changed=mode_changed,
        init_request=init_request,
        gait_select=gait_select,
    )
