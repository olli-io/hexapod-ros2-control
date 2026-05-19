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

A rising edge on the mode-toggle button flips the mode. Holding the
button does not retoggle; the user must release and press again.

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
    axis_dpad_y: int
    dpad_up_sign: float
    mode_toggle_button: int
    init_button: int
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
    posture_wiggle_pivot_forward_m: float
    posture_height_max: float
    posture_height_min: float
    posture_height_rate: float


@dataclass
class JoyState:
    mode: str = POSTURE
    prev_toggle: bool = False
    prev_init: bool = False
    yaw_current: float = 0.0
    wiggle_amount: float = 0.0
    # Persistent body-height offset, driven by the D-pad in POSTURE
    # mode. Unlike every other posture axis this value survives D-pad
    # release and a mode toggle into GAIT (the robot walks at the
    # lifted/lowered posture). Reset to zero on a Start press while
    # non-zero — see ``map_joy`` for the two-press semantics.
    height_current: float = 0.0


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


def map_joy(
    axes: Sequence[float],
    buttons: Sequence[int],
    cfg: JoyConfig,
    state: JoyState,
    dt: float,
) -> JoyOutput:
    pressed = _read_button(buttons, cfg.mode_toggle_button)
    mode_changed = pressed and not state.prev_toggle
    if mode_changed:
        state.mode = GAIT if state.mode == POSTURE else POSTURE
    state.prev_toggle = pressed

    # Start button: one-shot rising-edge trigger with two-press
    # semantics when the chassis is lifted/lowered.
    #
    #   * height != 0 — first press snaps state.height_current to 0
    #     and SUPPRESSES init_request (the gait engine's reseat
    #     ladder restores default joint geometry; no fold yet).
    #   * height == 0 — fires init_request, which the engine routes
    #     to start_initialize() if FOLDED or to a deferred fold
    #     request if STAND.
    #
    # So a "lifted-then-fold" sequence is two presses: snap, then
    # fold. Holding the button does nothing extra; the user must
    # release and press again.
    init_pressed = _read_button(buttons, cfg.init_button)
    init_edge = init_pressed and not state.prev_init
    state.prev_init = init_pressed
    init_request = False
    if init_edge:
        # Tolerance well below the integration step so a stale tiny
        # value from a very brief D-pad tap doesn't trap the user in
        # a perpetual "snap-then-snap" loop. 0.1 mm is far below the
        # min increment per tick at the YAML defaults
        # (rate=0.05 m/s * dt=0.02 s = 1 mm).
        if abs(state.height_current) > 1e-4:
            state.height_current = 0.0
        else:
            init_request = True

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

    if state.mode == POSTURE:
        # Tilt sign: stick-forward (ly > 0) → +pitch about +y (front
        # dips). stick-left (lx > 0) → -roll about +x (left side dips,
        # which is CCW about +x viewed from behind).
        #
        # Wiggle translation: rotation about a pivot at (+px, 0) in
        # the body frame is equivalent to (rotate about body centre)
        # + (translate by px*(1-cos θ), -px*sin θ). Scaled by the
        # eased wiggle scalar so the translation only appears when
        # the user actually wants the pivoting effect.
        px = cfg.posture_wiggle_pivot_forward_m
        wx = state.wiggle_amount * px * (1.0 - math.cos(state.yaw_current))
        wy = -state.wiggle_amount * px * math.sin(state.yaw_current)
        return JoyOutput(
            linear_x=0.0,
            linear_y=0.0,
            angular_z=0.0,
            pose_x=ry * cfg.posture_x_max + wx,
            pose_y=rx * cfg.posture_y_max + wy,
            pose_z=state.height_current,
            pose_yaw=state.yaw_current,
            pose_roll=-lx * cfg.posture_roll_max,
            pose_pitch=ly * cfg.posture_pitch_max,
            mode_changed=mode_changed,
            init_request=init_request,
        )
    # GAIT mode: posture axes are zero EXCEPT height, which is held
    # so the robot walks at the lifted/lowered posture.
    return JoyOutput(
        linear_x=ry * cfg.gait_linear_max,
        linear_y=rx * cfg.gait_linear_max,
        angular_z=lx * cfg.gait_angular_z_max,
        pose_x=0.0,
        pose_y=0.0,
        pose_z=state.height_current,
        pose_yaw=0.0,
        pose_roll=0.0,
        pose_pitch=0.0,
        mode_changed=mode_changed,
        init_request=init_request,
    )
