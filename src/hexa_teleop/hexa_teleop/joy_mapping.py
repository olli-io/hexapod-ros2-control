"""Pure mapping from a sensor_msgs/Joy snapshot to high-level commands.

The teleop node is the ROS glue around this module. The functions here
take plain sequences of axes and buttons (no rclpy types) so the logic
is unit-testable without spinning a ROS context.

Mode model:
  * ``posture`` — right stick translates the body in the x-y plane.
    ``/cmd_vel`` is zero so ``hexa_posture`` stays in pose mode.
  * ``gait`` — left stick X is spin rate, right stick is linear
    velocity of the body. ``/body/pose`` is zero so any standing
    translation decays back to the nominal stance.

A rising edge on the mode-toggle button flips the mode. Holding the
button does not retoggle; the user must release and press again.

Axis sign convention follows joy_node defaults: stick pushed
forward / left is positive — same as REP-103 body frame
(+x forward, +y left). Mapping is therefore unit-gain through the
sign; per-axis maxima come from YAML.
"""

from __future__ import annotations

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
    mode_toggle_button: int
    init_button: int
    deadband: float
    gait_linear_max: float
    gait_angular_z_max: float
    posture_x_max: float
    posture_y_max: float


@dataclass
class JoyState:
    mode: str = POSTURE
    prev_toggle: bool = False
    prev_init: bool = False


@dataclass(frozen=True)
class JoyOutput:
    linear_x: float
    linear_y: float
    angular_z: float
    pose_x: float
    pose_y: float
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


def map_joy(
    axes: Sequence[float],
    buttons: Sequence[int],
    cfg: JoyConfig,
    state: JoyState,
) -> JoyOutput:
    pressed = _read_button(buttons, cfg.mode_toggle_button)
    mode_changed = pressed and not state.prev_toggle
    if mode_changed:
        state.mode = GAIT if state.mode == POSTURE else POSTURE
    state.prev_toggle = pressed

    # Start button: one-shot rising-edge trigger that asks the gait
    # engine to switch between FOLDED and STAND — INITIALIZE on the
    # way up, FOLDING on the way down. Holding the button does nothing
    # extra; the user must release and press again.
    init_pressed = _read_button(buttons, cfg.init_button)
    init_request = init_pressed and not state.prev_init
    state.prev_init = init_pressed

    lx = _read_axis(axes, cfg.axis_left_x, cfg.deadband)
    rx = _read_axis(axes, cfg.axis_right_x, cfg.deadband)
    ry = _read_axis(axes, cfg.axis_right_y, cfg.deadband)

    if state.mode == POSTURE:
        return JoyOutput(
            linear_x=0.0,
            linear_y=0.0,
            angular_z=0.0,
            pose_x=ry * cfg.posture_x_max,
            pose_y=rx * cfg.posture_y_max,
            mode_changed=mode_changed,
            init_request=init_request,
        )
    return JoyOutput(
        linear_x=ry * cfg.gait_linear_max,
        linear_y=rx * cfg.gait_linear_max,
        angular_z=lx * cfg.gait_angular_z_max,
        pose_x=0.0,
        pose_y=0.0,
        mode_changed=mode_changed,
        init_request=init_request,
    )
