import math
from pathlib import Path

import pytest
import yaml

from hexa_webteleop import (
    NUM_BUTTONS,
    button_labels_for_mode,
    load_web_config,
    map_web,
)
from hexa_webteleop.web_mapping import GAIT, POSTURE, ANIMATION


DT = 0.02  # 50 Hz


# ─── Config fixture ────────────────────────────────────────────────

_WEBTELEOP_YAML = """
initial_mode: gait
gait_cycle: [tripod, surf, tetrapod, crawl, ripple]
default_gait: tripod
allow_unstable_gaits: false

server:
  port: 8080

base:
  deadband: 0.05
  trigger_threshold: 0.5
  buttons:
    btn_0: 0
    btn_1: 1
    btn_2: 2
    btn_3: 3
    btn_4: 4
    btn_5: 5
    btn_6: 6
    btn_7: 7
    btn_8: 8
  axes:
    left_stick_x: 0
    left_stick_y: 1
    right_stick_x: 2
    right_stick_y: 3
  axis_signs: {}
  bindings:
    btn_0: gait_mode
    btn_1: posture_mode
    btn_2: animation_mode

gait:
  bindings:
    btn_3: init
    btn_4: record
    btn_5: gait_prev
    btn_6: gait_next
    btn_7: height_up
    btn_8: height_down
    left_stick_y: drive_x
    left_stick_x: drive_y
    right_stick_x: drive_yaw

posture:
  bindings:
    btn_3: init
    btn_4: record
    btn_5: yaw_left
    btn_6: yaw_right
    btn_7: height_up
    btn_8: height_down
    left_stick_y: pose_x
    left_stick_x: pose_y
    right_stick_x: tilt_roll
    right_stick_y: tilt_pitch
  x_max: 0.035
  y_max: 0.035
  roll_max_deg: 12.0
  pitch_max_deg: 12.0
  yaw_max_deg: 20.0
  yaw_tau_s: 0.10
  revert_tau_s: 0.25
  wiggle_pivot_forward_m: 0.06
  height:
    max_m: 0.04
    min_m: -0.04
    rate_m_per_s: 0.05

animation:
  bindings:
    btn_3: init
    btn_4: record
    btn_5: animation_prev
    btn_6: animation_next
    btn_7: height_up
    btn_8: height_down
    left_stick_y: drive_x
    left_stick_x: drive_y
    right_stick_x: drive_yaw

arbitration:
  enabled: true
"""

_GAIT_YAML = """
stride_length: 0.1
min_swing_time: 0.30
max_swing_time: 0.4
angular_z_max: 3.00
yaw_bias: 0.6
default_gait: tripod
"""

_POSTURE_YAML = """
posture_node:
  ros__parameters:
    animation_mode_animations:
      - vertical_body_roll
      - horizontal_body_roll
      - body_roll_3d
"""


@pytest.fixture
def cfg(tmp_path):
    web_path = tmp_path / "webteleop.yaml"
    gait_path = tmp_path / "gait.yaml"
    posture_path = tmp_path / "posture.yaml"
    web_path.write_text(_WEBTELEOP_YAML)
    gait_path.write_text(_GAIT_YAML)
    posture_path.write_text(_POSTURE_YAML)
    loaded_cfg, initial_mode, default_gait, caps = load_web_config(
        web_path, gait_path, posture_path
    )
    return loaded_cfg, initial_mode, default_gait


def _buttons(*pressed_indices) -> tuple[int, ...]:
    out = [0] * NUM_BUTTONS
    for idx in pressed_indices:
        out[idx] = 1
    return tuple(out)


def _sticks(
    lx=0.0, ly=0.0, rx=0.0, ry=0.0
) -> tuple[tuple[float, float], tuple[float, float]]:
    return (lx, ly), (rx, ry)


# ─── Config loading ─────────────────────────────────────────────────

def test_load_config_returns_correct_initial_mode(cfg):
    _, initial_mode, _ = cfg
    assert initial_mode == "gait"


def test_load_config_button_count(cfg):
    loaded_cfg, _, _ = cfg
    assert len(loaded_cfg.base.button_index) == NUM_BUTTONS


def test_load_config_gait_cycle_filtered(cfg):
    loaded_cfg, _, _ = cfg
    # surf and crawl are unstable → filtered out with allow_unstable: false
    assert "tripod" in loaded_cfg.gait_cycle
    assert "ripple" in loaded_cfg.gait_cycle
    assert "surf" not in loaded_cfg.gait_cycle
    assert "crawl" not in loaded_cfg.gait_cycle


def test_load_config_animation_list(cfg):
    loaded_cfg, _, _ = cfg
    assert loaded_cfg.animation_list == (
        "vertical_body_roll", "horizontal_body_roll", "body_roll_3d"
    )


# ─── Button labels ──────────────────────────────────────────────────

def test_button_labels_gait_mode(cfg):
    loaded_cfg, _, _ = cfg
    labels = button_labels_for_mode(loaded_cfg, GAIT)
    assert labels == (
        "gait_mode", "posture_mode", "animation_mode",
        "init", "record", "gait_prev", "gait_next",
        "height_up", "height_down",
    )


def test_button_labels_posture_mode(cfg):
    loaded_cfg, _, _ = cfg
    labels = button_labels_for_mode(loaded_cfg, POSTURE)
    assert labels == (
        "gait_mode", "posture_mode", "animation_mode",
        "init", "record", "yaw_left", "yaw_right",
        "height_up", "height_down",
    )


def test_button_labels_animation_mode(cfg):
    loaded_cfg, _, _ = cfg
    labels = button_labels_for_mode(loaded_cfg, ANIMATION)
    assert labels == (
        "gait_mode", "posture_mode", "animation_mode",
        "init", "record", "animation_prev", "animation_next",
        "height_up", "height_down",
    )


# ─── map_web: gait mode stick mapping ───────────────────────────────

def test_gait_left_stick_y_maps_to_drive_x(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    left, right = _sticks(ly=0.5)
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    assert math.isclose(out.linear_x, 0.5 * loaded_cfg.gait_linear_max, rel_tol=1e-6)
    assert out.linear_y == 0.0


def test_gait_left_stick_x_maps_to_drive_y(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    left, right = _sticks(lx=0.5)
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    assert math.isclose(out.linear_y, 0.5 * loaded_cfg.gait_linear_max, rel_tol=1e-6)
    assert out.linear_x == 0.0


def test_gait_right_stick_x_maps_to_drive_yaw(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    left, right = _sticks(rx=0.5)
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    assert math.isclose(out.angular_z, 0.5 * loaded_cfg.gait_angular_z_max, rel_tol=1e-6)


def test_gait_deadband_zeros_small_inputs(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    left, right = _sticks(ly=0.03)  # below deadband 0.05
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    assert out.linear_x == 0.0


# ─── map_web: posture mode stick mapping ────────────────────────────

def test_posture_left_stick_maps_to_pose_xy(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=POSTURE)
    left, right = _sticks(lx=0.5, ly=0.5)
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    # left_stick_y → pose_x, left_stick_x → pose_y
    assert math.isclose(out.pose_x, 0.5 * loaded_cfg.posture.x_max, rel_tol=1e-6)
    assert math.isclose(out.pose_y, 0.5 * loaded_cfg.posture.y_max, rel_tol=1e-6)
    # cmd_vel is zero in posture mode
    assert out.linear_x == 0.0
    assert out.linear_y == 0.0
    assert out.angular_z == 0.0


def test_posture_right_stick_maps_to_tilt(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=POSTURE)
    left, right = _sticks(rx=0.5, ry=0.5)
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    # right_stick_x → tilt_roll, right_stick_y → tilt_pitch
    assert out.pose_roll != 0.0
    assert out.pose_pitch != 0.0


# ─── map_web: mode switching via top 3 buttons ──────────────────────

def test_mode_switch_gait_to_posture(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    # btn_1 = posture_mode, rising edge
    out = map_web((0, 0), (0, 0), _buttons(1), loaded_cfg, state, DT)
    assert out.mode_changed is True
    assert state.mode == POSTURE


def test_mode_switch_gait_to_animation(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    # btn_2 = animation_mode, rising edge (toggles gait ↔ animation)
    out = map_web((0, 0), (0, 0), _buttons(2), loaded_cfg, state, DT)
    assert out.mode_changed is True
    assert state.mode == ANIMATION


def test_mode_switch_no_retrigger_on_held_button(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    # First tick: rising edge
    map_web((0, 0), (0, 0), _buttons(1), loaded_cfg, state, DT)
    # Second tick: still held, no new edge
    out = map_web((0, 0), (0, 0), _buttons(1), loaded_cfg, state, DT)
    assert out.mode_changed is False


# ─── map_web: gait cycling via bottom buttons ───────────────────────

def test_gait_next_cycles_forward(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT, current_gait_idx=0)
    # btn_6 = gait_next, rising edge
    out = map_web((0, 0), (0, 0), _buttons(6), loaded_cfg, state, DT)
    assert out.gait_select is not None
    # gait_cycle filtered: [tripod, tetrapod, ripple] (surf, crawl removed)
    assert out.gait_select == "tetrapod"


def test_gait_prev_cycles_backward(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT, current_gait_idx=0)
    # btn_5 = gait_prev, rising edge
    out = map_web((0, 0), (0, 0), _buttons(5), loaded_cfg, state, DT)
    assert out.gait_select is not None
    # wraps to last
    assert out.gait_select == "ripple"


# ─── map_web: init button ───────────────────────────────────────────

def test_init_request_fires_when_posture_default(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    # btn_3 = init, rising edge
    out = map_web((0, 0), (0, 0), _buttons(3), loaded_cfg, state, DT)
    assert out.init_request is True


# ─── map_web: zero input produces zero output ───────────────────────

def test_zero_input_produces_zero_output(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    out = map_web((0, 0), (0, 0), _buttons(), loaded_cfg, state, DT)
    assert out.linear_x == 0.0
    assert out.linear_y == 0.0
    assert out.angular_z == 0.0
