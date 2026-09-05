import math
from pathlib import Path

import pytest
import yaml

from hexa_webteleop import (
    NUM_BUTTONS,
    battery_payload,
    button_labels_for_mode,
    input_is_stale,
    load_web_config,
    map_web,
    neutral_inputs,
    preset_descriptors,
    preset_payload,
    preset_pending_expired,
    resync_gait,
)
from hexa_teleop.joy_mapping import apply_deadband
from hexa_webteleop.web_mapping import GAIT, POSTURE, ANIMATION


# Independent of webteleop_node.PUBLISH_RATE_HZ — map_web is rate-correct.
# Tick counts below are written against this value.
DT = 0.02


# ─── Config fixture ────────────────────────────────────────────────

_WEBTELEOP_YAML = """
initial_mode: gait
allow_unstable_gaits: false
presets:
  default: normal
  switch_timeout_s: 4.0
  list:
    - id: normal
      label: NORMAL
      sub: six legs
      gait_cycle: [tripod, surf, tetrapod, crawl, ripple]
      default_gait: tripod
    - id: quad
      label: QUAD
      sub: four corners, middle pair parked
      gait_cycle: [quad_canter, quad_walk]
      default_gait: quad_canter

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
    btn_4: quadruped_mode
    btn_5: gait_prev
    btn_6: gait_next
    btn_7: height_up
    btn_8: height_down
    left_stick_y: drive_x
    left_stick_x: drive_y
    right_stick_x: drive_yaw
    right_stick_y: drive_x_aux

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
  height:
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
    right_stick_y: drive_x_aux

arbitration:
  enabled: true
"""

_GAIT_YAML = """
gait_node:
  ros__parameters:
    stride_length: 0.1
    min_swing_time: 0.30
    max_swing_time: 0.4
    yaw_bias: 0.6
    default_gait: tripod
    default_standing_pose:
      body_height: 0.04
      front:
        tip_reach: 0.135
        coxa_deg: 0
      middle:
        tip_reach: 0.135
        coxa_deg: 0
      rear:
        tip_reach: 0.135
        coxa_deg: 0
"""

# The angular stick cap is derived from the standing stance, so the caps loader
# needs the leg mounts too.
_GEOMETRY_YAML = """
mounts:
  l_front: { x: 0.083, y: 0.0575, yaw_deg: 30 }
  l_middle: { x: 0.00, y: 0.082, yaw_deg: 90 }
"""

_POSTURE_YAML = """
posture_node:
  ros__parameters:
    animation_mode_animations:
      - vertical_body_roll
      - horizontal_body_roll
      - body_roll_3d
    body_height_max_m: 0.13
    body_height_min_m: 0.01
    pose_limit_x: 0.05
    pose_limit_y: 0.05
    pose_limit_roll: 0.30
    pose_limit_pitch: 0.30
    pose_limit_yaw: 0.50

teleop_node:
  ros__parameters:
    posture:
      x_max: 0.035
      y_max: 0.035
      roll_max_deg: 12.0
      pitch_max_deg: 12.0
      yaw_max_deg: 20.0
      yaw_tau_s: 0.10
      revert_tau_s: 0.25
      wiggle_pivot_forward_m: 0.06
"""


def _load(tmp_path):
    web_path = tmp_path / "webteleop.yaml"
    gait_path = tmp_path / "gait.yaml"
    posture_path = tmp_path / "posture.yaml"
    geometry_path = tmp_path / "geometry.yaml"
    web_path.write_text(_WEBTELEOP_YAML)
    gait_path.write_text(_GAIT_YAML)
    posture_path.write_text(_POSTURE_YAML)
    geometry_path.write_text(_GEOMETRY_YAML)
    return load_web_config(web_path, gait_path, posture_path, geometry_path)


@pytest.fixture
def cfg(tmp_path):
    loaded_cfg, initial_mode, default_gait, _, _ = _load(tmp_path)
    return loaded_cfg, initial_mode, default_gait


@pytest.fixture
def cfg_and_caps(tmp_path):
    return _load(tmp_path)


def _buttons(*pressed_indices) -> tuple[int, ...]:
    out = [0] * NUM_BUTTONS
    for idx in pressed_indices:
        out[idx] = 1
    return tuple(out)


def _sticks(
    lx=0.0, ly=0.0, rx=0.0, ry=0.0
) -> tuple[tuple[float, float], tuple[float, float]]:
    return (lx, ly), (rx, ry)


def _stick(value: float, cfg) -> float:
    """What a stick reading is worth after the deadband rescales it.

    The routing tests below care which stick reaches which output, not what
    the input curve is, so they take the shaping from the mapping itself.
    """
    return apply_deadband(value, cfg.base.deadband)


# ─── Config loading ─────────────────────────────────────────────────

def test_load_config_returns_correct_initial_mode(cfg):
    _, initial_mode, _ = cfg
    assert initial_mode == "gait"


def test_load_config_height_limits_come_from_tuning_not_webteleop(cfg):
    """webteleop.yaml declares no height envelope; tuning.yaml owns it.

    The absolute belly heights (0.01 / 0.13) become pose offsets against the
    0.04 standing height, so the mapper saturates exactly where the posture
    stack clamps.
    """
    loaded_cfg, _, _ = cfg
    assert math.isclose(loaded_cfg.posture.height_max, 0.13 - 0.04)
    assert math.isclose(loaded_cfg.posture.height_min, 0.01 - 0.04)


def test_load_config_scalar_limits_come_from_tuning_not_webteleop(cfg):
    """webteleop.yaml declares no posture limits; tuning.yaml owns them.

    Same block the gamepad teleop reads, so both front ends pose the body
    over one range.
    """
    loaded_cfg, _, _ = cfg
    assert math.isclose(loaded_cfg.posture.x_max, 0.035)
    assert math.isclose(loaded_cfg.posture.roll_max, math.radians(12.0))
    assert math.isclose(loaded_cfg.posture.wiggle_pivot_forward_m, 0.06)


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


def test_the_two_rotations_stay_partitioned_by_leg_set(cfg):
    # The quad init is the only way onto four legs, exactly as on the gamepad,
    # and once there the cycler walks the four-leg rotation. A name in the
    # wrong list would be a prev/next the engine refuses, so load_web_config
    # rejects it — here we pin that the shipped lists are on the right sides.
    loaded_cfg, _, _ = cfg
    assert "quad_walk" not in loaded_cfg.gait_cycle
    assert "quad_canter" not in loaded_cfg.gait_cycle
    assert loaded_cfg.quadruped_gait_cycle == ("quad_canter", "quad_walk")
    assert loaded_cfg.default_quadruped_gait == "quad_canter"


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
        "init", "quadruped_mode", "gait_prev", "gait_next",
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
    assert math.isclose(out.linear_x, _stick(0.5, loaded_cfg) * loaded_cfg.gait_linear_max, rel_tol=1e-6)
    assert out.linear_y == 0.0


def test_gait_left_stick_x_maps_to_drive_y(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    left, right = _sticks(lx=0.5)
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    assert math.isclose(out.linear_y, _stick(0.5, loaded_cfg) * loaded_cfg.gait_linear_max, rel_tol=1e-6)
    assert out.linear_x == 0.0


def test_gait_right_stick_x_maps_to_drive_yaw(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    left, right = _sticks(rx=0.5)
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    assert math.isclose(out.angular_z, _stick(0.5, loaded_cfg) * loaded_cfg.gait_angular_z_max, rel_tol=1e-6)


def test_gait_right_stick_y_also_drives_forward(cfg):
    # drive_x_aux: the turning pad drives forward too, so either pad alone is
    # a complete drive control.
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    left, right = _sticks(ry=0.5)
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    assert math.isclose(
        out.linear_x, _stick(0.5, loaded_cfg) * loaded_cfg.gait_linear_max,
        rel_tol=1e-6,
    )
    assert out.linear_y == 0.0
    assert out.angular_z == 0.0


def test_gait_both_pads_sum_into_forward(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    left, right = _sticks(ly=1.0, ry=1.0)
    out = map_web(left, right, _buttons(), loaded_cfg, state, DT)
    # Clipped at the cap rather than doubling it.
    assert math.isclose(out.linear_x, loaded_cfg.gait_linear_max, rel_tol=1e-6)


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
    assert math.isclose(out.pose_x, _stick(0.5, loaded_cfg) * loaded_cfg.posture.x_max, rel_tol=1e-6)
    assert math.isclose(out.pose_y, _stick(0.5, loaded_cfg) * loaded_cfg.posture.y_max, rel_tol=1e-6)
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


# ─── map_web: quadruped init (btn_4 in gait mode) ───────────────────

def test_quadruped_init_asks_for_the_quadruped_gait(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT, current_gait_idx=1)
    out = map_web((0, 0), (0, 0), _buttons(4), loaded_cfg, state, DT)
    # The leg set rides the gait, and the init that goes with it is what
    # stands the robot up on that set.
    assert out.gait_select == "quad_canter"
    assert out.init_request is True
    assert out.init_quadruped is True
    assert state.quadruped is True


def test_quadruped_mode_cycles_its_own_rotation(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT, current_gait_idx=1)
    map_web((0, 0), (0, 0), _buttons(4), loaded_cfg, state, DT)
    map_web((0, 0), (0, 0), _buttons(), loaded_cfg, state, DT)
    assert state.quadruped is True

    # btn_6 = gait_next, rising edge — inside the four-leg rotation.
    out = map_web((0, 0), (0, 0), _buttons(6), loaded_cfg, state, DT)
    assert out.gait_select == "quad_walk"
    assert state.current_gait_idx == 1, "the six-leg slot must not drift"


def test_hexapod_init_asks_for_the_cycler_gait(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT, current_gait_idx=1)
    six_leg = loaded_cfg.gait_cycle[1]
    map_web((0, 0), (0, 0), _buttons(4), loaded_cfg, state, DT)
    map_web((0, 0), (0, 0), _buttons(), loaded_cfg, state, DT)
    # btn_3 = init, the six-leg half of the same button.
    out = map_web((0, 0), (0, 0), _buttons(3), loaded_cfg, state, DT)
    assert state.quadruped is False
    assert out.init_quadruped is False
    assert out.gait_select == six_leg


def test_quadruped_button_is_record_in_posture_mode(cfg):
    """btn_4 is `quadruped_mode` only in the gait section.

    The shared mapping resolves that function against the gait bindings in
    every mode to keep its edge tracker honest, but only acts on it in gait
    mode — so a posture-mode press of the same button must record the pose
    and leave the leg set alone.
    """
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=POSTURE)
    # Hold a pose offset on the left stick, then record it.
    map_web((0.0, 0.5), (0, 0), _buttons(), loaded_cfg, state, DT)
    out = map_web((0.0, 0.5), (0, 0), _buttons(4), loaded_cfg, state, DT)
    assert out.gait_select is None
    assert out.init_request is False
    assert out.init_quadruped is False
    assert state.quadruped is False
    assert state.recorded_x != 0.0


def test_quadruped_button_held_across_a_mode_switch_does_not_fire(cfg):
    """No spurious init when the button enters gait mode already held.

    The press starts in posture mode (where it means `record`); the edge
    tracker sees it there, so arriving in gait mode is not a rising edge.
    """
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=POSTURE)
    map_web((0, 0), (0, 0), _buttons(4), loaded_cfg, state, DT)
    # btn_0 = gait_mode, with btn_4 still held.
    map_web((0, 0), (0, 0), _buttons(0, 4), loaded_cfg, state, DT)
    assert state.mode == GAIT
    out = map_web((0, 0), (0, 0), _buttons(4), loaded_cfg, state, DT)
    assert out.init_quadruped is False
    assert state.quadruped is False


# ─── resync_gait: external /cmd_gait switches ───────────────────────

def test_resync_gait_updates_caps_and_cycler(cfg_and_caps):
    loaded_cfg, _, _, caps, registry = cfg_and_caps
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT, current_gait_idx=0)
    new_cfg = resync_gait("ripple", loaded_cfg, state, caps, registry)
    assert new_cfg is not None
    assert state.current_gait_idx == loaded_cfg.gait_cycle.index("ripple")
    assert math.isclose(new_cfg.gait_linear_max, caps.linear_max("ripple"))
    assert math.isclose(new_cfg.gait_angular_z_max, caps.angular_max("ripple"))


def test_resync_gait_unknown_name_returns_none(cfg_and_caps):
    # A foreign string on /cmd_gait: no cfg, cycler untouched.
    loaded_cfg, _, _, caps, registry = cfg_and_caps
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT, current_gait_idx=1)
    assert resync_gait("moonwalk", loaded_cfg, state, caps, registry) is None
    assert state.current_gait_idx == 1


def test_resync_gait_outside_cycle_updates_caps_only(cfg_and_caps):
    # crawl is in the catalog but filtered from this config's gait_cycle
    # (unstable) — the gamepad may still command it. Caps follow; the
    # cycler keeps its old position.
    loaded_cfg, _, _, caps, registry = cfg_and_caps
    assert "crawl" not in loaded_cfg.gait_cycle
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT, current_gait_idx=2)
    new_cfg = resync_gait("crawl", loaded_cfg, state, caps, registry)
    assert new_cfg is not None
    assert state.current_gait_idx == 2
    assert math.isclose(new_cfg.gait_linear_max, caps.linear_max("crawl"))


def test_resync_gait_own_loopback_is_idempotent(cfg_and_caps):
    # The node hears its own accepted publish back via loopback; map_joy
    # already advanced the cycler on the press, so the resync must land
    # on the same slot.
    loaded_cfg, _, _, caps, registry = cfg_and_caps
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT, current_gait_idx=0)
    out = map_web((0, 0), (0, 0), _buttons(6), loaded_cfg, state, DT)  # gait_next
    assert out.gait_select is not None
    idx_after_press = state.current_gait_idx
    new_cfg = resync_gait(out.gait_select, loaded_cfg, state, caps, registry)
    assert new_cfg is not None
    assert state.current_gait_idx == idx_after_press


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


# ─── Safety watchdog ────────────────────────────────────────────────

def test_input_is_stale_after_timeout():
    # last input at t=10.0, now t=10.6, timeout 0.5 → stale
    assert input_is_stale(10.0, 10.6, 0.5) is True


def test_input_is_fresh_within_timeout():
    assert input_is_stale(10.0, 10.4, 0.5) is False


def test_input_is_stale_at_startup():
    # last-input seed of 0.0 reads stale against any real monotonic clock
    assert input_is_stale(0.0, 1234.5, 0.5) is True


def test_battery_payload_reports_a_fresh_reading():
    assert battery_payload((8.3, 1.5, 100.0), 100.4, 5.0) == {
        "voltage": 8.3,
        "current": 1.5,
    }


def test_battery_payload_is_unknown_before_the_first_reading():
    assert battery_payload(None, 100.0, 5.0) == {"voltage": None, "current": None}


def test_battery_payload_goes_unknown_when_stale():
    # Hardware node died at t=100; at t=106 the last reading is a lie, not
    # merely old — the strip must fall back to a dash.
    assert battery_payload((8.3, 1.5, 100.0), 106.0, 5.0) == {
        "voltage": None,
        "current": None,
    }


def test_battery_payload_drops_non_finite_fields():
    # A bare NaN in the JSON would throw in the client's JSON.parse and take
    # the whole message with it.
    payload = battery_payload(
        (float("nan"), float("inf"), 100.0), 100.1, 5.0
    )
    assert payload == {"voltage": None, "current": None}
    import json

    assert json.loads(json.dumps(payload)) == payload


def test_neutral_inputs_are_centred_and_released():
    left, right, buttons = neutral_inputs()
    assert left == (0.0, 0.0)
    assert right == (0.0, 0.0)
    assert buttons == (0,) * NUM_BUTTONS


def test_neutral_inputs_map_to_zero_velocity(cfg):
    loaded_cfg, _, _ = cfg
    from hexa_teleop.joy_mapping import JoyState
    state = JoyState(mode=GAIT)
    # Drive forward, then apply the watchdog's neutral inputs: cmd_vel zeroes.
    map_web((0.0, 0.8), (0.0, 0.0), _buttons(), loaded_cfg, state, DT)
    left, right, buttons = neutral_inputs()
    out = map_web(left, right, buttons, loaded_cfg, state, DT)
    assert out.linear_x == 0.0
    assert out.linear_y == 0.0
    assert out.angular_z == 0.0


# ─── Mode view payloads ─────────────────────────────────────────────

def test_preset_payload_reads_the_applied_leg_set(cfg_and_caps):
    _, _, _, _, registry = cfg_and_caps
    payload = preset_payload(registry, "quadruped", None, None)
    assert payload["active"] == "quad"
    assert payload["leg_set"] == "quadruped"
    assert payload["pending"] is None
    assert payload["refused"] is None

    payload = preset_payload(registry, "hexapod", None, None)
    assert payload["active"] == "normal"


def test_preset_payload_is_blank_before_the_first_leg_set(cfg_and_caps):
    # /gait/leg_set is latched, but nothing has published it yet in sim before
    # the locomotion node starts. Showing no row beats guessing at one.
    _, _, _, _, registry = cfg_and_caps
    payload = preset_payload(registry, "", None, None)
    assert payload["active"] is None
    assert payload["leg_set"] is None


def test_preset_payload_carries_pending_and_refused(cfg_and_caps):
    _, _, _, _, registry = cfg_and_caps
    payload = preset_payload(registry, "hexapod", "quad", None)
    assert payload["active"] == "normal"
    assert payload["pending"] == "quad"

    payload = preset_payload(registry, "hexapod", None, "not while walking")
    assert payload["refused"] == "not while walking"
    # The active row never moves on a refusal — it is still what the robot is.
    assert payload["active"] == "normal"


def test_preset_descriptors_list_every_preset(cfg_and_caps):
    _, _, _, _, registry = cfg_and_caps
    rows = preset_descriptors(registry)
    assert [r["id"] for r in rows] == ["normal", "quad"]
    assert rows[0]["label"] == "NORMAL"
    assert rows[1]["leg_set"] == "quadruped"


def test_preset_pending_expired():
    # /gait/leg_set publishes on change only, so a refusal the node could not
    # predict arrives as silence; past the deadline that is the answer.
    assert preset_pending_expired(10.0, 10.5) is True
    assert preset_pending_expired(10.0, 9.5) is False
    # Nothing pending never expires.
    assert preset_pending_expired(None, 1e9) is False
