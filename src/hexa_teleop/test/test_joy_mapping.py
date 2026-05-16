import math

from hexa_teleop import GAIT, POSTURE, JoyConfig, JoyState, apply_deadband, map_joy


def _cfg(**overrides) -> JoyConfig:
    base = dict(
        axis_left_x=0,
        axis_left_y=1,
        axis_right_x=3,
        axis_right_y=4,
        mode_toggle_button=3,
        deadband=0.1,
        gait_linear_x_max=0.3,
        gait_linear_y_max=0.2,
        gait_angular_z_max=1.0,
        posture_x_max=0.05,
        posture_y_max=0.05,
    )
    base.update(overrides)
    return JoyConfig(**base)


def _axes(left_x=0.0, left_y=0.0, right_x=0.0, right_y=0.0) -> tuple[float, ...]:
    return (left_x, left_y, 0.0, right_x, right_y, 0.0, 0.0, 0.0)


def _buttons(toggle: bool = False) -> tuple[int, ...]:
    out = [0] * 11
    out[3] = int(toggle)
    return tuple(out)


def test_deadband_zeros_small_inputs():
    assert apply_deadband(0.05, 0.1) == 0.0
    assert apply_deadband(-0.05, 0.1) == 0.0
    assert apply_deadband(0.0, 0.1) == 0.0


def test_deadband_passes_through_above_threshold():
    assert math.isclose(apply_deadband(0.5, 0.1), 0.5)
    assert math.isclose(apply_deadband(-0.5, 0.1), -0.5)


def test_posture_right_stick_maps_to_body_xy_scaled():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    out = map_joy(_axes(right_x=1.0, right_y=1.0), _buttons(), cfg, state)
    # stick forward (right_y=+1) -> body +x; stick left (right_x=+1) -> body +y
    assert math.isclose(out.pose_x, cfg.posture_x_max)
    assert math.isclose(out.pose_y, cfg.posture_y_max)
    # gait channel stays zero in posture mode
    assert out.linear_x == 0.0
    assert out.linear_y == 0.0
    assert out.angular_z == 0.0


def test_posture_left_stick_is_ignored():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    out = map_joy(_axes(left_x=1.0, left_y=1.0), _buttons(), cfg, state)
    assert out.pose_x == 0.0
    assert out.pose_y == 0.0
    assert out.angular_z == 0.0


def test_gait_left_stick_x_drives_angular_z():
    cfg = _cfg()
    state = JoyState(mode=GAIT)
    out = map_joy(_axes(left_x=1.0), _buttons(), cfg, state)
    assert math.isclose(out.angular_z, cfg.gait_angular_z_max)
    assert out.linear_x == 0.0
    assert out.linear_y == 0.0
    # posture channel stays zero in gait mode
    assert out.pose_x == 0.0
    assert out.pose_y == 0.0


def test_gait_right_stick_drives_linear_xy():
    cfg = _cfg()
    state = JoyState(mode=GAIT)
    out = map_joy(_axes(right_x=0.5, right_y=1.0), _buttons(), cfg, state)
    assert math.isclose(out.linear_x, cfg.gait_linear_x_max)
    assert math.isclose(out.linear_y, 0.5 * cfg.gait_linear_y_max)
    assert out.angular_z == 0.0


def test_deadband_applied_before_scaling():
    cfg = _cfg(deadband=0.2)
    state = JoyState(mode=GAIT)
    # 0.15 magnitude is inside the deadband -> zero output
    out = map_joy(_axes(right_y=0.15), _buttons(), cfg, state)
    assert out.linear_x == 0.0


def test_mode_toggle_flips_on_rising_edge():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, prev_toggle=False)

    out = map_joy(_axes(), _buttons(toggle=True), cfg, state)
    assert out.mode_changed is True
    assert state.mode == GAIT

    out = map_joy(_axes(), _buttons(toggle=False), cfg, state)
    assert out.mode_changed is False
    assert state.mode == GAIT


def test_holding_toggle_does_not_retoggle():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, prev_toggle=False)

    # Initial press: flip
    map_joy(_axes(), _buttons(toggle=True), cfg, state)
    assert state.mode == GAIT

    # Held: must NOT flip again
    for _ in range(10):
        out = map_joy(_axes(), _buttons(toggle=True), cfg, state)
        assert out.mode_changed is False
        assert state.mode == GAIT

    # Release then press again: flip back
    map_joy(_axes(), _buttons(toggle=False), cfg, state)
    out = map_joy(_axes(), _buttons(toggle=True), cfg, state)
    assert out.mode_changed is True
    assert state.mode == POSTURE


def test_short_joy_message_does_not_crash():
    cfg = _cfg()
    state = JoyState(mode=GAIT)
    # Empty axes/buttons — should not raise; all outputs zero.
    out = map_joy((), (), cfg, state)
    assert out.linear_x == 0.0
    assert out.linear_y == 0.0
    assert out.angular_z == 0.0
    assert out.pose_x == 0.0
    assert out.pose_y == 0.0
    assert out.mode_changed is False
