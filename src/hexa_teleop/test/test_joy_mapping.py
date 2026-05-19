import math

from hexa_teleop import GAIT, POSTURE, JoyConfig, JoyState, apply_deadband, map_joy


DT = 0.02  # matches teleop_joy.PUBLISH_RATE_HZ = 50 Hz


def _cfg(**overrides) -> JoyConfig:
    base = dict(
        axis_left_x=0,
        axis_left_y=1,
        axis_right_x=3,
        axis_right_y=4,
        axis_dpad_y=7,
        dpad_up_sign=1.0,
        mode_toggle_button=3,
        init_button=7,
        yaw_left_button=4,
        yaw_right_button=5,
        wiggle_left_trigger_axis=2,
        wiggle_right_trigger_axis=5,
        wiggle_trigger_threshold=0.5,
        deadband=0.1,
        gait_linear_max=0.4,
        gait_angular_z_max=1.0,
        posture_x_max=0.05,
        posture_y_max=0.05,
        posture_roll_max=math.radians(15.0),
        posture_pitch_max=math.radians(15.0),
        posture_yaw_max=math.radians(20.0),
        posture_yaw_tau=0.10,
        posture_wiggle_pivot_forward_m=0.06,
        posture_height_max=0.04,
        posture_height_min=-0.04,
        posture_height_rate=0.05,
    )
    base.update(overrides)
    return JoyConfig(**base)


def _axes(
    left_x=0.0,
    left_y=0.0,
    right_x=0.0,
    right_y=0.0,
    lt=1.0,
    rt=1.0,
    dpad_y=0.0,
) -> tuple[float, ...]:
    # Trigger rest value is +1.0 (joy_node Xbox-style convention),
    # so defaults read as "not pressed". D-pad Y rest value is 0.0.
    return (left_x, left_y, lt, right_x, right_y, rt, 0.0, dpad_y)


def _buttons(
    toggle: bool = False,
    init: bool = False,
    yaw_left: bool = False,
    yaw_right: bool = False,
) -> tuple[int, ...]:
    out = [0] * 11
    out[3] = int(toggle)
    out[4] = int(yaw_left)
    out[5] = int(yaw_right)
    out[7] = int(init)
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
    out = map_joy(_axes(right_x=1.0, right_y=1.0), _buttons(), cfg, state, DT)
    # stick forward (right_y=+1) -> body +x; stick left (right_x=+1) -> body +y
    assert math.isclose(out.pose_x, cfg.posture_x_max)
    assert math.isclose(out.pose_y, cfg.posture_y_max)
    # right stick alone leaves tilt at zero
    assert out.pose_roll == 0.0
    assert out.pose_pitch == 0.0
    assert out.pose_yaw == 0.0
    # gait channel stays zero in posture mode
    assert out.linear_x == 0.0
    assert out.linear_y == 0.0
    assert out.angular_z == 0.0


def test_posture_left_stick_x_drives_roll():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    out = map_joy(_axes(left_x=1.0), _buttons(), cfg, state, DT)
    # stick left (left_x=+1) -> body tilts left (left side dips),
    # which is negative roll about +x.
    assert math.isclose(out.pose_roll, -cfg.posture_roll_max)
    assert out.pose_pitch == 0.0
    assert out.pose_x == 0.0
    assert out.pose_y == 0.0
    assert out.angular_z == 0.0


def test_posture_left_stick_y_drives_pitch():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    out = map_joy(_axes(left_y=1.0), _buttons(), cfg, state, DT)
    # stick forward (left_y=+1) -> body tilts forward (front dips),
    # which is positive pitch about +y.
    assert math.isclose(out.pose_pitch, cfg.posture_pitch_max)
    assert out.pose_roll == 0.0
    assert out.pose_x == 0.0
    assert out.pose_y == 0.0


def test_posture_both_sticks_apply_simultaneously():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    out = map_joy(
        _axes(left_x=1.0, left_y=1.0, right_x=1.0, right_y=1.0),
        _buttons(),
        cfg,
        state,
        DT,
    )
    assert math.isclose(out.pose_x, cfg.posture_x_max)
    assert math.isclose(out.pose_y, cfg.posture_y_max)
    assert math.isclose(out.pose_roll, -cfg.posture_roll_max)
    assert math.isclose(out.pose_pitch, cfg.posture_pitch_max)


def test_gait_left_stick_x_drives_angular_z():
    cfg = _cfg()
    state = JoyState(mode=GAIT)
    out = map_joy(_axes(left_x=1.0), _buttons(), cfg, state, DT)
    assert math.isclose(out.angular_z, cfg.gait_angular_z_max)
    assert out.linear_x == 0.0
    assert out.linear_y == 0.0
    # posture channel stays zero in gait mode
    assert out.pose_x == 0.0
    assert out.pose_y == 0.0
    assert out.pose_yaw == 0.0
    assert out.pose_roll == 0.0
    assert out.pose_pitch == 0.0


def test_gait_right_stick_drives_linear_xy():
    cfg = _cfg()
    state = JoyState(mode=GAIT)
    out = map_joy(_axes(right_x=0.5, right_y=1.0), _buttons(), cfg, state, DT)
    # Linear cap is isotropic — same scale for x and y.
    assert math.isclose(out.linear_x, cfg.gait_linear_max)
    assert math.isclose(out.linear_y, 0.5 * cfg.gait_linear_max)
    assert out.angular_z == 0.0


def test_deadband_applied_before_scaling():
    cfg = _cfg(deadband=0.2)
    state = JoyState(mode=GAIT)
    # 0.15 magnitude is inside the deadband -> zero output
    out = map_joy(_axes(right_y=0.15), _buttons(), cfg, state, DT)
    assert out.linear_x == 0.0


def test_mode_toggle_flips_on_rising_edge():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, prev_toggle=False)

    out = map_joy(_axes(), _buttons(toggle=True), cfg, state, DT)
    assert out.mode_changed is True
    assert state.mode == GAIT

    out = map_joy(_axes(), _buttons(toggle=False), cfg, state, DT)
    assert out.mode_changed is False
    assert state.mode == GAIT


def test_holding_toggle_does_not_retoggle():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, prev_toggle=False)

    # Initial press: flip
    map_joy(_axes(), _buttons(toggle=True), cfg, state, DT)
    assert state.mode == GAIT

    # Held: must NOT flip again
    for _ in range(10):
        out = map_joy(_axes(), _buttons(toggle=True), cfg, state, DT)
        assert out.mode_changed is False
        assert state.mode == GAIT

    # Release then press again: flip back
    map_joy(_axes(), _buttons(toggle=False), cfg, state, DT)
    out = map_joy(_axes(), _buttons(toggle=True), cfg, state, DT)
    assert out.mode_changed is True
    assert state.mode == POSTURE


def test_short_joy_message_does_not_crash():
    cfg = _cfg()
    state = JoyState(mode=GAIT)
    # Empty axes/buttons — should not raise; all outputs zero.
    out = map_joy((), (), cfg, state, DT)
    assert out.linear_x == 0.0
    assert out.linear_y == 0.0
    assert out.angular_z == 0.0
    assert out.pose_x == 0.0
    assert out.pose_y == 0.0
    assert out.pose_yaw == 0.0
    assert out.pose_roll == 0.0
    assert out.pose_pitch == 0.0
    assert out.mode_changed is False
    assert out.init_request is False


def test_init_request_fires_on_rising_edge():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, prev_init=False)

    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is True

    # Held: must NOT re-fire while the button stays down.
    for _ in range(5):
        out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
        assert out.init_request is False

    # Release: still no fire.
    out = map_joy(_axes(), _buttons(init=False), cfg, state, DT)
    assert out.init_request is False

    # Re-press after release: fires again.
    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is True


def test_init_request_independent_of_mode():
    # The init button works the same in posture and gait modes — the
    # cold-start gate is orthogonal to teleop modes.
    cfg = _cfg()
    for mode in (POSTURE, GAIT):
        state = JoyState(mode=mode, prev_init=False)
        out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
        assert out.init_request is True


def test_posture_yaw_button_eases_toward_max():
    # L1 held: yaw climbs monotonically toward +posture_yaw_max but does
    # NOT snap there on the first tick — that's the whole point of the
    # easing. At tau=0.10s and dt=0.02s, alpha≈0.181, so one tick should
    # land at ~18% of the cap and many ticks asymptote to the cap.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)

    out = map_joy(_axes(), _buttons(yaw_left=True), cfg, state, DT)
    assert 0.0 < out.pose_yaw < cfg.posture_yaw_max
    assert math.isclose(out.pose_yaw, cfg.posture_yaw_max * 0.18126, abs_tol=1e-4)

    # Saturate by holding for several time constants.
    for _ in range(200):
        out = map_joy(_axes(), _buttons(yaw_left=True), cfg, state, DT)
    assert math.isclose(out.pose_yaw, cfg.posture_yaw_max, rel_tol=1e-6)


def test_posture_yaw_right_button_is_negative():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    for _ in range(200):
        out = map_joy(_axes(), _buttons(yaw_right=True), cfg, state, DT)
    assert math.isclose(out.pose_yaw, -cfg.posture_yaw_max, rel_tol=1e-6)


def test_posture_yaw_both_buttons_cancel_to_zero():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, yaw_current=cfg.posture_yaw_max)
    # Both pressed -> target 0; output eases down from saturated state.
    out = map_joy(_axes(), _buttons(yaw_left=True, yaw_right=True), cfg, state, DT)
    assert 0.0 < out.pose_yaw < cfg.posture_yaw_max


def test_posture_yaw_eases_back_to_zero_on_release():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, yaw_current=cfg.posture_yaw_max)
    for _ in range(200):
        out = map_joy(_axes(), _buttons(), cfg, state, DT)
    assert math.isclose(out.pose_yaw, 0.0, abs_tol=1e-6)


def test_posture_yaw_inactive_in_gait_mode():
    # Pressing yaw buttons in gait mode must not produce a /body/pose
    # yaw offset — output stays at zero regardless of yaw state.
    cfg = _cfg()
    state = JoyState(mode=GAIT, yaw_current=cfg.posture_yaw_max)
    out = map_joy(_axes(), _buttons(yaw_left=True), cfg, state, DT)
    assert out.pose_yaw == 0.0
    # And the held state bleeds off so a mode flip back to posture
    # doesn't resurrect a stale offset.
    assert state.yaw_current < cfg.posture_yaw_max


# ---- Wiggle (L2 / R2) ----------------------------------------------------


def _press_lt(value: float = -1.0):
    """LT axis value below threshold (0.5) reads as 'pressed'."""
    return _axes(lt=value)


def _press_rt(value: float = -1.0):
    return _axes(rt=value)


def test_wiggle_lt_saturates_yaw_and_translates():
    # Holding L2 alone eases yaw to +posture_yaw_max (same as L1) and
    # adds the pivot-keeping translation. Steady-state values match
    # the closed-form (1 - cos θ, -sin θ) * pivot * 1.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    for _ in range(400):
        out = map_joy(_press_lt(), _buttons(), cfg, state, DT)
    px = cfg.posture_wiggle_pivot_forward_m
    assert math.isclose(out.pose_yaw, cfg.posture_yaw_max, rel_tol=1e-6)
    assert math.isclose(state.wiggle_amount, 1.0, rel_tol=1e-6)
    assert math.isclose(out.pose_x, px * (1.0 - math.cos(cfg.posture_yaw_max)))
    assert math.isclose(out.pose_y, -px * math.sin(cfg.posture_yaw_max))


def test_wiggle_rt_mirrors_lt():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    for _ in range(400):
        out = map_joy(_press_rt(), _buttons(), cfg, state, DT)
    px = cfg.posture_wiggle_pivot_forward_m
    assert math.isclose(out.pose_yaw, -cfg.posture_yaw_max, rel_tol=1e-6)
    # sin is odd, (1 - cos) is even, so x bob is the same direction
    # regardless of which trigger is held — the front always rolls
    # forward a hair as the rear swings.
    assert math.isclose(out.pose_x, px * (1.0 - math.cos(cfg.posture_yaw_max)))
    assert math.isclose(out.pose_y, -px * math.sin(-cfg.posture_yaw_max))


def test_wiggle_pivot_point_stays_stationary_during_ramp():
    # Compose translation + yaw on the pivot point (px, 0) on every
    # tick. With the eased ramp it should never drift more than float
    # noise from its starting position.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    px = cfg.posture_wiggle_pivot_forward_m
    for _ in range(50):
        out = map_joy(_press_lt(), _buttons(), cfg, state, DT)
        # Apply body offset to body-frame point (px, 0): rotate by
        # yaw, then translate.
        c, s = math.cos(out.pose_yaw), math.sin(out.pose_yaw)
        world_x = out.pose_x + c * px
        world_y = out.pose_y + s * px
        # During the easing ramp, wiggle_amount may still be < 1 so
        # the translation undercompensates the rotation slightly; the
        # drift is bounded by px * (1 - wiggle_amount), which the same
        # low-pass squeezes to zero.
        drift_bound = px * (1.0 - state.wiggle_amount) + 1e-9
        assert abs(world_x - px) <= drift_bound + 1e-9
        assert abs(world_y) <= drift_bound + 1e-9
    # By the end of the ramp the pivot is essentially planted.
    assert math.isclose(world_x, px, abs_tol=1e-6)
    assert math.isclose(world_y, 0.0, abs_tol=1e-6)


def test_wiggle_l1_plus_lt_does_not_stack_yaw():
    # L1 + L2 held together — yaw target is shared, so the steady
    # state matches L1 alone (posture_yaw_max), but the wiggle scalar
    # ramps to 1 so the translation is present.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    for _ in range(400):
        out = map_joy(_press_lt(), _buttons(yaw_left=True), cfg, state, DT)
    assert math.isclose(out.pose_yaw, cfg.posture_yaw_max, rel_tol=1e-6)
    assert math.isclose(state.wiggle_amount, 1.0, rel_tol=1e-6)
    px = cfg.posture_wiggle_pivot_forward_m
    assert math.isclose(out.pose_y, -px * math.sin(cfg.posture_yaw_max))


def test_wiggle_eases_back_on_release():
    cfg = _cfg()
    state = JoyState(
        mode=POSTURE,
        yaw_current=cfg.posture_yaw_max,
        wiggle_amount=1.0,
    )
    for _ in range(400):
        out = map_joy(_axes(), _buttons(), cfg, state, DT)
    assert math.isclose(state.wiggle_amount, 0.0, abs_tol=1e-6)
    assert math.isclose(out.pose_x, 0.0, abs_tol=1e-6)
    assert math.isclose(out.pose_y, 0.0, abs_tol=1e-6)
    assert math.isclose(out.pose_yaw, 0.0, abs_tol=1e-6)


def test_wiggle_inactive_in_gait_mode_but_state_bleeds():
    # In gait mode the trigger must not produce any pose output, and
    # the wiggle_amount state should ease toward zero so a flip back
    # to posture doesn't resurrect the wiggle.
    cfg = _cfg()
    state = JoyState(mode=GAIT, yaw_current=cfg.posture_yaw_max, wiggle_amount=1.0)
    out = map_joy(_press_lt(), _buttons(), cfg, state, DT)
    assert out.pose_x == 0.0
    assert out.pose_y == 0.0
    assert out.pose_yaw == 0.0
    assert state.wiggle_amount < 1.0
    assert state.yaw_current < cfg.posture_yaw_max


def test_wiggle_trigger_threshold_respected():
    # An axis value just above the threshold must NOT count as pressed.
    cfg = _cfg(wiggle_trigger_threshold=0.5)
    state = JoyState(mode=POSTURE)
    # 0.6 > 0.5 → released
    for _ in range(50):
        out = map_joy(_press_lt(0.6), _buttons(), cfg, state, DT)
    assert state.wiggle_amount == 0.0
    assert out.pose_yaw == 0.0
    # 0.4 < 0.5 → pressed; yaw + wiggle start to ramp on the next call.
    out = map_joy(_press_lt(0.4), _buttons(), cfg, state, DT)
    assert state.wiggle_amount > 0.0
    assert out.pose_yaw > 0.0


# ---- D-pad-driven body height ----------------------------------------------


def test_dpad_up_held_in_posture_integrates_height_up():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    # Hold D-pad up (+1) for 1 s — height integrates at rate * dt every
    # tick; 50 ticks at dt=0.02 and rate=0.05 m/s gives 1.0 m * 0.05 =
    # 0.05 m, which the clamp pins to posture_height_max = 0.04.
    for _ in range(50):
        out = map_joy(_axes(dpad_y=1.0), _buttons(), cfg, state, DT)
    assert math.isclose(state.height_current, cfg.posture_height_max)
    assert math.isclose(out.pose_z, cfg.posture_height_max)


def test_dpad_down_held_in_posture_integrates_height_down():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    for _ in range(50):
        out = map_joy(_axes(dpad_y=-1.0), _buttons(), cfg, state, DT)
    assert math.isclose(state.height_current, cfg.posture_height_min)
    assert math.isclose(out.pose_z, cfg.posture_height_min)


def test_dpad_release_holds_height():
    # After lifting halfway and releasing, the height stays put — it
    # does not decay like the other posture axes.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    # Lift for 10 ticks: 10 * 0.02 * 0.05 = 0.010 m
    for _ in range(10):
        map_joy(_axes(dpad_y=1.0), _buttons(), cfg, state, DT)
    held = state.height_current
    assert held > 0.0
    # Now release the D-pad — many ticks should not change height.
    for _ in range(200):
        out = map_joy(_axes(dpad_y=0.0), _buttons(), cfg, state, DT)
    assert math.isclose(state.height_current, held)
    assert math.isclose(out.pose_z, held)


def test_dpad_inactive_in_gait_mode_but_height_bleeds_through():
    # In GAIT mode the D-pad must NOT change the height (walking-time
    # height adjustments would force a reseat mid-walk). The already-
    # integrated height bleeds through unchanged into pose.z so the
    # robot walks at the lifted posture.
    cfg = _cfg()
    state = JoyState(mode=GAIT, height_current=0.02)
    out = map_joy(_axes(dpad_y=1.0), _buttons(), cfg, state, DT)
    assert math.isclose(state.height_current, 0.02)
    assert math.isclose(out.pose_z, 0.02)


def test_dpad_sign_can_be_flipped_via_config():
    # joy_node's sign on the D-pad Y axis varies by driver / build. If
    # the YAML's dpad_up_sign needs flipping to match the live joystick,
    # the integration sign follows.
    cfg = _cfg(dpad_up_sign=-1.0)
    state = JoyState(mode=POSTURE)
    out = map_joy(_axes(dpad_y=1.0), _buttons(), cfg, state, DT)
    # With dpad_up_sign=-1, +1 on the axis should LOWER the body.
    assert state.height_current < 0.0
    assert out.pose_z < 0.0


def test_height_clamps_at_max():
    # Once the integrator pins to max, additional held ticks do not
    # accumulate.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=cfg.posture_height_max)
    for _ in range(50):
        map_joy(_axes(dpad_y=1.0), _buttons(), cfg, state, DT)
    assert math.isclose(state.height_current, cfg.posture_height_max)


def test_height_clamps_at_min():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=cfg.posture_height_min)
    for _ in range(50):
        map_joy(_axes(dpad_y=-1.0), _buttons(), cfg, state, DT)
    assert math.isclose(state.height_current, cfg.posture_height_min)


def test_mode_toggle_preserves_height():
    # The whole point of height: it must survive a POSTURE → GAIT
    # toggle so the robot walks at the lifted posture.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=0.03)
    out = map_joy(_axes(), _buttons(toggle=True), cfg, state, DT)
    assert state.mode == GAIT
    assert math.isclose(state.height_current, 0.03)
    assert math.isclose(out.pose_z, 0.03)


# ---- Start button two-press semantics --------------------------------------


def test_start_at_zero_height_fires_init_request_in_stand():
    # Existing behaviour preserved: when height is at default, Start
    # publishes /gait/initialize as before so the gait engine can
    # fold or initialize per its current state.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=0.0)
    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is True
    assert state.height_current == 0.0


def test_start_at_nonzero_height_snaps_to_zero_and_suppresses_init():
    # New: first press while lifted just snaps height back to default
    # (the gait engine then runs its reseat ladder). No fold yet.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=0.03)
    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is False
    assert state.height_current == 0.0


def test_two_press_start_from_lifted_state():
    # Press 1 snaps height; press 2 (after release) fires init_request.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=0.03)

    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is False
    assert state.height_current == 0.0

    # Release the button so the next press is a rising edge.
    out = map_joy(_axes(), _buttons(init=False), cfg, state, DT)
    assert out.init_request is False

    # Second press: at zero height now, so init_request fires.
    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is True


def test_holding_start_at_nonzero_height_snaps_once():
    # Snap is a rising-edge action — holding the button doesn't
    # repeatedly clobber height (a brief D-pad press after the snap
    # should re-integrate, not be wiped on every tick).
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=0.03)
    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert state.height_current == 0.0
    # Held: even with D-pad held up, the integrator now climbs from 0.
    out = map_joy(_axes(dpad_y=1.0), _buttons(init=True), cfg, state, DT)
    assert state.height_current > 0.0
    assert out.init_request is False


def test_short_joy_message_zero_pose_z():
    # Regression: the pose_z field exists on JoyOutput and is zero for
    # the empty-input case.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    out = map_joy((), (), cfg, state, DT)
    assert out.pose_z == 0.0
    assert state.height_current == 0.0
