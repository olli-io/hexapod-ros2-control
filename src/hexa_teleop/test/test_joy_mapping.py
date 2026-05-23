import math

from hexa_teleop import (
    ANIMATION,
    GAIT,
    POSTURE,
    JoyConfig,
    JoyState,
    apply_deadband,
    map_joy,
)


DT = 0.02  # matches teleop_joy.PUBLISH_RATE_HZ = 50 Hz


def _cfg(**overrides) -> JoyConfig:
    base = dict(
        axis_left_x=0,
        axis_left_y=1,
        axis_right_x=3,
        axis_right_y=4,
        axis_dpad_x=6,
        axis_dpad_y=7,
        dpad_up_sign=1.0,
        dpad_right_sign=1.0,
        gait_mode_button=0,
        posture_mode_button=3,
        animation_mode_button=1,
        init_button=7,
        record_button=6,
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
        posture_revert_tau=0.50,
        posture_wiggle_pivot_forward_m=0.06,
        posture_height_max=0.04,
        posture_height_min=-0.04,
        posture_height_rate=0.05,
        gait_cycle=("wave", "ripple", "tripod"),
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
    dpad_x=0.0,
    dpad_y=0.0,
) -> tuple[float, ...]:
    # Trigger rest value is +1.0 (joy_node Xbox-style convention),
    # so defaults read as "not pressed". D-pad rest values are 0.0.
    return (left_x, left_y, lt, right_x, right_y, rt, dpad_x, dpad_y)


def _buttons(
    gait_mode: bool = False,
    posture_mode: bool = False,
    init: bool = False,
    record: bool = False,
    yaw_left: bool = False,
    yaw_right: bool = False,
) -> tuple[int, ...]:
    out = [0] * 11
    out[0] = int(gait_mode)
    out[3] = int(posture_mode)
    out[4] = int(yaw_left)
    out[5] = int(yaw_right)
    out[6] = int(record)
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


def test_gait_button_selects_gait_mode_on_rising_edge():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)

    out = map_joy(_axes(), _buttons(gait_mode=True), cfg, state, DT)
    assert out.mode_changed is True
    assert state.mode == GAIT

    out = map_joy(_axes(), _buttons(gait_mode=False), cfg, state, DT)
    assert out.mode_changed is False
    assert state.mode == GAIT


def test_posture_button_selects_posture_mode_on_rising_edge():
    cfg = _cfg()
    state = JoyState(mode=GAIT)

    out = map_joy(_axes(), _buttons(posture_mode=True), cfg, state, DT)
    assert out.mode_changed is True
    assert state.mode == POSTURE

    out = map_joy(_axes(), _buttons(posture_mode=False), cfg, state, DT)
    assert out.mode_changed is False
    assert state.mode == POSTURE


def test_mode_button_for_active_mode_is_noop():
    cfg = _cfg()
    state = JoyState(mode=GAIT)

    out = map_joy(_axes(), _buttons(gait_mode=True), cfg, state, DT)
    assert out.mode_changed is False
    assert state.mode == GAIT

    state.mode = POSTURE
    state.prev_posture_mode = False
    out = map_joy(_axes(), _buttons(posture_mode=True), cfg, state, DT)
    assert out.mode_changed is False
    assert state.mode == POSTURE


def test_holding_mode_button_does_not_retrigger():
    cfg = _cfg()
    state = JoyState(mode=POSTURE)

    # Initial press: switch to gait
    map_joy(_axes(), _buttons(gait_mode=True), cfg, state, DT)
    assert state.mode == GAIT

    # Held: must NOT re-fire
    for _ in range(10):
        out = map_joy(_axes(), _buttons(gait_mode=True), cfg, state, DT)
        assert out.mode_changed is False
        assert state.mode == GAIT

    # Release the gait button, then press posture: switch back
    map_joy(_axes(), _buttons(gait_mode=False), cfg, state, DT)
    out = map_joy(_axes(), _buttons(posture_mode=True), cfg, state, DT)
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


def test_mode_switch_preserves_height():
    # The whole point of height: it must survive a POSTURE → GAIT
    # switch so the robot walks at the lifted posture.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=0.03)
    out = map_joy(_axes(), _buttons(gait_mode=True), cfg, state, DT)
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


def test_start_at_nonzero_height_arms_smooth_revert():
    # First press while lifted arms a smooth revert (reverting flag
    # set, init suppressed). The height decays this tick — it does not
    # snap to zero — and continues to decay on subsequent ticks toward
    # default.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=0.03)
    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is False
    assert state.reverting is True
    # One tick of decay at tau=0.5s, dt=0.02s: exp(-0.04) ≈ 0.9608.
    expected = 0.03 * math.exp(-DT / cfg.posture_revert_tau)
    assert math.isclose(state.height_current, expected, rel_tol=1e-9)


def test_two_press_start_from_lifted_state():
    # Press 1 arms the smooth revert; after enough ticks for the decay
    # to settle below the 1e-4 tolerance, press 2 fires init_request.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=0.03)

    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is False
    assert state.reverting is True

    # Release the button so the next press would be a rising edge, and
    # let the revert run to completion. At tau=0.5s a 0.03 m offset
    # decays below 1e-4 in ~3 s; 250 ticks @ 20 ms is 5 s, plenty of
    # margin.
    for _ in range(250):
        map_joy(_axes(), _buttons(), cfg, state, DT)
    assert state.reverting is False
    assert state.height_current == 0.0

    # Second press: at zero height now, so init_request fires.
    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is True


def test_holding_start_at_nonzero_height_keeps_revert_armed():
    # Revert is armed by the rising edge — holding the Start button
    # across ticks doesn't re-arm or otherwise disturb the decay. The
    # height continues to decay smoothly while the button is held.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, height_current=0.03)
    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert state.reverting is True
    after_first = state.height_current
    # Hold the button for a few more ticks: decay continues normally,
    # init stays suppressed.
    for _ in range(3):
        out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is False
    assert state.reverting is True
    assert state.height_current < after_first


def test_short_joy_message_zero_pose_z():
    # Regression: the pose_z field exists on JoyOutput and is zero for
    # the empty-input case.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    out = map_joy((), (), cfg, state, DT)
    assert out.pose_z == 0.0
    assert state.height_current == 0.0


# ---- Select-button posture recording ----------------------------------------


def test_select_in_posture_records_current_joystick_pose():
    # Push left stick fully left → roll at -roll_max. Press Select,
    # then release the stick: the robot must hold the recorded tilt.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    map_joy(_axes(left_x=1.0), _buttons(record=True), cfg, state, DT)
    assert math.isclose(state.recorded_roll, -cfg.posture_roll_max)
    # Release stick AND release Select: output should still be at the
    # recorded tilt.
    out = map_joy(_axes(), _buttons(), cfg, state, DT)
    assert math.isclose(out.pose_roll, -cfg.posture_roll_max)


def test_recorded_pose_plus_stick_clamps_at_limit():
    # The user's example: tilt fully left, record, tilt fully left
    # again — the second push must have no further effect because the
    # baseline is already saturated.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    # First: record while pushing left.
    map_joy(_axes(left_x=1.0), _buttons(record=True), cfg, state, DT)
    # Release record button so a future press would re-record; keep
    # the stick pushed.
    out = map_joy(_axes(left_x=1.0), _buttons(), cfg, state, DT)
    assert math.isclose(out.pose_roll, -cfg.posture_roll_max)


def test_recorded_pose_plus_opposite_stick_unwinds():
    # Record at full left roll, then push right at full deflection:
    # the joystick fully cancels the baseline, output goes to zero.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    map_joy(_axes(left_x=1.0), _buttons(record=True), cfg, state, DT)
    # Release record, push opposite direction.
    out = map_joy(_axes(left_x=-1.0), _buttons(), cfg, state, DT)
    assert math.isclose(out.pose_roll, 0.0, abs_tol=1e-9)


def test_select_folds_height_into_recorded_z_and_zeros_height():
    # Lift halfway with D-pad, press Select: recorded_z absorbs the
    # height, height_current resets to zero, pose_z stays where it was.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    for _ in range(20):  # 20 * 0.02 * 0.05 = 0.020 m
        map_joy(_axes(dpad_y=1.0), _buttons(), cfg, state, DT)
    height_before = state.height_current
    assert height_before > 0.0
    out = map_joy(_axes(), _buttons(record=True), cfg, state, DT)
    assert math.isclose(state.recorded_z, height_before)
    assert state.height_current == 0.0
    assert math.isclose(out.pose_z, height_before)


def test_select_folds_yaw_current_into_recorded_yaw():
    # Hold L1 long enough that yaw_current saturates, press Select
    # while still holding L1: easing runs BEFORE the fold, so the
    # held-button case keeps yaw_current at the cap going INTO the
    # fold; the fold then absorbs the cap into recorded_yaw and zeros
    # yaw_current. Visible pose stays continuous because the output
    # reads recorded_yaw + yaw_current after the fold.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    for _ in range(400):
        map_joy(_axes(), _buttons(yaw_left=True), cfg, state, DT)
    yaw_before = state.yaw_current
    assert math.isclose(yaw_before, cfg.posture_yaw_max, rel_tol=1e-6)
    out = map_joy(_axes(), _buttons(record=True, yaw_left=True), cfg, state, DT)
    assert math.isclose(state.recorded_yaw, yaw_before, rel_tol=1e-6)
    assert state.yaw_current == 0.0
    assert math.isclose(out.pose_yaw, yaw_before, rel_tol=1e-6)
    # On the next tick the still-held L1 eases yaw_current back from 0
    # so the live state is alive again. The recorded baseline stops it
    # from accumulating past the cap.
    alpha = 1.0 - math.exp(-DT / cfg.posture_yaw_tau)
    out2 = map_joy(_axes(), _buttons(yaw_left=True), cfg, state, DT)
    assert math.isclose(state.yaw_current, alpha * cfg.posture_yaw_max, rel_tol=1e-6)
    assert math.isclose(out2.pose_yaw, cfg.posture_yaw_max, rel_tol=1e-6)


def test_select_in_gait_mode_is_noop():
    # Outside POSTURE mode the Select press must not capture anything;
    # the recorded state stays at default.
    cfg = _cfg()
    state = JoyState(mode=GAIT)
    out = map_joy(_axes(left_x=1.0), _buttons(record=True), cfg, state, DT)
    assert state.recorded_roll == 0.0
    assert state.recorded_x == 0.0
    assert state.recorded_y == 0.0
    assert state.recorded_pitch == 0.0
    assert state.recorded_yaw == 0.0
    assert out.pose_roll == 0.0


def test_recorded_pose_bleeds_through_to_gait_mode():
    # Record a non-zero posture in POSTURE, toggle to GAIT: the
    # recorded baseline still appears on every posture axis (like
    # height bleeds through today). Sticks now drive linear velocity,
    # not posture.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    # Push left + right sticks fully in their max-effect directions,
    # then record.
    map_joy(
        _axes(left_x=1.0, left_y=1.0, right_x=1.0, right_y=1.0),
        _buttons(record=True),
        cfg,
        state,
        DT,
    )
    # Release record, then switch to GAIT.
    map_joy(_axes(), _buttons(), cfg, state, DT)
    out = map_joy(_axes(right_x=1.0, right_y=1.0), _buttons(gait_mode=True), cfg, state, DT)
    assert state.mode == GAIT
    # Recorded posture bleeds through, sticks drive linear velocity.
    assert math.isclose(out.pose_x, cfg.posture_x_max)
    assert math.isclose(out.pose_y, cfg.posture_y_max)
    assert math.isclose(out.pose_roll, -cfg.posture_roll_max)
    assert math.isclose(out.pose_pitch, cfg.posture_pitch_max)
    assert math.isclose(out.linear_x, cfg.gait_linear_max)
    assert math.isclose(out.linear_y, cfg.gait_linear_max)


def test_start_with_recorded_pose_arms_revert_and_suppresses_init():
    # Record a non-zero posture, press Start: the reverting flag arms,
    # init_request is suppressed, and the persistent baseline starts
    # decaying (no instant snap).
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    # Get every axis non-zero in one shot.
    map_joy(_axes(left_x=1.0, left_y=1.0, right_x=1.0, right_y=1.0),
            _buttons(record=True), cfg, state, DT)
    # Release record, integrate some height, hold L1 for a few ticks.
    for _ in range(5):
        map_joy(_axes(dpad_y=1.0), _buttons(yaw_left=True), cfg, state, DT)
    pre_recorded_roll = state.recorded_roll
    pre_height = state.height_current
    assert pre_recorded_roll != 0.0
    assert pre_height > 0.0
    assert state.yaw_current > 0.0

    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is False
    assert state.reverting is True
    # One tick of decay leaves the baseline slightly reduced but
    # nowhere near zero.
    decay = math.exp(-DT / cfg.posture_revert_tau)
    assert math.isclose(state.recorded_roll, pre_recorded_roll * decay, rel_tol=1e-9)
    assert math.isclose(state.height_current, pre_height * decay, rel_tol=1e-9)


def test_revert_settles_to_zero_and_clears_flag():
    # Tick the revert long enough for every component to drop below
    # the 1e-4 tolerance: the flag clears and the persistent state
    # snaps to exactly zero so the next Start press fires init cleanly.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, recorded_roll=0.1, recorded_z=0.03)
    map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert state.reverting is True
    # 0.1 * exp(-N*0.04) < 1e-4 → N > ln(1e-3)/(-0.04) ≈ 173 ticks.
    # 250 ticks (5 s) is comfortably past that.
    for _ in range(250):
        map_joy(_axes(), _buttons(), cfg, state, DT)
    assert state.reverting is False
    assert state.recorded_roll == 0.0
    assert state.recorded_z == 0.0
    assert state.height_current == 0.0


def test_two_press_start_from_recorded_state():
    # Press 1 arms the revert; once the decay settles, press 2 at the
    # now-default state fires init_request.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    map_joy(_axes(left_x=1.0), _buttons(record=True), cfg, state, DT)
    map_joy(_axes(), _buttons(), cfg, state, DT)  # release record
    assert state.recorded_roll != 0.0

    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is False
    assert state.reverting is True

    # Release the button and let the revert run to completion.
    for _ in range(250):
        map_joy(_axes(), _buttons(), cfg, state, DT)
    assert state.reverting is False
    assert state.recorded_roll == 0.0

    out = map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert out.init_request is True


def test_select_during_revert_cancels_it():
    # Recording a fresh baseline mid-revert overrides the decay — the
    # user is explicitly setting a pose, which must not bleed away.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, recorded_roll=0.1)
    map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert state.reverting is True

    # A few decay ticks, then press Select with the stick at full
    # right (positive roll input).
    for _ in range(3):
        map_joy(_axes(), _buttons(), cfg, state, DT)
    map_joy(_axes(left_x=-1.0), _buttons(record=True), cfg, state, DT)
    assert state.reverting is False
    # The fold-in clamped to +roll_max (existing baseline + stick
    # contribution both push positive past the cap).
    assert math.isclose(state.recorded_roll, cfg.posture_roll_max)


def test_revert_runs_across_mode_switch():
    # A revert armed in POSTURE mode keeps decaying after a switch to
    # GAIT — the persistent baseline bleeds through into gait mode by
    # design, so the revert must keep running there too.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, recorded_roll=0.1)
    map_joy(_axes(), _buttons(init=True), cfg, state, DT)
    assert state.reverting is True
    # Switch to GAIT and tick the revert to completion there.
    map_joy(_axes(), _buttons(gait_mode=True), cfg, state, DT)
    assert state.mode == GAIT
    for _ in range(250):
        map_joy(_axes(), _buttons(), cfg, state, DT)
    assert state.reverting is False
    assert state.recorded_roll == 0.0


def test_select_rising_edge_only():
    # Holding Select across many ticks must NOT re-record on every
    # tick — the recording is a rising-edge action, identical to the
    # toggle/init buttons.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    # First tick at full left roll: records.
    map_joy(_axes(left_x=1.0), _buttons(record=True), cfg, state, DT)
    recorded_after_first = state.recorded_roll
    assert math.isclose(recorded_after_first, -cfg.posture_roll_max)
    # Hold Select for many more ticks with the stick at full left.
    # Without rising-edge gating, recorded_roll would saturate by
    # repeated folding — but the per-axis clamp at record time pins it
    # to -roll_max, so use a different signal: keep stick at NEUTRAL
    # while holding Select and check that recorded_roll does not drift
    # (it would drift toward 0 if folded again with stick=0).
    for _ in range(20):
        map_joy(_axes(), _buttons(record=True), cfg, state, DT)
    assert math.isclose(state.recorded_roll, recorded_after_first)


def test_recorded_pose_respects_per_axis_limit_at_record_time():
    # Set a tight roll limit, record with the baseline already at the
    # positive cap and the stick driving in the same direction: the
    # snapshot clamps at the cap rather than overflowing.
    cfg = _cfg(posture_roll_max=0.10)
    state = JoyState(mode=POSTURE, recorded_roll=0.10)
    # left_x = -1.0 → -lx * roll_max = +0.10, fold = clamp(0.10 + 0.10, ±0.10) = 0.10
    map_joy(_axes(left_x=-1.0), _buttons(record=True), cfg, state, DT)
    assert math.isclose(state.recorded_roll, 0.10)


# ---- D-pad X gait cycling ---------------------------------------------------


def test_dpad_right_rising_edge_advances_gait_index():
    # Cycle starts at "tripod" (index 2). D-right rising edge advances
    # the index wrap-around → "wave" (index 0).
    cfg = _cfg()
    state = JoyState(mode=POSTURE, current_gait_idx=2)
    out = map_joy(_axes(dpad_x=1.0), _buttons(), cfg, state, DT)
    assert out.gait_select == "wave"
    assert state.current_gait_idx == 0


def test_dpad_left_rising_edge_advances_backward():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, current_gait_idx=2)
    out = map_joy(_axes(dpad_x=-1.0), _buttons(), cfg, state, DT)
    assert out.gait_select == "ripple"
    assert state.current_gait_idx == 1


def test_dpad_x_hold_does_not_retrigger():
    # First tick at +1: fires. Subsequent ticks at +1 must NOT keep
    # cycling, matching the rising-edge contract of the other buttons.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, current_gait_idx=2)
    out = map_joy(_axes(dpad_x=1.0), _buttons(), cfg, state, DT)
    assert out.gait_select == "wave"
    for _ in range(20):
        out = map_joy(_axes(dpad_x=1.0), _buttons(), cfg, state, DT)
        assert out.gait_select is None
    # Release then press again → next slot.
    map_joy(_axes(dpad_x=0.0), _buttons(), cfg, state, DT)
    out = map_joy(_axes(dpad_x=1.0), _buttons(), cfg, state, DT)
    assert out.gait_select == "ripple"


def test_dpad_x_wraparound_full_cycle_returns_to_start():
    # Three D-right presses through wave→ripple→tripod gets the user
    # back to the starting selection.
    cfg = _cfg()
    state = JoyState(mode=POSTURE, current_gait_idx=2)
    names = []
    for _ in range(3):
        out = map_joy(_axes(dpad_x=1.0), _buttons(), cfg, state, DT)
        names.append(out.gait_select)
        # Release between presses to satisfy edge detection.
        map_joy(_axes(dpad_x=0.0), _buttons(), cfg, state, DT)
    assert names == ["wave", "ripple", "tripod"]
    assert state.current_gait_idx == 2


def test_dpad_x_works_in_gait_mode_too():
    # The pure mapping is mode-agnostic — the ROS layer is what filters
    # publishes on /gait/state == "stand". Both modes must report the
    # rising edge.
    cfg = _cfg()
    state = JoyState(mode=GAIT, current_gait_idx=0)
    out = map_joy(_axes(dpad_x=1.0), _buttons(), cfg, state, DT)
    assert out.gait_select == "ripple"


def test_dpad_x_no_select_when_axis_neutral():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, current_gait_idx=0)
    out = map_joy(_axes(), _buttons(), cfg, state, DT)
    assert out.gait_select is None
    assert state.current_gait_idx == 0


def test_dpad_x_sign_can_be_flipped_via_config():
    # joy_node's sign on D-pad X varies by driver. dpad_right_sign=-1
    # should reverse the cycle direction.
    cfg = _cfg(dpad_right_sign=-1.0)
    state = JoyState(mode=POSTURE, current_gait_idx=0)
    out = map_joy(_axes(dpad_x=1.0), _buttons(), cfg, state, DT)
    # With the flip, axis +1 should walk BACKWARD through the cycle
    # — from wave (0) to tripod (2).
    assert out.gait_select == "tripod"
    assert state.current_gait_idx == 2


def test_dpad_x_short_message_does_not_crash():
    cfg = _cfg()
    state = JoyState(mode=POSTURE, current_gait_idx=0)
    out = map_joy((), (), cfg, state, DT)
    assert out.gait_select is None
    assert state.current_gait_idx == 0


def test_dpad_x_empty_cycle_is_inert():
    # No registered gait list → mapper must not blow up; gait_select
    # stays None regardless of the axis.
    cfg = _cfg(gait_cycle=())
    state = JoyState(mode=POSTURE, current_gait_idx=0)
    out = map_joy(_axes(dpad_x=1.0), _buttons(), cfg, state, DT)
    assert out.gait_select is None


# ---- ANIMATION-mode D-pad animation selection -------------------------------


def test_animation_mode_dpad_up_selects_vertical_body_roll():
    cfg = _cfg()
    state = JoyState(mode=ANIMATION, animation_name="")
    out = map_joy(_axes(dpad_y=1.0), _buttons(), cfg, state, DT)
    assert out.animation_name == "vertical_body_roll"
    assert state.animation_name == "vertical_body_roll"


def test_animation_mode_dpad_down_selects_body_roll_3d():
    # D-down picks the body_roll_3d animation — vertical and horizontal
    # rolls combined with a quarter-cycle phase offset so the motion
    # traces a circle.
    cfg = _cfg()
    state = JoyState(mode=ANIMATION, animation_name="vertical_body_roll")
    out = map_joy(_axes(dpad_y=-1.0), _buttons(), cfg, state, DT)
    assert out.animation_name == "body_roll_3d"
    assert state.animation_name == "body_roll_3d"


def test_animation_mode_dpad_y_rising_edge_only():
    # Holding D-down must not re-publish on every tick.
    cfg = _cfg()
    state = JoyState(mode=ANIMATION, animation_name="vertical_body_roll")
    out = map_joy(_axes(dpad_y=-1.0), _buttons(), cfg, state, DT)
    assert out.animation_name == "body_roll_3d"
    for _ in range(20):
        out = map_joy(_axes(dpad_y=-1.0), _buttons(), cfg, state, DT)
        assert out.animation_name is None
    # Release then press again → re-fires only if the selection differs;
    # here the state is already body_roll_3d, so no republish.
    map_joy(_axes(dpad_y=0.0), _buttons(), cfg, state, DT)
    out = map_joy(_axes(dpad_y=-1.0), _buttons(), cfg, state, DT)
    assert out.animation_name is None
    # Toggle through D-up, release, then D-down again — the swap fires.
    map_joy(_axes(dpad_y=0.0), _buttons(), cfg, state, DT)
    map_joy(_axes(dpad_y=1.0), _buttons(), cfg, state, DT)
    map_joy(_axes(dpad_y=0.0), _buttons(), cfg, state, DT)
    out = map_joy(_axes(dpad_y=-1.0), _buttons(), cfg, state, DT)
    assert out.animation_name == "body_roll_3d"


def test_animation_mode_dpad_down_outside_animation_mode_is_inert():
    # D-down in POSTURE integrates height, never publishes animation
    # selection.
    cfg = _cfg()
    state = JoyState(mode=POSTURE)
    out = map_joy(_axes(dpad_y=-1.0), _buttons(), cfg, state, DT)
    assert out.animation_name is None
    assert state.animation_name == ""
