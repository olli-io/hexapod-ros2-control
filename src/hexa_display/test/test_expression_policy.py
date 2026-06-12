import pytest

from hexa_display.expression_policy import (
    DEFAULT_EXPRESSION_MAP,
    IDLE_TARGET,
    BatteryMonitor,
    DisplayTarget,
    PolicyConfig,
    PolicyInputs,
    decide,
    quantize_axis,
    select_face_animation,
)
from hexa_display.face_animation import FACE_ANIMATIONS
from hexa_display.protocol import Expression, Gaze

CONFIG = PolicyConfig(expression_map=dict(DEFAULT_EXPRESSION_MAP))


def make_inputs(**kwargs) -> PolicyInputs:
    defaults = dict(
        gait_state="stand",
        vx=0.0,
        vy=0.0,
        wz=0.0,
        animation_mode="",
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        battery_low=False,
        battery_critical=False,
    )
    defaults.update(kwargs)
    return PolicyInputs(**defaults)


@pytest.mark.parametrize("state,expression", list(DEFAULT_EXPRESSION_MAP.items()))
def test_gait_state_map_defaults(state, expression):
    target = decide(make_inputs(gait_state=state), CONFIG, IDLE_TARGET)
    assert target.expression == expression


def test_unknown_and_unseen_state_are_neutral():
    assert (
        decide(make_inputs(gait_state="warp"), CONFIG, IDLE_TARGET).expression
        == Expression.NEUTRAL
    )
    assert (
        decide(make_inputs(gait_state=None), CONFIG, IDLE_TARGET).expression
        == Expression.NEUTRAL
    )


def test_animation_mode_wins_over_gait_state():
    target = decide(
        make_inputs(gait_state="gait", animation_mode="body_roll_3d"),
        CONFIG,
        IDLE_TARGET,
    )
    assert target.expression == Expression.WOOZY


def test_battery_warning_only_when_idle():
    idle = decide(
        make_inputs(gait_state="stand", battery_low=True), CONFIG, IDLE_TARGET
    )
    assert idle.expression == Expression.SLEEPY
    walking = decide(
        make_inputs(gait_state="gait", vx=0.05, battery_low=True),
        CONFIG,
        IDLE_TARGET,
    )
    assert walking.expression == Expression.HAPPY
    animating = decide(
        make_inputs(battery_low=True, animation_mode="vertical_body_roll"),
        CONFIG,
        IDLE_TARGET,
    )
    assert animating.expression == Expression.WOOZY


def test_battery_critical_overrides_everything_and_centers_gaze():
    target = decide(
        make_inputs(
            gait_state="gait",
            vx=0.1,
            animation_mode="body_roll_3d",
            battery_low=True,
            battery_critical=True,
        ),
        CONFIG,
        IDLE_TARGET,
    )
    assert target == DisplayTarget(expression=Expression.DEAD, gaze=Gaze.CENTER)


@pytest.mark.parametrize(
    "vy,wz,gaze",
    [
        (0.1, 0.0, Gaze.LEFT),
        (-0.1, 0.0, Gaze.RIGHT),
        (0.0, 0.5, Gaze.LEFT),
        (0.0, -0.5, Gaze.RIGHT),
    ],
)
def test_gaze_follows_cmd_vel_horizontally(vy, wz, gaze):
    target = decide(
        make_inputs(gait_state="gait", vx=0.05, vy=vy, wz=wz),
        CONFIG,
        IDLE_TARGET,
    )
    assert target.gaze == gaze


def test_forward_backward_motion_keeps_gaze_level():
    forward = decide(make_inputs(gait_state="gait", vx=0.1), CONFIG, IDLE_TARGET)
    assert forward.gaze == Gaze.CENTER
    backward = decide(
        make_inputs(gait_state="gait", vx=-0.1), CONFIG, IDLE_TARGET
    )
    assert backward.gaze == Gaze.CENTER


def test_pitch_drives_vertical_gaze_while_gait_active():
    up = decide(
        make_inputs(gait_state="gait", vx=0.1, pitch=-0.2), CONFIG, IDLE_TARGET
    )
    assert up.gaze == Gaze.UP
    down_left = decide(
        make_inputs(gait_state="gait", vx=0.1, vy=0.1, pitch=0.2),
        CONFIG,
        IDLE_TARGET,
    )
    assert down_left.gaze == Gaze.DOWN_LEFT


def test_gaze_below_deadband_is_center():
    # 0.001 / vy_max 0.1 = 0.01 normalized, far under the 0.15 deadband.
    target = decide(
        make_inputs(gait_state="gait", vy=0.001), CONFIG, IDLE_TARGET
    )
    assert target.gaze == Gaze.CENTER


def test_gaze_hysteresis_holds_direction_in_exit_band():
    prev = DisplayTarget(expression=Expression.HAPPY, gaze=Gaze.LEFT)
    # 0.012 / 0.1 = 0.12 normalized: under deadband 0.15 but above the
    # exit level 0.15 * 0.6 = 0.09, so LEFT is held.
    held = decide(make_inputs(gait_state="gait", vy=0.012), CONFIG, prev)
    assert held.gaze == Gaze.LEFT
    # Fresh entry from CENTER at the same value stays CENTER.
    fresh = decide(make_inputs(gait_state="gait", vy=0.012), CONFIG, IDLE_TARGET)
    assert fresh.gaze == Gaze.CENTER
    # Below the exit level the held direction releases.
    released = decide(make_inputs(gait_state="gait", vy=0.005), CONFIG, prev)
    assert released.gaze == Gaze.CENTER


def test_pose_mode_gaze_follows_tilt():
    # Negative pitch = nose up → gaze UP; positive yaw = left → LEFT.
    up = decide(make_inputs(pitch=-0.2), CONFIG, IDLE_TARGET)
    assert up.gaze == Gaze.UP
    down = decide(make_inputs(pitch=0.2), CONFIG, IDLE_TARGET)
    assert down.gaze == Gaze.DOWN
    left = decide(make_inputs(yaw=0.2), CONFIG, IDLE_TARGET)
    assert left.gaze == Gaze.LEFT
    right = decide(make_inputs(roll=0.2), CONFIG, IDLE_TARGET)
    assert right.gaze == Gaze.RIGHT
    level = decide(make_inputs(), CONFIG, IDLE_TARGET)
    assert level.gaze == Gaze.CENTER


def test_breathing_selected_while_waiting_for_the_stack():
    assert (
        select_face_animation(make_inputs(gait_state=None), CONFIG)
        == "breathing"
    )


def test_idling_selected_when_standing_idle_and_level():
    assert (
        select_face_animation(make_inputs(gait_state="stand"), CONFIG)
        == "idling"
    )


@pytest.mark.parametrize(
    "inputs",
    [
        make_inputs(gait_state="gait", vx=0.1),  # walking
        make_inputs(gait_state="stand", wz=0.3),  # turning in place
        make_inputs(gait_state="stand", pitch=0.2),  # pose mode tilt
        make_inputs(gait_state="stand", yaw=0.2),
        make_inputs(gait_state="folded"),  # sleepy states stay still
        make_inputs(gait_state="paused"),
        make_inputs(gait_state="stand", animation_mode="body_roll_3d"),
        make_inputs(gait_state="stand", battery_low=True),
        make_inputs(gait_state=None, battery_critical=True),
    ],
)
def test_no_face_animation_when_busy_or_warning(inputs):
    assert select_face_animation(inputs, CONFIG) is None


def test_selected_face_animations_exist_in_registry():
    assert select_face_animation(make_inputs(gait_state=None), CONFIG) in (
        FACE_ANIMATIONS
    )
    assert select_face_animation(make_inputs(gait_state="stand"), CONFIG) in (
        FACE_ANIMATIONS
    )


def test_quantize_axis_basic_and_hysteresis():
    assert quantize_axis(0.2, 0, 0.15, 0.6) == 1
    assert quantize_axis(-0.2, 0, 0.15, 0.6) == -1
    assert quantize_axis(0.1, 0, 0.15, 0.6) == 0
    # In the hold band only the matching previous sign is held.
    assert quantize_axis(0.1, 1, 0.15, 0.6) == 1
    assert quantize_axis(0.1, -1, 0.15, 0.6) == 0
    assert quantize_axis(0.05, 1, 0.15, 0.6) == 0
    # Hard sign flip switches without passing through 0.
    assert quantize_axis(-0.2, 1, 0.15, 0.6) == -1


def test_battery_monitor_disabled_by_zero_thresholds():
    monitor = BatteryMonitor(warning_v=0.0, critical_v=0.0)
    assert monitor.update(0.1, 0.0) == (False, False)
    assert monitor.update(0.1, 100.0) == (False, False)


def test_battery_monitor_hold_time():
    monitor = BatteryMonitor(warning_v=7.0, critical_v=6.4, hold_s=3.0)
    assert monitor.update(6.8, 0.0) == (False, False)  # below, hold running
    assert monitor.update(6.8, 2.9) == (False, False)
    assert monitor.update(6.8, 3.0) == (True, False)
    # A dip that recovers before the hold expires never trips.
    monitor2 = BatteryMonitor(warning_v=7.0, critical_v=6.4, hold_s=3.0)
    assert monitor2.update(6.8, 0.0) == (False, False)
    assert monitor2.update(7.5, 1.0) == (False, False)
    assert monitor2.update(6.8, 2.0) == (False, False)  # hold restarted
    assert monitor2.update(6.8, 4.9) == (False, False)
    assert monitor2.update(6.8, 5.0) == (True, False)


def test_battery_monitor_hysteresis_on_recovery():
    monitor = BatteryMonitor(
        warning_v=7.0, critical_v=0.0, hysteresis_v=0.3, hold_s=0.0
    )
    assert monitor.update(6.9, 0.0) == (True, False)
    # Inside the hysteresis band the flag stays raised.
    assert monitor.update(7.2, 1.0) == (True, False)
    # Above threshold + hysteresis it clears immediately.
    assert monitor.update(7.4, 2.0) == (False, False)


def test_battery_monitor_critical_independent_of_warning():
    monitor = BatteryMonitor(warning_v=7.0, critical_v=6.4, hold_s=0.0)
    assert monitor.update(6.9, 0.0) == (True, False)
    assert monitor.update(6.3, 1.0) == (True, True)
