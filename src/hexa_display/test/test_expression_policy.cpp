// gtest port of the hexa_display expression_policy pytest suite. Exercises the
// pure decision policy directly — no rclcpp, no device.

#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "Expression.h"
#include "expression_policy.hpp"
#include "face_animation.hpp"

using namespace hexa::display;

namespace {

PolicyConfig makeConfig() {
    PolicyConfig config;
    config.expression_map = defaultExpressionMap();
    return config;
}

// Defaults mirror the pytest make_inputs(): standing, still, level, healthy.
PolicyInputs makeInputs() {
    PolicyInputs in;
    in.gait_state = "stand";
    return in;
}

const PolicyConfig CONFIG = makeConfig();

}  // namespace

TEST(ExpressionPolicy, GaitStateMapDefaults) {
    for (const auto& [state, expression] : defaultExpressionMap()) {
        auto in = makeInputs();
        in.gait_state = state;
        EXPECT_EQ(decide(in, CONFIG, kIdleTarget).expression, expression) << state;
    }
}

TEST(ExpressionPolicy, UnknownStateIsNeutralUnseenStateIsSleepy) {
    auto warp = makeInputs();
    warp.gait_state = "warp";
    EXPECT_EQ(decide(warp, CONFIG, kIdleTarget).expression, Expression::NEUTRAL);

    // The robot boots folded: until /gait/state is heard the eyes stay asleep.
    auto none = makeInputs();
    none.gait_state = std::nullopt;
    EXPECT_EQ(decide(none, CONFIG, kIdleTarget).expression, Expression::SLEEPY);
}

TEST(ExpressionPolicy, AnimationModeWinsOverGaitState) {
    auto in = makeInputs();
    in.gait_state = "gait";
    in.animation_mode = "body_roll_3d";
    EXPECT_EQ(decide(in, CONFIG, kIdleTarget).expression, Expression::WOOZY);
}

TEST(ExpressionPolicy, RunningGestureWearsItsExpression) {
    PolicyConfig config = CONFIG;
    config.gesture_expression_map["wave"] = Expression::LOVE;

    auto in = makeInputs();
    in.gait_state = "gesture";
    in.gesture = "wave";
    EXPECT_EQ(decide(in, config, kIdleTarget).expression, Expression::LOVE);

    in.animation_mode = "body_roll_3d";
    EXPECT_EQ(decide(in, config, kIdleTarget).expression, Expression::LOVE);

    in.gesture = "bow";  // no expression: the face carries on as before
    EXPECT_EQ(decide(in, config, kIdleTarget).expression, config.animation_expression);
}

TEST(ExpressionPolicy, BusyAndCriticalOutrankAGestureExpression) {
    PolicyConfig config = CONFIG;
    config.gesture_expression_map["wave"] = Expression::LOVE;
    auto in = makeInputs();
    in.gesture = "wave";
    in.battery_critical = true;
    EXPECT_EQ(decide(in, config, kIdleTarget).expression, Expression::DEAD);
    in.busy = true;
    EXPECT_EQ(decide(in, config, kIdleTarget).expression, Expression::SCANNING);
}

TEST(ExpressionPolicy, BatteryWarningOnlyWhenIdle) {
    auto idle = makeInputs();
    idle.gait_state = "stand";
    idle.battery_low = true;
    EXPECT_EQ(decide(idle, CONFIG, kIdleTarget).expression, Expression::SLEEPY);

    auto walking = makeInputs();
    walking.gait_state = "gait";
    walking.vx = 0.05;
    walking.battery_low = true;
    EXPECT_EQ(decide(walking, CONFIG, kIdleTarget).expression, Expression::NEUTRAL);

    auto animating = makeInputs();
    animating.battery_low = true;
    animating.animation_mode = "vertical_body_roll";
    EXPECT_EQ(decide(animating, CONFIG, kIdleTarget).expression, Expression::WOOZY);
}

TEST(ExpressionPolicy, BatteryCriticalOverridesEverythingAndCentersGaze) {
    auto in = makeInputs();
    in.gait_state = "gait";
    in.vx = 0.1;
    in.animation_mode = "body_roll_3d";
    in.battery_low = true;
    in.battery_critical = true;
    EXPECT_EQ(decide(in, CONFIG, kIdleTarget),
              (DisplayTarget{Expression::DEAD, GazeDirection::CENTER}));
}

TEST(ExpressionPolicy, GazeFollowsCmdVelHorizontally) {
    struct Case {
        double vy, wz;
        GazeDirection gaze;
    };
    const Case cases[] = {
        {0.1, 0.0, GazeDirection::LEFT},
        {-0.1, 0.0, GazeDirection::RIGHT},
        {0.0, 0.5, GazeDirection::LEFT},
        {0.0, -0.5, GazeDirection::RIGHT},
    };
    for (const auto& c : cases) {
        auto in = makeInputs();
        in.gait_state = "gait";
        in.vx = 0.05;
        in.vy = c.vy;
        in.wz = c.wz;
        EXPECT_EQ(decide(in, CONFIG, kIdleTarget).gaze, c.gaze)
            << "vy=" << c.vy << " wz=" << c.wz;
    }
}

TEST(ExpressionPolicy, ForwardBackwardMotionKeepsGazeLevel) {
    auto fwd = makeInputs();
    fwd.gait_state = "gait";
    fwd.vx = 0.1;
    EXPECT_EQ(decide(fwd, CONFIG, kIdleTarget).gaze, GazeDirection::CENTER);

    auto bwd = makeInputs();
    bwd.gait_state = "gait";
    bwd.vx = -0.1;
    EXPECT_EQ(decide(bwd, CONFIG, kIdleTarget).gaze, GazeDirection::CENTER);
}

TEST(ExpressionPolicy, PitchDrivesVerticalGazeWhileGaitActive) {
    auto up = makeInputs();
    up.gait_state = "gait";
    up.vx = 0.1;
    up.pitch = -0.2;
    EXPECT_EQ(decide(up, CONFIG, kIdleTarget).gaze, GazeDirection::UP);

    auto down_left = makeInputs();
    down_left.gait_state = "gait";
    down_left.vx = 0.1;
    down_left.vy = 0.1;
    down_left.pitch = 0.2;
    EXPECT_EQ(decide(down_left, CONFIG, kIdleTarget).gaze, GazeDirection::DOWN_LEFT);
}

TEST(ExpressionPolicy, GaitActiveGazeTravelIsProportionalToCmdVel) {
    auto in = makeInputs();
    in.gait_state = "gait";
    in.vy = CONFIG.gaze_vy_max * 0.3;
    const DisplayTarget small = decide(in, CONFIG, kIdleTarget);
    EXPECT_EQ(small.gaze, GazeDirection::LEFT);
    EXPECT_FLOAT_EQ(small.gaze_scale_x, 0.3f);

    in.vy = CONFIG.gaze_vy_max;
    in.wz = CONFIG.gaze_wz_max;
    EXPECT_FLOAT_EQ(decide(in, CONFIG, kIdleTarget).gaze_scale_x, 1.0f);
}

TEST(ExpressionPolicy, PoseModeGazeCountersTilt) {
    auto nose_up = makeInputs();
    nose_up.pitch = -0.2;
    EXPECT_EQ(decide(nose_up, CONFIG, kIdleTarget).gaze, GazeDirection::DOWN);

    auto nose_down = makeInputs();
    nose_down.pitch = 0.2;
    EXPECT_EQ(decide(nose_down, CONFIG, kIdleTarget).gaze, GazeDirection::UP);

    auto yaw_left = makeInputs();
    yaw_left.yaw = 0.2;
    EXPECT_EQ(decide(yaw_left, CONFIG, kIdleTarget).gaze, GazeDirection::RIGHT);

    auto lean_right = makeInputs();
    lean_right.roll = 0.2;
    EXPECT_EQ(decide(lean_right, CONFIG, kIdleTarget).gaze, GazeDirection::LEFT);

    auto shifted = makeInputs();
    shifted.x = 0.05;
    shifted.y = 0.05;
    EXPECT_EQ(decide(shifted, CONFIG, kIdleTarget).gaze, GazeDirection::CENTER);

    EXPECT_EQ(decide(makeInputs(), CONFIG, kIdleTarget).gaze, GazeDirection::CENTER);
}

TEST(ExpressionPolicy, PoseModeGazeTravelIsProportionalToTilt) {
    auto in = makeInputs();
    in.pitch = CONFIG.pose_pitch_full_rad * 0.5;
    in.roll = CONFIG.pose_tilt_full_rad * 0.25;
    const DisplayTarget half = decide(in, CONFIG, kIdleTarget);
    EXPECT_EQ(half.gaze, GazeDirection::UP_LEFT);
    EXPECT_FLOAT_EQ(half.gaze_scale_y, 0.5f);
    EXPECT_FLOAT_EQ(half.gaze_scale_x, 0.25f);

    in.pitch = CONFIG.pose_pitch_full_rad * 2.0;
    in.roll = 0.0;
    const DisplayTarget clamped = decide(in, CONFIG, kIdleTarget);
    EXPECT_EQ(clamped.gaze, GazeDirection::UP);
    EXPECT_FLOAT_EQ(clamped.gaze_scale_y, 1.0f);
}

// Pose mode is where the face answers the sticks: the gait state sits at
// "stand" the whole time the operator poses the body, so the map alone would
// leave the eyes NEUTRAL throughout.
TEST(ExpressionPolicy, PostureSticksDriveExpressionInPoseMode) {
    auto still = makeInputs();
    EXPECT_EQ(decide(still, CONFIG, kIdleTarget).expression, Expression::NEUTRAL);

    auto tilt = makeInputs();  // left stick: body tilt
    tilt.pitch = 0.05;
    EXPECT_EQ(decide(tilt, CONFIG, kIdleTarget).expression, Expression::HAPPY);

    auto roll = makeInputs();
    roll.roll = -0.05;
    EXPECT_EQ(decide(roll, CONFIG, kIdleTarget).expression, Expression::HAPPY);

    auto shift = makeInputs();  // right stick: body translation
    shift.y = 0.02;
    EXPECT_EQ(decide(shift, CONFIG, kIdleTarget).expression, Expression::LOVE);

    auto both = makeInputs();
    both.pitch = 0.05;
    both.x = -0.02;
    EXPECT_EQ(decide(both, CONFIG, kIdleTarget).expression, Expression::ANGRY);
}

// Yaw is L1/R1 and height is the D-pad — neither is a stick, so neither moves
// the expression off the gait state's.
TEST(ExpressionPolicy, YawAloneIsNotAPostureStick) {
    auto in = makeInputs();
    in.yaw = 0.2;
    EXPECT_EQ(decide(in, CONFIG, kIdleTarget).expression, Expression::NEUTRAL);
}

// Walking with a posed body (a recorded posture baseline bleeds through into
// gait mode) stays on the gait state's expression: this only applies in pose
// mode, where the sticks are what moved the body in the first place.
TEST(ExpressionPolicy, PostureExpressionOnlyInPoseMode) {
    auto walking = makeInputs();
    walking.gait_state = "gait";
    walking.vx = 0.05;
    walking.pitch = 0.05;
    walking.y = 0.02;
    EXPECT_EQ(decide(walking, CONFIG, kIdleTarget).expression, Expression::NEUTRAL);

    auto folded = makeInputs();
    folded.gait_state = "folded";
    folded.pitch = 0.05;
    folded.y = 0.02;
    EXPECT_EQ(decide(folded, CONFIG, kIdleTarget).expression, Expression::SLEEPY);
}

TEST(ExpressionPolicy, PostureStickHysteresisHoldsExpressionInExitBand) {
    auto held = makeInputs();
    held.pitch = 0.02;  // under the 0.03 threshold, above the 0.018 exit level
    const DisplayTarget prev{Expression::HAPPY, GazeDirection::CENTER};
    EXPECT_EQ(decide(held, CONFIG, prev).expression, Expression::HAPPY);
    EXPECT_EQ(decide(held, CONFIG, kIdleTarget).expression, Expression::NEUTRAL);

    auto released = makeInputs();
    released.pitch = 0.01;
    EXPECT_EQ(decide(released, CONFIG, prev).expression, Expression::NEUTRAL);

    // Each stick keeps its own band: the shift stick, still held, survives the
    // tilt stick's release.
    auto one_of_two = makeInputs();
    one_of_two.pitch = 0.01;
    one_of_two.x = 0.02;
    const DisplayTarget both{Expression::ANGRY, GazeDirection::CENTER};
    EXPECT_EQ(decide(one_of_two, CONFIG, both).expression, Expression::LOVE);
}

TEST(ExpressionPolicy, PostureStickThresholdOfZeroDisablesTheStick) {
    PolicyConfig config = CONFIG;
    config.posture_shift_threshold_m = 0.0;
    auto shift = makeInputs();
    shift.y = 0.035;  // full deflection
    EXPECT_EQ(decide(shift, config, kIdleTarget).expression, Expression::NEUTRAL);
}

TEST(ExpressionPolicy, BatteryAndBusyOutrankPostureSticks) {
    auto in = makeInputs();
    in.pitch = 0.05;
    in.x = -0.02;

    auto low = in;
    low.battery_low = true;
    EXPECT_EQ(decide(low, CONFIG, kIdleTarget).expression, Expression::SLEEPY);

    auto animating = in;
    animating.animation_mode = "body_roll_3d";
    EXPECT_EQ(decide(animating, CONFIG, kIdleTarget).expression, Expression::WOOZY);

    auto busy = in;
    busy.busy = true;
    EXPECT_EQ(decide(busy, CONFIG, kIdleTarget).expression, Expression::SCANNING);
}

TEST(ExpressionPolicy, PostureSticksSuppressIdling) {
    auto shift = makeInputs();
    shift.y = 0.02;  // level, so only the shift stick keeps idling away
    EXPECT_FALSE(selectFaceAnimation(shift, CONFIG).has_value());

    auto tilt = makeInputs();
    tilt.pitch = 0.05;  // under the gaze's 0.08 level threshold, still a stick
    EXPECT_FALSE(selectFaceAnimation(tilt, CONFIG).has_value());
}

TEST(ExpressionPolicy, ToScreenGazeFlipsHorizontalOnly) {
    EXPECT_EQ(toScreenGaze(GazeDirection::LEFT), GazeDirection::RIGHT);
    EXPECT_EQ(toScreenGaze(GazeDirection::RIGHT), GazeDirection::LEFT);
    EXPECT_EQ(toScreenGaze(GazeDirection::UP_LEFT), GazeDirection::UP_RIGHT);
    EXPECT_EQ(toScreenGaze(GazeDirection::DOWN_RIGHT), GazeDirection::DOWN_LEFT);
    EXPECT_EQ(toScreenGaze(GazeDirection::UP), GazeDirection::UP);
    EXPECT_EQ(toScreenGaze(GazeDirection::DOWN), GazeDirection::DOWN);
    EXPECT_EQ(toScreenGaze(GazeDirection::CENTER), GazeDirection::CENTER);

    for (int i = 0; i < static_cast<int>(GazeDirection::_COUNT); ++i) {
        const auto g = static_cast<GazeDirection>(i);
        EXPECT_EQ(toScreenGaze(toScreenGaze(g)), g) << "gaze " << gazeName(g);
    }
}

TEST(ExpressionPolicy, BreathingSelectedWhileSleeping) {
    // The sleeping face breathes: both before the stack is up and once folded.
    auto waiting = makeInputs();
    waiting.gait_state = std::nullopt;

    auto folded = makeInputs();
    folded.gait_state = "folded";

    for (const auto& in : {waiting, folded}) {
        auto sel = selectFaceAnimation(in, CONFIG);
        ASSERT_TRUE(sel.has_value());
        EXPECT_EQ(*sel, "breathing");
    }
}

TEST(ExpressionPolicy, IdlingSelectedWhenStandingIdleAndLevel) {
    auto sel = selectFaceAnimation(makeInputs(), CONFIG);
    ASSERT_TRUE(sel.has_value());
    EXPECT_EQ(*sel, "idling");
}

TEST(ExpressionPolicy, NoFaceAnimationWhenBusyOrWarning) {
    auto walking = makeInputs();
    walking.gait_state = "gait";
    walking.vx = 0.1;

    auto turning = makeInputs();
    turning.wz = 0.3;

    auto pose_tilt = makeInputs();
    pose_tilt.pitch = 0.2;

    auto pose_yaw = makeInputs();
    pose_yaw.yaw = 0.2;

    auto folding = makeInputs();
    folding.gait_state = "folding";

    auto settling = makeInputs();
    settling.gait_state = "settling";

    auto animating = makeInputs();
    animating.animation_mode = "body_roll_3d";

    auto low = makeInputs();
    low.battery_low = true;

    auto critical = makeInputs();
    critical.gait_state = std::nullopt;
    critical.battery_critical = true;

    for (const auto& in : {walking, turning, pose_tilt, pose_yaw, folding,
                           settling, animating, low, critical}) {
        EXPECT_FALSE(selectFaceAnimation(in, CONFIG).has_value());
    }
}

TEST(ExpressionPolicy, SelectedFaceAnimationsExistInRegistry) {
    auto waiting = makeInputs();
    waiting.gait_state = std::nullopt;
    auto a = selectFaceAnimation(waiting, CONFIG);
    ASSERT_TRUE(a.has_value());
    EXPECT_NE(findFaceAnimation(*a), nullptr);

    auto b = selectFaceAnimation(makeInputs(), CONFIG);
    ASSERT_TRUE(b.has_value());
    EXPECT_NE(findFaceAnimation(*b), nullptr);
}

TEST(BatteryMonitorTest, DisabledByZeroThresholds) {
    BatteryMonitor monitor(0.0, 0.0);
    EXPECT_EQ(monitor.update(0.1, 0.0), (BatteryMonitor::Flags{false, false}));
    EXPECT_EQ(monitor.update(0.1, 100.0), (BatteryMonitor::Flags{false, false}));
}

TEST(BatteryMonitorTest, HoldTime) {
    BatteryMonitor monitor(7.0, 6.4, 0.3, 3.0);
    EXPECT_EQ(monitor.update(6.8, 0.0), (BatteryMonitor::Flags{false, false}));
    EXPECT_EQ(monitor.update(6.8, 2.9), (BatteryMonitor::Flags{false, false}));
    EXPECT_EQ(monitor.update(6.8, 3.0), (BatteryMonitor::Flags{true, false}));

    // A dip that recovers before the hold expires never trips.
    BatteryMonitor monitor2(7.0, 6.4, 0.3, 3.0);
    EXPECT_EQ(monitor2.update(6.8, 0.0), (BatteryMonitor::Flags{false, false}));
    EXPECT_EQ(monitor2.update(7.5, 1.0), (BatteryMonitor::Flags{false, false}));
    EXPECT_EQ(monitor2.update(6.8, 2.0), (BatteryMonitor::Flags{false, false}));
    EXPECT_EQ(monitor2.update(6.8, 4.9), (BatteryMonitor::Flags{false, false}));
    EXPECT_EQ(monitor2.update(6.8, 5.0), (BatteryMonitor::Flags{true, false}));
}

TEST(BatteryMonitorTest, HysteresisOnRecovery) {
    BatteryMonitor monitor(7.0, 0.0, 0.3, 0.0);
    EXPECT_EQ(monitor.update(6.9, 0.0), (BatteryMonitor::Flags{true, false}));
    // Inside the hysteresis band the flag stays raised.
    EXPECT_EQ(monitor.update(7.2, 1.0), (BatteryMonitor::Flags{true, false}));
    // Above threshold + hysteresis it clears immediately.
    EXPECT_EQ(monitor.update(7.4, 2.0), (BatteryMonitor::Flags{false, false}));
}

TEST(BatteryMonitorTest, CriticalIndependentOfWarning) {
    BatteryMonitor monitor(7.0, 6.4, 0.3, 0.0);
    EXPECT_EQ(monitor.update(6.9, 0.0), (BatteryMonitor::Flags{true, false}));
    EXPECT_EQ(monitor.update(6.3, 1.0), (BatteryMonitor::Flags{true, true}));
}

// The busy flag is the top of the precedence ladder: it is the one state the
// operator asked for by hand (a front-panel hold — a pairing scan or a
// network-mode switch), so nothing may take the panel while they wait on the
// spinners. The node ORs /bluetooth/scanning and /display/busy into it, which
// is why the field is not named after either topic.
TEST(ExpressionPolicy, BusyWearsTheScanningExpressionGazeCentered) {
    auto in = makeInputs();
    in.busy = true;
    const DisplayTarget target = decide(in, CONFIG, kIdleTarget);
    EXPECT_EQ(target.expression, Expression::SCANNING);
    EXPECT_EQ(target.gaze, GazeDirection::CENTER);
}

TEST(ExpressionPolicy, BusyOutranksEvenACriticalPack) {
    auto in = makeInputs();
    in.busy = true;
    in.battery_critical = true;
    in.battery_low = true;
    EXPECT_EQ(decide(in, CONFIG, kIdleTarget).expression, Expression::SCANNING);
}

TEST(ExpressionPolicy, BusySuppressesFaceAnimations) {
    // A drift animation would fight the spinners for the gaze they own.
    auto in = makeInputs();
    in.busy = true;
    EXPECT_FALSE(selectFaceAnimation(in, CONFIG).has_value());
}
