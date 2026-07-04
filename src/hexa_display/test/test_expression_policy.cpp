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

TEST(ExpressionPolicy, UnknownAndUnseenStateAreNeutral) {
    auto warp = makeInputs();
    warp.gait_state = "warp";
    EXPECT_EQ(decide(warp, CONFIG, kIdleTarget).expression, Expression::NEUTRAL);

    auto none = makeInputs();
    none.gait_state = std::nullopt;
    EXPECT_EQ(decide(none, CONFIG, kIdleTarget).expression, Expression::NEUTRAL);
}

TEST(ExpressionPolicy, AnimationModeWinsOverGaitState) {
    auto in = makeInputs();
    in.gait_state = "gait";
    in.animation_mode = "body_roll_3d";
    EXPECT_EQ(decide(in, CONFIG, kIdleTarget).expression, Expression::WOOZY);
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
    EXPECT_EQ(decide(walking, CONFIG, kIdleTarget).expression, Expression::HAPPY);

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

TEST(ExpressionPolicy, GazeBelowDeadbandIsCenter) {
    auto in = makeInputs();
    in.gait_state = "gait";
    in.vy = 0.001;
    EXPECT_EQ(decide(in, CONFIG, kIdleTarget).gaze, GazeDirection::CENTER);
}

TEST(ExpressionPolicy, GazeHysteresisHoldsDirectionInExitBand) {
    const DisplayTarget prev{Expression::HAPPY, GazeDirection::LEFT};

    auto held = makeInputs();
    held.gait_state = "gait";
    held.vy = 0.012;  // under deadband 0.15 but above exit level 0.09 -> held
    EXPECT_EQ(decide(held, CONFIG, prev).gaze, GazeDirection::LEFT);

    auto fresh = makeInputs();
    fresh.gait_state = "gait";
    fresh.vy = 0.012;
    EXPECT_EQ(decide(fresh, CONFIG, kIdleTarget).gaze, GazeDirection::CENTER);

    auto released = makeInputs();
    released.gait_state = "gait";
    released.vy = 0.005;  // below exit level -> released
    EXPECT_EQ(decide(released, CONFIG, prev).gaze, GazeDirection::CENTER);
}

TEST(ExpressionPolicy, PoseModeGazeFollowsTilt) {
    auto up = makeInputs();
    up.pitch = -0.2;
    EXPECT_EQ(decide(up, CONFIG, kIdleTarget).gaze, GazeDirection::UP);

    auto down = makeInputs();
    down.pitch = 0.2;
    EXPECT_EQ(decide(down, CONFIG, kIdleTarget).gaze, GazeDirection::DOWN);

    auto left = makeInputs();
    left.yaw = 0.2;
    EXPECT_EQ(decide(left, CONFIG, kIdleTarget).gaze, GazeDirection::LEFT);

    auto right = makeInputs();
    right.roll = 0.2;
    EXPECT_EQ(decide(right, CONFIG, kIdleTarget).gaze, GazeDirection::RIGHT);

    EXPECT_EQ(decide(makeInputs(), CONFIG, kIdleTarget).gaze, GazeDirection::CENTER);
}

TEST(ExpressionPolicy, BreathingSelectedWhileWaitingForStack) {
    auto in = makeInputs();
    in.gait_state = std::nullopt;
    auto sel = selectFaceAnimation(in, CONFIG);
    ASSERT_TRUE(sel.has_value());
    EXPECT_EQ(*sel, "breathing");
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

    auto folded = makeInputs();
    folded.gait_state = "folded";

    auto paused = makeInputs();
    paused.gait_state = "paused";

    auto animating = makeInputs();
    animating.animation_mode = "body_roll_3d";

    auto low = makeInputs();
    low.battery_low = true;

    auto critical = makeInputs();
    critical.gait_state = std::nullopt;
    critical.battery_critical = true;

    for (const auto& in : {walking, turning, pose_tilt, pose_yaw, folded, paused,
                           animating, low, critical}) {
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

TEST(ExpressionPolicy, QuantizeAxisBasicAndHysteresis) {
    EXPECT_EQ(quantizeAxis(0.2, 0, 0.15, 0.6), 1);
    EXPECT_EQ(quantizeAxis(-0.2, 0, 0.15, 0.6), -1);
    EXPECT_EQ(quantizeAxis(0.1, 0, 0.15, 0.6), 0);
    // In the hold band only the matching previous sign is held.
    EXPECT_EQ(quantizeAxis(0.1, 1, 0.15, 0.6), 1);
    EXPECT_EQ(quantizeAxis(0.1, -1, 0.15, 0.6), 0);
    EXPECT_EQ(quantizeAxis(0.05, 1, 0.15, 0.6), 0);
    // Hard sign flip switches without passing through 0.
    EXPECT_EQ(quantizeAxis(-0.2, 1, 0.15, 0.6), -1);
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
