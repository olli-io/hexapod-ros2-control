// The Pico face's config glue: face_policy.hpp's buildPolicyConfig() and
// undervoltFlags(). This seam went stale once before, the shared policy growing
// fields the builder kept leaving at their defaults. src/hexa_display/test/
// covers what the policy DOES with a config; this covers whether the firmware
// hands it the right one — the transfer, not the values, so a retune in
// display.yaml moves both sides and the test still holds.

#include <string>

#include <gtest/gtest.h>

#include "face_policy.hpp"

namespace {

namespace cfg = hexa::config;
using face::buildPolicyConfig;

// A name that does not parse would silently become NEUTRAL via exprOrNeutral,
// turning a typo into a wrong face instead of a failure. gen_config.py validates
// at bake time; this is the runtime half of the same guard.
void expectParses(std::string_view name) {
    EXPECT_TRUE(face::parseExpression(std::string(name)).has_value())
        << "unparseable expression name: " << name;
}

TEST(FaceConfig, EveryBakedExpressionNameParses) {
    for (const auto& e : cfg::kFaceExpressionMap) expectParses(e.expression);
    expectParses(cfg::kFaceAnimationExpression);
    expectParses(cfg::kFaceBatteryWarningExpression);
    expectParses(cfg::kFaceBatteryCriticalExpression);
    expectParses(cfg::kFaceScanningExpression);
    expectParses(cfg::kFacePostureTiltExpression);
    expectParses(cfg::kFacePostureShiftExpression);
    expectParses(cfg::kFacePostureBothExpression);
}

TEST(FaceConfig, GaitStateExpressionMapTransfers) {
    const auto c = buildPolicyConfig();
    ASSERT_EQ(c.expression_map.size(), cfg::kFaceExpressionMap.size());
    for (const auto& e : cfg::kFaceExpressionMap) {
        const auto it = c.expression_map.find(std::string(e.state));
        ASSERT_NE(it, c.expression_map.end()) << "missing state: " << e.state;
        EXPECT_EQ(it->second, face::exprOrNeutral(e.expression)) << e.state;
    }
}

TEST(FaceConfig, StandaloneExpressionsTransfer) {
    const auto c = buildPolicyConfig();
    EXPECT_EQ(c.animation_expression,
              face::exprOrNeutral(cfg::kFaceAnimationExpression));
    EXPECT_EQ(c.battery_warning_expression,
              face::exprOrNeutral(cfg::kFaceBatteryWarningExpression));
    EXPECT_EQ(c.battery_critical_expression,
              face::exprOrNeutral(cfg::kFaceBatteryCriticalExpression));
    EXPECT_EQ(c.scanning_expression,
              face::exprOrNeutral(cfg::kFaceScanningExpression));
}

// The three that the pre-deletion builder never set at all.
TEST(FaceConfig, PostureExpressionsAndThresholdsTransfer) {
    const auto c = buildPolicyConfig();
    EXPECT_EQ(c.posture_tilt_expression,
              face::exprOrNeutral(cfg::kFacePostureTiltExpression));
    EXPECT_EQ(c.posture_shift_expression,
              face::exprOrNeutral(cfg::kFacePostureShiftExpression));
    EXPECT_EQ(c.posture_both_expression,
              face::exprOrNeutral(cfg::kFacePostureBothExpression));

    EXPECT_DOUBLE_EQ(c.posture_tilt_threshold_rad,
                     static_cast<double>(cfg::kFacePosture.tilt_threshold_rad));
    EXPECT_DOUBLE_EQ(c.posture_shift_threshold_m,
                     static_cast<double>(cfg::kFacePosture.shift_threshold_m));
    EXPECT_DOUBLE_EQ(c.posture_exit_ratio,
                     static_cast<double>(cfg::kFacePosture.exit_ratio));

    // A zero threshold disables that stick entirely, so a dropped transfer would
    // read as "posture expressions off" rather than as a failure.
    EXPECT_GT(c.posture_tilt_threshold_rad, 0.0);
    EXPECT_GT(c.posture_shift_threshold_m, 0.0);
    EXPECT_GT(c.posture_exit_ratio, 0.0);
}

TEST(FaceConfig, EveryGazeFieldTransfers) {
    const auto c = buildPolicyConfig();
    const auto& g = cfg::kFaceGaze;
    EXPECT_DOUBLE_EQ(c.gaze_deadband, static_cast<double>(g.gaze_deadband));
    EXPECT_DOUBLE_EQ(c.gaze_exit_ratio, static_cast<double>(g.gaze_exit_ratio));
    EXPECT_DOUBLE_EQ(c.gaze_wz_weight, static_cast<double>(g.gaze_wz_weight));
    EXPECT_DOUBLE_EQ(c.gaze_vy_max, static_cast<double>(g.gaze_vy_max));
    EXPECT_DOUBLE_EQ(c.gaze_wz_max, static_cast<double>(g.gaze_wz_max));
    EXPECT_DOUBLE_EQ(c.pose_pitch_threshold_rad,
                     static_cast<double>(g.pose_pitch_threshold_rad));
    EXPECT_DOUBLE_EQ(c.pose_tilt_threshold_rad,
                     static_cast<double>(g.pose_tilt_threshold_rad));
    EXPECT_DOUBLE_EQ(c.idling_start_delay_s,
                     static_cast<double>(g.idling_start_delay_s));
}

// Replaces Decision::battery_low/critical, which the supervisor no longer has.
TEST(FaceConfig, UndervoltLadderMapsToTheTwoFaceFlags) {
    using S = hexa::supervisor::UndervoltStage;

    EXPECT_FALSE(face::undervoltFlags(S::kNone).low);
    EXPECT_FALSE(face::undervoltFlags(S::kNone).critical);

    EXPECT_TRUE(face::undervoltFlags(S::kWarn).low);
    EXPECT_FALSE(face::undervoltFlags(S::kWarn).critical);

    // kFold and kCutoff both mean the robot is going down — critical, and still
    // low, so a consumer checking either flag sees it.
    EXPECT_TRUE(face::undervoltFlags(S::kFold).low);
    EXPECT_TRUE(face::undervoltFlags(S::kFold).critical);
    EXPECT_TRUE(face::undervoltFlags(S::kCutoff).low);
    EXPECT_TRUE(face::undervoltFlags(S::kCutoff).critical);
}

}  // namespace
