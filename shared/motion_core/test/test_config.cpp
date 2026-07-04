// Host spot-checks for the generated config (plan part 04).
//
// Guards that tools/gen_config.py keeps emitting the values the ROS2 loaders
// (description_loader.cpp / limits.cpp / joint_calibration.cpp) produce — the
// symmetry expansion, deg->rad conventions, derived velocity caps, and servo
// calibration. Not exhaustive; it pins the load-bearing constants and the
// tricky derived ones so a broken generator fails loudly.

#include <cmath>
#include <string_view>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "leg_index.hpp"

namespace cfg = hexa::config;
using hexa::Leg;

namespace {
constexpr float kTol = 1e-5f;
float deg(float d) { return d * static_cast<float>(M_PI) / 180.0f; }

const cfg::LegSpec& spec(Leg leg) {
  return cfg::kLegSpecs[static_cast<std::size_t>(leg)];
}
}  // namespace

TEST(LegIndex, NameRoundTrip) {
  EXPECT_EQ(hexa::leg_name(Leg::L_MIDDLE), "l_middle");
  EXPECT_EQ(hexa::leg_name(Leg::R_REAR), "r_rear");
  bool found = false;
  EXPECT_EQ(hexa::leg_from_name("r_rear", found), Leg::R_REAR);
  EXPECT_TRUE(found);
  hexa::leg_from_name("nope", found);
  EXPECT_FALSE(found);
  // Enum order must match the libs' LEG_NAMES.
  EXPECT_EQ(hexa::leg_index(Leg::L_FRONT), 0);
  EXPECT_EQ(hexa::leg_index(Leg::R_REAR), 5);
}

TEST(LegSpecs, ReferenceMounts) {
  // l_front reference mount from geometry.yaml (0.083, 0.0575, yaw 30deg).
  EXPECT_NEAR(spec(Leg::L_FRONT).mount_xyz.x, 0.083f, kTol);
  EXPECT_NEAR(spec(Leg::L_FRONT).mount_xyz.y, 0.0575f, kTol);
  EXPECT_NEAR(spec(Leg::L_FRONT).mount_yaw, deg(30.0f), kTol);
  // l_middle reference mount (0, 0.082, yaw 90deg).
  EXPECT_NEAR(spec(Leg::L_MIDDLE).mount_xyz.x, 0.0f, kTol);
  EXPECT_NEAR(spec(Leg::L_MIDDLE).mount_xyz.y, 0.082f, kTol);
  EXPECT_NEAR(spec(Leg::L_MIDDLE).mount_yaw, deg(90.0f), kTol);
}

TEST(LegSpecs, SymmetryExpansion) {
  // rear mirrors front about y: x -> -x, yaw -> pi - yaw.
  EXPECT_NEAR(spec(Leg::L_REAR).mount_xyz.x, -0.083f, kTol);
  EXPECT_NEAR(spec(Leg::L_REAR).mount_xyz.y, 0.0575f, kTol);
  EXPECT_NEAR(spec(Leg::L_REAR).mount_yaw, deg(150.0f), kTol);
  // right mirrors left about x: y -> -y, yaw -> -yaw.
  // r_rear = (-0.083, -0.0575, -150deg) — the plan's headline check.
  EXPECT_NEAR(spec(Leg::R_REAR).mount_xyz.x, -0.083f, kTol);
  EXPECT_NEAR(spec(Leg::R_REAR).mount_xyz.y, -0.0575f, kTol);
  EXPECT_NEAR(spec(Leg::R_REAR).mount_yaw, deg(-150.0f), kTol);
  EXPECT_NEAR(spec(Leg::R_FRONT).mount_yaw, deg(-30.0f), kTol);
  EXPECT_NEAR(spec(Leg::R_MIDDLE).mount_yaw, deg(-90.0f), kTol);
}

TEST(LegSpecs, SegmentLengths) {
  for (const auto& s : cfg::kLegSpecs) {
    EXPECT_NEAR(s.coxa_len, 0.042f, kTol);
    EXPECT_NEAR(s.femur_len, 0.08f, kTol);
    EXPECT_NEAR(s.tibia_len, 0.134f, kTol);
  }
  EXPECT_NEAR(cfg::kCoxaToBottom, 0.03f, kTol);
}

TEST(JointLimits, DegToRadConventions) {
  // coxa: deg -> rad, center 0, window [-90, 90] deg.
  EXPECT_NEAR(cfg::kJointLimits[0].center, 0.0f, kTol);
  EXPECT_NEAR(cfg::kJointLimits[0].lower, deg(-90.0f), kTol);
  EXPECT_NEAR(cfg::kJointLimits[0].upper, deg(90.0f), kTol);
  // femur: -deg. center = -radians(35). lower/upper reconciled after negation.
  EXPECT_NEAR(cfg::kJointLimits[1].center, -deg(35.0f), kTol);
  EXPECT_NEAR(cfg::kJointLimits[1].lower, -deg(125.0f), kTol);
  EXPECT_NEAR(cfg::kJointLimits[1].upper, -deg(-55.0f), kTol);
  // tibia: pi - deg. center = pi - radians(68).
  EXPECT_NEAR(cfg::kJointLimits[2].center,
              static_cast<float>(M_PI) - deg(68.0f), kTol);
}

TEST(Pose, StandingUniform) {
  EXPECT_NEAR(cfg::kStandingPose[0], 0.0f, kTol);
  EXPECT_NEAR(cfg::kStandingPose[1], -deg(35.0f), kTol);
  EXPECT_NEAR(cfg::kStandingPose[2], static_cast<float>(M_PI) - deg(68.0f), kTol);
}

TEST(Pose, InitialPerLegSymmetry) {
  // femur/tibia uniform; coxa per leg from l_front_deg=60, l_middle_deg=0.
  EXPECT_NEAR(cfg::kInitialPose[hexa::leg_index(Leg::L_FRONT)][0], deg(60.0f), kTol);
  EXPECT_NEAR(cfg::kInitialPose[hexa::leg_index(Leg::L_MIDDLE)][0], 0.0f, kTol);
  EXPECT_NEAR(cfg::kInitialPose[hexa::leg_index(Leg::L_REAR)][0], deg(-60.0f), kTol);
  EXPECT_NEAR(cfg::kInitialPose[hexa::leg_index(Leg::R_FRONT)][0], deg(-60.0f), kTol);
  EXPECT_NEAR(cfg::kInitialPose[hexa::leg_index(Leg::R_REAR)][0], deg(60.0f), kTol);
  // femur above_horizontal_deg=100 -> -radians(100); tibia interior 40.
  EXPECT_NEAR(cfg::kInitialPose[0][1], -deg(100.0f), kTol);
  EXPECT_NEAR(cfg::kInitialPose[0][2], static_cast<float>(M_PI) - deg(40.0f), kTol);
}

TEST(Engine, GaitYamlKnobs) {
  EXPECT_NEAR(cfg::kEngine.stride_length, 0.1f, kTol);
  EXPECT_NEAR(cfg::kEngine.step_height, 0.08f, kTol);
  EXPECT_NEAR(cfg::kEngine.min_swing_time, 0.3f, kTol);
  EXPECT_NEAR(cfg::kEngine.controller_dt, 0.005f, kTol);
  EXPECT_NEAR(cfg::kEngine.init_pair_swing_time, 0.4f, kTol);
  EXPECT_NEAR(cfg::kEngine.reseat_swing_clearance, 0.025f, kTol);
}

TEST(VelocityCaps, TripodDerived) {
  const auto& tripod = cfg::kGaits[0];
  EXPECT_EQ(std::string_view(tripod.name.data()), "tripod");
  EXPECT_NEAR(tripod.duty_factor, 0.5f, kTol);
  EXPECT_FALSE(tripod.unstable);
  // linear_max = stride*(1-duty)/(min_swing*duty) = 0.1*0.5/(0.3*0.5) = 1/3.
  EXPECT_NEAR(tripod.linear_max, 1.0f / 3.0f, 1e-4f);
  // yaw_bias_eff = 0.5 + (0.6-0.5)*(1.5-0.5) = 0.6.
  EXPECT_NEAR(tripod.yaw_bias, 0.6f, kTol);
  EXPECT_NEAR(cfg::kAngularMax, 3.0f, kTol);
}

TEST(VelocityCaps, StabilityFlags) {
  // surf and crawl are the unstable ones (registry.cpp source of truth).
  auto by_name = [](std::string_view n) -> const cfg::GaitSpec& {
    for (const auto& g : cfg::kGaits)
      if (std::string_view(g.name.data()) == n) return g;
    ADD_FAILURE() << "gait not found: " << n;
    return cfg::kGaits[0];
  };
  EXPECT_TRUE(by_name("surf").unstable);
  EXPECT_TRUE(by_name("crawl").unstable);
  EXPECT_FALSE(by_name("tetrapod").unstable);
  EXPECT_FALSE(by_name("ripple").unstable);
  EXPECT_NEAR(by_name("ripple").duty_factor, 5.0f / 6.0f, kTol);
}

TEST(Teleop, HardwareIdentity) {
  EXPECT_EQ(cfg::kButtons.start, 7);
  EXPECT_EQ(cfg::kButtons.y, 3);
  EXPECT_EQ(cfg::kAxes.right_stick_y, 4);
  EXPECT_EQ(cfg::kAxes.l2, 2);
  EXPECT_NEAR(cfg::kAxisSigns.left_stick_x, -1.0f, kTol);
  EXPECT_NEAR(cfg::kAxisSigns.l2, 1.0f, kTol);  // no sign entry -> default +1
  EXPECT_NEAR(cfg::kDeadband, 0.1f, kTol);
  EXPECT_NEAR(cfg::kTriggerThreshold, 0.5f, kTol);
  EXPECT_EQ(cfg::kInitialMode, "gait");
  // kGaitCycle is the runtime rotation, already filtered by
  // allow_unstable_gaits. With unstable gaits enabled, surf + crawl stay
  // in the cycle, so the firmware cycler matches the ROS teleop node's
  // accepted set.
  EXPECT_TRUE(cfg::kAllowUnstableGaits);
  ASSERT_EQ(cfg::kGaitCycle.size(), 5u);
  EXPECT_EQ(cfg::kGaitCycle[0], "tripod");
  EXPECT_EQ(cfg::kGaitCycle[1], "surf");
  EXPECT_EQ(cfg::kGaitCycle[2], "tetrapod");
  EXPECT_EQ(cfg::kGaitCycle[3], "crawl");
  EXPECT_EQ(cfg::kGaitCycle[4], "ripple");
}

TEST(Teleop, PostureLimits) {
  EXPECT_NEAR(cfg::kPostureLimits.x_max, 0.035f, kTol);
  EXPECT_NEAR(cfg::kPostureLimits.roll_max_deg, 12.0f, kTol);
  EXPECT_NEAR(cfg::kPostureLimits.height_max_m, 0.04f, kTol);
  EXPECT_NEAR(cfg::kPostureLimits.height_min_m, -0.04f, kTol);
}

TEST(Posture, AnimationStack) {
  ASSERT_EQ(cfg::kEnabledAnimations.size(), 3u);
  EXPECT_EQ(cfg::kEnabledAnimations[0], "still");
  EXPECT_EQ(cfg::kAnimationModeAnimations[0], "vertical_body_roll");
  EXPECT_NEAR(cfg::kPosture.gait_sway_strength, 0.4f, kTol);
  EXPECT_NEAR(cfg::kPosture.gait_bounce_arc_height, 0.02f, kTol);
  EXPECT_NEAR(cfg::kPosture.body_roll_3d_horizontal_phase_offset, 0.25f, kTol);
  // Posture layering fix: crossfade slew rate + layered-clamp reserves.
  EXPECT_NEAR(cfg::kPosture.gait_activation_slew_rate, 4.0f, kTol);
  EXPECT_NEAR(cfg::kPosture.animation_reserve_x, 0.02f, kTol);
  EXPECT_NEAR(cfg::kPosture.animation_reserve_roll, 0.20f, kTol);
}

TEST(Control, RampAndSnap) {
  EXPECT_NEAR(cfg::kControl.vmax_ramp_time_linear, 0.8f, kTol);
  EXPECT_NEAR(cfg::kControl.vmax_ramp_time_angular, 1.0f, kTol);
  EXPECT_NEAR(cfg::kControl.snap_tol_linear, 1e-4f, 1e-6f);
}

TEST(Hardware, ServoCalibration) {
  ASSERT_EQ(cfg::kJointCals.size(), 18u);
  // Pins sorted 1..18; table order is the joint order.
  for (std::size_t i = 0; i < cfg::kJointCals.size(); ++i) {
    EXPECT_EQ(cfg::kJointCals[i].pin, static_cast<std::uint8_t>(i + 1));
    EXPECT_NEAR(cfg::kJointCals[i].us_at_plus_45, 2000.0f, kTol);
    EXPECT_NEAR(cfg::kJointCals[i].us_at_minus_45, 1000.0f, kTol);
    EXPECT_EQ(cfg::kJointCals[i].min_us, 500);
    EXPECT_EQ(cfg::kJointCals[i].max_us, 2500);
  }
  // urdf_rad_at_center from deg_at_center (coxa 0, femur 35, tibia 68).
  EXPECT_NEAR(cfg::kJointCals[0].urdf_rad_at_center, 0.0f, kTol);          // coxa
  EXPECT_NEAR(cfg::kJointCals[1].urdf_rad_at_center, -deg(35.0f), kTol);   // femur
  EXPECT_NEAR(cfg::kJointCals[2].urdf_rad_at_center,
              static_cast<float>(M_PI) - deg(68.0f), kTol);                // tibia
  EXPECT_EQ(cfg::kRelayPin, 24);
  EXPECT_EQ(cfg::kBatteryVoltagePin, 26);
  EXPECT_EQ(cfg::kBatteryCurrentPin, 27);
  EXPECT_NEAR(cfg::kBatteryVoltageScale, 0.00366f, 1e-7f);
  EXPECT_NEAR(cfg::kBatteryCurrentScale, 0.00098f, 1e-7f);
}
