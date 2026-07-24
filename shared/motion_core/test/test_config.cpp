// Host spot-checks for the generated config (plan part 04).
//
// Guards that tools/gen_config.py keeps emitting the values the ROS2 loaders
// (description_loader.cpp / limits.cpp / joint_calibration.cpp) produce — the
// symmetry expansion, deg->rad conventions, derived velocity caps, and servo
// calibration. Not exhaustive; it pins the load-bearing constants and the
// tricky derived ones so a broken generator fails loudly.

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>
#include <vector>

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
  // coxa: deg -> rad, window [-90, 90] deg.
  EXPECT_NEAR(cfg::kJointLimits[0].lower, deg(-90.0f), kTol);
  EXPECT_NEAR(cfg::kJointLimits[0].upper, deg(90.0f), kTol);
  // femur: -deg, so lower/upper swap after negation ([-55, 125] deg).
  EXPECT_NEAR(cfg::kJointLimits[1].lower, -deg(125.0f), kTol);
  EXPECT_NEAR(cfg::kJointLimits[1].upper, -deg(-55.0f), kTol);
  // tibia: pi - deg, likewise swapped ([-22, 158] deg).
  EXPECT_NEAR(cfg::kJointLimits[2].lower,
              static_cast<float>(M_PI) - deg(158.0f), kTol);
  EXPECT_NEAR(cfg::kJointLimits[2].upper,
              static_cast<float>(M_PI) - deg(-22.0f), kTol);
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
  // femur above_horizontal_deg=120 -> -radians(120); tibia interior 35.
  EXPECT_NEAR(cfg::kInitialPose[0][1], -deg(120.0f), kTol);
  EXPECT_NEAR(cfg::kInitialPose[0][2], static_cast<float>(M_PI) - deg(35.0f), kTol);
}

TEST(Engine, GaitYamlKnobs) {
  EXPECT_NEAR(cfg::kEngine.stride_length, 0.1f, kTol);
  EXPECT_NEAR(cfg::kEngine.step_height, 0.08f, kTol);
  EXPECT_NEAR(cfg::kEngine.swing_apex_fraction, 0.45f, kTol);
  EXPECT_NEAR(cfg::kEngine.touchdown_velocity, 0.02f, kTol);
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

  // Row order is the pipeline's theta[] order (LEG_NAMES x {coxa, femur, tibia}),
  // NOT pin order — kJointCals[i] must calibrate theta[i]. The wiring is carried
  // per row in `pin`. This is the hand-authored half of hardware.yaml (`servos`:
  // pin + `reversed`), so it is worth pinning exactly; the endpoint pulse widths
  // are calibration output and are only range-checked below.
  struct Wiring {
    std::uint8_t pin;
    int direction;  // hardware.yaml `reversed: true` -> -1
  };
  constexpr Wiring kWiring[18] = {
      {13, -1}, {14, +1}, {15, -1},  // l_front  {coxa, femur, tibia}
      {7, -1},  {8, +1},  {9, -1},   // l_middle
      {1, -1},  {2, +1},  {3, -1},   // l_rear
      {16, -1}, {17, -1}, {18, +1},  // r_front
      {10, -1}, {11, -1}, {12, +1},  // r_middle
      {4, -1},  {5, -1},  {6, +1},   // r_rear
  };
  for (std::size_t i = 0; i < cfg::kJointCals.size(); ++i) {
    const auto& j = cfg::kJointCals[i];
    EXPECT_EQ(j.pin, kWiring[i].pin) << "row " << i;
    EXPECT_EQ(j.direction, kWiring[i].direction) << "row " << i;
    EXPECT_EQ(j.min_us, 500) << "row " << i;
    EXPECT_EQ(j.max_us, 2500) << "row " << i;
    // Endpoints are measured magnitudes — they move every calibration run, so
    // only the invariants are pinned: both inside the electrical clamp, and
    // distinct (equal endpoints would make the slope zero).
    EXPECT_GT(j.us_at_plus_45, static_cast<float>(j.min_us)) << "row " << i;
    EXPECT_LT(j.us_at_plus_45, static_cast<float>(j.max_us)) << "row " << i;
    EXPECT_GT(j.us_at_minus_45, static_cast<float>(j.min_us)) << "row " << i;
    EXPECT_LT(j.us_at_minus_45, static_cast<float>(j.max_us)) << "row " << i;
    EXPECT_NE(j.us_at_plus_45, j.us_at_minus_45) << "row " << i;
  }
  // Every servo pin is wired exactly once.
  bool used[19] = {};
  for (const auto& j : cfg::kJointCals) {
    ASSERT_GE(j.pin, 1);
    ASSERT_LE(j.pin, 18);
    EXPECT_FALSE(used[j.pin]) << "pin " << static_cast<int>(j.pin) << " wired twice";
    used[j.pin] = true;
  }
  // urdf_rad_at_center from hardware.yaml deg_at_center (coxa 0, femur 40,
  // tibia 68) — the sole source for the servo-center angle. Uniform per segment,
  // so check one leg and trust the loop above for the rest.
  EXPECT_NEAR(cfg::kJointCals[0].urdf_rad_at_center, 0.0f, kTol);          // coxa
  EXPECT_NEAR(cfg::kJointCals[1].urdf_rad_at_center, -deg(40.0f), kTol);   // femur
  EXPECT_NEAR(cfg::kJointCals[2].urdf_rad_at_center,
              static_cast<float>(M_PI) - deg(68.0f), kTol);                // tibia
  for (std::size_t i = 0; i < cfg::kJointCals.size(); ++i) {
    EXPECT_NEAR(cfg::kJointCals[i].urdf_rad_at_center,
                cfg::kJointCals[i % 3].urdf_rad_at_center, kTol)
        << "row " << i << " segment centre differs from leg 0";
  }
  // Chica command-index map + scales per protocol.md (hexapod-servo2040-driver):
  // CURR=24, VOLT=25 (consecutive), RELAY=26, STATUS=27; telemetry in 0.01
  // centi-units; STATUS trip current at 0.1 A/count.
  EXPECT_EQ(cfg::kRelayPin, 26);
  EXPECT_EQ(cfg::kBatteryCurrentPin, 24);
  EXPECT_EQ(cfg::kBatteryVoltagePin, 25);
  EXPECT_EQ(cfg::kStatusPin, 27);
  EXPECT_NEAR(cfg::kBatteryVoltageScale, 0.01f, 1e-7f);
  EXPECT_NEAR(cfg::kBatteryCurrentScale, 0.01f, 1e-7f);
  EXPECT_NEAR(cfg::kTripAmpsPerCount, 0.1f, 1e-7f);
}

TEST(Hardware, LegsAreWiredRearToFront) {
  // Sorting the rows by pin must yield the harness order the energize sweep
  // brings legs up in. Both consumers derive this the same way — the firmware's
  // servo_out leg_pin_order() off kJointCals, hexa_hardware's build_leg_order()
  // off hardware.yaml — so pinning it here pins both.
  std::array<std::size_t, hexa::kNumLegs> order{};
  for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
  auto lowest_pin = [](std::size_t leg) {
    std::uint8_t p = cfg::kJointCals[leg * 3].pin;
    for (std::size_t k = 1; k < 3; ++k) {
      p = std::min(p, cfg::kJointCals[leg * 3 + k].pin);
    }
    return p;
  };
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return lowest_pin(a) < lowest_pin(b);
  });

  std::vector<std::string_view> names;
  for (const std::size_t leg : order) {
    names.push_back(hexa::LEG_NAMES[leg]);
  }
  EXPECT_EQ(names, (std::vector<std::string_view>{"l_rear", "r_rear", "l_middle",
                                                  "r_middle", "l_front",
                                                  "r_front"}));
  // Each leg's three joints sit on consecutive pins, so a leg is one SET frame.
  for (std::size_t leg = 0; leg < hexa::kNumLegs; ++leg) {
    EXPECT_EQ(cfg::kJointCals[leg * 3 + 1].pin, cfg::kJointCals[leg * 3].pin + 1)
        << hexa::LEG_NAMES[leg];
    EXPECT_EQ(cfg::kJointCals[leg * 3 + 2].pin, cfg::kJointCals[leg * 3].pin + 2)
        << hexa::LEG_NAMES[leg];
  }
}

TEST(Hardware, StartupKnobs) {
  // hardware.yaml parser.get_period_ticks / init.sweep_leg_interval_ms — the two
  // host-side pacing knobs the firmware bakes rather than loads.
  EXPECT_EQ(cfg::kGetPeriodTicks, 10);
  // Non-negative by construction (gen_config rejects a negative); 0 opts out of
  // the per-leg energize stagger.
  EXPECT_GE(cfg::kSweepLegIntervalMs, 0);
  EXPECT_EQ(cfg::kSweepLegIntervalMs, 150);
}
