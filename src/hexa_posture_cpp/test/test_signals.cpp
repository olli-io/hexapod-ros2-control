// Port of hexa_posture/test/test_posture_node.py — the module constants and pure
// helper functions, which now live in signals.hpp.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include "hexa_posture_cpp/signals.hpp"
#include "hexa_posture_cpp/types.hpp"

namespace {

using hexa_posture::is_posture_active;
using hexa_posture::LEG_COUNT;
using hexa_posture::lpf_step_scalar;
using hexa_posture::lpf_step_xy;
using hexa_posture::max_swing_lift_z;
using hexa_posture::POSTURE_ACTIVE_STATES;
using hexa_posture::stance_centroid_xy;
using hexa_posture::twist_is_zero;
using hexa_posture::Vec3;

struct LegSample {
  double x = 0.0;
  double y = 0.0;
  bool stance = true;
  double z = 0.0;
};

std::pair<std::array<Vec3, LEG_COUNT>, std::array<bool, LEG_COUNT>> targets(
    const std::array<LegSample, LEG_COUNT>& samples) {
  std::array<Vec3, LEG_COUNT> feet{};
  std::array<bool, LEG_COUNT> stance{};
  for (std::size_t i = 0; i < LEG_COUNT; ++i) {
    feet[i] = Vec3(samples[i].x, samples[i].y, samples[i].z);
    stance[i] = samples[i].stance;
  }
  return {feet, stance};
}

// --- POSTURE_ACTIVE_STATES gating ---

TEST(Signals, PauseTrioPreservesBodyPose) {
  for (const std::string state : {"pausing", "paused", "resuming"}) {
    EXPECT_TRUE(is_posture_active(state)) << state;
    EXPECT_EQ(POSTURE_ACTIVE_STATES.count(state), 1u) << state;
  }
}

TEST(Signals, ReseatingPreservesBodyPose) {
  EXPECT_TRUE(is_posture_active("reseating"));
}

TEST(Signals, WalkingStatesAreActive) {
  for (const std::string state : {"stand", "engaging", "gait"}) {
    EXPECT_TRUE(is_posture_active(state)) << state;
  }
}

TEST(Signals, PreStandStatesEmitIdentity) {
  for (const std::string state : {"folded", "initialize", "folding"}) {
    EXPECT_FALSE(is_posture_active(state)) << state;
  }
  EXPECT_FALSE(is_posture_active(""));  // cold start, no state seen
}

// --- twist_is_zero ---

TEST(Signals, TwistIsZeroDetectsRest) {
  EXPECT_TRUE(twist_is_zero(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(twist_is_zero(1e-5, -1e-5, 0.0, 0.0, 0.0, 0.0));
}

TEST(Signals, TwistIsZeroDetectsMotion) {
  EXPECT_FALSE(twist_is_zero(0.1, 0.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(twist_is_zero(0.0, 0.0, 0.0, 0.0, 0.0, 0.05));
}

// --- stance_centroid_xy ---

TEST(Signals, StanceCentroidIsMeanOfStanceLegs) {
  auto [feet, stance] = targets({{
      {1.0, 0.0, true},
      {0.0, 1.0, true},
      {-1.0, -1.0, true},
      {99.0, 99.0, false},
      {99.0, 99.0, false},
      {99.0, 99.0, false},
  }});
  const auto c = stance_centroid_xy(feet, stance);
  ASSERT_TRUE(c.has_value());
  EXPECT_NEAR(c->first, 0.0, 1e-12);
  EXPECT_NEAR(c->second, 0.0, 1e-12);
}

TEST(Signals, StanceCentroidReturnsNoneWhenPolygonDegenerate) {
  auto [feet, stance] = targets({{
      {0.1, 0.1, true},
      {-0.1, 0.0, true},
      {0.0, 0.0, false},
      {0.0, 0.0, false},
      {0.0, 0.0, false},
      {0.0, 0.0, false},
  }});
  EXPECT_FALSE(stance_centroid_xy(feet, stance).has_value());
}

// --- max_swing_lift_z ---

TEST(Signals, MaxSwingLiftPicksHighestSwingFootAboveStanceMean) {
  auto [feet, stance] = targets({{
      {0.0, 0.0, false, 0.04},
      {0.0, 0.0, true, 0.0},
      {0.0, 0.0, true, 0.0},
      {0.0, 0.0, false, 0.06},
      {0.0, 0.0, true, 0.0},
      {0.0, 0.0, true, 0.0},
  }});
  const auto lift = max_swing_lift_z(feet, stance);
  ASSERT_TRUE(lift.has_value());
  EXPECT_NEAR(*lift, 0.06, 1e-12);
}

TEST(Signals, MaxSwingLiftOffsetsAgainstStanceMean) {
  auto [feet, stance] = targets({{
      {0.0, 0.0, false, -0.04},
      {0.0, 0.0, true, -0.10},
      {0.0, 0.0, true, -0.10},
      {0.0, 0.0, true, -0.10},
      {0.0, 0.0, true, -0.10},
      {0.0, 0.0, true, -0.10},
  }});
  const auto lift = max_swing_lift_z(feet, stance);
  ASSERT_TRUE(lift.has_value());
  EXPECT_NEAR(*lift, 0.06, 1e-12);
}

TEST(Signals, MaxSwingLiftZeroWhenNoLegsSwinging) {
  auto [feet, stance] = targets({{
      {0.0, 0.0, true},
      {0.0, 0.0, true},
      {0.0, 0.0, true},
      {0.0, 0.0, true},
      {0.0, 0.0, true},
      {0.0, 0.0, true},
  }});
  const auto lift = max_swing_lift_z(feet, stance);
  ASSERT_TRUE(lift.has_value());
  EXPECT_EQ(*lift, 0.0);
}

TEST(Signals, MaxSwingLiftReturnsNoneWhenPolygonDegenerate) {
  auto [feet, stance] = targets({{
      {0.0, 0.0, true, 0.0},
      {0.0, 0.0, true, 0.0},
      {0.0, 0.0, false, 0.05},
      {0.0, 0.0, false, 0.05},
      {0.0, 0.0, false, 0.05},
      {0.0, 0.0, false, 0.05},
  }});
  EXPECT_FALSE(max_swing_lift_z(feet, stance).has_value());
}

TEST(Signals, MaxSwingLiftClampsNegativeToZero) {
  auto [feet, stance] = targets({{
      {0.0, 0.0, false, -0.01},
      {0.0, 0.0, true, 0.0},
      {0.0, 0.0, true, 0.0},
      {0.0, 0.0, true, 0.0},
      {0.0, 0.0, true, 0.0},
      {0.0, 0.0, true, 0.0},
  }});
  const auto lift = max_swing_lift_z(feet, stance);
  ASSERT_TRUE(lift.has_value());
  EXPECT_EQ(*lift, 0.0);
}

// --- Low-pass filters ---

TEST(Signals, LpfSeedsOnFirstSample) {
  const auto out = lpf_step_xy(std::nullopt, std::make_pair(0.02, -0.01), 0.1,
                               0.02);
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->first, 0.02);
  EXPECT_EQ(out->second, -0.01);
}

TEST(Signals, LpfHoldsPreviousWhenRawMissing) {
  const auto prev = std::make_pair(0.03, 0.01);
  const auto out = lpf_step_xy(prev, std::nullopt, 0.1, 0.02);
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(*out, prev);
}

TEST(Signals, LpfScalarSeedsOnFirstSample) {
  const auto out = lpf_step_scalar(std::nullopt, 0.05, 0.05, 0.02);
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(*out, 0.05);
}

TEST(Signals, LpfScalarHoldsPreviousWhenRawMissing) {
  const auto out = lpf_step_scalar(0.03, std::nullopt, 0.05, 0.02);
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(*out, 0.03);
}

TEST(Signals, LpfScalarSettlesAfterAFewTau) {
  const double tau = 0.05;
  const double dt = 0.005;
  const double target = 0.06;
  std::optional<double> state = 0.0;
  const int n_steps = static_cast<int>(std::lround((4.0 * tau) / dt));
  for (int i = 0; i < n_steps; ++i) {
    state = lpf_step_scalar(state, target, tau, dt);
  }
  ASSERT_TRUE(state.has_value());
  EXPECT_LE(std::abs(*state - target), 0.05 * target);
}

TEST(Signals, LpfSettlesAfterAFewTau) {
  const double tau = 0.1;
  const double dt = 0.005;
  const double target = 0.04;
  std::optional<std::pair<double, double>> state = std::make_pair(0.0, 0.0);
  const int n_steps = static_cast<int>(std::lround((4.0 * tau) / dt));
  for (int i = 0; i < n_steps; ++i) {
    state = lpf_step_xy(state, std::make_pair(target, 0.0), tau, dt);
  }
  ASSERT_TRUE(state.has_value());
  EXPECT_LE(std::abs(state->first - target), 0.05 * target);
  EXPECT_NEAR(state->second, 0.0, 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
