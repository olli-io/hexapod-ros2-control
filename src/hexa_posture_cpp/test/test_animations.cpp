// Port of hexa_posture/test/test_animations.py, extended with direct coverage
// for the three body-roll animations (which the Python suite lacks) and for
// build_animation_stack.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hexa_posture_cpp/animations.hpp"
#include "hexa_posture_cpp/pose.hpp"

namespace {

using hexa_posture::AnimationContext;
using hexa_posture::AnimationPtr;
using hexa_posture::BodyPose;
using hexa_posture::BodyRoll3D;
using hexa_posture::Breathing;
using hexa_posture::build_animation_stack;
using hexa_posture::GaitBounce;
using hexa_posture::GaitSway;
using hexa_posture::HorizontalBodyRoll;
using hexa_posture::IDENTITY;
using hexa_posture::Stack;
using hexa_posture::Still;
using hexa_posture::VerticalBodyRoll;

AnimationContext ctx(
    double t = 0.0, bool walking = false,
    std::optional<std::string> gait_name = std::nullopt,
    std::optional<std::pair<double, double>> support_centroid_xy = std::nullopt,
    std::optional<double> swing_lift_z = std::nullopt,
    std::optional<double> master_phase = std::nullopt) {
  AnimationContext c;
  c.t = t;
  c.walking = walking;
  c.gait_name = std::move(gait_name);
  c.support_centroid_xy = support_centroid_xy;
  c.swing_lift_z = swing_lift_z;
  c.master_phase = master_phase;
  return c;
}

// --- Still / Breathing ---

TEST(Animations, StillAlwaysIdentity) {
  Still s;
  EXPECT_EQ(s(ctx(0.0)), IDENTITY);
  EXPECT_EQ(s(ctx(10.0, true)), IDENTITY);
}

TEST(Animations, BreathingSilentWhileWalking) {
  Breathing b;
  EXPECT_EQ(b(ctx(1.0, true)), IDENTITY);
  EXPECT_EQ(b(ctx(2.7, true)), IDENTITY);
}

TEST(Animations, BreathingEmitsZOffsetWhileIdle) {
  Breathing b(0.005, 4.0);
  // Quarter-period — sin(pi/2) = 1, so z hits the amplitude peak.
  const BodyPose out = b(ctx(1.0, false));
  EXPECT_NEAR(out.z, 0.005, 1e-12);
  EXPECT_EQ(out.x, 0.0);
  EXPECT_EQ(out.y, 0.0);
  EXPECT_EQ(out.roll, 0.0);
  EXPECT_EQ(out.pitch, 0.0);
  EXPECT_EQ(out.yaw, 0.0);
}

TEST(Animations, BreathingCompletesACycle) {
  Breathing b(0.005, 4.0);
  EXPECT_NEAR(b(ctx(0.0)).z, 0.0, 1e-12);
  EXPECT_NEAR(b(ctx(4.0)).z, 0.0, 1e-9);
}

// --- Stack ---

TEST(Animations, StackSumsLayerOutputs) {
  auto b1 = std::make_shared<Breathing>(0.003, 4.0);
  auto b2 = std::make_shared<Breathing>(0.002, 4.0);
  Stack stack(std::vector<AnimationPtr>{b1, b2});
  EXPECT_NEAR(stack(ctx(1.0, false)).z, 0.005, 1e-12);
}

TEST(Animations, StackWithNoLayersIsIdentity) {
  Stack empty;
  EXPECT_EQ(empty(ctx()), IDENTITY);
}

// --- GaitSway ---

TEST(Animations, GaitSwayIdentityWhenNotWalking) {
  GaitSway sway(1.0, 1.0);
  EXPECT_EQ(sway(ctx(0.0, false, std::nullopt,
                     std::make_pair(0.02, -0.01))),
            IDENTITY);
}

TEST(Animations, GaitSwayIdentityWhenCentroidMissing) {
  GaitSway sway(1.0, 1.0);
  EXPECT_EQ(sway(ctx(0.0, true, std::nullopt, std::nullopt)), IDENTITY);
}

TEST(Animations, GaitSwayDefaultStrengthIsHalf) {
  // Documenting the conservative default: out of the box GaitSway applies only
  // 50% of the centroid feedforward.
  GaitSway sway;  // defaults gain=1.0, strength=0.5
  const BodyPose out =
      sway(ctx(0.0, true, std::nullopt, std::make_pair(0.04, -0.02)));
  EXPECT_NEAR(out.x, 0.5 * 0.04, 1e-12);
  EXPECT_NEAR(out.y, 0.5 * -0.02, 1e-12);
}

TEST(Animations, GaitSwayIsLinearInCentroid) {
  const double gain = 0.8;
  GaitSway sway(gain, 1.0);
  const double cx = 0.02;
  const double cy = -0.015;
  const BodyPose out =
      sway(ctx(0.0, true, std::nullopt, std::make_pair(cx, cy)));
  EXPECT_NEAR(out.x, gain * cx, 1e-12);
  EXPECT_NEAR(out.y, gain * cy, 1e-12);
  EXPECT_EQ(out.z, 0.0);
  EXPECT_EQ(out.roll, 0.0);
  EXPECT_EQ(out.pitch, 0.0);
  EXPECT_EQ(out.yaw, 0.0);
}

TEST(Animations, GaitSwayStrengthScalesOutput) {
  const double gain = 1.0;
  const double strength = 0.5;
  GaitSway sway(gain, strength);
  const double cx = 0.04;
  const double cy = -0.02;
  const BodyPose out =
      sway(ctx(0.0, true, std::nullopt, std::make_pair(cx, cy)));
  EXPECT_NEAR(out.x, gain * strength * cx, 1e-12);
  EXPECT_NEAR(out.y, gain * strength * cy, 1e-12);
}

TEST(Animations, GaitSwayStrengthZeroIsIdentity) {
  GaitSway sway(1.0, 0.0);
  EXPECT_EQ(sway(ctx(0.0, true, std::nullopt, std::make_pair(0.05, 0.03))),
            IDENTITY);
}

// --- GaitBounce ---

TEST(Animations, GaitBounceIdentityWhenNotWalking) {
  GaitBounce bounce(0.02, 0.06);
  EXPECT_EQ(bounce(ctx(0.0, false, "tripod", std::nullopt, 0.06)), IDENTITY);
}

TEST(Animations, GaitBounceIdentityWhenSignalMissing) {
  GaitBounce bounce(0.02, 0.06);
  EXPECT_EQ(bounce(ctx(0.0, true, "tripod", std::nullopt, std::nullopt)),
            IDENTITY);
}

TEST(Animations, GaitBounceIdentityWhenGaitUnknown) {
  GaitBounce bounce(0.02, 0.06);
  EXPECT_EQ(bounce(ctx(0.0, true, std::nullopt, std::nullopt, 0.06)), IDENTITY);
}

TEST(Animations, GaitBounceIdentityUnderNonTripodGaits) {
  GaitBounce bounce(0.02, 0.06);
  for (const std::string name : {"tetrapod", "ripple", "crawl", "surf"}) {
    EXPECT_EQ(bounce(ctx(0.0, true, name, std::nullopt, 0.06)), IDENTITY)
        << "GaitBounce must stay silent under " << name;
  }
}

TEST(Animations, GaitBounceZeroAtRest) {
  GaitBounce bounce(0.02, 0.06);
  EXPECT_EQ(bounce(ctx(0.0, true, "tripod", std::nullopt, 0.0)), IDENTITY);
}

TEST(Animations, GaitBouncePeaksAtApex) {
  GaitBounce bounce(0.02, 0.06);
  const BodyPose out = bounce(ctx(0.0, true, "tripod", std::nullopt, 0.06));
  EXPECT_NEAR(out.z, 0.02, 1e-12);
  EXPECT_EQ(out.x, 0.0);
  EXPECT_EQ(out.y, 0.0);
  EXPECT_EQ(out.roll, 0.0);
  EXPECT_EQ(out.pitch, 0.0);
  EXPECT_EQ(out.yaw, 0.0);
}

TEST(Animations, GaitBounceIsLinearInSwingLift) {
  GaitBounce bounce(0.02, 0.06);
  EXPECT_NEAR(bounce(ctx(0.0, true, "tripod", std::nullopt, 0.03)).z, 0.01,
              1e-12);
}

TEST(Animations, GaitBounceClampsAboveReferenceHeight) {
  GaitBounce bounce(0.02, 0.06);
  EXPECT_NEAR(bounce(ctx(0.0, true, "tripod", std::nullopt, 0.10)).z, 0.02,
              1e-12);
}

TEST(Animations, GaitBounceArcHeightZeroIsIdentity) {
  GaitBounce bounce(0.0, 0.06);
  EXPECT_EQ(bounce(ctx(0.0, true, "tripod", std::nullopt, 0.06)), IDENTITY);
}

// --- Stack composition regressions ---

TEST(Animations, GaitBounceStacksWithGaitSwayOnIndependentAxes) {
  auto sway = std::make_shared<GaitSway>(1.0, 1.0);
  auto bounce = std::make_shared<GaitBounce>(0.02, 0.06);
  Stack stack(std::vector<AnimationPtr>{sway, bounce});
  const BodyPose out =
      stack(ctx(0.0, true, "tripod", std::make_pair(0.03, -0.02), 0.06));
  EXPECT_NEAR(out.x, 0.03, 1e-12);
  EXPECT_NEAR(out.y, -0.02, 1e-12);
  EXPECT_NEAR(out.z, 0.02, 1e-12);
}

TEST(Animations, GaitSwayStacksAdditivelyWithBreathing) {
  auto sway = std::make_shared<GaitSway>(1.0, 1.0);
  auto breath = std::make_shared<Breathing>(0.005, 4.0);
  Stack stack(std::vector<AnimationPtr>{sway, breath});
  const double cx = 0.03;
  const double cy = 0.01;
  const BodyPose out =
      stack(ctx(1.0, true, std::nullopt, std::make_pair(cx, cy)));
  EXPECT_NEAR(out.x, cx, 1e-12);
  EXPECT_NEAR(out.y, cy, 1e-12);
  EXPECT_NEAR(out.z, 0.0, 1e-12);
}

// --- Body-roll animations (new coverage) ---

constexpr double kTwoPi = 2.0 * M_PI;

TEST(Animations, VerticalBodyRollGates) {
  VerticalBodyRoll roll;
  EXPECT_EQ(roll(ctx(0.0, false, "tripod", std::nullopt, std::nullopt, 0.0)),
            IDENTITY);  // not walking
  EXPECT_EQ(roll(ctx(0.0, true, "tripod", std::nullopt, std::nullopt,
                     std::nullopt)),
            IDENTITY);  // no master phase
  EXPECT_EQ(roll(ctx(0.0, true, "ripple", std::nullopt, std::nullopt, 0.0)),
            IDENTITY);  // non-tripod
}

TEST(Animations, VerticalBodyRollValues) {
  VerticalBodyRoll roll(0.02, 0.1745, 0.0);
  // phi = 0: z = -z_amp*cos(0) = -z_amp; pitch = -amp*sin(0) = 0.
  BodyPose a = roll(ctx(0.0, true, "tripod", std::nullopt, std::nullopt, 0.0));
  EXPECT_NEAR(a.z, -0.02, 1e-12);
  EXPECT_NEAR(a.pitch, 0.0, 1e-12);
  // phi = 0.25: z = -z_amp*cos(pi/2) = 0; pitch = -amp*sin(pi/2) = -amp.
  BodyPose b = roll(ctx(0.0, true, "tripod", std::nullopt, std::nullopt, 0.25));
  EXPECT_NEAR(b.z, 0.0, 1e-12);
  EXPECT_NEAR(b.pitch, -0.1745, 1e-12);
  EXPECT_EQ(b.y, 0.0);
  EXPECT_EQ(b.yaw, 0.0);
}

TEST(Animations, HorizontalBodyRollValues) {
  HorizontalBodyRoll roll(0.02, 0.1745, 0.0);
  // phi = 0: y = -y_amp; yaw = +amp*sin(0) = 0.
  BodyPose a = roll(ctx(0.0, true, "tripod", std::nullopt, std::nullopt, 0.0));
  EXPECT_NEAR(a.y, -0.02, 1e-12);
  EXPECT_NEAR(a.yaw, 0.0, 1e-12);
  // phi = 0.25: y = 0; yaw = +amp*sin(pi/2) = +amp (positive convention).
  BodyPose b = roll(ctx(0.0, true, "tripod", std::nullopt, std::nullopt, 0.25));
  EXPECT_NEAR(b.y, 0.0, 1e-12);
  EXPECT_NEAR(b.yaw, 0.1745, 1e-12);
  EXPECT_EQ(b.z, 0.0);
  EXPECT_EQ(b.pitch, 0.0);
}

TEST(Animations, HorizontalBodyRollGatedNonTripod) {
  HorizontalBodyRoll roll;
  EXPECT_EQ(roll(ctx(0.0, true, "crawl", std::nullopt, std::nullopt, 0.3)),
            IDENTITY);
}

TEST(Animations, BodyRoll3DValues) {
  BodyRoll3D roll(0.02, 0.1745, 0.02, 0.1745, 0.25, 0.0, 0.0);
  // phi = 0, phi_h = 0.25:
  //   z = -z_amp*cos(0) = -z_amp
  //   pitch = -amp*sin(0) = 0
  //   y = -y_amp*cos(pi/2) = 0
  //   yaw = +amp*sin(pi/2) = +amp
  const BodyPose p =
      roll(ctx(0.0, true, "tripod", std::nullopt, std::nullopt, 0.0));
  EXPECT_NEAR(p.z, -0.02, 1e-12);
  EXPECT_NEAR(p.pitch, 0.0, 1e-12);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_NEAR(p.yaw, 0.1745, 1e-12);
}

TEST(Animations, BodyRoll3DGatedNotWalking) {
  BodyRoll3D roll;
  EXPECT_EQ(roll(ctx(0.0, false, "tripod", std::nullopt, std::nullopt, 0.1)),
            IDENTITY);
}

// --- build_animation_stack ---

TEST(Animations, BuildAnimationStackUnknownNameThrows) {
  EXPECT_THROW(build_animation_stack({"still", "nope"}), std::invalid_argument);
}

TEST(Animations, BuildAnimationStackDefaultConstructs) {
  Stack s = build_animation_stack({"still", "breathing"});
  EXPECT_EQ(s.layers().size(), 2u);
  // Breathing default amplitude 0.005 at quarter period.
  EXPECT_NEAR(s(ctx(1.0, false)).z, 0.005, 1e-12);
}

TEST(Animations, BuildAnimationStackOverrideWins) {
  // Default gait_sway strength is 0.5; the override forces 1.0, so the stack
  // output tracks the centroid one-for-one.
  std::map<std::string, AnimationPtr> overrides = {
      {"gait_sway", std::make_shared<GaitSway>(1.0, 1.0)}};
  Stack s = build_animation_stack({"gait_sway"}, overrides);
  const BodyPose out =
      s(ctx(0.0, true, std::nullopt, std::make_pair(0.04, -0.02)));
  EXPECT_NEAR(out.x, 0.04, 1e-12);
  EXPECT_NEAR(out.y, -0.02, 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
