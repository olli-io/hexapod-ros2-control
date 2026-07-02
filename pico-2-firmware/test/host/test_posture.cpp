// Host tests for the float posture port (plan part 08).
//
// Two tiers:
//   1. Behavioural unit tests — a direct port of the three hexa_posture pytest
//      suites (test_pose / test_animations / test_posture_node). The expected
//      values are copied verbatim from those tests, which are the reference
//      behaviour, so this tier verifies the port with no transcription-of-logic
//      risk on the leaf math.
//   2. A golden-trace integration test — drives one PostureController across a
//      recorded engine trace and asserts the full body_pose_target matches the
//      real Python hexa_posture animation stack (posture_golden_generated.hpp,
//      emitted by gen_posture_golden.py) within tolerance.
//
// Float-only, so it builds under -Wdouble-promotion (the same gate as the
// firmware), proving the port never silently promotes to double.

#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "gait/engine.hpp"
#include "gait/types.hpp"
#include "posture/animations.hpp"
#include "posture/posture.hpp"
#include "posture/pose.hpp"

#include "posture_golden_generated.hpp"

namespace posture = hexa::posture;
namespace gait = hexa::gait;
using posture::BodyPose;
using posture::IDENTITY;

namespace {

constexpr float kTol = 1e-6f;

void expect_pose_eq(const BodyPose& a, const BodyPose& b, float tol = kTol) {
  EXPECT_NEAR(a.x, b.x, tol);
  EXPECT_NEAR(a.y, b.y, tol);
  EXPECT_NEAR(a.z, b.z, tol);
  EXPECT_NEAR(a.roll, b.roll, tol);
  EXPECT_NEAR(a.pitch, b.pitch, tol);
  EXPECT_NEAR(a.yaw, b.yaw, tol);
}

void expect_identity(const BodyPose& p) { expect_pose_eq(p, IDENTITY); }

posture::AnimationContext ctx_walking(bool walking) {
  posture::AnimationContext c;
  c.walking = walking;
  return c;
}

// Build a six-leg output map from (name, x, y, stance, z) samples.
std::map<std::string, gait::LegOutput> make_legs(
    std::initializer_list<std::tuple<const char*, float, float, bool, float>>
        samples) {
  std::map<std::string, gait::LegOutput> legs;
  for (const auto& [name, x, y, stance, z] : samples) {
    gait::LegOutput o;
    o.foot_target = hexa::Vec3(x, y, z);
    o.stance = stance;
    legs[name] = o;
  }
  return legs;
}

}  // namespace

// ── pose algebra (port of test_pose.py) ─────────────────────────────────────

TEST(PosturePose, IdentityIsAllZeros) { expect_identity(IDENTITY); }

TEST(PosturePose, AddIsComponentWise) {
  BodyPose a{0.01f, 0.02f, 0.03f, 0.1f, 0.2f, 0.3f};
  BodyPose b{-0.005f, 0.0f, 0.01f, -0.05f, 0.0f, 0.1f};
  BodyPose s = posture::add(a, b);
  expect_pose_eq(s, BodyPose{0.005f, 0.02f, 0.04f, 0.05f, 0.2f, 0.4f});
}

TEST(PosturePose, ScaleIsUniform) {
  BodyPose p{0.04f, -0.02f, 0.01f, 0.1f, -0.1f, 0.2f};
  BodyPose s = posture::scale(p, 0.5f);
  expect_pose_eq(s, BodyPose{0.02f, -0.01f, 0.005f, 0.05f, -0.05f, 0.1f});
}

TEST(PosturePose, ClampCapsEachAxisSymmetrically) {
  posture::PoseLimits limits;
  BodyPose runaway{10.0f, -10.0f, 1.0f, 5.0f, -5.0f, 2.0f};
  BodyPose c = posture::clamp(runaway, limits);
  expect_pose_eq(c, BodyPose{0.05f, -0.05f, 0.04f, 0.30f, -0.30f, 0.50f});
}

TEST(PosturePose, ClampPassesThroughInEnvelopeValues) {
  posture::PoseLimits limits;
  BodyPose inside{0.01f, -0.02f, 0.005f, 0.05f, -0.05f, 0.1f};
  expect_pose_eq(posture::clamp(inside, limits), inside);
}

// ── animations (port of test_animations.py) ─────────────────────────────────

TEST(PostureAnimations, StillAlwaysIdentity) {
  posture::Still s;
  expect_identity(s.eval(ctx_walking(false)));
  expect_identity(s.eval(ctx_walking(true)));
}

TEST(PostureAnimations, BreathingSilentWhileWalking) {
  posture::Breathing b;
  posture::AnimationContext c = ctx_walking(true);
  c.t = 1.0f;
  expect_identity(b.eval(c));
}

TEST(PostureAnimations, BreathingEmitsZOffsetWhileIdle) {
  posture::Breathing b(0.005f, 4.0f);
  posture::AnimationContext c = ctx_walking(false);
  c.t = 1.0f;  // quarter period -> sin(pi/2) = 1 -> z = amplitude
  BodyPose out = b.eval(c);
  EXPECT_NEAR(out.z, 0.005f, 1e-7f);
  EXPECT_EQ(out.x, 0.0f);
  EXPECT_EQ(out.y, 0.0f);
  EXPECT_EQ(out.roll, 0.0f);
}

TEST(PostureAnimations, BreathingCompletesACycle) {
  posture::Breathing b(0.005f, 4.0f);
  posture::AnimationContext c0 = ctx_walking(false);
  c0.t = 0.0f;
  EXPECT_NEAR(b.eval(c0).z, 0.0f, 1e-7f);
  posture::AnimationContext c4 = ctx_walking(false);
  c4.t = 4.0f;
  EXPECT_NEAR(b.eval(c4).z, 0.0f, 1e-6f);
}

TEST(PostureAnimations, GaitSwayIdentityWhenNotWalking) {
  posture::GaitSway sway(1.0f, 1.0f);
  posture::AnimationContext c = ctx_walking(false);
  c.support_centroid_xy = std::make_pair(0.02f, -0.01f);
  expect_identity(sway.eval(c));
}

TEST(PostureAnimations, GaitSwayIdentityWhenCentroidMissing) {
  posture::GaitSway sway(1.0f, 1.0f);
  expect_identity(sway.eval(ctx_walking(true)));
}

TEST(PostureAnimations, GaitSwayLinearInCentroid) {
  const float gain = 0.8f;
  posture::GaitSway sway(gain, 1.0f);
  posture::AnimationContext c = ctx_walking(true);
  c.support_centroid_xy = std::make_pair(0.02f, -0.015f);
  BodyPose out = sway.eval(c);
  EXPECT_NEAR(out.x, gain * 0.02f, 1e-7f);
  EXPECT_NEAR(out.y, gain * -0.015f, 1e-7f);
  EXPECT_EQ(out.z, 0.0f);
}

TEST(PostureAnimations, GaitSwayStrengthScalesOutput) {
  posture::GaitSway sway(1.0f, 0.5f);
  posture::AnimationContext c = ctx_walking(true);
  c.support_centroid_xy = std::make_pair(0.04f, -0.02f);
  BodyPose out = sway.eval(c);
  EXPECT_NEAR(out.x, 0.5f * 0.04f, 1e-7f);
  EXPECT_NEAR(out.y, 0.5f * -0.02f, 1e-7f);
}

TEST(PostureAnimations, GaitSwayStrengthZeroIsIdentity) {
  posture::GaitSway sway(1.0f, 0.0f);
  posture::AnimationContext c = ctx_walking(true);
  c.support_centroid_xy = std::make_pair(0.05f, 0.03f);
  expect_identity(sway.eval(c));
}

TEST(PostureAnimations, GaitBounceGates) {
  posture::GaitBounce bounce(0.02f, 0.06f);
  // Not walking.
  posture::AnimationContext c = ctx_walking(false);
  c.gait_name = "tripod";
  c.swing_lift_z = 0.06f;
  expect_identity(bounce.eval(c));
  // Missing signal.
  c = ctx_walking(true);
  c.gait_name = "tripod";
  expect_identity(bounce.eval(c));
  // Unknown gait (empty name).
  c = ctx_walking(true);
  c.swing_lift_z = 0.06f;
  expect_identity(bounce.eval(c));
  // Non-tripod gaits.
  for (const char* name : {"tetrapod", "ripple", "crawl", "surf"}) {
    SCOPED_TRACE(name);
    c = ctx_walking(true);
    c.gait_name = name;
    c.swing_lift_z = 0.06f;
    expect_identity(bounce.eval(c));
  }
}

TEST(PostureAnimations, GaitBounceZeroAtRest) {
  posture::GaitBounce bounce(0.02f, 0.06f);
  posture::AnimationContext c = ctx_walking(true);
  c.gait_name = "tripod";
  c.swing_lift_z = 0.0f;
  expect_identity(bounce.eval(c));
}

TEST(PostureAnimations, GaitBouncePeaksAtApex) {
  posture::GaitBounce bounce(0.02f, 0.06f);
  posture::AnimationContext c = ctx_walking(true);
  c.gait_name = "tripod";
  c.swing_lift_z = 0.06f;
  BodyPose out = bounce.eval(c);
  EXPECT_NEAR(out.z, 0.02f, 1e-7f);
  EXPECT_EQ(out.x, 0.0f);
}

TEST(PostureAnimations, GaitBounceLinearAndClamped) {
  posture::GaitBounce bounce(0.02f, 0.06f);
  posture::AnimationContext c = ctx_walking(true);
  c.gait_name = "tripod";
  c.swing_lift_z = 0.03f;  // half apex -> half height
  EXPECT_NEAR(bounce.eval(c).z, 0.01f, 1e-7f);
  c.swing_lift_z = 0.10f;  // beyond reference -> clamps to arc_height
  EXPECT_NEAR(bounce.eval(c).z, 0.02f, 1e-7f);
}

TEST(PostureAnimations, GaitBounceArcHeightZeroIsIdentity) {
  posture::GaitBounce bounce(0.0f, 0.06f);
  posture::AnimationContext c = ctx_walking(true);
  c.gait_name = "tripod";
  c.swing_lift_z = 0.06f;
  expect_identity(bounce.eval(c));
}

TEST(PostureAnimations, VerticalBodyRollPhaseLocked) {
  // z = -A cos(2*pi*phi); pitch = -B sin(2*pi*(phi+off)).
  posture::VerticalBodyRoll roll(0.02f, 0.1745f, 0.0f);
  posture::AnimationContext c = ctx_walking(true);
  c.gait_name = "tripod";
  c.master_phase = 0.0f;  // trough of the cosine
  BodyPose out = roll.eval(c);
  EXPECT_NEAR(out.z, -0.02f, 1e-6f);
  EXPECT_NEAR(out.pitch, 0.0f, 1e-6f);
  c.master_phase = 0.25f;  // z crosses zero, pitch at -B
  out = roll.eval(c);
  EXPECT_NEAR(out.z, 0.0f, 1e-6f);
  EXPECT_NEAR(out.pitch, -0.1745f, 1e-6f);
}

TEST(PostureAnimations, VerticalBodyRollGatesOnTripodAndWalking) {
  posture::VerticalBodyRoll roll(0.02f, 0.1745f, 0.0f);
  posture::AnimationContext c = ctx_walking(false);
  c.gait_name = "tripod";
  c.master_phase = 0.3f;
  expect_identity(roll.eval(c));  // not walking
  c = ctx_walking(true);
  c.master_phase = 0.3f;  // no gait_name
  expect_identity(roll.eval(c));
  c = ctx_walking(true);
  c.gait_name = "ripple";
  c.master_phase = 0.3f;
  expect_identity(roll.eval(c));  // non-tripod
}

TEST(PostureAnimations, HorizontalBodyRollMirrorsVertical) {
  posture::HorizontalBodyRoll roll(0.02f, 0.1745f, 0.0f);
  posture::AnimationContext c = ctx_walking(true);
  c.gait_name = "tripod";
  c.master_phase = 0.0f;
  BodyPose out = roll.eval(c);
  EXPECT_NEAR(out.y, -0.02f, 1e-6f);
  EXPECT_NEAR(out.yaw, 0.0f, 1e-6f);
  c.master_phase = 0.25f;
  out = roll.eval(c);
  EXPECT_NEAR(out.y, 0.0f, 1e-6f);
  EXPECT_NEAR(out.yaw, 0.1745f, 1e-6f);  // +A sin
}

TEST(PostureAnimations, BodyRoll3DCombinesWithQuarterOffset) {
  posture::BodyRoll3D roll(0.02f, 0.1745f, 0.02f, 0.1745f, 0.25f, 0.0f, 0.0f);
  posture::AnimationContext c = ctx_walking(true);
  c.gait_name = "tripod";
  c.master_phase = 0.0f;
  BodyPose out = roll.eval(c);
  // Vertical pair at phi=0: z=-A, pitch=0. Horizontal pair at phi_h=0.25:
  // y = -A cos(pi/2) = 0; yaw = A sin(pi/2) = A.
  EXPECT_NEAR(out.z, -0.02f, 1e-6f);
  EXPECT_NEAR(out.pitch, 0.0f, 1e-6f);
  EXPECT_NEAR(out.y, 0.0f, 1e-6f);
  EXPECT_NEAR(out.yaw, 0.1745f, 1e-6f);
}

TEST(PostureAnimations, StackSumsLayerOutputs) {
  auto b1 = std::make_shared<posture::Breathing>(0.003f, 4.0f);
  auto b2 = std::make_shared<posture::Breathing>(0.002f, 4.0f);
  posture::Stack stack({b1, b2});
  posture::AnimationContext c = ctx_walking(false);
  c.t = 1.0f;
  EXPECT_NEAR(stack.eval(c).z, 0.005f, 1e-7f);
}

TEST(PostureAnimations, StackWithNoLayersIsIdentity) {
  posture::Stack stack;
  expect_identity(stack.eval(ctx_walking(false)));
}

TEST(PostureAnimations, SwayAndBounceStackOnIndependentAxes) {
  auto sway = std::make_shared<posture::GaitSway>(1.0f, 1.0f);
  auto bounce = std::make_shared<posture::GaitBounce>(0.02f, 0.06f);
  posture::Stack stack({sway, bounce});
  posture::AnimationContext c = ctx_walking(true);
  c.gait_name = "tripod";
  c.support_centroid_xy = std::make_pair(0.03f, -0.02f);
  c.swing_lift_z = 0.06f;
  BodyPose out = stack.eval(c);
  EXPECT_NEAR(out.x, 0.03f, 1e-7f);
  EXPECT_NEAR(out.y, -0.02f, 1e-7f);
  EXPECT_NEAR(out.z, 0.02f, 1e-7f);
}

// ── signal derivation + filters (port of test_posture_node.py) ──────────────

TEST(PostureSignals, StanceCentroidIsMeanOfStanceLegs) {
  auto legs = make_legs({{"l_front", 1.0f, 0.0f, true, 0.0f},
                         {"l_middle", 0.0f, 1.0f, true, 0.0f},
                         {"l_rear", -1.0f, -1.0f, true, 0.0f},
                         {"r_front", 99.0f, 99.0f, false, 0.0f},
                         {"r_middle", 99.0f, 99.0f, false, 0.0f},
                         {"r_rear", 99.0f, 99.0f, false, 0.0f}});
  auto c = posture::stance_centroid_xy(legs);
  ASSERT_TRUE(c.has_value());
  EXPECT_NEAR(c->first, 0.0f, 1e-6f);
  EXPECT_NEAR(c->second, 0.0f, 1e-6f);
}

TEST(PostureSignals, StanceCentroidNoneWhenDegenerate) {
  auto legs = make_legs({{"l_front", 0.1f, 0.1f, true, 0.0f},
                         {"l_middle", -0.1f, 0.0f, true, 0.0f},
                         {"l_rear", 0.0f, 0.0f, false, 0.0f},
                         {"r_front", 0.0f, 0.0f, false, 0.0f},
                         {"r_middle", 0.0f, 0.0f, false, 0.0f},
                         {"r_rear", 0.0f, 0.0f, false, 0.0f}});
  EXPECT_FALSE(posture::stance_centroid_xy(legs).has_value());
}

TEST(PostureSignals, MaxSwingLiftPicksHighestAboveStanceMean) {
  auto legs = make_legs({{"l_front", 0.0f, 0.0f, false, 0.04f},
                         {"l_middle", 0.0f, 0.0f, true, 0.0f},
                         {"l_rear", 0.0f, 0.0f, true, 0.0f},
                         {"r_front", 0.0f, 0.0f, false, 0.06f},
                         {"r_middle", 0.0f, 0.0f, true, 0.0f},
                         {"r_rear", 0.0f, 0.0f, true, 0.0f}});
  auto lift = posture::max_swing_lift_z(legs);
  ASSERT_TRUE(lift.has_value());
  EXPECT_NEAR(*lift, 0.06f, 1e-6f);
}

TEST(PostureSignals, MaxSwingLiftOffsetsAgainstStanceMean) {
  auto legs = make_legs({{"l_front", 0.0f, 0.0f, false, -0.04f},
                         {"l_middle", 0.0f, 0.0f, true, -0.10f},
                         {"l_rear", 0.0f, 0.0f, true, -0.10f},
                         {"r_front", 0.0f, 0.0f, true, -0.10f},
                         {"r_middle", 0.0f, 0.0f, true, -0.10f},
                         {"r_rear", 0.0f, 0.0f, true, -0.10f}});
  auto lift = posture::max_swing_lift_z(legs);
  ASSERT_TRUE(lift.has_value());
  EXPECT_NEAR(*lift, 0.06f, 1e-6f);  // ground=-0.10, apex=-0.04
}

TEST(PostureSignals, MaxSwingLiftZeroWhenNoneSwinging) {
  auto legs = make_legs({{"l_front", 0.0f, 0.0f, true, 0.0f},
                         {"l_middle", 0.0f, 0.0f, true, 0.0f},
                         {"l_rear", 0.0f, 0.0f, true, 0.0f},
                         {"r_front", 0.0f, 0.0f, true, 0.0f},
                         {"r_middle", 0.0f, 0.0f, true, 0.0f},
                         {"r_rear", 0.0f, 0.0f, true, 0.0f}});
  auto lift = posture::max_swing_lift_z(legs);
  ASSERT_TRUE(lift.has_value());
  EXPECT_EQ(*lift, 0.0f);
}

TEST(PostureSignals, MaxSwingLiftNoneWhenDegenerate) {
  auto legs = make_legs({{"l_front", 0.0f, 0.0f, true, 0.0f},
                         {"l_middle", 0.0f, 0.0f, true, 0.0f},
                         {"l_rear", 0.0f, 0.0f, false, 0.05f},
                         {"r_front", 0.0f, 0.0f, false, 0.05f},
                         {"r_middle", 0.0f, 0.0f, false, 0.05f},
                         {"r_rear", 0.0f, 0.0f, false, 0.05f}});
  EXPECT_FALSE(posture::max_swing_lift_z(legs).has_value());
}

TEST(PostureSignals, MaxSwingLiftClampsNegativeToZero) {
  auto legs = make_legs({{"l_front", 0.0f, 0.0f, false, -0.01f},
                         {"l_middle", 0.0f, 0.0f, true, 0.0f},
                         {"l_rear", 0.0f, 0.0f, true, 0.0f},
                         {"r_front", 0.0f, 0.0f, true, 0.0f},
                         {"r_middle", 0.0f, 0.0f, true, 0.0f},
                         {"r_rear", 0.0f, 0.0f, true, 0.0f}});
  auto lift = posture::max_swing_lift_z(legs);
  ASSERT_TRUE(lift.has_value());
  EXPECT_EQ(*lift, 0.0f);
}

TEST(PostureFilters, LpfSeedsOnFirstSample) {
  auto out = posture::lpf_step_xy(std::nullopt, std::make_pair(0.02f, -0.01f),
                                  0.1f, 0.02f);
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->first, 0.02f);
  EXPECT_EQ(out->second, -0.01f);
}

TEST(PostureFilters, LpfHoldsPreviousWhenRawMissing) {
  auto prev = std::make_pair(0.03f, 0.01f);
  auto out = posture::lpf_step_xy(prev, std::nullopt, 0.1f, 0.02f);
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->first, 0.03f);
  EXPECT_EQ(out->second, 0.01f);
}

TEST(PostureFilters, LpfScalarSeedsAndHolds) {
  EXPECT_EQ(posture::lpf_step_scalar(std::nullopt, 0.05f, 0.05f, 0.02f), 0.05f);
  EXPECT_EQ(posture::lpf_step_scalar(0.03f, std::nullopt, 0.05f, 0.02f), 0.03f);
}

TEST(PostureFilters, LpfScalarSettlesAfterAFewTau) {
  const float tau = 0.05f;
  const float dt = 0.005f;
  const float target = 0.06f;
  std::optional<float> state = 0.0f;
  const int n = static_cast<int>(std::lround((4.0f * tau) / dt));
  for (int i = 0; i < n; ++i) {
    state = posture::lpf_step_scalar(state, target, tau, dt);
  }
  EXPECT_LE(std::fabs(*state - target), 0.05f * target);
}

TEST(PostureGate, ActiveStatesMatchReference) {
  using E = gait::EngineState;
  for (E e : {E::STAND, E::ENGAGING, E::GAIT, E::PAUSING, E::PAUSED,
              E::RESUMING, E::RESEATING}) {
    EXPECT_TRUE(posture::posture_active(e))
        << gait::state_value(e) << " should be active";
  }
  for (E e : {E::FOLDED, E::INITIALIZE, E::FOLDING}) {
    EXPECT_FALSE(posture::posture_active(e))
        << gait::state_value(e) << " should be inactive";
  }
}

// ── golden-trace integration ────────────────────────────────────────────────

namespace {
gait::EngineState state_from_string(std::string_view s) {
  using E = gait::EngineState;
  for (E e : {E::FOLDED, E::INITIALIZE, E::STAND, E::ENGAGING, E::GAIT,
              E::PAUSING, E::PAUSED, E::RESUMING, E::FOLDING, E::RESEATING}) {
    if (gait::state_value(e) == s) {
      return e;
    }
  }
  ADD_FAILURE() << "unknown state string: " << s;
  return E::FOLDED;
}
}  // namespace

TEST(PostureGolden, MatchesPythonReferenceTrace) {
  posture::PostureController controller;
  float t = 0.0f;
  // Loosened tolerance: Python reference is double, the port is float, and the
  // low-pass recursion accumulates across the trace.
  constexpr float kGoldenTol = 1e-4f;

  for (int i = 0; i < posture_golden::kNumFrames; ++i) {
    const auto& f = posture_golden::kFrames[i];
    const auto& e = posture_golden::kExpected[i];

    std::map<std::string, gait::LegOutput> legs;
    for (int L = 0; L < 6; ++L) {
      const auto& s = f.legs[L];
      gait::LegOutput o;
      o.foot_target = hexa::Vec3(s.x, s.y, s.z);
      o.stance = s.stance;
      legs[std::string(gait::LEG_NAMES[static_cast<std::size_t>(L)])] = o;
    }

    controller.set_user_pose(BodyPose{f.user_pose[0], f.user_pose[1],
                                      f.user_pose[2], f.user_pose[3],
                                      f.user_pose[4], f.user_pose[5]});
    EXPECT_TRUE(controller.set_animation_mode(f.animation_mode))
        << "frame " << i << " mode " << f.animation_mode;

    BodyPose out =
        controller.update(legs, f.master_phase, f.walking,
                          state_from_string(f.state), f.gait_name, f.dt, t);
    t += f.dt;

    SCOPED_TRACE("frame " + std::to_string(i));
    expect_pose_eq(out, BodyPose{e.x, e.y, e.z, e.roll, e.pitch, e.yaw},
                   kGoldenTol);
  }
}
