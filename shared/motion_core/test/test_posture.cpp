// Posture controller — gait-active body-animation master switch, and the
// body-pose clamp envelope.
//
// Pins the one behaviour tuning.yaml's `gait_body_animations_enabled` promises:
// with it false the default stack holds the body still through a walk (no sway,
// no bounce — but an explicitly selected ANIMATION-mode stack still runs), and
// with it true the same tick moves the body. Everything else about the stack
// (filters, engine-state gating) is exercised through test_pipeline.
//
// The clamp cases below pin the absolute-to-offset conversion: tuning.yaml
// states belly clearance off the ground, BodyPose::z is a delta from the
// stance, and the controller's constructor is the only place the two meet.
#include "posture/posture.hpp"

#include <gtest/gtest.h>

#include <map>
#include <string>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gait/types.hpp"

namespace {

using hexa::gait::EngineState;
using hexa::gait::LegOutput;
using hexa::posture::BodyPose;
using hexa::posture::PostureController;

constexpr float kDt = 0.005f;

// One tripod tick: the left tripod in swing at apex, the right tripod in stance
// and offset in +y, so the support centroid is off-body-centre (GaitSway) and the
// swing lift is a full step height (GaitBounce). Both animations therefore have
// something to say whenever the walking regime is evaluated.
std::map<std::string, LegOutput> tripod_legs() {
  std::map<std::string, LegOutput> legs;
  for (const auto& name : hexa::gait::LEG_NAMES) {
    const bool swing = name == "l_front" || name == "r_middle" ||
                       name == "l_rear";
    LegOutput leg;
    leg.stance = !swing;
    leg.foot_target = hexa::Vec3{0.1f, swing ? 0.0f : 0.12f,
                                 swing ? -0.02f : -0.08f};
    legs[name] = leg;
  }
  return legs;
}

// Run the controller far past the activation slew (4/s => 0.25 s) so the
// crossfade has fully settled on the walking regime.
BodyPose walk_until_settled(PostureController& posture) {
  const auto legs = tripod_legs();
  BodyPose out;
  float t = 0.0f;
  for (int i = 0; i < 400; ++i) {
    out = posture.update(legs, 0.25f, /*walking=*/true, EngineState::GAIT,
                         "tripod", kDt, t);
    t += kDt;
  }
  return out;
}

bool is_identity(const BodyPose& p) {
  constexpr float kTol = 1e-6f;
  return std::abs(p.x) < kTol && std::abs(p.y) < kTol && std::abs(p.z) < kTol &&
         std::abs(p.roll) < kTol && std::abs(p.pitch) < kTol &&
         std::abs(p.yaw) < kTol;
}

hexa::config::PostureConfig with_switch(bool enabled) {
  hexa::config::PostureConfig p = hexa::config::kPosture;
  p.gait_body_animations_enabled = enabled;
  return p;
}

TEST(PostureGaitAnimationSwitch, DisabledHoldsBodyStillWhileWalking) {
  PostureController posture{with_switch(false)};
  EXPECT_TRUE(is_identity(walk_until_settled(posture)))
      << "gait_body_animations_enabled=false must leave no body offset at all "
         "while gait-active";
}

TEST(PostureGaitAnimationSwitch, EnabledMovesTheBodyWhileWalking) {
  PostureController posture{with_switch(true)};
  EXPECT_FALSE(is_identity(walk_until_settled(posture)))
      << "the default stack (gait_sway + gait_bounce) must still animate when "
         "the switch is on";
}

// The switch governs only the DEFAULT stack's implicit gait animations. An
// explicitly selected ANIMATION-mode stack is exempt: the user asked for that
// animation, so the demo rolls run while gait-active regardless of the switch.
TEST(PostureGaitAnimationSwitch, AnimationModeExemptFromSwitch) {
  PostureController posture{with_switch(false)};
  ASSERT_TRUE(posture.set_animation_mode("body_roll_3d"));
  EXPECT_FALSE(is_identity(walk_until_settled(posture)))
      << "gait_body_animations_enabled=false must not silence an explicitly "
         "selected ANIMATION-mode stack";

  // Deselecting back to the default stack re-arms the switch.
  ASSERT_TRUE(posture.set_animation_mode(""));
  EXPECT_TRUE(is_identity(walk_until_settled(posture)));
}

// ── pose clamp envelope ───────────────────────────────────────────────────

// A posture config with an explicit, deliberately asymmetric height envelope:
// nominal belly at 0.05, ceiling at 0.14, floor at 0.02 — so the usable pose
// offsets are +0.09 up and only -0.03 down. Animations are off and the reserve
// is zeroed so the user pose gets the whole envelope and the assertions read
// against the configured numbers directly.
hexa::config::PostureConfig with_height_envelope() {
  hexa::config::PostureConfig p = hexa::config::kPosture;
  p.gait_body_animations_enabled = false;
  p.nominal_body_height = 0.05f;
  p.body_height_max = 0.14f;
  p.body_height_min = 0.02f;
  p.animation_reserve_x = 0.0f;
  p.animation_reserve_y = 0.0f;
  p.animation_reserve_z = 0.0f;
  p.animation_reserve_roll = 0.0f;
  p.animation_reserve_pitch = 0.0f;
  p.animation_reserve_yaw = 0.0f;
  return p;
}

// Settle a still (non-walking) tick so the returned pose is the clamped user
// pose and nothing else.
BodyPose idle_pose(PostureController& posture, const BodyPose& user) {
  posture.set_user_pose(user);
  const auto legs = tripod_legs();
  BodyPose out;
  float t = 0.0f;
  for (int i = 0; i < 400; ++i) {
    out = posture.update(legs, 0.0f, /*walking=*/false, EngineState::STAND,
                         "tripod", kDt, t);
    t += kDt;
  }
  return out;
}

TEST(PosturePoseClamp, HeightSaturatesAsymmetricallyAroundTheStance) {
  PostureController posture{with_height_envelope()};
  constexpr float kTol = 1e-5f;

  // Far past the ceiling: pinned to (0.14 - 0.05) = +0.09 of lift.
  EXPECT_NEAR(idle_pose(posture, BodyPose{0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f}).z,
              0.09f, kTol);

  // Far past the floor: pinned to (0.02 - 0.05) = -0.03 of drop. The two ends
  // are NOT mirror images — that asymmetry is the whole point.
  EXPECT_NEAR(idle_pose(posture, BodyPose{0.0f, 0.0f, -0.5f, 0.0f, 0.0f, 0.0f}).z,
              -0.03f, kTol);
}

TEST(PosturePoseClamp, MidRangeHeightPassesThroughUntouched) {
  PostureController posture{with_height_envelope()};
  // +0.06 of lift = belly 0.11 m, inside the envelope: no clamping at all.
  EXPECT_NEAR(idle_pose(posture, BodyPose{0.0f, 0.0f, 0.06f, 0.0f, 0.0f, 0.0f}).z,
              0.06f, 1e-5f);
}

// The reserve is spent from both ends of an asymmetric envelope, not mirrored
// off the larger one: with 0.01 held back the user keeps +0.08 / -0.02.
TEST(PosturePoseClamp, AnimationReserveShrinksBothEndsOfTheHeightRange) {
  hexa::config::PostureConfig p = with_height_envelope();
  p.animation_reserve_z = 0.01f;
  PostureController posture{p};
  constexpr float kTol = 1e-5f;

  EXPECT_NEAR(idle_pose(posture, BodyPose{0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f}).z,
              0.08f, kTol);
  EXPECT_NEAR(idle_pose(posture, BodyPose{0.0f, 0.0f, -0.5f, 0.0f, 0.0f, 0.0f}).z,
              -0.02f, kTol);
}

// The five symmetric axes come from the same config block; a runaway teleop
// value must still be pinned to the tuning.yaml envelope.
TEST(PosturePoseClamp, SymmetricAxesClampToTheConfiguredLimits) {
  hexa::config::PostureConfig p = with_height_envelope();
  PostureController posture{p};
  constexpr float kTol = 1e-5f;

  const BodyPose out =
      idle_pose(posture, BodyPose{9.0f, -9.0f, 0.0f, 9.0f, -9.0f, 9.0f});
  EXPECT_NEAR(out.x, p.pose_limit_x, kTol);
  EXPECT_NEAR(out.y, -p.pose_limit_y, kTol);
  EXPECT_NEAR(out.roll, p.pose_limit_roll, kTol);
  EXPECT_NEAR(out.pitch, -p.pose_limit_pitch, kTol);
  EXPECT_NEAR(out.yaw, p.pose_limit_yaw, kTol);
}

}  // namespace
