// Posture controller — gait-active body-animation master switch.
//
// Pins the one behaviour tuning.yaml's `gait_body_animations_enabled` promises:
// with it false the body holds still through a walk (no sway, no bounce, no
// ANIMATION-mode roll), and with it true the same tick moves the body. Everything
// else about the stack (filters, layered clamp, engine-state gating) is exercised
// through test_pipeline.
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

// The switch covers the ANIMATION-mode stacks too, not just the default one:
// "no body animation while gait-active" means the demo rolls stay off as well.
TEST(PostureGaitAnimationSwitch, DisabledAlsoSilencesAnimationMode) {
  PostureController posture{with_switch(false)};
  ASSERT_TRUE(posture.set_animation_mode("body_roll_3d"));
  EXPECT_TRUE(is_identity(walk_until_settled(posture)));

  PostureController enabled{with_switch(true)};
  ASSERT_TRUE(enabled.set_animation_mode("body_roll_3d"));
  EXPECT_FALSE(is_identity(walk_until_settled(enabled)));
}

}  // namespace
