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
//
// The PosturePoseFilter cases pin the spring/inertia smoother on the commanded
// pose — its transient shape, its independence from the tick rate, and that it
// saturates without winding up.
#include "posture/posture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
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
// offsets are +0.09 up and only -0.03 down. Animations are off so the user pose
// gets the whole envelope and the assertions read against the configured
// numbers directly.
hexa::config::PostureConfig with_height_envelope() {
  hexa::config::PostureConfig p = hexa::config::kPosture;
  p.gait_body_animations_enabled = false;
  p.nominal_body_height = 0.05f;
  p.body_height_max = 0.14f;
  p.body_height_min = 0.02f;
  return p;
}

// Settle a still (non-walking) tick so the returned pose is the clamped user
// pose and nothing else.
//
// 1000 ticks = 5 s: the pose filter needs ~3.7 s to reach the 1e-5 these cases
// assert. Steady-state only; the transient is pinned by PosturePoseFilter.
BodyPose idle_pose(PostureController& posture, const BodyPose& user) {
  posture.set_user_pose(user);
  const auto legs = tripod_legs();
  BodyPose out;
  float t = 0.0f;
  for (int i = 0; i < 1000; ++i) {
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

// ── pose input filter ─────────────────────────────────────────────────────

// A step to +0.06 of lift: inside the with_height_envelope() ceiling (+0.09)
// even after the ~35% overshoot zeta = 0.32 produces, so these cases see the
// filter's own response rather than the clamp's.
constexpr float kStepZ = 0.06f;
const BodyPose kStepUp{0.0f, 0.0f, kStepZ, 0.0f, 0.0f, 0.0f};

// Sim time -> pose z, from a cold controller, at an arbitrary tick period.
float step_response_z_at(float horizon_s, float dt) {
  PostureController posture{with_height_envelope()};
  posture.set_user_pose(kStepUp);
  const auto legs = tripod_legs();
  const int ticks = static_cast<int>(horizon_s / dt + 0.5f);
  BodyPose out;
  float t = 0.0f;
  for (int i = 0; i < ticks; ++i) {
    out = posture.update(legs, 0.0f, /*walking=*/false, EngineState::STAND,
                         "tripod", dt, t);
    t += dt;
  }
  return out.z;
}

// The headline behaviour: a stepped command is not a stepped output. It
// accelerates in, crosses the target (zeta < 1), and settles back onto it.
TEST(PosturePoseFilter, StepResponseOvershootsThenSettles) {
  PostureController posture{with_height_envelope()};
  posture.set_user_pose(kStepUp);
  const auto legs = tripod_legs();

  const BodyPose first = posture.update(legs, 0.0f, /*walking=*/false,
                                        EngineState::STAND, "tripod", kDt, 0.0f);
  EXPECT_LT(first.z, 0.1f * kStepZ)
      << "a stepped /body/pose must not arrive at the servos in one tick";

  float peak = first.z;
  float t = kDt;
  for (int i = 1; i < 200; ++i) {  // 1 s covers the first overshoot
    peak = std::max(peak, posture.update(legs, 0.0f, /*walking=*/false,
                                         EngineState::STAND, "tripod", kDt, t)
                              .z);
    t += kDt;
  }
  // zeta = 0.32 predicts exp(-pi*zeta/sqrt(1-zeta^2)) = ~35% overshoot. Bounded
  // on both sides: too little means it is creeping (over-damped), too much
  // means the damping term has been lost.
  EXPECT_GT(peak, 1.15f * kStepZ) << "the response must cross the target";
  EXPECT_LT(peak, 1.5f * kStepZ) << "overshoot must stay bounded";

  for (int i = 0; i < 800; ++i) {  // out to 5 s
    posture.update(legs, 0.0f, /*walking=*/false, EngineState::STAND, "tripod",
                   kDt, t);
    t += kDt;
  }
  EXPECT_NEAR(posture.update(legs, 0.0f, /*walking=*/false, EngineState::STAND,
                             "tripod", kDt, t)
                  .z,
              kStepZ, 1e-5f);
}

// Why the filter takes (tau, zeta) instead of the reference implementation's
// constants, which resolve to a tick-period-dependent omega_n and zeta. One
// sim-time trajectory sampled at 200 Hz and 800 Hz must trace the same curve;
// a literal port fails this.
TEST(PosturePoseFilter, FrameRateInvarianceOfTheResponse) {
  for (const float horizon : {0.05f, 0.1f, 0.2f, 0.4f, 0.8f}) {
    const float coarse = step_response_z_at(horizon, 0.005f);   // 200 Hz
    const float fine = step_response_z_at(horizon, 0.00125f);   // 800 Hz
    EXPECT_NEAR(coarse, fine, 0.02f * kStepZ)
        << "step response diverges between tick rates at t = " << horizon << " s";
  }
}

// A command parked outside the envelope must not bank velocity behind the
// clamp: the moment it is withdrawn the body has to move, not spend time
// unwinding momentum it was never allowed to express.
TEST(PosturePoseFilter, SaturatedAxisDoesNotWindUp) {
  PostureController posture{with_height_envelope()};
  const auto legs = tripod_legs();
  constexpr float kCeiling = 0.09f;  // 0.14 - 0.05
  posture.set_user_pose(BodyPose{0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f});

  BodyPose out;
  float highest = 0.0f;
  float t = 0.0f;
  for (int i = 0; i < 400; ++i) {  // 2 s hard against the ceiling
    out = posture.update(legs, 0.0f, /*walking=*/false, EngineState::STAND,
                         "tripod", kDt, t);
    highest = std::max(highest, out.z);
    t += kDt;
  }
  EXPECT_LE(highest, kCeiling + 1e-6f) << "the filter must saturate, not ring "
                                          "through the envelope";
  ASSERT_NEAR(out.z, kCeiling, 1e-6f);

  posture.set_user_pose(BodyPose{});
  const BodyPose next = posture.update(legs, 0.0f, /*walking=*/false,
                                       EngineState::STAND, "tripod", kDt, t);
  EXPECT_LT(next.z, kCeiling) << "withdrawing the command must move the body on "
                                 "the very next tick";
}

// While posture is inactive the controller emits IDENTITY, so the filter's own
// state has to agree — otherwise the body springs from a stale offset (with a
// stale velocity) the moment it re-activates.
TEST(PosturePoseFilter, InactiveStateResetsTheFilter) {
  PostureController posture{with_height_envelope()};
  const auto legs = tripod_legs();

  ASSERT_NEAR(idle_pose(posture, kStepUp).z, kStepZ, 1e-5f);

  const BodyPose folded = posture.update(legs, 0.0f, /*walking=*/false,
                                         EngineState::FOLDED, "tripod", kDt, 0.0f);
  EXPECT_TRUE(is_identity(folded));

  // Same command still held; the ramp must start over from the stance.
  const BodyPose first = posture.update(legs, 0.0f, /*walking=*/false,
                                        EngineState::STAND, "tripod", kDt, 0.0f);
  EXPECT_LT(first.z, 0.1f * kStepZ);
}

// tau <= 0 is the off switch, in place of a separate enable flag.
TEST(PosturePoseFilter, ZeroTauBypassesTheFilter) {
  hexa::config::PostureConfig p = with_height_envelope();
  p.pose_filter_tau_translation = 0.0f;
  p.pose_filter_tau_rotation = 0.0f;
  PostureController posture{p};
  posture.set_user_pose(kStepUp);

  const BodyPose first = posture.update(tripod_legs(), 0.0f, /*walking=*/false,
                                        EngineState::STAND, "tripod", kDt, 0.0f);
  EXPECT_NEAR(first.z, kStepZ, 1e-6f);
}

}  // namespace
