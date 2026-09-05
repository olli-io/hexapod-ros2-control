// Posture controller: the gait-active body-animation master switch, the
// body-pose clamp envelope, and the pose smoother.
//
// `gait_body_animations_enabled` false holds the body still through a walk while
// an explicitly selected ANIMATION-mode stack still runs; true moves it on the
// same tick. The clamp cases pin the absolute-to-offset conversion, whose only
// meeting point is the controller's constructor. The PosturePoseFilter cases pin
// the smoother's transient shape, its independence from the tick rate, and that
// it saturates without winding up. Everything else goes through test_pipeline.
#include "posture/posture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

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
                         "tripod", hexa::gait::LegSet::HEXAPOD, kDt, t);
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
                         "tripod", hexa::gait::LegSet::HEXAPOD, kDt, t);
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
// value must still be pinned to the tuning.yaml envelope. One axis at a time,
// because x-y and roll-pitch share a disc — a runaway on BOTH members of a pair
// is the PairsClampToAnInscribedDisc case below, not this one.
TEST(PosturePoseClamp, SymmetricAxesClampToTheConfiguredLimits) {
  hexa::config::PostureConfig p = with_height_envelope();
  constexpr float kTol = 1e-5f;

  PostureController px{p};
  EXPECT_NEAR(idle_pose(px, BodyPose{9.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}).x,
              p.pose_limit_x, kTol);
  PostureController py{p};
  EXPECT_NEAR(idle_pose(py, BodyPose{0.0f, -9.0f, 0.0f, 0.0f, 0.0f, 0.0f}).y,
              -p.pose_limit_y, kTol);
  PostureController pr{p};
  EXPECT_NEAR(idle_pose(pr, BodyPose{0.0f, 0.0f, 0.0f, 9.0f, 0.0f, 0.0f}).roll,
              p.pose_limit_roll, kTol);
  PostureController pp{p};
  EXPECT_NEAR(idle_pose(pp, BodyPose{0.0f, 0.0f, 0.0f, 0.0f, -9.0f, 0.0f}).pitch,
              -p.pose_limit_pitch, kTol);
  PostureController pyaw{p};
  EXPECT_NEAR(idle_pose(pyaw, BodyPose{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.0f}).yaw,
              p.pose_limit_yaw, kTol);
}

// x-y and roll-pitch ease as pairs, so their envelope is the disc inscribed in
// the box, not the box: a diagonal runaway lands at limit/sqrt(2) on each
// member, not at the corner. Deliberate — reach is then the same in every
// direction. This case exists so reintroducing a box clamp fails loudly.
TEST(PosturePoseClamp, PairsClampToAnInscribedDisc) {
  hexa::config::PostureConfig p = with_height_envelope();
  PostureController posture{p};
  constexpr float kTol = 1e-5f;
  const float kInvSqrt2 = 0.70710678f;

  const BodyPose out =
      idle_pose(posture, BodyPose{9.0f, -9.0f, 0.0f, 9.0f, -9.0f, 0.0f});
  EXPECT_NEAR(std::hypot(out.x, out.y), p.pose_limit_x, kTol);
  EXPECT_NEAR(out.x, p.pose_limit_x * kInvSqrt2, kTol);
  EXPECT_NEAR(out.y, -p.pose_limit_y * kInvSqrt2, kTol);
  EXPECT_NEAR(std::hypot(out.roll, out.pitch), p.pose_limit_roll, kTol);
  EXPECT_NEAR(out.roll, p.pose_limit_roll * kInvSqrt2, kTol);
  EXPECT_NEAR(out.pitch, -p.pose_limit_pitch * kInvSqrt2, kTol);
}

// ── pose input filter ─────────────────────────────────────────────────────

// A step to +0.06 of lift: inside the with_height_envelope() ceiling (+0.09)
// even after the overshoot the configured damping produces, so these cases see
// the filter's own response rather than the clamp's.
constexpr float kStepZ = 0.06f;
const BodyPose kStepUp{0.0f, 0.0f, kStepZ, 0.0f, 0.0f, 0.0f};

// Standard second-order overshoot, as a fraction of the step. Read off the
// configured zeta rather than pinned to a literal, so retuning damping_ratio in
// tuning.yaml moves the expectation with it instead of failing this.
float predicted_overshoot(float zeta) {
  return std::exp(-3.14159265f * zeta / std::sqrt(1.0f - zeta * zeta));
}

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
                         "tripod", hexa::gait::LegSet::HEXAPOD, dt, t);
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
                                        EngineState::STAND, "tripod", hexa::gait::LegSet::HEXAPOD, kDt, 0.0f);
  EXPECT_LT(first.z, 0.1f * kStepZ)
      << "a stepped /body/pose must not arrive at the servos in one tick";

  float peak = first.z;
  float t = kDt;
  for (int i = 1; i < 200; ++i) {  // 1 s covers the first overshoot
    peak = std::max(peak, posture.update(legs, 0.0f, /*walking=*/false,
                                         EngineState::STAND, "tripod", hexa::gait::LegSet::HEXAPOD, kDt, t)
                              .z);
    t += kDt;
  }
  // Bracketed either side of what the configured zeta predicts: too little
  // means it is creeping (over-damped), too much means the damping term has
  // been lost. The slack absorbs the 200 Hz grid missing the exact peak.
  const float overshoot =
      predicted_overshoot(with_height_envelope().pose_filter_damping_ratio);
  EXPECT_GT(peak, kStepZ * (1.0f + 0.75f * overshoot))
      << "the response must cross the target";
  EXPECT_LT(peak, kStepZ * (1.0f + 1.25f * overshoot))
      << "overshoot must stay bounded";

  for (int i = 0; i < 800; ++i) {  // out to 5 s
    posture.update(legs, 0.0f, /*walking=*/false, EngineState::STAND, "tripod",
                   hexa::gait::LegSet::HEXAPOD, kDt, t);
    t += kDt;
  }
  EXPECT_NEAR(posture.update(legs, 0.0f, /*walking=*/false, EngineState::STAND,
                             "tripod", hexa::gait::LegSet::HEXAPOD, kDt, t)
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
                         "tripod", hexa::gait::LegSet::HEXAPOD, kDt, t);
    highest = std::max(highest, out.z);
    t += kDt;
  }
  EXPECT_LE(highest, kCeiling + 1e-6f) << "the filter must saturate, not ring "
                                          "through the envelope";
  ASSERT_NEAR(out.z, kCeiling, 1e-6f);

  posture.set_user_pose(BodyPose{});
  const BodyPose next = posture.update(legs, 0.0f, /*walking=*/false,
                                       EngineState::STAND, "tripod", hexa::gait::LegSet::HEXAPOD, kDt, t);
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
                                         EngineState::FOLDED, "tripod", hexa::gait::LegSet::HEXAPOD, kDt, 0.0f);
  EXPECT_TRUE(is_identity(folded));

  // Same command still held; the ramp must start over from the stance.
  const BodyPose first = posture.update(legs, 0.0f, /*walking=*/false,
                                        EngineState::STAND, "tripod", hexa::gait::LegSet::HEXAPOD, kDt, 0.0f);
  EXPECT_LT(first.z, 0.1f * kStepZ);
}

// tau <= 0 is the off switch, in place of a separate enable flag. Checked on a
// pair as well as on z: the pair bypass is a separate branch in step_polar.
TEST(PosturePoseFilter, ZeroTauBypassesTheFilter) {
  hexa::config::PostureConfig p = with_height_envelope();
  p.pose_filter_tau = 0.0f;
  PostureController posture{p};
  posture.set_user_pose(BodyPose{0.03f, 0.0f, kStepZ, 0.2f, 0.0f, 0.0f});

  const BodyPose first = posture.update(tripod_legs(), 0.0f, /*walking=*/false,
                                        EngineState::STAND, "tripod", hexa::gait::LegSet::HEXAPOD, kDt, 0.0f);
  EXPECT_NEAR(first.z, kStepZ, 1e-6f);
  EXPECT_NEAR(first.x, 0.03f, 1e-6f);
  EXPECT_NEAR(first.roll, 0.2f, 1e-6f);
}

// ── polar pair easing ─────────────────────────────────────────────────────
//
// These drive PoseSmoother directly rather than through PostureController: the
// assertions are about the shape of the smoother's trajectory, and the
// animation stack has nothing to contribute to them.

using hexa::posture::PoseLimits;
using hexa::posture::PoseSmoother;
using hexa::posture::PoseSmootherConfig;

constexpr PoseLimits kEnv{0.05f, 0.05f, 0.09f, -0.03f, 0.30f, 0.30f, 0.50f};

// Hold `target` for `ticks` and return every intermediate pose.
std::vector<BodyPose> run_to(PoseSmoother& s, const BodyPose& target,
                             int ticks) {
  std::vector<BodyPose> out;
  out.reserve(static_cast<size_t>(ticks));
  for (int i = 0; i < ticks; ++i) {
    out.push_back(s.step(hexa::posture::clamp(target, kEnv), kEnv, kDt));
  }
  return out;
}

BodyPose xy_pose(float x, float y) {
  return BodyPose{x, y, 0.0f, 0.0f, 0.0f, 0.0f};
}

// The headline case. Per-axis easing is isotropic, so its path between two
// commands is a straight line: swinging a held reach 90 degrees traces the
// chord, and the magnitude dips to sqrt((1-p)^2 + p^2) of the reach — 0.707 at
// the crossing. That 29% collapse IS the jerk this change removes.
TEST(PosturePolarFilter, DirectionSweepHoldsItsReach) {
  PoseSmoother s{PoseSmootherConfig{}};
  constexpr float kR = 0.04f;
  run_to(s, xy_pose(kR, 0.0f), 2000);
  ASSERT_NEAR(s.value().x, kR, 1e-5f);

  for (const BodyPose& p : run_to(s, xy_pose(0.0f, kR), 2000)) {
    EXPECT_NEAR(std::hypot(p.x, p.y), kR, 0.05f * kR);
    // Never behind the heading it started from — it sweeps the short way and
    // rings about the destination, it does not set off backwards. (It does
    // overshoot: zeta < 1 applies to the heading spring as much as any other.)
    EXPECT_GE(std::atan2(p.y, p.x), -1e-6f);
  }
  EXPECT_NEAR(s.value().y, kR, 1e-5f);
  EXPECT_NEAR(s.value().x, 0.0f, 1e-5f);
}

// Same for the tilt pair, so nobody wires only x-y.
TEST(PosturePolarFilter, TiltDirectionSweepHoldsItsReach) {
  PoseSmoother s{PoseSmootherConfig{}};
  constexpr float kR = 0.20f;
  const BodyPose roll_out{0.0f, 0.0f, 0.0f, kR, 0.0f, 0.0f};
  const BodyPose pitch_out{0.0f, 0.0f, 0.0f, 0.0f, kR, 0.0f};
  run_to(s, roll_out, 2000);
  ASSERT_NEAR(s.value().roll, kR, 1e-5f);

  for (const BodyPose& p : run_to(s, pitch_out, 2000)) {
    EXPECT_NEAR(std::hypot(p.roll, p.pitch), kR, 0.05f * kR);
  }
  EXPECT_NEAR(s.value().pitch, kR, 1e-5f);
}

// A direction step wider than a half-turn must go the short way, not unwind
// the long way round. At +175 -> -175 degrees the short way stays on the -x
// side throughout; the long way would carry the body through +x.
TEST(PosturePolarFilter, DirectionTakesTheShortWayAcrossTheWrap) {
  PoseSmoother s{PoseSmootherConfig{}};
  constexpr float kR = 0.04f;
  const float a0 = 175.0f * 3.14159265f / 180.0f;
  run_to(s, xy_pose(kR * std::cos(a0), kR * std::sin(a0)), 2000);
  ASSERT_LT(s.value().x, 0.0f);

  for (const BodyPose& p : run_to(s, xy_pose(kR * std::cos(-a0),
                                             kR * std::sin(-a0)), 2000)) {
    EXPECT_LT(p.x, 0.0f);
  }
  EXPECT_NEAR(s.value().y, kR * std::sin(-a0), 1e-5f);
}

// A target at the origin has no direction, so the pair retracts along its own
// line instead of spinning to centre. The line runs through the origin and out
// the far side — x and y stay equal on both sides of it.
TEST(PosturePolarFilter, WithdrawnPairRetractsAlongItsOwnLine) {
  PoseSmoother s{PoseSmootherConfig{}};
  run_to(s, xy_pose(0.03f, 0.03f), 2000);

  for (const BodyPose& p : run_to(s, xy_pose(0.0f, 0.0f), 2000)) {
    EXPECT_NEAR(p.x, p.y, 1e-6f);  // heading frozen at 45 degrees
  }
  EXPECT_NEAR(s.value().x, 0.0f, 1e-6f);
}

// The magnitude is not floored at the origin: a withdrawal eases through it
// and rings down, so the return keeps the settle a floor would have clipped
// off. Damping alone bounds the excursion — the depth past centre is the
// standard second-order overshoot of the reach withdrawn.
TEST(PosturePolarFilter, MagnitudeEasesThroughTheOriginInsteadOfStopping) {
  PoseSmootherConfig cfg;
  PoseSmoother s{cfg};
  constexpr float kR = 0.04f;
  run_to(s, xy_pose(kR, 0.0f), 2000);
  ASSERT_NEAR(s.value().x, kR, 1e-5f);

  float depth = 0.0f;
  for (const BodyPose& p : run_to(s, xy_pose(0.0f, 0.0f), 400)) {
    depth = std::min(depth, p.x);
    EXPECT_NEAR(p.y, 0.0f, 1e-6f);  // straight through, no spin
  }
  const float predicted = -kR * predicted_overshoot(cfg.damping_ratio);
  EXPECT_NEAR(depth, predicted, 0.03f * kR);

  // And it rings down onto centre rather than parking at the rebound.
  run_to(s, xy_pose(0.0f, 0.0f), 2000);
  ASSERT_NEAR(s.value().x, 0.0f, 1e-6f);

  const BodyPose next = s.step(xy_pose(kR, 0.0f), kEnv, kDt);
  EXPECT_GT(next.x, 0.0f);
}

// Re-commanding a pair that is mid-rebound must not make it travel. Caught on
// the far side of the origin the magnitude is carried negative, so the stored
// heading is a half-turn from where the body actually stands; a target on that
// far side is then the pair's own position and must read as zero error, not as
// a 180-degree sweep back through the origin and around.
TEST(PosturePolarFilter, RecommandingMidReboundDoesNotSweepTheLongWay) {
  PoseSmoother s{PoseSmootherConfig{}};
  constexpr float kR = 0.04f;
  run_to(s, xy_pose(kR, 0.0f), 2000);

  // ~pi/omega_d after the withdrawal: the deepest point of the rebound.
  run_to(s, xy_pose(0.0f, 0.0f), 90);
  ASSERT_LT(s.value().x, 0.0f) << "expected to be past centre by now";

  for (const BodyPose& p : run_to(s, xy_pose(-kR, 0.0f), 2000)) {
    EXPECT_NEAR(p.y, 0.0f, 1e-6f) << "swept around instead of easing in place";
    EXPECT_LT(p.x, 1e-6f) << "went back through the origin first";
  }
  EXPECT_NEAR(s.value().x, -kR, 1e-5f);
}

// The pair twin of SaturatedAxisDoesNotWindUp: parked far outside, the
// magnitude pins to the disc and still responds on the tick after withdrawal.
TEST(PosturePolarFilter, SaturatedMagnitudeDoesNotWindUp) {
  PoseSmoother s{PoseSmootherConfig{}};
  for (const BodyPose& p : run_to(s, xy_pose(10.0f, 10.0f), 2000)) {
    EXPECT_LE(std::hypot(p.x, p.y), kEnv.xy_radius() + 1e-6f);
  }
  ASSERT_NEAR(std::hypot(s.value().x, s.value().y), kEnv.xy_radius(), 1e-6f);

  const float before = s.value().x;
  const BodyPose next = s.step(xy_pose(0.0f, 0.0f), kEnv, kDt);
  EXPECT_LT(next.x, before);
}

// A pair leaving the origin has no direction of its own to unwind from: it
// must go straight out along the commanded heading, not spiral off the seeded
// zero angle.
TEST(PosturePolarFilter, PairLeavesTheOriginAlongTheCommandedHeading) {
  PoseSmoother s{PoseSmootherConfig{}};
  for (const BodyPose& p : run_to(s, xy_pose(0.0f, 0.04f), 2000)) {
    EXPECT_NEAR(p.x, 0.0f, 1e-6f);
  }
  EXPECT_NEAR(s.value().y, 0.04f, 1e-5f);
}

// Single-axis motion must be numerically what it always was — the heading
// snaps on the first tick and never moves again, leaving the magnitude spring
// exactly the old scalar spring. This is what keeps the shipped feel of a
// plain forward lean identical.
TEST(PosturePolarFilter, SingleAxisMoveMatchesTheScalarKernel) {
  PoseSmoother s{PoseSmootherConfig{}};
  // Symmetric z bounds, so the z axis cannot clamp before x does.
  const PoseLimits env{0.05f, 0.05f, 0.05f, -0.05f, 0.30f, 0.30f, 0.50f};
  const BodyPose target{0.03f, 0.0f, 0.03f, 0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 1000; ++i) {
    const BodyPose p = s.step(target, env, kDt);
    ASSERT_NEAR(p.x, p.z, 1e-6f);
  }
}

// The one tau governs the heading spring as well as the magnitude one — the
// direction is a separate integrator, not a free ride on the radial move, so a
// change to tau has to show up in a pure sweep at constant reach.
TEST(PosturePolarFilter, SharedTauSlowsTheDirectionSweep) {
  constexpr float kR = 0.04f;
  auto ticks_to_quarter_turn = [&](float tau) {
    PoseSmootherConfig cfg;
    cfg.tau = tau;
    PoseSmoother s{cfg};
    run_to(s, xy_pose(kR, 0.0f), 2000);
    int n = 0;
    while (n < 4000 && std::atan2(s.value().y, s.value().x) < 0.25f * 3.14159f) {
      s.step(xy_pose(0.0f, kR), kEnv, kDt);
      ++n;
    }
    return n;
  };
  EXPECT_GT(ticks_to_quarter_turn(0.27f), 1.5 * ticks_to_quarter_turn(0.135f));
}

// reset() has to seed the polar state, not just the Cartesian pose, or the
// first tick springs the heading away from a stale zero.
TEST(PosturePolarFilter, ResetSeedsThePolarStateFromThePose) {
  PoseSmoother s{PoseSmootherConfig{}};
  const BodyPose seed = xy_pose(0.0f, 0.04f);
  s.reset(seed);

  const BodyPose held = s.step(seed, kEnv, kDt);
  EXPECT_NEAR(held.x, 0.0f, 1e-6f);
  EXPECT_NEAR(held.y, 0.04f, 1e-6f);

  // A quarter turn away from the seeded heading holds its reach, which it
  // could not do from a heading that had been reset to zero.
  for (const BodyPose& p : run_to(s, xy_pose(0.04f, 0.0f), 2000)) {
    EXPECT_NEAR(std::hypot(p.x, p.y), 0.04f, 0.05f * 0.04f);
  }
}

// ── settle deadband ───────────────────────────────────────────────────────

// A spring's tail is asymptotic: without a deadband a released stick leaves the
// body creeping through fractions of a servo count long after it has visibly
// arrived. Every axis group has to land on exactly zero — the two eased pairs
// and the two lone axes alike.
TEST(PostureSettleSnap, WithdrawnPoseArrivesExactlyAtZero) {
  PoseSmoother s{PoseSmootherConfig{}};
  const BodyPose reach{0.03f, 0.02f, 0.02f, 0.10f, 0.05f, 0.20f};
  run_to(s, reach, 2000);
  ASSERT_NEAR(s.value().x, reach.x, 1e-5f);

  run_to(s, hexa::posture::IDENTITY, 2000);
  const BodyPose p = s.value();
  EXPECT_EQ(p.x, 0.0f);
  EXPECT_EQ(p.y, 0.0f);
  EXPECT_EQ(p.z, 0.0f);
  EXPECT_EQ(p.roll, 0.0f);
  EXPECT_EQ(p.pitch, 0.0f);
  EXPECT_EQ(p.yaw, 0.0f);

  // Snapped, not frozen: the next command still moves it.
  EXPECT_GT(s.step(reach, kEnv, kDt).x, 0.0f);
}

// The deadband must not become the floor the magnitude deliberately does not
// have. Even a fat one — 2 mm, an eighth of the rebound — leaves the crossing
// alone, because a pair travelling through the origin at 0.3 m/s is plainly not
// settling. Only the arrival at the end of the ring-down qualifies.
TEST(PostureSettleSnap, DeadbandDoesNotClipTheRebound) {
  PoseSmootherConfig cfg;
  cfg.snap_tol_linear = 0.002f;
  PoseSmoother s{cfg};
  constexpr float kR = 0.04f;
  run_to(s, xy_pose(kR, 0.0f), 2000);

  float depth = 0.0f;
  for (const BodyPose& p : run_to(s, xy_pose(0.0f, 0.0f), 400)) {
    depth = std::min(depth, p.x);
  }
  EXPECT_NEAR(depth, -kR * predicted_overshoot(cfg.damping_ratio), 0.03f * kR);

  run_to(s, xy_pose(0.0f, 0.0f), 2000);
  EXPECT_EQ(s.value().x, 0.0f);
}

// A command inside the band is one below what a servo can express, so it snaps
// too — the test is on the command as well as the position, and an axis parked
// just off zero would otherwise never settle.
TEST(PostureSettleSnap, SubToleranceCommandReadsAsZero) {
  PoseSmoother s{PoseSmootherConfig{}};
  run_to(s, BodyPose{2.0e-5f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0e-5f}, 2000);
  EXPECT_EQ(s.value().x, 0.0f);
  EXPECT_EQ(s.value().yaw, 0.0f);
}

// Zero disables it, so the tail can be had back from YAML without touching code.
TEST(PostureSettleSnap, ZeroToleranceLeavesTheTailAlone) {
  PoseSmootherConfig cfg;
  cfg.snap_tol_linear = 0.0f;
  cfg.snap_tol_angular = 0.0f;
  PoseSmoother s{cfg};
  run_to(s, xy_pose(0.03f, 0.0f), 2000);
  run_to(s, xy_pose(0.0f, 0.0f), 600);
  EXPECT_NE(s.value().x, 0.0f);
  EXPECT_LT(std::fabs(s.value().x), 1.0e-4f);
}

}  // namespace

// ── Quadruped support shift ──

namespace {

// A convex quadrilateral of four corner feet plus a parked middle pair, with
// each corner's phase supplied so the anticipation weights can be exercised.
std::map<std::string, LegOutput> quad_legs(
    const std::map<std::string, float>& phases,
    const std::map<std::string, bool>& swing = {}) {
  const std::map<std::string, std::pair<float, float>> xy = {
      {"l_front", {0.16f, 0.17f}},
      {"r_front", {0.16f, -0.17f}},
      {"l_rear", {-0.16f, 0.17f}},
      {"r_rear", {-0.16f, -0.17f}},
  };
  std::map<std::string, LegOutput> legs;
  for (const auto& name : hexa::gait::LEG_NAMES) {
    LegOutput leg;
    if (hexa::gait::leg_is_parked(hexa::gait::LegSet::QUADRUPED, name)) {
      leg.parked = true;
      leg.stance = false;
      // Where the parked pose actually puts it: high, and outboard.
      leg.foot_target = hexa::Vec3{0.0f, name == "l_middle" ? 0.17f : -0.17f,
                                   0.19f};
      legs[name] = leg;
      continue;
    }
    const auto it = swing.find(name);
    leg.stance = (it == swing.end() || !it->second);
    leg.phase = phases.count(name) ? phases.at(name) : 0.5f;
    leg.foot_target =
        hexa::Vec3{xy.at(name).first, xy.at(name).second,
                   leg.stance ? -0.08f : -0.04f};
    legs[name] = leg;
  }
  return legs;
}

// Signed area test: is p strictly inside the convex hull of the stance feet,
// taken in the fixed corner order that traces the quadrilateral.
bool inside_stance_polygon(const std::map<std::string, LegOutput>& legs,
                           float px, float py, float margin) {
  std::vector<std::pair<float, float>> hull;
  for (const char* name : {"l_front", "r_front", "r_rear", "l_rear"}) {
    const auto& leg = legs.at(name);
    if (leg.parked || !leg.stance) {
      continue;
    }
    hull.push_back({leg.foot_target.x, leg.foot_target.y});
  }
  if (hull.size() < 3) {
    return false;
  }
  // The corner order above traces the quadrilateral clockwise in the REP-103
  // body frame, so every interior cross product is negative. Taking the sign off
  // the first edge keeps the test honest if that order is ever reversed.
  float sign = 0.0f;
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];
    const float ex = b.first - a.first;
    const float ey = b.second - a.second;
    const float len = std::hypot(ex, ey);
    if (len <= 0.0f) {
      return false;
    }
    const float d = (ex * (py - a.second) - ey * (px - a.first)) / len;
    if (sign == 0.0f) {
      sign = d < 0.0f ? -1.0f : 1.0f;
    }
    if (sign * d < margin) {
      return false;
    }
  }
  return true;
}

}  // namespace

// The property the whole creep rests on: every weight is non-negative and at
// least one is positive, so the target is a convex combination of grounded feet
// and can never leave the polygon they enclose. Swept over which leg is
// airborne and where the remaining three are in their cycles.
TEST(SupportShift, AnticipatedSupportStaysInsideTheStancePolygon) {
  for (const char* airborne :
       {"l_front", "r_front", "l_rear", "r_rear", ""}) {
    for (int a = 0; a < 5; ++a) {
      for (int b = 0; b < 5; ++b) {
        std::map<std::string, float> phases;
        std::map<std::string, bool> swing;
        int i = 0;
        for (const char* name : {"l_front", "r_front", "l_rear", "r_rear"}) {
          phases[name] = static_cast<float>((a + i * b) % 5) / 5.0f;
          swing[name] = (std::string(name) == airborne);
          ++i;
        }
        const auto legs = quad_legs(phases, swing);
        const auto p = hexa::posture::anticipated_support_xy(legs, 0.05f);
        ASSERT_TRUE(p.has_value());
        EXPECT_TRUE(inside_stance_polygon(legs, p->first, p->second, 1e-4f))
            << "airborne='" << airborne << "' a=" << a << " b=" << b
            << " target (" << p->first << ", " << p->second << ")";
      }
    }
  }
}

// The reason the support shift is filtered in polar. Given a quarter turn to
// track, the per-axis lag collapses the magnitude on the way through (the two
// axes cross their midpoints together), which on this signal means the body
// cutting the corner INTO the polygon edge it is trying to stay away from. The
// polar lag holds the radius and sweeps the angle.
TEST(SupportShift, PolarLagArcsWhereThePerAxisLagCutsTheChord) {
  constexpr float kR = 0.04f;
  constexpr float kTau = 0.14f;
  constexpr float kDt = 0.005f;
  const std::pair<float, float> start{kR, 0.0f};
  const std::pair<float, float> target{0.0f, kR};

  std::optional<std::pair<float, float>> cartesian = start;
  std::optional<std::pair<float, float>> polar = start;
  float worst_cartesian = kR;
  float worst_polar = kR;
  for (int i = 0; i < 400; ++i) {
    cartesian = hexa::posture::lpf_step_xy(cartesian, target, kTau, kDt);
    polar = hexa::posture::lpf_step_polar_xy(polar, target, kTau, kDt);
    worst_cartesian =
        std::min(worst_cartesian, std::hypot(cartesian->first, cartesian->second));
    worst_polar = std::min(worst_polar, std::hypot(polar->first, polar->second));
  }
  // The chord of a quarter turn sits at 1/sqrt(2) of the radius.
  EXPECT_NEAR(worst_cartesian, kR * 0.7071f, 0.01f * kR);
  EXPECT_NEAR(worst_polar, kR, 1e-4f * kR);
  // Both still get there: the path differs, the destination does not.
  EXPECT_NEAR(polar->first, target.first, 1e-4f);
  EXPECT_NEAR(polar->second, target.second, 1e-4f);
}

// Round the back of the circle the short way, not the long one: the raw target
// crosses +/-pi whenever the creep's handover passes straight behind the robot.
TEST(SupportShift, PolarLagTakesTheShortWayAcrossTheWrap) {
  constexpr float kR = 0.04f;
  const float a0 = 3.0f;  // just short of +pi
  std::optional<std::pair<float, float>> v =
      std::make_pair(kR * std::cos(a0), kR * std::sin(a0));
  const std::pair<float, float> target{kR * std::cos(-a0), kR * std::sin(-a0)};
  for (int i = 0; i < 400; ++i) {
    v = hexa::posture::lpf_step_polar_xy(v, target, 0.14f, 0.005f);
    // The short way keeps y's sign or crosses through the far side; it never
    // sweeps back down through y = 0 on the +x half, which is the long way.
    EXPECT_LE(v->first, kR * std::cos(a0) + 1e-6f);
  }
  EXPECT_NEAR(v->first, target.first, 1e-4f);
  EXPECT_NEAR(v->second, target.second, 1e-4f);
}

// A zero lead makes every weight 1, which is the plain centroid — the degenerate
// case the knob has to reach so it can be turned off.
TEST(SupportShift, ZeroLeadReproducesTheStanceCentroid) {
  const auto legs = quad_legs({{"l_front", 0.1f},
                               {"r_front", 0.35f},
                               {"l_rear", 0.6f},
                               {"r_rear", 0.85f}});
  const auto anticipated = hexa::posture::anticipated_support_xy(legs, 0.0f);
  const auto centroid = hexa::posture::stance_centroid_xy(legs);
  ASSERT_TRUE(anticipated.has_value());
  ASSERT_TRUE(centroid.has_value());
  EXPECT_NEAR(anticipated->first, centroid->first, 1e-6f);
  EXPECT_NEAR(anticipated->second, centroid->second, 1e-6f);
}

// The dangerous one: a parked foot sits ~0.19 m above the stance plane. Read as
// a swing leg it would peg max_swing every tick and heave the body with it.
TEST(SupportShift, MaxSwingLiftIgnoresParkedLegs) {
  const auto legs = quad_legs({{"l_front", 0.5f}});
  const auto lift = hexa::posture::max_swing_lift_z(legs);
  ASSERT_TRUE(lift.has_value());
  EXPECT_NEAR(*lift, 0.0f, 1e-6f) << "a parked foot counted as a swing";
}

// A parked foot carries no weight, so it is not a vertex of the support polygon.
TEST(SupportShift, StanceCentroidIgnoresParkedLegs) {
  const auto legs = quad_legs({});
  const auto c = hexa::posture::stance_centroid_xy(legs);
  ASSERT_TRUE(c.has_value());
  // The four corners are symmetric about the origin; the parked pair is not on
  // the same plane, but is symmetric in y, so only x would betray it.
  EXPECT_NEAR(c->first, 0.0f, 1e-6f);
  EXPECT_NEAR(c->second, 0.0f, 1e-6f);
}

// The quadruped stack is exempt from gait_body_animations_enabled: what holds
// the robot up is not an embellishment the operator may decline.
TEST(SupportShift, QuadrupedStackRunsWithGaitAnimationsDisabled) {
  auto cfg = hexa::config::kPosture;
  cfg.gait_body_animations_enabled = false;
  PostureController posture(cfg);

  // l_front is inside the 0.05-cycle lead, so its weight is down to 0.2 and the
  // target has already moved off centre — which a running stack will report and
  // a suppressed one will not.
  const auto legs = quad_legs({{"l_front", 0.99f},
                               {"r_front", 0.2f},
                               {"l_rear", 0.4f},
                               {"r_rear", 0.6f}});
  BodyPose quad;
  BodyPose hexa;
  float t = 0.0f;
  for (int i = 0; i < 400; ++i) {
    quad = posture.update(legs, 0.25f, /*walking=*/true, EngineState::GAIT,
                          "quad_walk", hexa::gait::LegSet::QUADRUPED, kDt,
                          t);
    t += kDt;
  }
  PostureController hexa_posture(cfg);
  t = 0.0f;
  for (int i = 0; i < 400; ++i) {
    hexa = hexa_posture.update(legs, 0.25f, /*walking=*/true,
                               EngineState::GAIT, "quad_walk",
                               hexa::gait::LegSet::HEXAPOD, kDt, t);
    t += kDt;
  }
  EXPECT_GT(std::hypot(quad.x, quad.y), 1e-3f)
      << "the support shift was suppressed by the master switch";
  EXPECT_NEAR(std::hypot(hexa.x, hexa.y), 0.0f, 1e-6f)
      << "the default stack must still be suppressed";
}

// Posture mode is live on four feet: the user pose reaches the output as it
// does on six. The support shift is not entitled to the whole envelope — what
// keeps its travel intact is upstream, the teleop refusing a posture RECORD
// while quadruped, so no offset rides into the creep.
TEST(SupportShift, QuadrupedAppliesTheUserPose) {
  PostureController posture(hexa::config::kPosture);
  const BodyPose user{0.02f, -0.015f, 0.01f, 0.1f, -0.05f, 0.3f};
  posture.set_user_pose(user);

  // Equal phases put every anticipation weight at 1, so the support shift
  // target is the symmetric centroid and what comes out is the user pose alone.
  const auto legs = quad_legs({{"l_front", 0.5f},
                               {"r_front", 0.5f},
                               {"l_rear", 0.5f},
                               {"r_rear", 0.5f}});
  BodyPose out;
  float t = 0.0f;
  for (int i = 0; i < 1000; ++i) {
    out = posture.update(legs, 0.25f, /*walking=*/false, EngineState::GAIT,
                         "quad_walk", hexa::gait::LegSet::QUADRUPED, kDt,
                         t);
    t += kDt;
  }
  EXPECT_NEAR(out.x, user.x, 1e-3f);
  EXPECT_NEAR(out.y, user.y, 1e-3f);
  EXPECT_NEAR(out.z, user.z, 1e-3f);
  EXPECT_NEAR(out.roll, user.roll, 1e-3f);
  EXPECT_NEAR(out.pitch, user.pitch, 1e-3f);
  EXPECT_NEAR(out.yaw, user.yaw, 1e-3f);
}
