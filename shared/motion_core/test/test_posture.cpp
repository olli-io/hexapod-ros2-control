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

// tau <= 0 is the off switch, in place of a separate enable flag. Checked on a
// pair as well as on z: the pair bypass is a separate branch in step_polar.
TEST(PosturePoseFilter, ZeroTauBypassesTheFilter) {
  hexa::config::PostureConfig p = with_height_envelope();
  p.pose_filter_tau_translation = 0.0f;
  p.pose_filter_tau_rotation = 0.0f;
  PostureController posture{p};
  posture.set_user_pose(BodyPose{0.03f, 0.0f, kStepZ, 0.2f, 0.0f, 0.0f});

  const BodyPose first = posture.update(tripod_legs(), 0.0f, /*walking=*/false,
                                        EngineState::STAND, "tripod", kDt, 0.0f);
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
// line instead of spinning to centre.
TEST(PosturePolarFilter, WithdrawnPairRetractsAlongItsOwnLine) {
  PoseSmoother s{PoseSmootherConfig{}};
  run_to(s, xy_pose(0.03f, 0.03f), 2000);

  for (const BodyPose& p : run_to(s, xy_pose(0.0f, 0.0f), 2000)) {
    EXPECT_NEAR(p.x, p.y, 1e-6f);  // heading frozen at 45 degrees
    EXPECT_GE(p.x, -1e-6f);        // and never past the origin
  }
  EXPECT_NEAR(s.value().x, 0.0f, 1e-6f);
}

// The magnitude floor absorbs the zeta = 0.32 undershoot on a return to
// centre. A deliberate change: the old per-axis filter overshot THROUGH the
// origin and came out the far side. The floor must not wind up, though — the
// pose has to move again on the very next tick.
TEST(PosturePolarFilter, MagnitudeStopsAtTheOriginInsteadOfSwingingThrough) {
  PoseSmoother s{PoseSmootherConfig{}};
  run_to(s, xy_pose(0.04f, 0.0f), 2000);

  for (const BodyPose& p : run_to(s, xy_pose(0.0f, 0.0f), 400)) {
    EXPECT_GE(p.x, -1e-6f);
  }
  ASSERT_NEAR(s.value().x, 0.0f, 1e-6f);

  const BodyPose next = s.step(xy_pose(0.04f, 0.0f), kEnv, kDt);
  EXPECT_GT(next.x, 0.0f);
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

// The direction tau is its own knob because tangential travel is magnitude
// times heading rate: at equal taus a half-turn at full reach covers ~pi times
// the path of a full radial move in the same time. Raising it must slow the
// sweep and nothing else.
TEST(PosturePolarFilter, DirectionTauSlowsTheSweepOnItsOwn) {
  constexpr float kR = 0.04f;
  auto ticks_to_quarter_turn = [&](float tau_angle) {
    PoseSmootherConfig cfg;
    cfg.tau_xy_angle = tau_angle;
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

}  // namespace
