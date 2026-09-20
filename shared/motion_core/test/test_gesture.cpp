// Gestures: keyframe sampling (the two segment shapes and their joins), the
// player (live start / home knots and the blend weights next to them, hold
// resolution, per-leg tracks) and the engine's GESTURE state with its guards.

#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gesture/keyframe.hpp"
#include "gesture/player.hpp"
#include "gesture/validate.hpp"
#include "kinematics/body_transform.hpp"
#include "kinematics/leg_ik.hpp"

namespace g = hexa::gait;
namespace gs = hexa::gesture;
using gs::BodyKeyframe;
using gs::GestureTransition;
using gs::LegKeyframe;

namespace {

constexpr float kDt = 0.02f;
constexpr float kTol = 1e-4f;

// Drive the engine cold-start ladder to STAND (cmd_vel held at zero).
void run_to_stand(g::Engine& e) {
  ASSERT_TRUE(e.start_initialize());
  for (int i = 0; i < 200 && e.state() != g::EngineState::STAND; ++i) {
    e.update(kDt, {0.0f, 0.0f}, 0.0f);
  }
  ASSERT_EQ(e.state(), g::EngineState::STAND);
}

// Stand, then run the engine at zero command until it is standing again.
// Returns the states seen, deduplicated.
std::vector<g::EngineState> run_until_stand(g::Engine& e, int max_ticks = 1000) {
  std::vector<g::EngineState> seen;
  for (int i = 0; i < max_ticks; ++i) {
    e.update(kDt, {0.0f, 0.0f}, 0.0f);
    if (seen.empty() || seen.back() != e.state()) {
      seen.push_back(e.state());
    }
    if (e.state() == g::EngineState::STAND) {
      break;
    }
  }
  return seen;
}

LegKeyframe key(float t, float value, GestureTransition tr) {
  LegKeyframe k{};
  k.t = t;
  k.coxa = value;
  k.femur = value;
  k.tibia = value;
  k.transition = tr;
  return k;
}

float angle_of(const LegKeyframe& k) { return k.coxa; }

float sample(const std::vector<LegKeyframe>& keys, float t) {
  return gs::track_value(keys.data(), keys.size(), t, angle_of);
}

float slope(const std::vector<LegKeyframe>& keys, float t, float h = 1e-3f) {
  return (sample(keys, t + h) - sample(keys, t - h)) / (2.0f * h);
}

float one_sided_slope(const std::vector<LegKeyframe>& keys, float t, float dir,
                      float h = 1e-3f) {
  return (sample(keys, t + dir * h) - sample(keys, t)) / (dir * h);
}

LegKeyframe hold_key(float t, GestureTransition tr = GestureTransition::EASE) {
  LegKeyframe k{};
  k.t = t;
  k.transition = tr;
  k.hold = true;
  return k;
}

LegKeyframe home_key(float t, GestureTransition tr = GestureTransition::EASE) {
  LegKeyframe k{};
  k.t = t;
  k.transition = tr;
  k.home = true;
  return k;
}

LegKeyframe start_key(float t) {
  LegKeyframe k{};
  k.t = t;
  k.transition = GestureTransition::EASE;
  k.start = true;
  return k;
}

// The l_front foot the pipeline would solve a live knot to with no body pose
// on: nominal, as the player reports it.
hexa::Vec3 foot_of(const g::LegOutput& out, const std::string& leg) {
  const auto specs = g::leg_specs_from_config();
  return hexa::leg_to_body(hexa::forward_kinematics(out.joints, specs.at(leg)),
                           specs.at(leg));
}

BodyKeyframe body_home_key(float t) {
  BodyKeyframe k{};
  k.t = t;
  k.transition = GestureTransition::EASE;
  k.home = true;
  return k;
}

// A gesture spec moving one leg through `keys`, then home `return_time` after
// the last of them.
gs::GestureSpec one_leg_spec(hexa::Leg leg, std::vector<LegKeyframe> keys,
                             float return_time = 0.5f) {
  gs::GestureSpec spec;
  spec.id = "test";
  keys.push_back(home_key(keys.back().t + return_time));
  gs::LegTrack track;
  track.leg = leg;
  track.keys = std::move(keys);
  spec.legs.push_back(track);
  return spec;
}

LegKeyframe joint_key(float t, const hexa::JointAngles& a,
                      GestureTransition tr = GestureTransition::EASE) {
  LegKeyframe k{};
  k.t = t;
  k.coxa = a[0];
  k.femur = a[1];
  k.tibia = a[2];
  k.transition = tr;
  return k;
}

// Joint angles that put `leg`'s foot at its default-preset stance, swivelled
// by dangle about the coxa axis, moved dreach radially and lifted by height.
// Tests think in foot geometry; the tracks are written in joints.
hexa::JointAngles joints_at(const std::string& leg, float dangle, float dreach,
                            float height) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const hexa::Vec3 in_leg = hexa::body_to_leg(nominal.at(leg), specs.at(leg));
  const float angle = std::atan2(in_leg.y, in_leg.x) + dangle;
  const float reach = std::hypot(in_leg.x, in_leg.y) + dreach;
  return hexa::inverse_kinematics(
      hexa::Vec3(reach * std::cos(angle), reach * std::sin(angle),
                 in_leg.z + height),
      specs.at(leg));
}

hexa::JointAngles joints_of(const g::LegOutput& out) { return out.joints; }

bool near(const hexa::Vec3& a, const hexa::Vec3& b, float tol = kTol) {
  return (a - b).norm() <= tol;
}

}  // namespace

// ── Keyframe sampling ──

TEST(Keyframe, EaseSegmentHitsEndpointsWithZeroEndVelocity) {
  const std::vector<LegKeyframe> keys = {
      key(0.0f, 0.0f, GestureTransition::EASE),
      key(1.0f, 1.0f, GestureTransition::EASE)};
  EXPECT_NEAR(sample(keys, 0.0f), 0.0f, kTol);
  EXPECT_NEAR(sample(keys, 1.0f), 1.0f, kTol);
  EXPECT_NEAR(sample(keys, 0.5f), 0.5f, kTol);
  EXPECT_NEAR(one_sided_slope(keys, 0.0f, 1.0f), 0.0f, 1e-2f);
  EXPECT_NEAR(one_sided_slope(keys, 1.0f, -1.0f), 0.0f, 1e-2f);
  // Held outside the table.
  EXPECT_NEAR(sample(keys, -1.0f), 0.0f, kTol);
  EXPECT_NEAR(sample(keys, 2.0f), 1.0f, kTol);
}

TEST(Keyframe, ContinuousRunPassesThroughEveryKnot) {
  const std::vector<LegKeyframe> keys = {
      key(0.0f, 0.0f, GestureTransition::EASE),
      key(1.0f, 1.0f, GestureTransition::CONTINUOUS),
      key(2.0f, 3.0f, GestureTransition::CONTINUOUS),
      key(3.0f, 2.0f, GestureTransition::CONTINUOUS),
      key(4.0f, 0.0f, GestureTransition::EASE)};
  for (const auto& k : keys) {
    EXPECT_NEAR(sample(keys, k.t), k.coxa, kTol) << "t=" << k.t;
  }
}

TEST(Keyframe, ContinuousRunIsC1AtInteriorKnots) {
  const std::vector<LegKeyframe> keys = {
      key(0.0f, 0.0f, GestureTransition::EASE),
      key(1.0f, 1.0f, GestureTransition::CONTINUOUS),
      key(2.0f, 3.0f, GestureTransition::CONTINUOUS),
      key(3.0f, 6.0f, GestureTransition::CONTINUOUS),
      key(4.0f, 7.0f, GestureTransition::CONTINUOUS)};
  for (float t : {1.0f, 2.0f, 3.0f}) {
    const float left = one_sided_slope(keys, t, -1.0f);
    const float right = one_sided_slope(keys, t, 1.0f);
    EXPECT_NEAR(left, right, 2e-2f) << "t=" << t;
  }
  // An interior knot with agreeing secants carries a real slope.
  EXPECT_GT(slope(keys, 2.0f), 1.0f);
}

TEST(Keyframe, ContinuousRunHasZeroTangentAtRunBorders) {
  const std::vector<LegKeyframe> keys = {
      key(0.0f, 0.0f, GestureTransition::EASE),
      key(1.0f, 1.0f, GestureTransition::EASE),
      key(2.0f, 3.0f, GestureTransition::CONTINUOUS),
      key(3.0f, 6.0f, GestureTransition::CONTINUOUS),
      key(4.0f, 7.0f, GestureTransition::EASE)};
  // The run starts at t=1 (arrived at by an ease segment) and ends at t=3
  // (left by one): both borders have a zero slope.
  EXPECT_NEAR(one_sided_slope(keys, 1.0f, 1.0f), 0.0f, 2e-2f);
  EXPECT_NEAR(one_sided_slope(keys, 3.0f, -1.0f), 0.0f, 2e-2f);
  EXPECT_NEAR(one_sided_slope(keys, 3.0f, 1.0f), 0.0f, 2e-2f);
}

TEST(Keyframe, UnevenKnotSpacingUsesTimeParameterisedTangents) {
  // Linear data on uneven knots: time-parameterised tangents reproduce the line
  // exactly on the interior segment; index-parameterised ones would bulge it.
  const std::vector<LegKeyframe> keys = {
      key(0.0f, 0.0f, GestureTransition::CONTINUOUS),
      key(1.0f, 1.0f, GestureTransition::CONTINUOUS),
      key(3.0f, 3.0f, GestureTransition::CONTINUOUS),
      key(4.0f, 4.0f, GestureTransition::CONTINUOUS)};
  EXPECT_NEAR(sample(keys, 2.0f), 2.0f, kTol);
  EXPECT_NEAR(sample(keys, 1.5f), 1.5f, kTol);
  EXPECT_NEAR(sample(keys, 2.5f), 2.5f, kTol);
}

TEST(Keyframe, EqualConsecutiveKnotsHoldInsideAContinuousRun) {
  const std::vector<LegKeyframe> keys = {
      key(0.0f, 0.0f, GestureTransition::EASE),
      key(1.0f, 1.0f, GestureTransition::CONTINUOUS),
      key(2.0f, 1.0f, GestureTransition::CONTINUOUS),
      key(3.0f, 2.0f, GestureTransition::CONTINUOUS)};
  for (float t = 1.0f; t <= 2.0f; t += 0.05f) {
    EXPECT_NEAR(sample(keys, t), 1.0f, kTol) << "t=" << t;
  }
}

TEST(Keyframe, TurningPointDoesNotOvershoot) {
  const std::vector<LegKeyframe> keys = {
      key(0.0f, 0.0f, GestureTransition::EASE),
      key(1.0f, 2.0f, GestureTransition::CONTINUOUS),
      key(2.0f, 0.0f, GestureTransition::CONTINUOUS),
      key(3.0f, 2.0f, GestureTransition::CONTINUOUS)};
  for (float t = 0.0f; t <= 3.0f; t += 0.01f) {
    const float v = sample(keys, t);
    EXPECT_LE(v, 2.0f + kTol) << "t=" << t;
    EXPECT_GE(v, -kTol) << "t=" << t;
  }
}

TEST(Keyframe, MonotoneUnevenKnotsStayInsideTheirRange) {
  // A steep step after a shallow one: an uncapped Catmull-Rom tangent at the
  // middle knot would pull the shallow segment below its start.
  const std::vector<LegKeyframe> keys = {
      key(0.0f, 0.0f, GestureTransition::CONTINUOUS),
      key(1.0f, 1.0f, GestureTransition::CONTINUOUS),
      key(2.0f, 100.0f, GestureTransition::CONTINUOUS),
      key(3.0f, 101.0f, GestureTransition::CONTINUOUS)};
  for (std::size_t i = 1; i < keys.size(); ++i) {
    for (float t = keys[i - 1].t; t <= keys[i].t; t += 0.01f) {
      const float v = sample(keys, t);
      EXPECT_GE(v, keys[i - 1].coxa - kTol) << "t=" << t;
      EXPECT_LE(v, keys[i].coxa + kTol) << "t=" << t;
    }
  }
}

// ── Player ──

TEST(Player, TrackedLegIsDirectAndUntrackedIsNot) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const auto k1 = joints_at("l_front", 0.0f, 0.0f, 0.05f);
  const auto k2 = joints_at("l_front", 0.2f, 0.0f, 0.05f);
  const auto spec = one_leg_spec(hexa::Leg::L_FRONT,
                                 {joint_key(0.5f, k1), joint_key(1.0f, k2)});
  gs::GesturePlayer player(spec, nominal, specs);
  while (!player.done()) {
    const auto out = player.update(kDt);
    const float t = player.t();
    const auto& l = out.at("l_front");
    if (t > 0.5f && t < 1.0f) {
      // Between two fixed knots: fully direct, foot_target the joints' FK.
      EXPECT_TRUE(l.direct) << "t=" << t;
      EXPECT_EQ(l.direct_weight, 1.0f) << "t=" << t;
      EXPECT_TRUE(near(l.foot_target, foot_of(l, "l_front"), 1e-6f)) << "t=" << t;
    } else if (t < 0.5f) {
      // Easing out of the live start: the first knot's angles at the ease
      // weight, anchored on the stance.
      EXPECT_TRUE(l.direct) << "t=" << t;
      EXPECT_GT(l.direct_weight, 0.0f) << "t=" << t;
      EXPECT_LT(l.direct_weight, 1.0f) << "t=" << t;
      for (std::size_t j = 0; j < 3; ++j) EXPECT_FLOAT_EQ(l.joints[j], k1[j]);
      EXPECT_TRUE(near(l.foot_target, nominal.at("l_front"), 1e-7f)) << "t=" << t;
    }
    for (const auto& leg : g::LEG_NAMES) {
      if (leg != "l_front") {
        EXPECT_FALSE(out.at(leg).direct) << leg;
      }
    }
  }
}

TEST(Player, StartsLiveAndReturnsLive) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT,
      {joint_key(0.5f, joints_at("l_front", 0.3f, -0.02f, 0.05f))});
  gs::GesturePlayer player(spec, nominal, specs);

  // Just off the start the weight is as good as zero: the leg is the stance.
  auto out = player.update(1e-5f);
  EXPECT_LT(out.at("l_front").direct_weight * float(out.at("l_front").direct), 1e-6f);
  EXPECT_TRUE(near(out.at("l_front").foot_target, nominal.at("l_front"), 1e-7f));
  EXPECT_TRUE(out.at("l_front").stance);
  EXPECT_FALSE(player.done());

  for (int i = 0; i < 200 && !player.done(); ++i) {
    out = player.update(kDt);
  }
  EXPECT_TRUE(player.done());
  EXPECT_NEAR(player.duration(), 1.0f, 1e-6f);
  EXPECT_FALSE(out.at("l_front").direct);
  EXPECT_TRUE(near(out.at("l_front").foot_target, nominal.at("l_front"), 1e-7f));
  EXPECT_TRUE(out.at("l_front").stance);
  EXPECT_NEAR(out.at("l_front").phase, 1.0f, 1e-6f);
}

TEST(Player, BlendWeightsEaseOutOfStartAndBackIntoHome) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const auto k = joints_at("l_front", 0.0f, 0.0f, 0.05f);
  const auto spec = one_leg_spec(hexa::Leg::L_FRONT, {joint_key(0.5f, k)}, 0.5f);
  gs::GesturePlayer player(spec, nominal, specs);
  for (int i = 0; i < 12; ++i) player.update(kDt);  // t = 0.24
  auto l = player.update(0.01f).at("l_front");        // t = 0.25, u = 0.5
  EXPECT_TRUE(l.direct);
  EXPECT_NEAR(l.direct_weight, gs::ease5(0.5f), 1e-5f);
  for (std::size_t j = 0; j < 3; ++j) EXPECT_FLOAT_EQ(l.joints[j], k[j]);
  l = player.update(0.25f).at("l_front");  // t = 0.5, at the knot
  EXPECT_NEAR(l.direct_weight, 1.0f, 1e-5f);
  l = player.update(0.25f).at("l_front");  // t = 0.75, halfway home
  EXPECT_TRUE(l.direct);
  EXPECT_NEAR(l.direct_weight, 1.0f - gs::ease5(0.5f), 1e-5f);
  for (std::size_t j = 0; j < 3; ++j) EXPECT_FLOAT_EQ(l.joints[j], k[j]);
  EXPECT_TRUE(near(l.foot_target, nominal.at("l_front"), 1e-7f));
}

TEST(Player, UntrackedLegsStayPlanted) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT, {joint_key(0.5f, joints_at("l_front", 0.0f, 0.0f, 0.05f))});
  gs::GesturePlayer player(spec, nominal, specs);
  while (!player.done()) {
    const auto out = player.update(kDt);
    for (const auto& leg : g::LEG_NAMES) {
      if (leg == "l_front") continue;
      EXPECT_TRUE(near(out.at(leg).foot_target, nominal.at(leg), 1e-7f)) << leg;
      EXPECT_TRUE(out.at(leg).stance) << leg;
      EXPECT_EQ(out.at(leg).phase, 0.0f) << leg;
      EXPECT_FALSE(out.at(leg).parked) << leg;
    }
  }
}

TEST(Player, LiftedLegReportsSwingWhileOffTheGround) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT, {joint_key(0.5f, joints_at("l_front", 0.0f, 0.0f, 0.05f))});
  gs::GesturePlayer player(spec, nominal, specs);
  auto out = player.update(1e-5f);
  EXPECT_TRUE(out.at("l_front").stance) << "planted at the start";
  // Planted, then in the air, then planted again: exactly two flips, and the
  // knot itself is in the air.
  int flips = 0;
  bool last = true;
  while (!player.done()) {
    out = player.update(kDt);
    const bool stance = out.at("l_front").stance;
    flips += stance != last;
    last = stance;
    if (std::fabs(player.t() - 0.5f) < kDt / 2) {
      EXPECT_FALSE(stance) << "at the lifted knot";
    }
  }
  EXPECT_EQ(flips, 2);
  EXPECT_TRUE(out.at("l_front").stance) << "planted at the end";
}

TEST(Player, BodyTrackReturnsToIdentity) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  gs::GestureSpec spec;
  spec.id = "bow";
  BodyKeyframe k{};
  k.t = 0.5f;
  k.pitch = -0.2f;
  k.x = 0.02f;
  k.transition = GestureTransition::EASE;
  spec.body = {k, body_home_key(1.0f)};
  gs::GesturePlayer player(spec, nominal, specs);

  for (int i = 0; i < 25; ++i) player.update(kDt);  // t = 0.5
  EXPECT_NEAR(player.body().pitch, -0.2f, 1e-4f);
  EXPECT_NEAR(player.body().x, 0.02f, 1e-4f);
  while (!player.done()) player.update(kDt);
  EXPECT_NEAR(player.body().pitch, 0.0f, 1e-6f);
  EXPECT_NEAR(player.body().x, 0.0f, 1e-6f);
  // Every foot is planted on nominal throughout: a body track moves no leg.
  const auto out = player.update(kDt);
  for (const auto& leg : g::LEG_NAMES) {
    EXPECT_TRUE(near(out.at(leg).foot_target, nominal.at(leg), 1e-7f)) << leg;
  }
}

TEST(Player, LegAbsentFromAMiddleKeyframeInterpolatesAcrossIt) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const hexa::JointAngles a = joints_at("l_front", 0.0f, 0.0f, 0.02f);
  const hexa::JointAngles c = joints_at("l_front", 0.0f, 0.0f, 0.06f);
  const hexa::JointAngles b = joints_at("r_front", 0.0f, 0.0f, 0.03f);
  gs::GestureSpec spec;
  spec.id = "test";
  // l_front is named at A (0.5) and C (1.5); r_front alone at B (1.0). The
  // flattened tracks carry that directly, so l_front eases A -> C over B.
  gs::LegTrack left;
  left.leg = hexa::Leg::L_FRONT;
  left.keys = {joint_key(0.5f, a), joint_key(1.5f, c), home_key(2.0f)};
  gs::LegTrack right;
  right.leg = hexa::Leg::R_FRONT;
  right.keys = {joint_key(1.0f, b), home_key(1.5f)};
  spec.legs = {left, right};
  gs::GesturePlayer player(spec, nominal, specs);
  for (int i = 0; i < 50; ++i) player.update(kDt);  // t = 1.0
  const auto out = player.update(0.0f);
  for (std::size_t j = 0; j < 3; ++j) {
    EXPECT_NEAR(joints_of(out.at("l_front"))[j], 0.5f * (a[j] + c[j]), 1e-4f)
        << "joint " << j << " midway A -> C, ease5(0.5) = 0.5";
    EXPECT_NEAR(joints_of(out.at("r_front"))[j], b[j], 1e-4f) << "joint " << j;
  }
}

TEST(Player, TwoLegsInOneKeyframeMoveTogether) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  gs::GestureSpec spec;
  spec.id = "test";
  gs::LegTrack left;
  left.leg = hexa::Leg::L_FRONT;
  left.keys = {joint_key(0.8f, joints_at("l_front", 0.0f, 0.0f, 0.04f)),
               home_key(1.3f)};
  gs::LegTrack right;
  right.leg = hexa::Leg::R_FRONT;
  right.keys = {joint_key(0.8f, joints_at("r_front", 0.0f, 0.0f, 0.04f)),
                home_key(1.3f)};
  spec.legs = {left, right};
  gs::GesturePlayer player(spec, nominal, specs);
  while (!player.done()) {
    const auto out = player.update(kDt);
    const float lift_l = out.at("l_front").foot_target.z - nominal.at("l_front").z;
    const float lift_r = out.at("r_front").foot_target.z - nominal.at("r_front").z;
    EXPECT_NEAR(lift_l, lift_r, 1e-6f) << "t=" << player.t();
    EXPECT_EQ(out.at("l_front").phase, out.at("r_front").phase);
  }
}

TEST(Player, HoldKnotHoldsThePreviousValue) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT,
      {joint_key(0.5f, joints_at("l_front", 0.2f, 0.0f, 0.05f)), hold_key(1.0f)});
  gs::GesturePlayer player(spec, nominal, specs);
  for (int i = 0; i < 25; ++i) player.update(kDt);  // t = 0.5
  const hexa::Vec3 at_knot = player.update(0.0f).at("l_front").foot_target;
  for (int i = 0; i < 25; ++i) {
    const auto out = player.update(kDt);
    EXPECT_TRUE(near(out.at("l_front").foot_target, at_knot, 1e-5f))
        << "t=" << player.t();
  }
  EXPECT_NEAR(player.duration(), 1.5f, 1e-6f);
}

TEST(Player, StartKnotKeepsTheLegLiveUntilItsTime) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const auto k = joints_at("l_front", 0.0f, 0.0f, 0.05f);
  const auto spec = one_leg_spec(hexa::Leg::L_FRONT,
                                 {start_key(0.5f), joint_key(1.0f, k)});
  gs::GesturePlayer player(spec, nominal, specs);
  for (int i = 0; i < 24; ++i) {  // to t = 0.48, short of the start knot
    const auto out = player.update(kDt);
    EXPECT_FALSE(out.at("l_front").direct) << "t=" << player.t();
    EXPECT_TRUE(near(out.at("l_front").foot_target, nominal.at("l_front"), 1e-7f))
        << "t=" << player.t();
    EXPECT_TRUE(out.at("l_front").stance) << "t=" << player.t();
    EXPECT_GT(out.at("l_front").phase, 0.0f) << "still a tracked leg";
  }
  // The ease into the first knot runs from the start knot, not from t = 0.
  const auto l = player.update(0.75f - player.t()).at("l_front");  // u = 0.5
  EXPECT_TRUE(l.direct);
  EXPECT_NEAR(l.direct_weight, gs::ease5(0.5f), 1e-5f);
  for (std::size_t j = 0; j < 3; ++j) EXPECT_FLOAT_EQ(l.joints[j], k[j]);
}

TEST(Player, HoldAfterStartIsLive) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT,
      {hold_key(0.5f), joint_key(1.0f, joints_at("l_front", 0.0f, 0.0f, 0.05f))});
  gs::GesturePlayer player(spec, nominal, specs);
  for (int i = 0; i < 24; ++i) {  // to t = 0.48, short of the hold knot
    const auto out = player.update(kDt);
    EXPECT_FALSE(out.at("l_front").direct) << "t=" << player.t();
  }
  EXPECT_TRUE(player.update(0.27f).at("l_front").direct);
}

TEST(Player, BodyHoldKnotHoldsThePreviousPose) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  gs::GestureSpec spec;
  spec.id = "test";
  BodyKeyframe a{};
  a.t = 0.5f;
  a.roll = 0.1f;
  a.y = -0.01f;
  a.transition = GestureTransition::EASE;
  BodyKeyframe hold{};
  hold.t = 1.5f;
  hold.transition = GestureTransition::EASE;
  hold.hold = true;
  spec.body = {a, hold, body_home_key(2.0f)};
  gs::GesturePlayer player(spec, nominal, specs);
  for (int i = 0; i < 25; ++i) player.update(kDt);  // t = 0.5
  for (int i = 0; i < 50; ++i) {
    player.update(kDt);
    EXPECT_NEAR(player.body().roll, 0.1f, 1e-5f) << "t=" << player.t();
    EXPECT_NEAR(player.body().y, -0.01f, 1e-5f) << "t=" << player.t();
  }
  while (!player.done()) player.update(kDt);
  EXPECT_NEAR(player.body().roll, 0.0f, 1e-6f);
}

TEST(Player, BakedTableRoundTripsThroughTheSpecs) {
  const auto specs = gs::gesture_specs_from_config();
  ASSERT_EQ(specs.size(), hexa::config::kGestures.size());
  for (std::size_t i = 0; i < specs.size(); ++i) {
    EXPECT_EQ(specs[i].id, std::string(hexa::config::kGestures[i].id));
    EXPECT_EQ(specs[i].legs.size(), hexa::config::kGestures[i].leg_track_count);
    EXPECT_EQ(specs[i].body.size(), hexa::config::kGestures[i].body_key_count);
  }
}

// ── Validation ──

TEST(Validate, AcceptsTheBakedTable) {
  EXPECT_NO_THROW(gs::validate_gestures(
      gs::gesture_specs_from_config(), g::leg_specs_from_config(),
      g::nominal_stance_from_config(),
      hexa::posture::PoseLimits{}));
}

TEST(Validate, RejectsAKnotPastAJointLimit) {
  hexa::JointAngles a = joints_at("l_front", 0.0f, 0.0f, 0.05f);
  a[1] = hexa::config::kJointLimits[1].lower - 0.01f;
  const auto spec = one_leg_spec(hexa::Leg::L_FRONT, {joint_key(0.5f, a)});
  try {
    gs::validate_gestures({spec}, g::leg_specs_from_config(),
                          g::nominal_stance_from_config(),
                          hexa::posture::PoseLimits{});
    FAIL() << "accepted a femur past its limit";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("l_front"), std::string::npos) << msg;
    EXPECT_NE(msg.find("femur"), std::string::npos) << msg;
  }
}

TEST(Validate, RejectsAKnotBelowTheGroundPlane) {
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT, {joint_key(0.5f, joints_at("l_front", 0.0f, 0.0f, -0.01f))});
  EXPECT_THROW(gs::validate_gestures(
                   {spec}, g::leg_specs_from_config(),
                   g::nominal_stance_from_config(),
                   hexa::posture::PoseLimits{}),
               std::invalid_argument);
}

TEST(Validate, StandInKnotsAreNotChecked) {
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT,
      {start_key(0.2f), joint_key(0.5f, joints_at("l_front", 0.0f, 0.0f, 0.05f)),
       hold_key(1.0f)});
  EXPECT_NO_THROW(gs::validate_gestures(
      {spec}, g::leg_specs_from_config(), g::nominal_stance_from_config(),
      hexa::posture::PoseLimits{}));
}

TEST(Validate, RejectsAStartAfterAJointKeyframe) {
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT,
      {joint_key(0.5f, joints_at("l_front", 0.0f, 0.0f, 0.05f)), start_key(1.0f)});
  EXPECT_THROW(gs::validate_gestures({spec}, g::leg_specs_from_config(),
                                     g::nominal_stance_from_config(),
                                     hexa::posture::PoseLimits{}),
               std::invalid_argument);
}

TEST(Validate, RejectsAContinuousKeyframeAfterALiveKnot) {
  const auto a = joints_at("l_front", 0.0f, 0.0f, 0.05f);
  const auto b = joints_at("l_front", 0.2f, 0.0f, 0.05f);
  // After the implicit start, after a start, after a home and after a hold
  // of one; fine after a joint keyframe or a hold of one.
  for (const auto& keys : std::vector<std::vector<LegKeyframe>>{
           {joint_key(0.5f, a, GestureTransition::CONTINUOUS), joint_key(1.0f, b)},
           {start_key(0.2f), joint_key(0.5f, a, GestureTransition::CONTINUOUS),
            joint_key(1.0f, b)},
           {joint_key(0.5f, a), home_key(1.0f),
            joint_key(1.5f, b, GestureTransition::CONTINUOUS), joint_key(2.0f, a)},
           {hold_key(0.2f), joint_key(0.5f, a, GestureTransition::CONTINUOUS),
            joint_key(1.0f, b)}}) {
    EXPECT_THROW(gs::validate_gestures({one_leg_spec(hexa::Leg::L_FRONT, keys)},
                                       g::leg_specs_from_config(),
                                       g::nominal_stance_from_config(),
                                       hexa::posture::PoseLimits{}),
                 std::invalid_argument);
  }
  for (const auto& keys : std::vector<std::vector<LegKeyframe>>{
           {joint_key(0.5f, a), joint_key(1.0f, b, GestureTransition::CONTINUOUS),
            joint_key(1.5f, a, GestureTransition::CONTINUOUS)},
           {joint_key(0.5f, a), hold_key(0.8f),
            joint_key(1.0f, b, GestureTransition::CONTINUOUS),
            joint_key(1.5f, a, GestureTransition::CONTINUOUS)}}) {
    EXPECT_NO_THROW(gs::validate_gestures({one_leg_spec(hexa::Leg::L_FRONT, keys)},
                                          g::leg_specs_from_config(),
                                          g::nominal_stance_from_config(),
                                          hexa::posture::PoseLimits{}));
  }
}

TEST(Validate, RejectsATrackThatDoesNotEndAtHome) {
  gs::GestureSpec spec;
  spec.id = "test";
  gs::LegTrack track;
  track.leg = hexa::Leg::L_FRONT;
  track.keys = {joint_key(0.5f, joints_at("l_front", 0.0f, 0.0f, 0.05f))};
  spec.legs.push_back(track);
  EXPECT_THROW(gs::validate_gestures({spec}, g::leg_specs_from_config(),
                                     g::nominal_stance_from_config(),
                                     hexa::posture::PoseLimits{}),
               std::invalid_argument);
  spec.legs.clear();
  BodyKeyframe k{};
  k.t = 0.5f;
  k.pitch = -0.1f;
  k.transition = GestureTransition::EASE;
  spec.body = {k};
  EXPECT_THROW(gs::validate_gestures({spec}, g::leg_specs_from_config(),
                                     g::nominal_stance_from_config(),
                                     hexa::posture::PoseLimits{}),
               std::invalid_argument);
}

TEST(Player, HomeKnotLandsOnNominalAtItsOwnTime) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  // Two legs with different home times: each is home at its own t.
  gs::GestureSpec spec;
  spec.id = "test";
  gs::LegTrack left;
  left.leg = hexa::Leg::L_FRONT;
  left.keys = {joint_key(0.5f, joints_at("l_front", 0.0f, 0.0f, 0.05f)),
               home_key(1.0f)};
  gs::LegTrack right;
  right.leg = hexa::Leg::R_FRONT;
  right.keys = {joint_key(0.5f, joints_at("r_front", 0.0f, 0.0f, 0.05f)),
                home_key(2.0f)};
  spec.legs = {left, right};
  gs::GesturePlayer player(spec, nominal, specs);
  EXPECT_NEAR(player.duration(), 2.0f, 1e-6f);
  for (int i = 0; i < 50; ++i) player.update(kDt);  // t = 1.0
  auto out = player.update(0.0f);
  EXPECT_FALSE(out.at("l_front").direct) << "home: the stance under the body";
  EXPECT_TRUE(near(out.at("l_front").foot_target, nominal.at("l_front"), 1e-7f));
  EXPECT_TRUE(out.at("l_front").stance);
  EXPECT_TRUE(out.at("r_front").direct);
  EXPECT_FALSE(out.at("r_front").stance);
  while (!player.done()) out = player.update(kDt);
  EXPECT_FALSE(out.at("r_front").direct);
  EXPECT_TRUE(near(out.at("r_front").foot_target, nominal.at("r_front"), 1e-7f));
}

// ── Engine ──

TEST(Engine, GestureRefusedUnlessStanding) {
  auto e = g::make_default_engine("tripod");
  ASSERT_FALSE(hexa::config::kGestures.empty());
  const std::string id(hexa::config::kGestures[0].id);
  EXPECT_FALSE(e->request_gesture(id)) << "from the belly";
  run_to_stand(*e);
  EXPECT_FALSE(e->request_gesture("no_such_gesture"));
  // Walking.
  for (int i = 0; i < 50; ++i) e->update(kDt, {0.05f, 0.0f}, 0.0f);
  ASSERT_NE(e->state(), g::EngineState::STAND);
  EXPECT_FALSE(e->request_gesture(id)) << "while walking";
}

TEST(Engine, GestureRunsFromStandAndHandsBackAStand) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  const auto nominal = g::nominal_stance_from_config();
  ASSERT_TRUE(e->request_gesture("wave"));
  EXPECT_EQ(e->state(), g::EngineState::STAND) << "latched, not started";

  bool lifted = false;
  std::vector<g::EngineState> seen;
  std::map<std::string, g::LegOutput> out;
  for (int i = 0; i < 1000; ++i) {
    out = e->update(kDt, {0.0f, 0.0f}, 0.0f);
    if (seen.empty() || seen.back() != e->state()) seen.push_back(e->state());
    if (e->state() == g::EngineState::GESTURE) {
      const auto progress = e->gesture();
      ASSERT_TRUE(progress.has_value());
      EXPECT_EQ(progress->id, "wave");
      lifted = lifted || !out.at("l_front").stance;
      for (const auto& leg : g::LEG_NAMES) {
        if (leg != "l_front") {
          EXPECT_TRUE(near(out.at(leg).foot_target, nominal.at(leg), 1e-7f)) << leg;
        }
      }
    }
    if (i > 0 && e->state() == g::EngineState::STAND) break;
  }
  ASSERT_EQ(seen.size(), 2u);
  EXPECT_EQ(seen[0], g::EngineState::GESTURE);
  EXPECT_EQ(seen[1], g::EngineState::STAND);
  EXPECT_TRUE(lifted);
  EXPECT_FALSE(e->gesture().has_value());
  for (const auto& leg : g::LEG_NAMES) {
    EXPECT_TRUE(near(out.at(leg).foot_target, nominal.at(leg), 1e-5f)) << leg;
    EXPECT_TRUE(out.at(leg).stance) << leg;
  }
  EXPECT_EQ(g::state_value(g::EngineState::GESTURE), "gesture");
}

TEST(Engine, CommandIsIgnoredUntilTheGestureEnds) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  ASSERT_TRUE(e->request_gesture("wave"));
  e->update(kDt, {0.0f, 0.0f}, 0.0f);
  ASSERT_EQ(e->state(), g::EngineState::GESTURE);
  for (int i = 0; i < 20; ++i) {
    e->update(kDt, {0.05f, 0.0f}, 0.0f);
    EXPECT_EQ(e->state(), g::EngineState::GESTURE);
  }
  // Wait it out at zero, then the same command walks.
  run_until_stand(*e);
  ASSERT_EQ(e->state(), g::EngineState::STAND);
  e->update(kDt, {0.05f, 0.0f}, 0.0f);
  EXPECT_EQ(e->state(), g::EngineState::ENGAGING);
}

TEST(Engine, ACommandArrivingWithTheRequestWins) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  ASSERT_TRUE(e->request_gesture("wave"));
  e->update(kDt, {0.05f, 0.0f}, 0.0f);
  EXPECT_EQ(e->state(), g::EngineState::ENGAGING);
  EXPECT_FALSE(e->gesture().has_value());
}

TEST(Engine, FoldRequestedMidGestureRunsAfterIt) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  ASSERT_TRUE(e->request_gesture("bow"));
  e->update(kDt, {0.0f, 0.0f}, 0.0f);
  ASSERT_EQ(e->state(), g::EngineState::GESTURE);
  EXPECT_TRUE(e->request_fold());
  const auto seen = run_until_stand(*e);
  ASSERT_FALSE(seen.empty());
  EXPECT_EQ(seen.back(), g::EngineState::STAND);
  e->update(kDt, {0.0f, 0.0f}, 0.0f);
  EXPECT_EQ(e->state(), g::EngineState::FOLDING);
}

TEST(Engine, PresetAndGaitRefusedMidGesture) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  ASSERT_TRUE(e->request_gesture("wave"));
  e->update(kDt, {0.0f, 0.0f}, 0.0f);
  ASSERT_EQ(e->state(), g::EngineState::GESTURE);
  EXPECT_FALSE(e->request_preset("offroad"));
  EXPECT_FALSE(e->set_strategy("ripple"));
  EXPECT_FALSE(e->start_fold());
  EXPECT_FALSE(e->request_gesture("bow")) << "one at a time";
  EXPECT_EQ(e->strategy_name(), "tripod");
  EXPECT_EQ(e->preset_id(), "normal");
}

TEST(Engine, FaultAbortsTheGesture) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  ASSERT_TRUE(e->request_gesture("wave"));
  e->update(kDt, {0.0f, 0.0f}, 0.0f);
  ASSERT_EQ(e->state(), g::EngineState::GESTURE);
  e->enter_fault();
  EXPECT_EQ(e->state(), g::EngineState::FAULT);
  EXPECT_FALSE(e->gesture().has_value());
  // A latched request dies with the fault too.
  auto f = g::make_default_engine("tripod");
  run_to_stand(*f);
  ASSERT_TRUE(f->request_gesture("wave"));
  f->enter_fault();
  run_to_stand(*f);
  f->update(kDt, {0.0f, 0.0f}, 0.0f);
  EXPECT_EQ(f->state(), g::EngineState::STAND);
}

TEST(Engine, GestureRefusedOffTheDefaultPreset) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  ASSERT_TRUE(e->request_preset("fast"));
  run_until_stand(*e, 6000);
  ASSERT_EQ(e->state(), g::EngineState::STAND);
  ASSERT_EQ(e->preset_id(), "fast");
  EXPECT_FALSE(e->request_gesture("wave"));
  ASSERT_TRUE(e->request_preset("normal"));
  run_until_stand(*e, 6000);
  ASSERT_EQ(e->preset_id(), "normal");
  EXPECT_TRUE(e->request_gesture("wave"));

  // Quadruped, from the belly.
  auto q = g::make_default_engine("tripod");
  ASSERT_TRUE(q->request_preset("quad"));
  ASSERT_TRUE(q->set_strategy("quad_walk"));
  ASSERT_TRUE(q->start_initialize());
  for (int i = 0; i < 4000 && q->state() != g::EngineState::STAND; ++i) {
    q->update(kDt, {0.0f, 0.0f}, 0.0f);
  }
  ASSERT_EQ(q->state(), g::EngineState::STAND);
  ASSERT_EQ(q->leg_set(), g::LegSet::QUADRUPED);
  EXPECT_FALSE(q->request_gesture("wave"));
}

TEST(Engine, GestureRefusedWhileAPresetChangeIsArmed) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  ASSERT_TRUE(e->request_preset("fast"));
  EXPECT_FALSE(e->request_gesture("wave"));
}
