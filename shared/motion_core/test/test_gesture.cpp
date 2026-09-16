// Gestures: keyframe sampling (the two segment shapes and their joins), the
// player (implicit start / return knots, preserve resolution, per-leg tracks)
// and the engine's GESTURE state with its guards.

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gesture/keyframe.hpp"
#include "gesture/player.hpp"
#include "kinematics/body_transform.hpp"

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
  k.angle = value;
  k.reach = value;
  k.height = value;
  k.transition = tr;
  return k;
}

float angle_of(const LegKeyframe& k) { return k.angle; }

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

// A gesture spec moving one leg through one keyframe.
gs::GestureSpec one_leg_spec(hexa::Leg leg, const std::vector<LegKeyframe>& keys,
                             float return_time = 0.5f) {
  gs::GestureSpec spec;
  spec.id = "test";
  spec.return_time = return_time;
  gs::LegTrack track;
  track.leg = leg;
  track.keys = keys;
  spec.legs.push_back(track);
  return spec;
}

LegKeyframe polar_key(float t, float angle, float reach, float height,
                      GestureTransition tr = GestureTransition::EASE) {
  LegKeyframe k{};
  k.t = t;
  k.angle = angle;
  k.reach = reach;
  k.height = height;
  k.transition = tr;
  return k;
}

LegKeyframe preserve_key(float t, GestureTransition tr = GestureTransition::EASE) {
  LegKeyframe k{};
  k.t = t;
  k.transition = tr;
  k.preserve = true;
  return k;
}

// The default preset's l_front foot in its own polar currency.
gs::LegPolar nominal_polar(const std::string& leg) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const hexa::Vec3 in_leg = hexa::body_to_leg(nominal.at(leg), specs.at(leg));
  return gs::polar_from_leg_frame(in_leg, in_leg.z);
}

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
    EXPECT_NEAR(sample(keys, k.t), k.angle, kTol) << "t=" << k.t;
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

// ── Player ──

TEST(Player, PolarRoundTripReproducesNominalExactly) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  for (const auto& leg : g::LEG_NAMES) {
    const hexa::Vec3 in_leg = hexa::body_to_leg(nominal.at(leg), specs.at(leg));
    const gs::LegPolar p = gs::polar_from_leg_frame(in_leg, in_leg.z);
    EXPECT_NEAR(p.height, 0.0f, 1e-7f) << leg;
    const hexa::Vec3 back =
        hexa::leg_to_body(gs::leg_frame_from_polar(p, in_leg.z), specs.at(leg));
    EXPECT_TRUE(near(back, nominal.at(leg), 1e-6f)) << leg;
  }
}

TEST(Player, StartsAtTheCurrentFootAndReturnsToNominal) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  auto start = nominal;
  start["l_front"] = nominal.at("l_front") + hexa::Vec3(0.01f, -0.005f, 0.0f);

  const gs::LegPolar home = nominal_polar("l_front");
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT,
      {polar_key(0.5f, home.angle + 0.3f, home.reach - 0.02f, 0.05f)});
  gs::GesturePlayer player(spec, start, nominal, specs);

  auto out = player.update(1e-5f);
  EXPECT_TRUE(near(out.at("l_front").foot_target, start.at("l_front"), 1e-3f));
  EXPECT_FALSE(player.done());

  for (int i = 0; i < 200 && !player.done(); ++i) {
    out = player.update(kDt);
  }
  EXPECT_TRUE(player.done());
  EXPECT_NEAR(player.duration(), 1.0f, 1e-6f);
  EXPECT_TRUE(near(out.at("l_front").foot_target, nominal.at("l_front"), 1e-5f));
  EXPECT_TRUE(out.at("l_front").stance);
  EXPECT_NEAR(out.at("l_front").phase, 1.0f, 1e-6f);
}

TEST(Player, UntrackedLegsStayPlanted) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const gs::LegPolar home = nominal_polar("l_front");
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT, {polar_key(0.5f, home.angle, home.reach, 0.05f)});
  gs::GesturePlayer player(spec, nominal, nominal, specs);
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

TEST(Player, LiftedLegReportsSwingOnlyWhileAboveTheGround) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const gs::LegPolar home = nominal_polar("l_front");
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT, {polar_key(0.5f, home.angle, home.reach, 0.05f)});
  gs::GesturePlayer player(spec, nominal, nominal, specs);
  auto out = player.update(1e-5f);
  EXPECT_TRUE(out.at("l_front").stance) << "planted at the start";
  bool saw_swing = false;
  while (!player.done()) {
    out = player.update(kDt);
    const float lift = out.at("l_front").foot_target.z - nominal.at("l_front").z;
    EXPECT_EQ(out.at("l_front").stance, lift <= gs::kPlantedHeight);
    saw_swing = saw_swing || !out.at("l_front").stance;
  }
  EXPECT_TRUE(saw_swing);
  EXPECT_TRUE(out.at("l_front").stance) << "planted at the end";
}

TEST(Player, BodyTrackReturnsToIdentity) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  gs::GestureSpec spec;
  spec.id = "bow";
  spec.return_time = 0.5f;
  BodyKeyframe k{};
  k.t = 0.5f;
  k.pitch = -0.2f;
  k.x = 0.02f;
  k.transition = GestureTransition::EASE;
  spec.body.push_back(k);
  gs::GesturePlayer player(spec, nominal, nominal, specs);

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
  const gs::LegPolar l = nominal_polar("l_front");
  const gs::LegPolar r = nominal_polar("r_front");
  gs::GestureSpec spec;
  spec.id = "test";
  spec.return_time = 0.5f;
  // l_front is named at A (0.5) and C (1.5); r_front alone at B (1.0). The
  // flattened tracks carry that directly, so l_front eases A -> C over B.
  gs::LegTrack left;
  left.leg = hexa::Leg::L_FRONT;
  left.keys = {polar_key(0.5f, l.angle, l.reach, 0.02f),
               polar_key(1.5f, l.angle, l.reach, 0.06f)};
  gs::LegTrack right;
  right.leg = hexa::Leg::R_FRONT;
  right.keys = {polar_key(1.0f, r.angle, r.reach, 0.03f)};
  spec.legs = {left, right};
  gs::GesturePlayer player(spec, nominal, nominal, specs);
  for (int i = 0; i < 50; ++i) player.update(kDt);  // t = 1.0
  const auto out = player.update(0.0f);
  const float lift_l = out.at("l_front").foot_target.z - nominal.at("l_front").z;
  const float lift_r = out.at("r_front").foot_target.z - nominal.at("r_front").z;
  EXPECT_NEAR(lift_l, 0.04f, 1e-4f) << "midway A -> C, ease5(0.5) = 0.5";
  EXPECT_NEAR(lift_r, 0.03f, 1e-4f);
}

TEST(Player, TwoLegsInOneKeyframeMoveTogether) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const gs::LegPolar l = nominal_polar("l_front");
  const gs::LegPolar r = nominal_polar("r_front");
  gs::GestureSpec spec;
  spec.id = "test";
  spec.return_time = 0.5f;
  gs::LegTrack left;
  left.leg = hexa::Leg::L_FRONT;
  left.keys = {polar_key(0.8f, l.angle, l.reach, 0.04f)};
  gs::LegTrack right;
  right.leg = hexa::Leg::R_FRONT;
  right.keys = {polar_key(0.8f, r.angle, r.reach, 0.04f)};
  spec.legs = {left, right};
  gs::GesturePlayer player(spec, nominal, nominal, specs);
  while (!player.done()) {
    const auto out = player.update(kDt);
    const float lift_l = out.at("l_front").foot_target.z - nominal.at("l_front").z;
    const float lift_r = out.at("r_front").foot_target.z - nominal.at("r_front").z;
    EXPECT_NEAR(lift_l, lift_r, 1e-6f) << "t=" << player.t();
    EXPECT_EQ(out.at("l_front").phase, out.at("r_front").phase);
  }
}

TEST(Player, PreserveKnotHoldsThePreviousValue) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const gs::LegPolar home = nominal_polar("l_front");
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT,
      {polar_key(0.5f, home.angle + 0.2f, home.reach, 0.05f), preserve_key(1.0f)});
  gs::GesturePlayer player(spec, nominal, nominal, specs);
  for (int i = 0; i < 25; ++i) player.update(kDt);  // t = 0.5
  const hexa::Vec3 at_knot = player.update(0.0f).at("l_front").foot_target;
  for (int i = 0; i < 25; ++i) {
    const auto out = player.update(kDt);
    EXPECT_TRUE(near(out.at("l_front").foot_target, at_knot, 1e-5f))
        << "t=" << player.t();
  }
  EXPECT_NEAR(player.duration(), 1.5f, 1e-6f);
}

TEST(Player, LeadingPreserveKnotHoldsTheStartPosition) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  auto start = nominal;
  start["l_front"] = nominal.at("l_front") + hexa::Vec3(0.0f, 0.0f, 0.02f);
  const gs::LegPolar home = nominal_polar("l_front");
  const auto spec = one_leg_spec(
      hexa::Leg::L_FRONT,
      {preserve_key(0.5f), polar_key(1.0f, home.angle, home.reach, 0.05f)});
  gs::GesturePlayer player(spec, start, nominal, specs);
  for (int i = 0; i < 25; ++i) {
    const auto out = player.update(kDt);
    EXPECT_TRUE(near(out.at("l_front").foot_target, start.at("l_front"), 1e-5f))
        << "t=" << player.t();
  }
}

TEST(Player, BodyPreserveKnotHoldsThePreviousPose) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  gs::GestureSpec spec;
  spec.id = "test";
  spec.return_time = 0.5f;
  BodyKeyframe a{};
  a.t = 0.5f;
  a.roll = 0.1f;
  a.y = -0.01f;
  a.transition = GestureTransition::EASE;
  BodyKeyframe hold{};
  hold.t = 1.5f;
  hold.transition = GestureTransition::EASE;
  hold.preserve = true;
  spec.body = {a, hold};
  gs::GesturePlayer player(spec, nominal, nominal, specs);
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
