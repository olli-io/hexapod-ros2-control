// Unit tests for the folded <-> standing ladders (InitializeController /
// FoldController).
//
// The ladders sequence legs as three mirrored pairs (PAIR_ORDER), so the thing
// worth pinning is coverage: every leg belongs to exactly one pair, and every
// leg has actually left the folded pose by the time the controller reports DONE.
// A leg that silently sat out its pair would look exactly like a leg whose
// commands never reached the servos, so this suite is what tells the two apart.
//
// Float-only, built through hexa_host_test() under -Wdouble-promotion.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gait/stand_transition.hpp"
#include "kinematics/leg_ik.hpp"
#include "leg_index.hpp"

namespace g = hexa::gait;

namespace {

constexpr float kDt = 0.005f;  // controller_dt
constexpr float kTol = 1e-4f;

struct Ladder {
  std::map<std::string, g::Vec3> initial;
  std::map<std::string, g::Vec3> nominal;
  hexa::gait::EngineConfig cfg;
};

Ladder baked() {
  return {g::initial_stance_from_config(), g::nominal_stance_from_config(),
          g::engine_config_from_config()};
}

g::InitializeController make_initialize(const Ladder& l) {
  return g::InitializeController(
      l.initial, l.nominal, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, l.cfg.init_pair_swing_time,
      l.cfg.init_lift_body_time, l.cfg.init_swing_clearance, l.cfg.swing_width,
      l.cfg.controller_dt);
}

g::FoldController make_fold(const Ladder& l) {
  return g::FoldController(l.initial, l.nominal, hexa::config::kCoxaToBottom,
                           hexa::config::kFootRadius, l.cfg.init_pair_swing_time,
                           l.cfg.init_lift_body_time, l.cfg.init_swing_clearance,
                           l.cfg.swing_width, l.cfg.controller_dt);
}

void expect_near(const g::Vec3& got, const g::Vec3& want,
                 const std::string& who) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(got[i], want[i], kTol) << who << "[" << i << "]";
  }
}

// Upper bound on ticks: three pair swings + the body ramp, with slack.
int max_ticks(const Ladder& l) {
  return static_cast<int>(
             (3.0f * l.cfg.init_pair_swing_time + l.cfg.init_lift_body_time) /
             kDt) +
         50;
}

// One leg's z through a body ramp, sampled per tick. Every leg shares the same
// ramp, so one is the whole story.
constexpr const char* kProbeLeg = "l_middle";

// Body-frame IK target z at which the robot meets the floor. The target is the
// foot sphere's centre, so it sits one radius above the contact point at
// -kCoxaToBottom — and both ladders are built so this is an *endpoint* of the
// body ramp, never a point inside it.
float contact_z() {
  return hexa::ik_z_for_contact(-hexa::config::kCoxaToBottom,
                                hexa::config::kFootRadius);
}

// One leg's z, sampled per tick, across a controller's body ramp. Every leg
// shares the same ramp, so one is the whole story.
std::vector<float> initialize_ramp(g::InitializeController& init, int limit) {
  std::vector<float> z;
  for (int i = 0; i < limit; ++i) {
    const auto out = init.update(kDt);
    if (init.state() == g::InitializeState::PLACE_FEET) continue;
    z.push_back(out.at(kProbeLeg).foot_target[2]);
    if (init.done()) break;
  }
  return z;
}

std::vector<float> fold_ramp(g::FoldController& fold, int limit) {
  std::vector<float> z;
  for (int i = 0; i < limit; ++i) {
    const auto out = fold.update(kDt);
    z.push_back(out.at(kProbeLeg).foot_target[2]);
    if (fold.state() != g::FoldState::LOWER_BODY) break;
  }
  return z;
}

// Largest per-tick |dz| anywhere in a ramp.
float peak_step(const std::vector<float>& z) {
  float peak = 0.0f;
  for (std::size_t i = 1; i < z.size(); ++i) {
    peak = std::max(peak, std::fabs(z[i] - z[i - 1]));
  }
  return peak;
}

}  // namespace

// ── PAIR_ORDER covers the whole robot ───────────────────────────────────────

TEST(PairOrder, CoversEveryLegExactlyOnce) {
  std::multiset<std::string> seen;
  for (const auto& pair : g::PAIR_ORDER) {
    for (const auto& name : pair) seen.insert(name);
  }
  ASSERT_EQ(seen.size(), static_cast<std::size_t>(hexa::kNumLegs));
  for (const auto& name : g::LEG_NAMES) {
    EXPECT_EQ(seen.count(std::string(name)), 1u)
        << name << " is missing from PAIR_ORDER or listed twice";
  }
}

TEST(PairOrder, EachPairMirrorsAcrossTheBody) {
  // The pairs exist to keep the CoM near the chassis centre: middles together,
  // then each diagonal. A pair with two same-side legs would tip the belly.
  for (const auto& pair : g::PAIR_ORDER) {
    EXPECT_NE(pair[0].substr(0, 2), pair[1].substr(0, 2))
        << pair[0] << " + " << pair[1] << " are on the same side";
  }
}

// ── InitializeController: folded -> standing ────────────────────────────────

TEST(InitializeLadder, EveryLegReachesNominalStance) {
  const Ladder l = baked();
  auto init = make_initialize(l);

  std::map<std::string, g::LegOutput> out;
  int ticks = 0;
  const int limit = max_ticks(l);
  while (!init.done() && ticks < limit) {
    out = init.update(kDt);
    ++ticks;
  }
  ASSERT_TRUE(init.done()) << "ladder never finished in " << limit << " ticks";

  for (const auto& name : g::LEG_NAMES) {
    const std::string leg(name);
    expect_near(out.at(leg).foot_target, l.nominal.at(leg), leg);
  }
}

TEST(InitializeLadder, EveryLegLeavesTheFoldedPose) {
  // The regression that matters: a leg whose pair never ran would still be
  // sitting on its initial_stance entry at DONE. r_front is the last pair, so
  // it is the one a truncated ladder would strand.
  const Ladder l = baked();
  auto init = make_initialize(l);

  std::map<std::string, bool> moved;
  for (const auto& name : g::LEG_NAMES) moved[std::string(name)] = false;

  const int limit = max_ticks(l);
  for (int i = 0; i < limit && !init.done(); ++i) {
    const auto out = init.update(kDt);
    for (const auto& name : g::LEG_NAMES) {
      const std::string leg(name);
      const g::Vec3 d = out.at(leg).foot_target - l.initial.at(leg);
      if (std::hypot(std::hypot(d[0], d[1]), d[2]) > 1e-3f) moved[leg] = true;
    }
  }
  ASSERT_TRUE(init.done());
  for (const auto& [leg, did] : moved) {
    EXPECT_TRUE(did) << leg << " never left the folded pose";
  }
}

TEST(InitializeLadder, PairsSwingOneAtATime) {
  // Only the active pair is airborne; the other four report stance. This is what
  // keeps the belly load predictable during PLACE_FEET.
  const Ladder l = baked();
  auto init = make_initialize(l);

  bool saw_swing = false;
  for (int i = 0; i < max_ticks(l) && !init.done(); ++i) {
    const auto out = init.update(kDt);
    if (init.state() != g::InitializeState::PLACE_FEET) continue;
    int swinging = 0;
    for (const auto& name : g::LEG_NAMES) {
      if (!out.at(std::string(name)).stance) ++swinging;
    }
    EXPECT_LE(swinging, 2) << "more than one pair airborne at tick " << i;
    if (swinging > 0) saw_swing = true;
  }
  EXPECT_TRUE(saw_swing) << "PLACE_FEET never lifted a foot";
}

TEST(InitializeLadder, PlaceFeetLandsFeetOnTheFloor) {
  // Every foot must be planted at the same z before LIFT_BODY starts, or the
  // body ramp would drag one leg along the floor.
  //
  // That z is the floor itself. Placing the feet above it — the ladder used to
  // hold them 12 mm up — does not avoid the ground contact, it only moves the
  // contact into the middle of the LIFT_BODY ramp, where the curve is at speed
  // and nothing is slowing down for it. Here the swing arc does the landing,
  // and it already arrives with zero vertical velocity.
  const Ladder l = baked();
  auto init = make_initialize(l);

  const float want_z = contact_z();
  std::map<std::string, g::LegOutput> last;
  for (int i = 0; i < max_ticks(l); ++i) {
    const auto out = init.update(kDt);
    if (init.state() != g::InitializeState::PLACE_FEET) {
      last = out;
      break;
    }
  }
  ASSERT_FALSE(last.empty()) << "never left PLACE_FEET";
  for (const auto& name : g::LEG_NAMES) {
    const std::string leg(name);
    EXPECT_NEAR(last.at(leg).foot_target[2], want_z, kTol) << leg;
    EXPECT_NEAR(last.at(leg).foot_target[0], l.nominal.at(leg)[0], kTol) << leg;
    EXPECT_NEAR(last.at(leg).foot_target[1], l.nominal.at(leg)[1], kTol) << leg;
  }
}

TEST(InitializeLadder, BodyRampStartsOnTheFloorAndLeavesItStationary) {
  // The regression this pins is the one that made the cold start slip: ground
  // contact anywhere but an endpoint of the ramp. It belongs at tau = 0 here,
  // where the ramp is not yet moving — the feet are already down when the legs
  // start taking the body's weight, and they never come back up.
  const Ladder l = baked();
  auto init = make_initialize(l);

  const std::vector<float> z = initialize_ramp(init, max_ticks(l));
  ASSERT_TRUE(init.done());
  ASSERT_GE(z.size(), 3u);

  EXPECT_NEAR(z.front(), contact_z(), kTol) << "LIFT_BODY did not start on the floor";
  const float peak = peak_step(z);
  ASSERT_GT(peak, 0.0f);
  EXPECT_LT(std::fabs(z[1] - z[0]), 0.05f * peak)
      << "the ramp leaves the floor at " << 100.0f * std::fabs(z[1] - z[0]) / peak
      << "% of its peak speed";
  for (std::size_t i = 0; i < z.size(); ++i) {
    EXPECT_LE(z[i], contact_z() + kTol) << "foot rose off the floor at sample " << i;
  }
}

TEST(InitializeLadder, BodyRampIsMonotonicAndFinishesOnTime) {
  // A quintic must not become a reversal, and must not lengthen the ladder —
  // it is still three pair swings plus init_lift_body_time.
  const Ladder l = baked();
  auto init = make_initialize(l);

  const std::vector<float> z = initialize_ramp(init, max_ticks(l));
  ASSERT_TRUE(init.done());

  for (std::size_t i = 1; i < z.size(); ++i) {
    EXPECT_LE(z[i], z[i - 1] + kTol) << "ramp reversed at sample " << i;
  }
  const int want_ticks = static_cast<int>(l.cfg.init_lift_body_time / kDt);
  EXPECT_NEAR(static_cast<int>(z.size()), want_ticks + 1, 2);
}

// ── FoldController: standing -> folded ──────────────────────────────────────

TEST(FoldLadder, EveryLegReachesInitialStance) {
  const Ladder l = baked();
  auto fold = make_fold(l);

  std::map<std::string, g::LegOutput> out;
  int ticks = 0;
  const int limit = max_ticks(l);
  while (!fold.done() && ticks < limit) {
    out = fold.update(kDt);
    ++ticks;
  }
  ASSERT_TRUE(fold.done()) << "ladder never finished in " << limit << " ticks";

  for (const auto& name : g::LEG_NAMES) {
    const std::string leg(name);
    expect_near(out.at(leg).foot_target, l.initial.at(leg), leg);
  }
}

TEST(FoldLadder, ReversesTheInitializePairOrder) {
  // Fold lifts feet in the mirror of the cold-start order, so the last leg down
  // is the first leg back up.
  const Ladder l = baked();
  auto fold = make_fold(l);

  std::vector<std::string> lift_order;
  for (int i = 0; i < max_ticks(l) && !fold.done(); ++i) {
    const auto out = fold.update(kDt);
    if (fold.state() != g::FoldState::LIFT_FEET) continue;
    for (const auto& name : g::LEG_NAMES) {
      const std::string leg(name);
      if (out.at(leg).stance) continue;
      if (std::find(lift_order.begin(), lift_order.end(), leg) ==
          lift_order.end()) {
        lift_order.push_back(leg);
      }
    }
  }
  ASSERT_EQ(lift_order.size(), static_cast<std::size_t>(hexa::kNumLegs));
  // Pairs come off in reverse PAIR_ORDER; within a pair the order is incidental.
  for (std::size_t p = 0; p < g::PAIR_ORDER.size(); ++p) {
    const auto& want = g::PAIR_ORDER[g::PAIR_ORDER.size() - 1 - p];
    const std::set<std::string> got{lift_order[p * 2], lift_order[p * 2 + 1]};
    const std::set<std::string> expected{want[0], want[1]};
    EXPECT_EQ(got, expected) << "pair " << p << " lifted out of order";
  }
}

TEST(FoldLadder, BodyRampMeetsTheFloorStationaryAtItsEnd) {
  // The mirror: the belly arrives exactly as LOWER_BODY runs out, so the ramp
  // is already stopped when it lands rather than still descending into it. The
  // fold used to overshoot the contact and put the landing mid-ramp.
  const Ladder l = baked();
  auto fold = make_fold(l);

  const std::vector<float> z = fold_ramp(fold, max_ticks(l));
  ASSERT_NE(fold.state(), g::FoldState::LOWER_BODY) << "LOWER_BODY never ended";
  ASSERT_GE(z.size(), 3u);

  EXPECT_NEAR(z.back(), contact_z(), kTol) << "LOWER_BODY did not end on the floor";
  const float peak = peak_step(z);
  ASSERT_GT(peak, 0.0f);
  const float last = std::fabs(z[z.size() - 1] - z[z.size() - 2]);
  EXPECT_LT(last, 0.05f * peak)
      << "the belly lands at " << 100.0f * last / peak << "% of the ramp's "
      << "peak speed";
  for (std::size_t i = 0; i < z.size(); ++i) {
    EXPECT_LE(z[i], contact_z() + kTol) << "body dipped past the floor at " << i;
  }
}

TEST(FoldLadder, IsTheTimeReverseOfInitialize) {
  // Both ladders visit the same belly-height footprint, from opposite ends.
  const Ladder l = baked();
  auto init = make_initialize(l);
  auto fold = make_fold(l);

  std::map<std::string, g::LegOutput> init_end;
  for (int i = 0; i < max_ticks(l); ++i) {
    const auto out = init.update(kDt);
    if (init.state() != g::InitializeState::PLACE_FEET) {
      init_end = out;
      break;
    }
  }
  std::map<std::string, g::LegOutput> fold_mid;
  for (int i = 0; i < max_ticks(l); ++i) {
    const auto out = fold.update(kDt);
    if (fold.state() == g::FoldState::LIFT_FEET) {
      fold_mid = out;
      break;
    }
  }
  ASSERT_FALSE(init_end.empty());
  ASSERT_FALSE(fold_mid.empty());
  for (const auto& name : g::LEG_NAMES) {
    const std::string leg(name);
    expect_near(fold_mid.at(leg).foot_target, init_end.at(leg).foot_target, leg);
  }
}

// ── eased_ramp ──────────────────────────────────────────────────────────────

TEST(RampCurve, PinsBothEndsAndIsMonotonic) {
  EXPECT_NEAR(g::eased_ramp(-0.5f), 0.0f, kTol);
  EXPECT_NEAR(g::eased_ramp(0.0f), 0.0f, kTol);
  EXPECT_NEAR(g::eased_ramp(1.0f), 1.0f, kTol);
  EXPECT_NEAR(g::eased_ramp(1.5f), 1.0f, kTol);
  EXPECT_NEAR(g::eased_ramp(0.5f), 0.5f, kTol);

  constexpr int kSamples = 1000;
  float prev = g::eased_ramp(0.0f);
  for (int i = 1; i <= kSamples; ++i) {
    const float cur = g::eased_ramp(static_cast<float>(i) / kSamples);
    // On the value, not the slope: dividing float noise at 1.0 by the sample
    // step turns an epsilon into a visibly negative slope.
    EXPECT_GE(cur, prev - 1e-6f) << "not monotonic at sample " << i;
    prev = cur;
  }
}

TEST(RampCurve, LeavesTheFloorWithVanishingAcceleration) {
  // What separates the quintic from the cubic smoothstep it replaced. Both
  // start at zero velocity, so velocity alone cannot tell them apart; the
  // cubic's curvature at tau = 0 is its maximum, which is a jerk step exactly
  // where the legs begin taking the body's weight. The quintic's is zero.
  //
  // Measured at the tau = 0 end only: the curve is symmetric, and near tau = 1
  // the values sit at 1.0 where float spacing (1.2e-7) swamps a second
  // difference this small.
  constexpr float h = 1e-3f;
  const float d2_at_zero = std::fabs(g::eased_ramp(2.0f * h) -
                                     2.0f * g::eased_ramp(h) + g::eased_ramp(0.0f));

  float d2_peak = 0.0f;
  for (int i = 1; i < 999; ++i) {
    const float t = static_cast<float>(i) / 1000.0f;
    d2_peak = std::max(d2_peak, std::fabs(g::eased_ramp(t + h) -
                                          2.0f * g::eased_ramp(t) +
                                          g::eased_ramp(t - h)));
  }
  ASSERT_GT(d2_peak, 0.0f);
  EXPECT_LT(d2_at_zero, 0.05f * d2_peak)
      << "curvature at the floor is " << 100.0f * d2_at_zero / d2_peak
      << "% of the ramp's peak; a cubic smoothstep scores ~100%";
}
