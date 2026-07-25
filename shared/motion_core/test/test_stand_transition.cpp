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
      l.cfg.init_pair_swing_time, l.cfg.init_lift_body_time,
      l.cfg.init_swing_clearance, l.cfg.init_place_feet_clearance,
      l.cfg.swing_width, l.cfg.controller_dt);
}

g::FoldController make_fold(const Ladder& l) {
  return g::FoldController(l.initial, l.nominal, hexa::config::kCoxaToBottom,
                           l.cfg.init_pair_swing_time, l.cfg.init_lift_body_time,
                           l.cfg.init_swing_clearance,
                           l.cfg.init_place_feet_clearance, l.cfg.swing_width,
                           l.cfg.controller_dt);
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

TEST(InitializeLadder, PlaceFeetLandsFeetAtTheBellyHeight) {
  // Every foot must be planted at the same z before LIFT_BODY starts, or the
  // body ramp would drag one leg along the floor.
  const Ladder l = baked();
  auto init = make_initialize(l);

  const float want_z =
      -hexa::config::kCoxaToBottom + l.cfg.init_place_feet_clearance;
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
