// The folded <-> standing ladders. Both directions pass through the initialized
// pose and are time-reverses of each other, their one asymmetry being the cold
// start's init_place_clearance.
//
// The ground-facing rung is three mirrored pairs, so the thing worth pinning is
// coverage: every leg belongs to exactly one pair and has actually left its
// starting pose by DONE. A leg that silently sat out its pair looks exactly like
// a leg whose commands never reached the servos, and this suite is what tells the
// two apart. It also pins the mirror, and that folded is genuinely tucked inside
// initialized with both clear of the floor.

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
  std::map<std::string, g::Vec3> folded;
  std::map<std::string, g::Vec3> initialized;
  std::map<std::string, g::Vec3> nominal;
  hexa::gait::EngineConfig cfg;
};

Ladder baked() {
  return {g::folded_stance_from_config(), g::initialized_stance_from_config(),
          g::nominal_stance_from_config(), g::engine_config_from_config()};
}

g::RestPoseMove make_unfold(const Ladder& l) {
  return g::RestPoseMove(l.folded, l.initialized, l.cfg.init_unfold_time);
}

g::InitializeController make_initialize(const Ladder& l) {
  return g::InitializeController(
      l.folded, l.initialized, l.nominal, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, l.cfg.init_pair_swing_time,
      l.cfg.init_lift_body_time, l.cfg.init_unfold_time,
      l.cfg.init_place_clearance, l.cfg.init_swing_clearance, l.cfg.swing_width,
      l.cfg.touchdown_velocity, l.cfg.touchdown_probe_fraction,
      l.cfg.controller_dt);
}

g::FoldController make_fold(const Ladder& l) {
  return g::FoldController(
      l.folded, l.initialized, l.nominal, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, l.cfg.init_pair_swing_time,
      l.cfg.init_lift_body_time, l.cfg.init_unfold_time,
      l.cfg.init_swing_clearance, l.cfg.swing_width, l.cfg.touchdown_velocity,
      l.cfg.touchdown_probe_fraction, l.cfg.controller_dt);
}

void expect_near(const g::Vec3& got, const g::Vec3& want,
                 const std::string& who) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(got[i], want[i], kTol) << who << "[" << i << "]";
  }
}

// Upper bound on ticks for a whole ladder: the body ramp, three pair swings and
// the rest-pose move, with slack.
int max_ticks(const Ladder& l) {
  return static_cast<int>((l.cfg.init_lift_body_time +
                           3.0f * l.cfg.init_pair_swing_time +
                           l.cfg.init_unfold_time) /
                          kDt) +
         50;
}

// One leg's z through a body ramp, sampled per tick. Every leg shares the same
// ramp, so one is the whole story.
constexpr const char* kProbeLeg = "l_middle";

// Body-frame IK target z at which the robot meets the floor. The target is the
// foot sphere's centre, so it sits one radius above the contact point at
// -kCoxaToBottom — and the fold's body ramp is built so this is an *endpoint*
// of it, never a point inside.
float contact_z() {
  return hexa::ik_z_for_contact(-hexa::config::kCoxaToBottom,
                                hexa::config::kFootRadius);
}

// Where the cold start parks its feet: init_place_clearance above contact_z().
// Unlike the fold, this *is* a point inside the body ramp — the ramp's opening
// stretch is what spends it, which is why the initialize ramps on a septic.
float place_z(const Ladder& l) {
  return hexa::ik_z_for_contact(
      -hexa::config::kCoxaToBottom + l.cfg.init_place_clearance,
      hexa::config::kFootRadius);
}

// One leg's z, sampled per tick, across a controller's body ramp.
std::vector<float> initialize_ramp(g::InitializeController& init, int limit) {
  std::vector<float> z;
  for (int i = 0; i < limit; ++i) {
    const auto out = init.update(kDt);
    if (init.state() != g::InitializeState::LIFT_BODY) continue;
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

// ── The two rest poses ──────────────────────────────────────────────────────

TEST(RestPoses, FoldedIsTuckedInsideInitialized) {
  // The whole point of the split: the robot energizes drawn in tighter than the
  // pose the unfold takes it to. Measured as the foot's planar reach from the
  // coxa axis, which is what "tucked" means mechanically — comparing raw joint
  // angles would pass on a femur that folded the wrong way.
  const Ladder l = baked();
  for (const auto& name : g::LEG_NAMES) {
    const std::string leg(name);
    const g::Vec3& folded = l.folded.at(leg);
    const g::Vec3& initialized = l.initialized.at(leg);
    EXPECT_LT(std::hypot(folded[0], folded[1]),
              std::hypot(initialized[0], initialized[1]))
        << leg << " folded reaches no further in than initialized";
  }
}

TEST(RestPoses, BothKeepTheFeetOffTheFloor) {
  // Both are belly-rest poses: the chassis bottom is on the ground and every
  // foot must clear it, or the robot energizes standing on a leg tip and the
  // unfold drags it across the floor.
  const float floor_z = -hexa::config::kCoxaToBottom;
  const std::array<std::pair<const char*, std::map<std::string, g::Vec3>>, 2>
      poses = {{
          {"folded", g::folded_stance_from_config()},
          {"initialized", g::initialized_stance_from_config()},
      }};
  for (const auto& [name, stance] : poses) {
    for (const auto& leg : g::LEG_NAMES) {
      EXPECT_GT(stance.at(std::string(leg))[2] - hexa::config::kFootRadius,
                floor_z)
          << name << " " << leg << " touches the floor";
    }
  }
}

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

// ── RestPoseMove: folded <-> initialized ────────────────────────────────────

TEST(RestPoseMove, EveryLegReachesTheTargetPose) {
  const Ladder l = baked();
  auto unfold = make_unfold(l);

  std::map<std::string, g::LegOutput> out;
  int ticks = 0;
  const int limit = max_ticks(l);
  while (!unfold.done() && ticks < limit) {
    out = unfold.update(kDt);
    ++ticks;
  }
  ASSERT_TRUE(unfold.done()) << "move never finished in " << limit << " ticks";
  EXPECT_NEAR(ticks, static_cast<int>(l.cfg.init_unfold_time / kDt), 2);

  for (const auto& name : g::LEG_NAMES) {
    const std::string leg(name);
    expect_near(out.at(leg).foot_target, l.initialized.at(leg), leg);
  }
}

TEST(RestPoseMove, MovesAllSixLegsTogetherAlongAStraightChord) {
  // No pair sequencing and no arc: the belly is carrying the robot, so every
  // leg travels at once, and each one stays on the segment between the two
  // poses. A stray lift here would be a foot scuffing the floor it is parked
  // just above.
  const Ladder l = baked();
  auto unfold = make_unfold(l);

  std::map<std::string, bool> moved;
  for (const auto& name : g::LEG_NAMES) moved[std::string(name)] = false;

  int samples = 0;
  for (int i = 0; i < max_ticks(l) && !unfold.done(); ++i) {
    const auto out = unfold.update(kDt);
    ++samples;
    for (const auto& name : g::LEG_NAMES) {
      const std::string leg(name);
      const g::Vec3& from = l.folded.at(leg);
      const g::Vec3& to = l.initialized.at(leg);
      const g::Vec3 chord = to - from;
      const g::Vec3 travelled = out.at(leg).foot_target - from;
      // Distance off the chord: |travelled - (travelled.chord/|chord|^2) chord|.
      const float t = travelled.dot(chord) / chord.dot(chord);
      const g::Vec3 off = travelled - chord * t;
      EXPECT_LT(off.norm(), kTol) << leg << " left the chord at tick " << i;
      EXPECT_GE(t, -kTol) << leg << " ran backwards at tick " << i;
      EXPECT_LE(t, 1.0f + kTol) << leg << " overshot at tick " << i;
      if (travelled.norm() > 1e-3f) moved[leg] = true;
      // Nothing is airborne: the belly bears the load the whole way.
      EXPECT_TRUE(out.at(leg).stance) << leg << " reported airborne";
    }
  }
  ASSERT_TRUE(unfold.done());
  ASSERT_GT(samples, 2);
  for (const auto& [leg, did] : moved) {
    EXPECT_TRUE(did) << leg << " never left the folded pose";
  }
}

TEST(RestPoseMove, StartsAndEndsStationary) {
  // ease5 at both ends, for the same reason the body ramps use it: the servos
  // take up and give back the motion without a jerk step.
  const Ladder l = baked();
  auto unfold = make_unfold(l);

  std::vector<float> r;
  for (int i = 0; i < max_ticks(l); ++i) {
    const auto out = unfold.update(kDt);
    const g::Vec3 p = out.at(kProbeLeg).foot_target;
    r.push_back(std::hypot(p[0], p[1]));
    if (unfold.done()) break;
  }
  ASSERT_GE(r.size(), 3u);

  const float peak = peak_step(r);
  ASSERT_GT(peak, 0.0f);
  EXPECT_LT(std::fabs(r[1] - r[0]), 0.05f * peak)
      << "the move leaves the folded pose at "
      << 100.0f * std::fabs(r[1] - r[0]) / peak << "% of its peak speed";
  const float last = std::fabs(r[r.size() - 1] - r[r.size() - 2]);
  EXPECT_LT(last, 0.05f * peak)
      << "the move arrives at " << 100.0f * last / peak << "% of peak speed";
}

// ── InitializeController: folded -> initialized -> standing ────────────────

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

TEST(InitializeLadder, PassesThroughTheInitializedPose) {
  // Mechanically required, not cosmetic: the legs deploy to the initialized
  // pose in one belly-borne move, and only from there does a pair reach for the
  // floor. The mirror of the fold's TUCK handover.
  const Ladder l = baked();
  auto init = make_initialize(l);

  std::map<std::string, g::LegOutput> handover;
  for (int i = 0; i < max_ticks(l); ++i) {
    const auto out = init.update(kDt);
    if (init.state() == g::InitializeState::PLACE_FEET) {
      handover = out;
      break;
    }
  }
  ASSERT_FALSE(handover.empty()) << "UNFOLD never handed over to PLACE_FEET";
  for (const auto& name : g::LEG_NAMES) {
    const std::string leg(name);
    expect_near(handover.at(leg).foot_target, l.initialized.at(leg), leg);
  }
}

TEST(InitializeLadder, EveryLegLeavesTheFoldedPose) {
  // The regression that matters: a leg whose pair never ran would still be
  // sitting on its initialized-pose entry at DONE. r_front is the last pair, so
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
      const g::Vec3 d = out.at(leg).foot_target - l.folded.at(leg);
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

TEST(InitializeLadder, PlaceFeetParksEveryFootOneClearanceAboveTheFloor) {
  // Every foot at the same z before LIFT_BODY, or the body ramp drags one leg
  // along the floor. That z is init_place_clearance above the floor, not on it,
  // so no pair takes load while later pairs are still swinging; all six meet the
  // floor together on the body ramp instead.
  const Ladder l = baked();
  auto init = make_initialize(l);

  const float want_z = place_z(l);
  ASSERT_GT(l.cfg.init_place_clearance, 0.0f) << "clearance configured away";
  ASSERT_GT(want_z, contact_z()) << "place target is not above the floor";
  std::map<std::string, g::LegOutput> last;
  for (int i = 0; i < max_ticks(l); ++i) {
    const auto out = init.update(kDt);
    if (init.state() == g::InitializeState::LIFT_BODY) {
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

TEST(InitializeLadder, BodyRampStartsClearAndTakesTheFloorInItsOpening) {
  // The ramp starts place_clearance above the floor, so its ground contact is an
  // interior point, and what has to hold is where that point falls: inside the
  // opening stretch, while the septic is still accelerating and well short of its
  // peak. A contact past the peak is the regression that made the old 12 mm cold
  // start slip. This pins the shape only — the absolute contact speed is
  // init_lift_body_time against init_place_clearance, not the choice of curve.
  const Ladder l = baked();
  auto init = make_initialize(l);

  const std::vector<float> z = initialize_ramp(init, max_ticks(l));
  ASSERT_TRUE(init.done());
  ASSERT_GE(z.size(), 3u);

  EXPECT_NEAR(z.front(), place_z(l), kTol)
      << "LIFT_BODY did not start one clearance above the floor";
  EXPECT_LT(z.back(), contact_z()) << "the ramp never reached the floor";

  // The sample the ramp crosses the floor on, and its speed there.
  const float peak = peak_step(z);
  ASSERT_GT(peak, 0.0f);
  std::size_t cross = 0;
  while (cross + 1 < z.size() && z[cross] > contact_z()) ++cross;
  ASSERT_GT(cross, 0u) << "the ramp started below the floor";
  ASSERT_LT(cross, z.size() - 1) << "the ramp never crossed the floor";

  const float cross_step = std::fabs(z[cross] - z[cross - 1]);
  EXPECT_LT(cross_step, 0.5f * peak)
      << "the feet take the floor at " << 100.0f * cross_step / peak
      << "% of the ramp's peak speed";
  // Still accelerating there: the crossing is in the ramp's first half, before
  // the septic's midpoint peak.
  EXPECT_LT(cross, z.size() / 2)
      << "the floor contact fell past the middle of the ramp";
}

TEST(InitializeLadder, BodyRampIsMonotonicAndFinishesOnTime) {
  // A septic must not become a reversal, and must not lengthen the ladder —
  // the ramp itself is still exactly init_lift_body_time. Swapping ease5 for
  // ease7 changes the schedule along the travel, never the travel or its
  // duration.
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

TEST(InitializeLadder, IsTheTimeReverseOfTheFoldUpToThePlaceClearance) {
  // Rung for rung: TUCK starts where UNFOLD ends, and LOWER_BODY ends where
  // LIFT_BODY starts — same XY, same z but for the cold start's place_clearance.
  //
  // That clearance is the ladders' one asymmetry, and deliberate: the fold's feet
  // leave from where they were standing, carrying the robot until the belly takes
  // over, so there is nothing to hold clear of. Only the cold start has pairs
  // arriving one at a time with nothing yet to bear.
  const Ladder l = baked();
  auto init = make_initialize(l);
  auto fold = make_fold(l);

  std::map<std::string, g::LegOutput> init_lift_start;
  for (int i = 0; i < max_ticks(l); ++i) {
    const auto out = init.update(kDt);
    if (init.state() == g::InitializeState::LIFT_BODY) {
      init_lift_start = out;
      break;
    }
  }
  std::map<std::string, g::LegOutput> fold_lower_end;
  for (int i = 0; i < max_ticks(l); ++i) {
    const auto out = fold.update(kDt);
    if (fold.state() == g::FoldState::LIFT_FEET) {
      fold_lower_end = out;
      break;
    }
  }
  ASSERT_FALSE(init_lift_start.empty());
  ASSERT_FALSE(fold_lower_end.empty());
  for (const auto& name : g::LEG_NAMES) {
    const std::string leg(name);
    const g::Vec3& down = fold_lower_end.at(leg).foot_target;
    const g::Vec3& up = init_lift_start.at(leg).foot_target;
    // Same footprint either way round.
    EXPECT_NEAR(down[0], up[0], kTol) << leg;
    EXPECT_NEAR(down[1], up[1], kTol) << leg;
    // Same floor, offset by exactly the clearance the cold start holds.
    EXPECT_NEAR(up[2] - down[2], l.cfg.init_place_clearance, kTol) << leg;
    EXPECT_NEAR(down[2], contact_z(), kTol) << leg << " fold left the floor";
  }
}

// ── FoldController: standing -> initialized -> folded ───────────────────────

TEST(FoldLadder, EveryLegReachesTheFoldedPose) {
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
    expect_near(out.at(leg).foot_target, l.folded.at(leg), leg);
  }
}

TEST(FoldLadder, PassesThroughTheInitializedPose) {
  // Mechanically required, not cosmetic: the pairs come off the floor to the
  // initialized pose and only then draw in to folded. Going straight to folded
  // would take a leg through the tuck while its own pair is the only thing off
  // the ground.
  const Ladder l = baked();
  auto fold = make_fold(l);

  std::map<std::string, g::LegOutput> handover;
  for (int i = 0; i < max_ticks(l); ++i) {
    const auto out = fold.update(kDt);
    if (fold.state() == g::FoldState::TUCK) {
      handover = out;
      break;
    }
  }
  ASSERT_FALSE(handover.empty()) << "LIFT_FEET never handed over to TUCK";
  for (const auto& name : g::LEG_NAMES) {
    const std::string leg(name);
    expect_near(handover.at(leg).foot_target, l.initialized.at(leg), leg);
  }
}

TEST(FoldLadder, LiftsPairsInReversePairOrder) {
  // Fold lifts feet in the mirror of the order a reseat plants them, so the
  // last leg down is the first leg back up.
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

TEST(FoldLadder, OnlyOnePairIsAirborneAtATime) {
  // The other four hold the body up until the belly is down. This is the
  // opposite requirement to the cold start's all-six landing, and the reason
  // the two directions cannot share a ladder.
  const Ladder l = baked();
  auto fold = make_fold(l);

  bool saw_swing = false;
  for (int i = 0; i < max_ticks(l) && !fold.done(); ++i) {
    const auto out = fold.update(kDt);
    if (fold.state() != g::FoldState::LIFT_FEET) continue;
    int swinging = 0;
    for (const auto& name : g::LEG_NAMES) {
      if (!out.at(std::string(name)).stance) ++swinging;
    }
    EXPECT_LE(swinging, 2) << "more than one pair airborne at tick " << i;
    if (swinging > 0) saw_swing = true;
  }
  EXPECT_TRUE(saw_swing) << "LIFT_FEET never lifted a foot";
}

TEST(FoldLadder, BodyRampMeetsTheFloorStationaryAtItsEnd) {
  // The belly arrives exactly as LOWER_BODY runs out, so the ramp is already
  // stopped when it lands rather than still descending into it. The fold used
  // to overshoot the contact and put the landing mid-ramp.
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
