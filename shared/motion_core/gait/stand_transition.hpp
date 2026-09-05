// Folded <-> standing body transitions, via the initialized pose in between. The
// two controllers are exact time-reverses of each other, rung for rung, and
// share the same pair-swing + eased-ramp machinery. Both take the leg set they
// are standing up (or folding) — the same ladder either way, minus the middle
// pair, which quadruped mode leaves folded where it already is:
//
//   InitializeController (cold start): UNFOLD out to the initialized pose,
//   PLACE_FEET swings three mirroring pairs onto the standing footprint while
//   the belly rests, stopping place_clearance short of the floor, LIFT_BODY
//   ramps the body-frame z.
//
//   FoldController (warm shutdown): LOWER_BODY down onto the belly, LIFT_FEET
//   the same three pairs in reverse, TUCK the unfold run backwards.
//
// The pairs are what makes the middle rung safe both ways: while the feet carry
// the body, only one mirrored pair is ever off the ground.
//
//   PairFoldController (leg-set change): the middle pair alone, between the
//   ground and the folded pose, with the body standing on the four corners. Not
//   a ladder — one rung, and that rung is PAIR_ORDER's own first one.
#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "gait/gaits/base.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

// Three sequential mirroring pairs, ordered to keep the CoM near the chassis
// centre: middle pair first, then each diagonal. Reused by reseat.cpp.
inline const std::array<std::array<std::string, 2>, 3> PAIR_ORDER = {{
    {"l_middle", "r_middle"},
    {"l_front", "r_rear"},
    {"r_front", "l_rear"},
}};

// The same diagonals in the same order, with the middle pair dropped: the rungs
// quadruped mode's ladders climb. It must never name a middle — that pair is
// folded and has no ground to re-plant on.
inline const std::array<std::array<std::string, 2>, 2> QUAD_PAIR_ORDER = {{
    {"l_front", "r_rear"},
    {"r_front", "l_rear"},
}};

// One rung of a ladder: the legs it lifts together. A runtime value, since the
// tables differ in both length and rung size. Its flattened legs are also the
// set the ladder manages.
using Rung = std::vector<std::string>;
using RungList = std::vector<Rung>;

// Mirrored pairs. Safe wherever the legs outside the rung still make a support
// polygon around the body — six feet lifting two, or (on the belly, where the
// stand ladders swing them) four.
inline RungList pair_list(LegSet set) {
  RungList out;
  if (set == LegSet::QUADRUPED) {
    for (const auto& p : QUAD_PAIR_ORDER) {
      out.push_back({p[0], p[1]});
    }
    return out;
  }
  for (const auto& p : PAIR_ORDER) {
    out.push_back({p[0], p[1]});
  }
  return out;
}

// Quadruped mode's middle pair never leaves the folded pose: it powers up there
// and both ladders drive the four corners around it. Applied to whatever a rung
// emitted, so no rung has to know which legs it is not moving.
inline void pin_parked(LegSet set, const std::map<std::string, Vec3>& folded,
                       std::map<std::string, LegOutput>& out) {
  if (set != LegSet::QUADRUPED) {
    return;
  }
  for (const auto& name : PARKED_LEGS) {
    out[name] = LegOutput{folded.at(name), 0.0f, false, true};
  }
}

// The reseat ladder's rungs, which are the whole support while it runs. On four
// feet a mirrored pair is half of that, so the quadruped set goes one leg at a
// time, around the perimeter. The body stands at the centroid of the three that
// are down, opposite the one that is up, so adjacent corners hand it a quarter
// turn where a diagonal step would send it across the middle.
inline RungList reseat_rungs(LegSet set) {
  if (set == LegSet::QUADRUPED) {
    return RungList{{"l_rear"}, {"l_front"}, {"r_front"}, {"r_rear"}};
  }
  return pair_list(set);
}

// Eased ramp for the fold and the rest-pose moves. The fold's LOWER_BODY ends on
// the belly arriving, so the ramp's own endpoint is where it has to be
// stationary — ease5 is, in acceleration as well as velocity, so the legs give
// the weight up without a jerk step.
inline float eased_ramp(float tau) {
  if (tau <= 0.0f) {
    return 0.0f;
  }
  if (tau >= 1.0f) {
    return 1.0f;
  }
  return ease5(tau);
}

// Body-lift ramp for the cold start's LIFT_BODY, and the one place the two
// ladders are not each other's mirror: PLACE_FEET stops place_clearance short of
// the floor, so the contact falls *inside* the travel rather than at an end.
// ease7 opens more slowly, so the contact lands at a smaller share of the ramp's
// peak speed — a smaller share, not a slower contact, which lift_body_time
// against place_clearance sets.
inline float lift_ramp(float tau) {
  if (tau <= 0.0f) {
    return 0.0f;
  }
  if (tau >= 1.0f) {
    return 1.0f;
  }
  return ease7(tau);
}

// One eased move of all six feet at once, along the chord between the two
// belly-rest poses; the unfold and the tuck are this class constructed either
// way round. Both ends are airborne and the belly carries the robot throughout,
// so there is nothing to sequence and nothing to land. Feeding it a pose whose
// feet are on the ground is a caller error.
class RestPoseMove {
 public:
  RestPoseMove(std::map<std::string, Vec3> from_stance,
               std::map<std::string, Vec3> to_stance, float move_time);

  bool done() const { return done_; }

  std::map<std::string, LegOutput> update(float dt);

 private:
  std::map<std::string, Vec3> from_;
  std::map<std::string, Vec3> to_;
  float move_time_;

  float t_ = 0.0f;
  bool done_ = false;
};

// Which way the pair is going. FOLD lifts it off the ground to the folded pose
// (the far half of a hexapod -> quadruped change); UNFOLD brings it back down.
enum class PairFoldDirection { FOLD, UNFOLD };

// UNFOLD alone has a SET_DOWN: the move runs at zero clearance, which forgoes
// the swing profile's own probe, so the last few millimetres are their own
// segment rather than the arc's tail.
enum class PairFoldState { DWELL, MOVE, SET_DOWN, DONE };

// The middle pair between the folded pose and the ground, moved while the body
// stands on the four corners. The other half of a leg-set change; the reseat
// does the corners.
//
// Both middles move together, for the reason PAIR_ORDER's own first rung gives:
// the four corners hold the body either way and the two reactions cancel. The
// path is a single eased chord with no waypoint, because standing (unlike on the
// belly, where the initialized pose has to deploy the legs before they can swing
// down past a chassis on the floor) it is a near-vertical line well outboard of
// the corner feet, with every joint triple along it inside its limits.
//
// It emits all six legs every tick. The four corners come from the stance it was
// handed, planted and un-parked; the caller owns nothing here.
//
// The pair is emitted `parked = false` throughout, even at the folded end: a
// parked leg's joint angles bypass the body pose, an unparked one's do not, so
// the two only agree at a neutral pose. The caller is responsible for having
// reverted the operator's pose before building this — which is also what makes
// the handover at either end exact, since IK round-trips the folded pose.
class PairFoldController {
 public:
  // `held_stance` is where all six legs stand right now. `folded_stance` and
  // `nominal_stance` are the pair's two endpoints; which is the origin and which
  // the target follows from `direction`.
  //
  // The UNFOLD origin comes from `folded_stance`, never from `held_stance`: on a
  // quadruped stand the caller's last targets carry the ground placeholder for a
  // middle, not the pose its foot is actually at. The mode's own invariant — the
  // pair does not leave the folded pose — is the only trustworthy source.
  //
  // `swing` must have zero clearance; see EngineConfig::pair_fold_profile for
  // why that is a fact about the folded pose and not a tuning choice.
  // `probe_band` is how far above its target the unfold hands over to the braked
  // descent, which runs at the profile's own touchdown_velocity.
  PairFoldController(PairFoldDirection direction,
                     std::map<std::string, Vec3> held_stance,
                     std::map<std::string, Vec3> folded_stance,
                     std::map<std::string, Vec3> nominal_stance,
                     float swing_time, float dwell_time, float probe_band,
                     const SwingProfile& swing, float controller_dt);

  PairFoldDirection direction() const { return direction_; }
  PairFoldState state() const { return state_; }
  bool done() const { return state_ == PairFoldState::DONE; }

  std::map<std::string, LegOutput> update(float dt);

 private:
  std::map<std::string, LegOutput> emit(float pair_phase,
                                        bool pair_stance) const;

  PairFoldDirection direction_;
  std::map<std::string, Vec3> held_;
  // The pair's three waypoints: where it starts, where the chord ends, and where
  // it finishes. The middle two differ only on an UNFOLD, by probe_band.
  std::map<std::string, Vec3> origin_;
  std::map<std::string, Vec3> chord_end_;
  std::map<std::string, Vec3> final_;
  float swing_time_;
  float dwell_time_;
  // probe_band / touchdown_velocity, or 0 when there is no SET_DOWN to run.
  float set_down_time_ = 0.0f;
  SwingProfile swing_;
  float controller_dt_;

  std::map<std::string, Vec3> pair_pos_;
  PairFoldState state_ = PairFoldState::DWELL;
  float t_ = 0.0f;
};

enum class InitializeState { UNFOLD, PLACE_FEET, LIFT_BODY, DONE };

class InitializeController {
 public:
  InitializeController(LegSet leg_set,
                       std::map<std::string, Vec3> folded_stance,
                       std::map<std::string, Vec3> initialized_stance,
                       std::map<std::string, Vec3> nominal_stance,
                       float coxa_to_bottom, float foot_radius,
                       float pair_swing_time, float lift_body_time,
                       float unfold_time, float place_clearance,
                       float swing_clearance, float swing_width,
                       float touchdown_velocity,
                       float touchdown_probe_fraction, float controller_dt);

  InitializeState state() const { return state_; }
  bool done() const { return state_ == InitializeState::DONE; }

  std::map<std::string, LegOutput> update(float dt);

 private:
  std::map<std::string, LegOutput> tick_unfold(float dt);
  std::map<std::string, LegOutput> tick_place_feet(float dt);
  std::map<std::string, LegOutput> tick_lift_body(float dt);
  std::map<std::string, LegOutput> emit_nominal() const;

  LegSet leg_set_;
  RungList rungs_;
  std::map<std::string, Vec3> folded_;
  std::map<std::string, Vec3> initialized_;
  std::map<std::string, Vec3> nominal_;
  // Where PLACE_FEET parks the feet: place_clearance above the floor, not on it.
  float lift_start_z_;
  std::map<std::string, Vec3> ground_targets_;
  float pair_swing_time_;
  float lift_body_time_;
  SwingProfile swing_;
  float controller_dt_;

  RestPoseMove unfold_;
  std::map<std::string, Vec3> positions_;
  InitializeState state_ = InitializeState::UNFOLD;
  std::size_t pair_idx_ = 0;
  float t_in_pair_ = 0.0f;
  float t_in_lift_ = 0.0f;
};

enum class FoldState { LOWER_BODY, LIFT_FEET, TUCK, DONE };

class FoldController {
 public:
  FoldController(LegSet leg_set, std::map<std::string, Vec3> folded_stance,
                 std::map<std::string, Vec3> initialized_stance,
                 std::map<std::string, Vec3> nominal_stance,
                 float coxa_to_bottom, float foot_radius,
                 float pair_swing_time, float lift_body_time, float tuck_time,
                 float swing_clearance, float swing_width,
                 float touchdown_velocity, float touchdown_probe_fraction,
                 float controller_dt);

  FoldState state() const { return state_; }
  bool done() const { return state_ == FoldState::DONE; }

  std::map<std::string, LegOutput> update(float dt);

 private:
  std::map<std::string, LegOutput> tick_lower_body(float dt);
  std::map<std::string, LegOutput> tick_lift_feet(float dt);
  std::map<std::string, LegOutput> tick_tuck(float dt);
  std::map<std::string, LegOutput> emit_folded() const;

  LegSet leg_set_;
  RungList rungs_;
  std::map<std::string, Vec3> folded_;
  std::map<std::string, Vec3> initialized_;
  std::map<std::string, Vec3> nominal_;
  float lower_end_z_;
  std::map<std::string, Vec3> ground_targets_;
  float pair_swing_time_;
  float lift_body_time_;
  float tuck_time_;
  SwingProfile swing_;
  float controller_dt_;

  std::map<std::string, Vec3> positions_;
  // Built when LIFT_FEET hands over, from the pose the pairs left the legs in.
  std::optional<RestPoseMove> tuck_;
  FoldState state_ = FoldState::LOWER_BODY;
  std::size_t pair_idx_ = 0;
  float t_in_pair_ = 0.0f;
  float t_in_lower_ = 0.0f;
};

}  // namespace hexa::gait
