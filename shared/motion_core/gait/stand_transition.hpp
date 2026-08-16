// Folded <-> standing body transitions. Float fork of stand_transition.hpp
// (plan part 06). Ports initialize.py and fold.py.
//
// Neither direction goes between the folded pose and standing in one step: the
// initialized pose sits between them both ways. The two controllers are exact
// time-reverses of each other, rung for rung, and share the same pair-swing +
// eased-ramp machinery, so they live together:
//
//   InitializeController (cold start, folded -> initialized -> standing):
//   UNFOLD is a RestPoseMove out to the initialized pose; PLACE_FEET swings
//   three sequential mirroring pairs onto the standing footprint while the body
//   rests on its belly, stopping place_clearance short of the floor; LIFT_BODY
//   ramps the body-frame z via lift_ramp, taking the floor up in its opening
//   stretch; DONE emits nominal_stance.
//
//   FoldController (warm shutdown, standing -> initialized -> folded):
//   LOWER_BODY ramps the body-frame z up to the belly height, LIFT_FEET swings
//   the same three pairs in reverse back to the initialized pose, and TUCK is
//   the unfold's RestPoseMove run backwards; DONE emits folded_stance.
//
// The pairs are what makes the middle rung safe in both directions: until the
// belly is down (or before it comes up) the feet carry the body, so only one
// mirrored pair is ever off the ground.
//
// PAIR_ORDER is also reused by reseat.cpp.
#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>

#include "gait/gaits/base.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

// Three sequential mirroring pairs, ordered to keep the CoM near the chassis
// centre while it rests on its belly: middle pair first, then each diagonal.
inline const std::array<std::array<std::string, 2>, 3> PAIR_ORDER = {{
    {"l_middle", "r_middle"},
    {"l_front", "r_rear"},
    {"r_front", "l_rear"},
}};

// Eased ramp for the fold and the rest-pose moves: a quintic across the whole
// travel.
//
// The fold's LOWER_BODY ends on a ground contact — the belly arriving — which is
// what makes the ramp's own endpoint the right place to be stationary. ease5 is
// stationary there in acceleration as well as velocity, so the legs give the
// body's weight up without a jerk step. A cubic smoothstep, which this replaced,
// only vanishes in velocity. RestPoseMove reuses it for the same reason at both
// of its ends, neither of which touches the ground.
inline float eased_ramp(float tau) {
  if (tau <= 0.0f) {
    return 0.0f;
  }
  if (tau >= 1.0f) {
    return 1.0f;
  }
  return ease5(tau);
}

// Body-lift ramp for the cold start's LIFT_BODY: a septic across the same
// travel, and the one place the two ladders are not each other's mirror.
//
// The fold's ramp has its ground contact at an *end*, where any ease is
// stationary. This one no longer does: PLACE_FEET now stops the feet
// place_clearance short of the floor, so the six of them take the robot's
// weight a short way *into* the travel, off both endpoints. ease7 is the better
// curve for an interior contact — its third derivative vanishes at the ends
// too, so the opening stays in acceleration longer and the contact lands at a
// smaller share of the ramp's peak speed.
//
// A smaller share, not a slow contact. Covering the same travel in the same
// time gives the septic a higher peak than the quintic, so the two roughly
// cancel in absolute terms: what actually sets the contact speed is
// lift_body_time against place_clearance, and at the configured pair the feet
// still meet the floor several times faster than a gait swing lands.
inline float lift_ramp(float tau) {
  if (tau <= 0.0f) {
    return 0.0f;
  }
  if (tau >= 1.0f) {
    return 1.0f;
  }
  return ease7(tau);
}

// One eased move of all six feet at once, straight along the chord between the
// two belly-rest poses. The **unfold** (folded -> initialized) and the **tuck**
// (initialized -> folded) are the same move in opposite directions, so they are
// the same class constructed either way round.
//
// Both ends are airborne and the belly carries the robot the whole way, so
// there is nothing to sequence, nothing to land, and no arc to climb: a pair
// ladder here would only make the robot look hesitant. Feeding it a pose whose
// feet are on the ground is a caller error — nothing in here watches for
// contact.
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

enum class InitializeState { UNFOLD, PLACE_FEET, LIFT_BODY, DONE };

class InitializeController {
 public:
  InitializeController(std::map<std::string, Vec3> folded_stance,
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

  std::map<std::string, Vec3> initialized_;
  std::map<std::string, Vec3> nominal_;
  // Where PLACE_FEET parks the feet and LIFT_BODY starts: place_clearance above
  // the floor, not on it.
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
  FoldController(std::map<std::string, Vec3> folded_stance,
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
  // Built when LIFT_FEET hands over, so it starts from the pose the pairs
  // actually left the legs in.
  std::optional<RestPoseMove> tuck_;
  FoldState state_ = FoldState::LOWER_BODY;
  std::size_t pair_idx_ = 0;
  float t_in_pair_ = 0.0f;
  float t_in_lower_ = 0.0f;
};

}  // namespace hexa::gait
