// Folded <-> standing body transitions, via the initialized pose in between. The
// two controllers are exact time-reverses of each other, rung for rung, and
// share the same pair-swing + eased-ramp machinery:
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
#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>

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
  // Built when LIFT_FEET hands over, from the pose the pairs left the legs in.
  std::optional<RestPoseMove> tuck_;
  FoldState state_ = FoldState::LOWER_BODY;
  std::size_t pair_idx_ = 0;
  float t_in_pair_ = 0.0f;
  float t_in_lower_ = 0.0f;
};

}  // namespace hexa::gait
