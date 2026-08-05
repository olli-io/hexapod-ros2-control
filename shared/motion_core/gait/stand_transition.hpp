// Folded <-> standing body transitions. Float fork of stand_transition.hpp
// (plan part 06). Ports initialize.py and fold.py.
//
// The two controllers are exact time-reverses and share the same pair-swing +
// eased-ramp machinery, so they live together:
//
//   InitializeController (cold start, initial_pose -> standing): PLACE_FEET
//   swings three sequential mirroring pairs onto the standing footprint while
//   the body rests on its belly; LIFT_BODY ramps the body-frame z via
//   eased_ramp; DONE emits nominal_stance.
//
//   FoldController (warm shutdown, standing -> initial_pose): LOWER_BODY ramps
//   the body-frame z up to the belly height, then LIFT_FEET swings the same
//   three pairs in reverse back to the folded initial_pose; DONE emits
//   initial_stance.
//
// PAIR_ORDER is also reused by reseat.cpp.
#pragma once

#include <array>
#include <map>
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

// Body-z ramp for the two ladders: a quintic across the whole travel.
//
// Both ends of this ramp are a ground contact — the feet already planted at the
// start of a cold-start LIFT_BODY, the belly arriving at the end of a fold's
// LOWER_BODY — which is what makes the ramp's own endpoints the right place to
// be stationary. ease5 is stationary there in acceleration as well as velocity,
// so the legs take up the body's weight without a jerk step. A cubic
// smoothstep, which this replaced, only vanishes in velocity.
inline float eased_ramp(float tau) {
  if (tau <= 0.0f) {
    return 0.0f;
  }
  if (tau >= 1.0f) {
    return 1.0f;
  }
  return ease5(tau);
}

enum class InitializeState { PLACE_FEET, LIFT_BODY, DONE };

class InitializeController {
 public:
  InitializeController(std::map<std::string, Vec3> initial_stance,
                       std::map<std::string, Vec3> nominal_stance,
                       float coxa_to_bottom, float foot_radius,
                       float pair_swing_time, float lift_body_time,
                       float swing_clearance, float swing_width,
                       float controller_dt);

  InitializeState state() const { return state_; }
  bool done() const { return state_ == InitializeState::DONE; }

  std::map<std::string, LegOutput> update(float dt);

 private:
  std::map<std::string, LegOutput> tick_place_feet(float dt);
  std::map<std::string, LegOutput> tick_lift_body(float dt);
  std::map<std::string, LegOutput> emit_nominal() const;

  std::map<std::string, Vec3> initial_;
  std::map<std::string, Vec3> nominal_;
  float lift_start_z_;
  std::map<std::string, Vec3> ground_targets_;
  float pair_swing_time_;
  float lift_body_time_;
  float swing_clearance_;
  float swing_width_;
  float controller_dt_;

  std::map<std::string, Vec3> positions_;
  InitializeState state_ = InitializeState::PLACE_FEET;
  std::size_t pair_idx_ = 0;
  float t_in_pair_ = 0.0f;
  float t_in_lift_ = 0.0f;
};

enum class FoldState { LOWER_BODY, LIFT_FEET, DONE };

class FoldController {
 public:
  FoldController(std::map<std::string, Vec3> initial_stance,
                 std::map<std::string, Vec3> nominal_stance,
                 float coxa_to_bottom, float foot_radius,
                 float pair_swing_time, float lift_body_time,
                 float swing_clearance, float swing_width,
                 float controller_dt);

  FoldState state() const { return state_; }
  bool done() const { return state_ == FoldState::DONE; }

  std::map<std::string, LegOutput> update(float dt);

 private:
  std::map<std::string, LegOutput> tick_lower_body(float dt);
  std::map<std::string, LegOutput> tick_lift_feet(float dt);
  std::map<std::string, LegOutput> emit_initial() const;

  std::map<std::string, Vec3> initial_;
  std::map<std::string, Vec3> nominal_;
  float lower_end_z_;
  std::map<std::string, Vec3> ground_targets_;
  float pair_swing_time_;
  float lift_body_time_;
  float swing_clearance_;
  float swing_width_;
  float controller_dt_;

  std::map<std::string, Vec3> positions_;
  FoldState state_ = FoldState::LOWER_BODY;
  std::size_t pair_idx_ = 0;
  float t_in_pair_ = 0.0f;
  float t_in_lower_ = 0.0f;
};

}  // namespace hexa::gait
