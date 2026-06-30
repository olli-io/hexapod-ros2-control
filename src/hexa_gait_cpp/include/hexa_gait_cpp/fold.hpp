// Warm shutdown: standing -> initial_pose. Port of fold.py. Time-reverse of
// InitializeController: LOWER_BODY ramps the body-frame z up to the belly
// height, then LIFT_FEET swings three pairs (reverse PAIR_ORDER) back to the
// folded initial_pose; DONE emits initial_stance.
#pragma once

#include <map>
#include <string>

#include "hexa_gait_cpp/leg_output.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

enum class FoldState { LOWER_BODY, LIFT_FEET, DONE };

class FoldController {
 public:
  FoldController(std::map<std::string, Vec3> initial_stance,
                 std::map<std::string, Vec3> nominal_stance,
                 double coxa_to_bottom, double pair_swing_time,
                 double lift_body_time, double swing_clearance,
                 double place_feet_clearance, double swing_width,
                 double controller_dt);

  FoldState state() const { return state_; }
  bool done() const { return state_ == FoldState::DONE; }

  std::map<std::string, LegOutput> update(double dt);

 private:
  std::map<std::string, LegOutput> tick_lower_body(double dt);
  std::map<std::string, LegOutput> tick_lift_feet(double dt);
  std::map<std::string, LegOutput> emit_initial() const;

  std::map<std::string, Vec3> initial_;
  std::map<std::string, Vec3> nominal_;
  double lower_end_z_;
  std::map<std::string, Vec3> ground_targets_;
  double pair_swing_time_;
  double lift_body_time_;
  double swing_clearance_;
  double swing_width_;
  double controller_dt_;

  std::map<std::string, Vec3> positions_;
  FoldState state_ = FoldState::LOWER_BODY;
  std::size_t pair_idx_ = 0;
  double t_in_pair_ = 0.0;
  double t_in_lower_ = 0.0;
};

}  // namespace hexa_gait
