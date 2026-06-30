// Cold-start initialization: initial_pose -> standing. Port of initialize.py.
//
// PLACE_FEET swings three sequential mirroring pairs onto the standing
// footprint while the body rests on its belly; LIFT_BODY ramps the body-frame z
// via a smoothstep S-curve; DONE emits nominal_stance. PAIR_ORDER and
// smoothstep are shared with fold.py's FoldController.
#pragma once

#include <array>
#include <map>
#include <string>

#include "hexa_gait_cpp/leg_output.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

// Three sequential mirroring pairs, ordered to keep the CoM near the chassis
// centre while it rests on its belly: middle pair first, then each diagonal.
inline const std::array<std::array<std::string, 2>, 3> PAIR_ORDER = {{
    {"l_middle", "r_middle"},
    {"l_front", "r_rear"},
    {"r_front", "l_rear"},
}};

// Hermite smoothstep 3t^2 - 2t^3 on [0, 1]. Shared envelope across the
// cold-start transients (initialize, fold) and the engagement controller.
inline double smoothstep(double t) {
  if (t <= 0.0) {
    return 0.0;
  }
  if (t >= 1.0) {
    return 1.0;
  }
  return t * t * (3.0 - 2.0 * t);
}

enum class InitializeState { PLACE_FEET, LIFT_BODY, DONE };

class InitializeController {
 public:
  InitializeController(std::map<std::string, Vec3> initial_stance,
                       std::map<std::string, Vec3> nominal_stance,
                       double coxa_to_bottom, double pair_swing_time,
                       double lift_body_time, double swing_clearance,
                       double place_feet_clearance, double swing_width,
                       double controller_dt);

  InitializeState state() const { return state_; }
  bool done() const { return state_ == InitializeState::DONE; }

  std::map<std::string, LegOutput> update(double dt);

 private:
  std::map<std::string, LegOutput> tick_place_feet(double dt);
  std::map<std::string, LegOutput> tick_lift_body(double dt);
  std::map<std::string, LegOutput> emit_nominal() const;

  std::map<std::string, Vec3> initial_;
  std::map<std::string, Vec3> nominal_;
  double lift_start_z_;
  std::map<std::string, Vec3> ground_targets_;
  double coxa_to_bottom_;
  double place_feet_clearance_;
  double pair_swing_time_;
  double lift_body_time_;
  double swing_clearance_;
  double swing_width_;
  double controller_dt_;

  std::map<std::string, Vec3> positions_;
  InitializeState state_ = InitializeState::PLACE_FEET;
  std::size_t pair_idx_ = 0;
  double t_in_pair_ = 0.0;
  double t_in_lift_ = 0.0;
};

}  // namespace hexa_gait
