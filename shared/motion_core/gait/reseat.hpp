// Reseat ladder: arbitrary current foot positions -> a target stance. Float fork
// of reseat.hpp (plan part 06). Used by two engine paths (posture-height change,
// paused->standing cleanup) with the same ladder mechanics. The pair order
// mirrors InitializeController.PLACE_FEET.
#pragma once

#include <map>
#include <string>

#include "gait/kinematics.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

// Frozen snapshot of the standing pose's geometry. Captures the tibia-from-
// vertical lean and foot depth so every reseat target follows from a single
// target_height scalar.
struct ReseatGeometry {
  float coxa_len = 0.0f;
  float femur_len = 0.0f;
  float tibia_len = 0.0f;
  float tibia_from_vertical = 0.0f;
  float default_foot_depth = 0.0f;
};

// Derive the reseat geometry from a standing-pose joint-angle triple
// (theta_coxa, theta_femur, theta_tibia) and a leg's segment lengths (FK).
ReseatGeometry default_geometry_from_pose(const JointAngles& standing_angles,
                                          const kin::LegSpec& leg_spec);

// Body-frame nominal stance per leg at a target body height. Throws
// std::invalid_argument if target_height_m is outside the geometrically feasible
// range (arcsin argument leaves [-1, 1]).
std::map<std::string, Vec3> reseat_nominal_stance(
    float target_height_m, const ReseatGeometry& geometry,
    const std::map<std::string, kin::LegSpec>& leg_specs);

class ReseatController {
 public:
  ReseatController(std::map<std::string, Vec3> current_stance,
                   std::map<std::string, Vec3> target_stance,
                   float pair_swing_time, float pair_dwell_time,
                   float swing_clearance, float controller_dt);

  bool done() const { return done_; }
  std::map<std::string, LegOutput> update(float dt);

 private:
  void seed_pair_origin();

  std::map<std::string, Vec3> target_;
  float pair_swing_time_;
  float pair_dwell_time_;
  float swing_clearance_;
  float controller_dt_;

  std::map<std::string, Vec3> positions_;
  std::map<std::string, Vec3> pair_origin_;
  std::size_t pair_idx_ = 0;
  float t_in_pair_ = 0.0f;
  float dwell_remaining_ = 0.0f;
  bool done_ = false;
};

}  // namespace hexa::gait
