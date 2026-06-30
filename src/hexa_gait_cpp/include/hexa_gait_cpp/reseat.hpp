// Reseat ladder: arbitrary current foot positions -> a target stance. Port of
// reseat.py. Used by two engine paths (posture-height change, paused->standing
// cleanup) with the same ladder mechanics. The pair order mirrors
// InitializeController.PLACE_FEET.
#pragma once

#include <map>
#include <string>

#include "hexa_gait_cpp/kinematics_stub.hpp"
#include "hexa_gait_cpp/leg_output.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

// Frozen snapshot of the YAML default standing pose's geometry. Captures the
// tibia-from-vertical lean and foot depth so every reseat target follows from a
// single target_height scalar.
struct ReseatGeometry {
  double coxa_len = 0.0;
  double femur_len = 0.0;
  double tibia_len = 0.0;
  double tibia_from_vertical = 0.0;
  double default_foot_depth = 0.0;
};

// Derive the reseat geometry from a standing-pose joint-angle triple
// (theta_coxa, theta_femur, theta_tibia) and a leg's segment lengths (FK).
ReseatGeometry default_geometry_from_pose(const JointAngles& standing_angles,
                                          const kin::LegSpec& leg_spec);

// Body-frame nominal stance per leg at a target body height. Throws
// std::invalid_argument if target_height_m is outside the geometrically
// feasible range (arcsin argument leaves [-1, 1]).
std::map<std::string, Vec3> reseat_nominal_stance(
    double target_height_m, const ReseatGeometry& geometry,
    const std::map<std::string, kin::LegSpec>& leg_specs);

class ReseatController {
 public:
  ReseatController(std::map<std::string, Vec3> current_stance,
                   std::map<std::string, Vec3> target_stance,
                   double pair_swing_time, double pair_dwell_time,
                   double swing_clearance, double controller_dt);

  bool done() const { return done_; }
  std::map<std::string, LegOutput> update(double dt);

 private:
  void seed_pair_origin();

  std::map<std::string, Vec3> target_;
  double pair_swing_time_;
  double pair_dwell_time_;
  double swing_clearance_;
  double controller_dt_;

  std::map<std::string, Vec3> positions_;
  std::map<std::string, Vec3> pair_origin_;
  std::size_t pair_idx_ = 0;
  double t_in_pair_ = 0.0;
  double dwell_remaining_ = 0.0;
  bool done_ = false;
};

}  // namespace hexa_gait
