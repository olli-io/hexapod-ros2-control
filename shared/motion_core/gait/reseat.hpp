// Reseat ladder: arbitrary current foot positions -> a target stance. Float fork
// of reseat.hpp (plan part 06). Used by two engine paths (posture-height change,
// paused->standing cleanup) with the same ladder mechanics. The pair order
// mirrors InitializeController.PLACE_FEET.
#pragma once

#include <array>
#include <map>
#include <string>

#include "gait/gaits/base.hpp"
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

// One ReseatGeometry per leg, indexed by Leg (leg_index.hpp order). Legs in
// different standing-pose groups reach out different distances, so they lean
// their tibias differently and cannot share a snapshot.
using ReseatGeometryByLeg = std::array<ReseatGeometry, kNumLegs>;

// Derive the reseat geometry from a standing-pose joint-angle triple
// (theta_coxa, theta_femur, theta_tibia) and a leg's segment lengths (FK).
// theta_coxa is deliberately unused: depth and tibia lean are both invariant
// under the leg's swivel.
ReseatGeometry default_geometry_from_pose(const JointAngles& standing_angles,
                                          const kin::LegSpec& leg_spec);

// Body-frame nominal stance per leg at a target body height. Each leg keeps the
// swivel it currently stands at — current_stance supplies the azimuth, and only
// the radius and depth move — so the standing splay survives a height change.
// The radius is re-solved per leg from that leg's own geometry, so a stance whose
// groups reach out different distances keeps those distances distinct.
// Throws std::invalid_argument if target_height_m is outside the geometrically
// feasible range for any leg (arcsin argument leaves [-1, 1]).
std::map<std::string, Vec3> reseat_nominal_stance(
    float target_height_m, const ReseatGeometryByLeg& geometry,
    const std::map<std::string, kin::LegSpec>& leg_specs,
    const std::map<std::string, Vec3>& current_stance);

class ReseatController {
 public:
  // swing shapes the pair's arc exactly as it shapes a gait swing; its width is
  // ignored (a reseat is direction-agnostic, so the arc stays a vertical lift
  // over a straight chord).
  //
  // A pair already standing on its targets is skipped outright, costing neither
  // a swing nor a dwell: after a settle hands its leftovers over, some legs are
  // already home, and lifting a foot to put it back where it is reads as a stray
  // step. A height change moves all six, so nothing is skipped there.
  ReseatController(std::map<std::string, Vec3> current_stance,
                   std::map<std::string, Vec3> target_stance,
                   float pair_swing_time, float pair_dwell_time,
                   const SwingProfile& swing, float controller_dt);

  bool done() const { return done_; }
  std::map<std::string, LegOutput> update(float dt);

 private:
  // Advance past any pair that is already on its targets, then latch the origins
  // of the one that is left. Sets done_ if none is.
  void seed_pair_origin();
  bool pair_needs_moving(std::size_t idx) const;
  bool remaining_pair_needs_moving() const;

  std::map<std::string, Vec3> target_;
  float pair_swing_time_;
  float pair_dwell_time_;
  SwingProfile swing_;
  float controller_dt_;

  std::map<std::string, Vec3> positions_;
  std::map<std::string, Vec3> pair_origin_;
  std::size_t pair_idx_ = 0;
  float t_in_pair_ = 0.0f;
  float dwell_remaining_ = 0.0f;
  bool done_ = false;
};

}  // namespace hexa::gait
