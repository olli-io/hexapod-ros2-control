// Reseat ladder: arbitrary current foot positions -> a target stance. Three
// callers (posture-height change, settle cleanup, abandoned engagement) share
// the mechanics. The pair order is FoldController's, reversed.
//
// An abandoned engagement hands over feet still in the air, so the ladder lands
// those before it lifts any foot that is down.
#pragma once

#include <array>
#include <map>
#include <string>

#include "gait/gaits/base.hpp"
#include "gait/kinematics.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

// Frozen snapshot of the standing pose, so every reseat target follows from a
// single target_height scalar.
struct ReseatGeometry {
  float coxa_len = 0.0f;
  float femur_len = 0.0f;
  float tibia_len = 0.0f;
  float tibia_from_vertical = 0.0f;
  float default_foot_depth = 0.0f;
};

// Indexed by Leg. Legs in different standing-pose groups lean their tibias
// differently, so they cannot share a snapshot.
using ReseatGeometryByLeg = std::array<ReseatGeometry, kNumLegs>;

// FK from a standing-pose joint-angle triple. theta_coxa is deliberately
// unused: depth and tibia lean are invariant under the leg's swivel.
ReseatGeometry default_geometry_from_pose(const JointAngles& standing_angles,
                                          const kin::LegSpec& leg_spec);

// Body-frame nominal stance per leg at a target body height. Each leg keeps the
// swivel it currently stands at and re-solves its radius from its own geometry,
// so both the splay and the per-group reach survive a height change. Throws
// std::invalid_argument if target_height_m is infeasible for any leg.
std::map<std::string, Vec3> reseat_nominal_stance(
    float target_height_m, const ReseatGeometryByLeg& geometry,
    const std::map<std::string, kin::LegSpec>& leg_specs,
    const std::map<std::string, Vec3>& current_stance);

class ReseatController {
 public:
  // swing shapes the pair's arc as it shapes a gait swing; its width is ignored
  // (a reseat is direction-agnostic). A pair already on its targets is skipped
  // outright — after a settle some legs are already home, and lifting a foot to
  // put it back where it is reads as a stray step.
  ReseatController(std::map<std::string, Vec3> current_stance,
                   std::map<std::string, Vec3> target_stance,
                   float pair_swing_time, float pair_dwell_time,
                   const SwingProfile& swing, float controller_dt);

  bool done() const { return done_; }
  std::map<std::string, LegOutput> update(float dt);

 private:
  // Skip to the first pair that needs moving and latch its origins; sets done_
  // if there is none.
  void seed_pair_origin();
  bool pair_needs_moving(std::size_t idx) const;
  bool remaining_pair_needs_moving() const;
  // Latch the origins of every foot seeded above its target. Height is the only
  // contact signal the ladder has, and every target is on the same ground plane.
  void seed_landing();
  float contact_band() const;
  std::map<std::string, LegOutput> tick_landing(float dt);
  LegOutput held(const std::string& name) const;
  std::map<std::string, LegOutput> emit_held() const;

  std::map<std::string, Vec3> target_;
  float pair_swing_time_;
  float pair_dwell_time_;
  SwingProfile swing_;
  // swing_ with the climb taken out: a landing foot must never rise.
  SwingProfile landing_swing_;
  float controller_dt_;

  std::map<std::string, Vec3> positions_;
  std::map<std::string, Vec3> pair_origin_;
  std::map<std::string, Vec3> landing_origin_;
  std::size_t pair_idx_ = 0;
  // Shared by the landing stage and the pair swings; they never overlap.
  float t_in_pair_ = 0.0f;
  float dwell_remaining_ = 0.0f;
  bool landing_ = false;
  bool done_ = false;
};

}  // namespace hexa::gait
