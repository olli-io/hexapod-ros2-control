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
#include <vector>

#include "gait/gaits/base.hpp"
#include "gait/kinematics.hpp"
#include "gait/stand_transition.hpp"
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
  // swing shapes the rung's arc as it shapes a gait swing; its width is ignored
  // (a reseat is direction-agnostic). A rung already on its targets is skipped
  // outright — after a settle some legs are already home, and lifting a foot to
  // put it back where it is reads as a stray step.
  // `rung_order` is both the sequence and the ladder's leg set: the flattened
  // rungs are the only legs it reads, moves or reports. An empty list takes the
  // six-leg PAIR_ORDER. Legs outside it (a parked middle) are left to the
  // caller, which is what keeps a foot 0.27 m up out of the landing stage.
  //
  // `pre_lift_shift_time` holds every foot planted for that long before each
  // rung lifts, announcing the coming lift-off through the rung's emitted phase
  // so the support shift carries the body off it first. The rung that just
  // landed takes back exactly the weight over the same ramp, so the body crosses
  // straight from one rung's support to the next one's; `shift_lead` is
  // posture's support_shift_lead, which is what makes that exact. Zero — the
  // six-leg case, where a mirrored pair leaves the body inside what is left —
  // skips both.
  ReseatController(std::map<std::string, Vec3> current_stance,
                   std::map<std::string, Vec3> target_stance,
                   float pair_swing_time, float pair_dwell_time,
                   const SwingProfile& swing, float controller_dt,
                   RungList rung_order = {},
                   float pre_lift_shift_time = 0.0f, float shift_lead = 0.0f);

  bool done() const { return done_; }
  std::map<std::string, LegOutput> update(float dt);

 private:
  // Skip to the first rung that needs moving and latch its origins; sets done_
  // if there is none, and arms the pre-lift shift hold if one is owed.
  void seed_pair_origin();
  bool pair_needs_moving(std::size_t idx) const;
  bool remaining_pair_needs_moving() const;
  std::map<std::string, LegOutput> tick_shift(float dt);
  void begin_restore(const Rung& landed);
  // Latch the origins of every foot seeded above its target. Height is the only
  // contact signal the ladder has, and every target is on the same ground plane.
  void seed_landing();
  float contact_band() const;
  std::map<std::string, LegOutput> tick_landing(float dt);
  LegOutput held(const std::string& name) const;
  std::map<std::string, LegOutput> emit_held() const;

  RungList pair_order_;
  // pair_order_ flattened: every leg this ladder owns.
  std::vector<std::string> legs_;
  std::map<std::string, Vec3> target_;
  float pair_swing_time_;
  float pair_dwell_time_;
  float pre_lift_shift_time_;
  float shift_lead_;
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
  // Counts up through the pre-lift shift hold; -1 when no hold is running, so a
  // zero-length one is still distinguishable from none at all.
  float t_in_shift_ = -1.0f;
  // The rung that landed last, and the phase it is still emitted at: it holds
  // the body where its own lift-off put it until the next rung takes it on.
  // Empty outside a hold.
  Rung restoring_;
  float restoring_phase_ = 0.0f;
  bool landing_ = false;
  bool done_ = false;
};

}  // namespace hexa::gait
