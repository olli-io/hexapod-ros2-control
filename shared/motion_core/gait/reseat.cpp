#include "gait/reseat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "gait/gaits/base.hpp"
#include "gait/stand_transition.hpp"

namespace hexa::gait {

namespace {
// How close to its target a foot counts as already in place. A settle leaves the
// legs it got to exactly on nominal; everything else is millimetres out.
constexpr float kInPlaceEpsilon = 1e-5f;

// Share of the pre-lift shift hold spent running the rung's phase up to
// lift-off. The rest is the body arriving: the support shift only starts moving
// once the phase enters its own lead window, and what it tracks is low-passed,
// so a hold that ended with the announcement would lift on a body still in
// transit. Announce over the first half, settle over the second.
constexpr float kShiftRampFraction = 0.5f;
}  // namespace

ReseatGeometry default_geometry_from_pose(const JointAngles& standing_angles,
                                          const kin::LegSpec& leg_spec) {
  const float th_c = standing_angles[0];
  const float th_f = standing_angles[1];
  const float th_t = standing_angles[2];
  const Vec3 foot_leg = kin::forward_kinematics({th_c, th_f, th_t}, leg_spec);
  const float default_foot_depth = -foot_leg[2];
  // From straight down, positive toward +r.
  const float tibia_from_vertical =
      static_cast<float>(M_PI) / 2.0f - (th_f + th_t);
  ReseatGeometry g;
  g.coxa_len = leg_spec.coxa_len;
  g.femur_len = leg_spec.femur_len;
  g.tibia_len = leg_spec.tibia_len;
  g.tibia_from_vertical = tibia_from_vertical;
  g.default_foot_depth = default_foot_depth;
  return g;
}

std::map<std::string, Vec3> reseat_nominal_stance(
    float target_height_m, const ReseatGeometryByLeg& geometry,
    const std::map<std::string, kin::LegSpec>& leg_specs,
    const std::map<std::string, Vec3>& current_stance) {
  require_all_legs(current_stance, "current_stance");

  std::map<std::string, Vec3> out;
  for (std::size_t i = 0; i < static_cast<std::size_t>(kNumLegs); ++i) {
    const std::string& name = LEG_NAMES[i];
    auto it = leg_specs.find(name);
    if (it == leg_specs.end()) {
      throw std::invalid_argument("leg_specs missing " + name);
    }

    // Per leg: legs reaching out different distances lean their tibias
    // differently, so one shared solve would drag the whole stance onto one
    // group's radius.
    const ReseatGeometry& g = geometry[i];
    const float d_new = g.default_foot_depth + target_height_m;
    const float arg =
        (g.tibia_len * std::cos(g.tibia_from_vertical) - d_new) / g.femur_len;
    if (arg < -1.0f || arg > 1.0f) {
      throw std::invalid_argument(
          "target_height_m is outside the geometrically feasible reseat range "
          "for " +
          name + " (arcsin arg not in [-1, 1])");
    }
    const float alpha = std::asin(arg);
    const float r_new = g.coxa_len + g.femur_len * std::cos(alpha) +
                        g.tibia_len * std::sin(g.tibia_from_vertical);

    // Keep the leg pointing where it already points, so the standing splay
    // survives and only radius and depth follow the new height.
    const Vec3 cur_leg = kin::body_to_leg(current_stance.at(name), it->second);
    const float az = (std::hypot(cur_leg[0], cur_leg[1]) > 1e-6f)
                         ? std::atan2(cur_leg[1], cur_leg[0])
                         : 0.0f;
    const Vec3 body_xyz = kin::leg_to_body(
        Vec3(r_new * std::cos(az), r_new * std::sin(az), -d_new), it->second);
    // Cancels apply_body_pose's z-subtraction, landing the foot at -d_new.
    out[name] = Vec3(body_xyz[0], body_xyz[1], body_xyz[2] + target_height_m);
  }
  return out;
}

ReseatController::ReseatController(std::map<std::string, Vec3> current_stance,
                                   std::map<std::string, Vec3> target_stance,
                                   float pair_swing_time, float pair_dwell_time,
                                   const SwingProfile& swing,
                                   float controller_dt, RungList rung_order,
                                   float pre_lift_shift_time, float shift_lead)
    : pair_order_(rung_order.empty() ? pair_list(LegSet::HEXAPOD)
                                     : std::move(rung_order)),
      pair_swing_time_(pair_swing_time),
      pair_dwell_time_(pair_dwell_time),
      pre_lift_shift_time_(pre_lift_shift_time),
      shift_lead_(shift_lead),
      swing_(swing),
      controller_dt_(controller_dt) {
  // Direction-agnostic, so the lateral arch is dropped; the rest of the shape is
  // the gait's, so a foot re-plants as gently as it lands mid-walk.
  swing_.width = 0.0f;
  // Zero clearance drops the vertical shaping, so the landing rides the plain
  // eased blend down onto its target and never climbs over its own start. It
  // forgoes the probe with it: endpoint-soft rather than probe-gentle.
  landing_swing_ = swing_;
  landing_swing_.clearance = 0.0f;
  for (const auto& pair : pair_order_) {
    for (const auto& name : pair) {
      legs_.push_back(name);
    }
  }
  for (const auto& name : legs_) {
    if (current_stance.find(name) == current_stance.end()) {
      throw std::invalid_argument("current_stance missing leg: " + name);
    }
    if (target_stance.find(name) == target_stance.end()) {
      throw std::invalid_argument("target_stance missing leg: " + name);
    }
  }
  if (pair_swing_time <= 0.0f) {
    throw std::invalid_argument("pair_swing_time must be positive");
  }
  if (pair_dwell_time < 0.0f) {
    throw std::invalid_argument("pair_dwell_time must be non-negative");
  }
  if (pre_lift_shift_time < 0.0f) {
    throw std::invalid_argument("pre_lift_shift_time must be non-negative");
  }
  for (const auto& name : legs_) {
    target_[name] = target_stance.at(name);
    positions_[name] = current_stance.at(name);
  }
  seed_landing();
  if (!landing_) {
    seed_pair_origin();
  }
}

std::map<std::string, LegOutput> ReseatController::update(float dt) {
  if (done_) {
    std::map<std::string, LegOutput> out;
    for (const auto& name : legs_) {
      out[name] = LegOutput{target_[name], 0.0f, true};
    }
    return out;
  }

  if (landing_) {
    return tick_landing(dt);
  }

  if (dwell_remaining_ > 0.0f) {
    // Held between two pair swings; the next pair seeds when the dwell expires.
    dwell_remaining_ -= dt;
    if (dwell_remaining_ <= 0.0f) {
      dwell_remaining_ = 0.0f;
      seed_pair_origin();
    }
    return emit_held();
  }

  if (t_in_shift_ >= 0.0f) {
    return tick_shift(dt);
  }

  t_in_pair_ += dt;
  const float phase = t_in_pair_ / pair_swing_time_;
  const Rung& active = pair_order_[pair_idx_];

  if (phase >= 1.0f) {
    for (const auto& name : active) {
      positions_[name] = target_[name];
    }
    begin_restore(active);
    pair_idx_ += 1;
    t_in_pair_ = 0.0f;
    // The dwell lets this pair land before the next one lifts, so it is only
    // owed if a next one will lift.
    if (pair_dwell_time_ > 0.0f && remaining_pair_needs_moving()) {
      dwell_remaining_ = pair_dwell_time_;
    } else {
      seed_pair_origin();
    }
    return emit_held();
  }

  // Mid-rung: a rest-to-rest swing arc from the rung-start origin to the target.
  // A leg already standing on its target stays down — a mirrored pair is
  // mirrored to keep the body balanced, and lifting fewer feet only helps that.
  std::map<std::string, LegOutput> out;
  for (const auto& name : legs_) {
    const bool is_active =
        std::find(active.begin(), active.end(), name) != active.end();
    if (is_active && (pair_origin_.at(name) - target_.at(name)).norm() >
                         kInPlaceEpsilon) {
      const Vec3 origin = pair_origin_[name];
      const Vec3 target = target_[name];
      const Vec3 point =
          swing_arc(phase, origin, target, identity_y_sign(target),
                    pair_swing_time_, swing_, Vec3::Zero(), Vec3::Zero());
      positions_[name] = point;
      out[name] = LegOutput{point, phase, false};
    } else {
      out[name] = held(name);
    }
  }
  return out;
}

std::map<std::string, LegOutput> ReseatController::tick_landing(float dt) {
  t_in_pair_ += dt;
  const float phase = t_in_pair_ / pair_swing_time_;

  if (phase >= 1.0f) {
    Rung landed;
    for (const auto& entry : landing_origin_) {
      positions_[entry.first] = target_[entry.first];
      landed.push_back(entry.first);
    }
    // An airborne foot carried nothing either, so it comes back on the same ramp.
    begin_restore(landed);
    landing_origin_.clear();
    landing_ = false;
    t_in_pair_ = 0.0f;
    if (pair_dwell_time_ > 0.0f && remaining_pair_needs_moving()) {
      dwell_remaining_ = pair_dwell_time_;
    } else {
      seed_pair_origin();
    }
    return emit_held();
  }

  std::map<std::string, LegOutput> out;
  for (const auto& name : legs_) {
    auto it = landing_origin_.find(name);
    if (it == landing_origin_.end()) {
      out[name] = held(name);
      continue;
    }
    // Onto the full target rather than straight down: an airborne foot carries
    // no weight, and arriving home lets seed_pair_origin skip its pair.
    const Vec3 target = target_[name];
    const Vec3 point =
        swing_arc(phase, it->second, target, identity_y_sign(target),
                  pair_swing_time_, landing_swing_, Vec3::Zero(), Vec3::Zero());
    positions_[name] = point;
    out[name] = LegOutput{point, phase, false};
  }
  return out;
}

// How far above its target a foot may sit and still be carrying weight. The
// touchdown probe is that height by definition, and is already tuned to clear
// servo resolution and inter-leg height error.
float ReseatController::contact_band() const {
  return std::max(swing_.probe_band(pair_swing_time_), kInPlaceEpsilon);
}

LegOutput ReseatController::held(const std::string& name) const {
  const Vec3& p = positions_.at(name);
  const bool restoring =
      std::find(restoring_.begin(), restoring_.end(), name) != restoring_.end();
  return LegOutput{p, restoring ? restoring_phase_ : 0.0f,
                   p[2] <= target_.at(name)[2] + contact_band()};
}

void ReseatController::begin_restore(const Rung& landed) {
  // Without a hold there is no ramp to run it back down, and a foot left
  // announcing a lift-off it will not make is dropped from the support for good.
  if (pre_lift_shift_time_ <= 0.0f || shift_lead_ <= 0.0f) {
    return;
  }
  restoring_ = landed;
  restoring_phase_ = 1.0f;
}

std::map<std::string, LegOutput> ReseatController::emit_held() const {
  std::map<std::string, LegOutput> out;
  for (const auto& name : legs_) {
    out[name] = held(name);
  }
  return out;
}

void ReseatController::seed_landing() {
  for (const auto& name : legs_) {
    if (positions_[name][2] > target_[name][2] + contact_band()) {
      landing_origin_[name] = positions_[name];
    }
  }
  landing_ = !landing_origin_.empty();
}

bool ReseatController::remaining_pair_needs_moving() const {
  for (std::size_t i = pair_idx_; i < pair_order_.size(); ++i) {
    if (pair_needs_moving(i)) {
      return true;
    }
  }
  return false;
}

bool ReseatController::pair_needs_moving(std::size_t idx) const {
  for (const auto& name : pair_order_[idx]) {
    if ((positions_.at(name) - target_.at(name)).norm() > kInPlaceEpsilon) {
      return true;
    }
  }
  return false;
}

void ReseatController::seed_pair_origin() {
  while (pair_idx_ < pair_order_.size() && !pair_needs_moving(pair_idx_)) {
    // Already there: snap out the float dust, no swing, no dwell and no shift.
    for (const auto& name : pair_order_[pair_idx_]) {
      positions_[name] = target_[name];
    }
    pair_idx_ += 1;
  }
  if (pair_idx_ >= pair_order_.size()) {
    done_ = true;
    return;
  }
  const Rung& active = pair_order_[pair_idx_];
  pair_origin_.clear();
  for (const auto& name : active) {
    pair_origin_[name] = positions_[name];
  }
  // Armed here rather than at the swing, so the origins are already latched when
  // the hold starts emitting the rung's phase.
  t_in_shift_ = pre_lift_shift_time_ > 0.0f ? 0.0f : -1.0f;
}

// Every foot stays exactly where it is; only the phases move. The rung about to
// lift runs up to lift-off over the ramp and sits there for the rest, which is
// what tells the support shift to stop counting these feet and carry the body
// onto the ones that will still be down; the rung that landed last takes back
// exactly the weight this one gives up, which slides the anticipated support
// from one triangle straight into the next instead of through the centroid.
std::map<std::string, LegOutput> ReseatController::tick_shift(float dt) {
  t_in_shift_ += dt;
  const float ramp = kShiftRampFraction * pre_lift_shift_time_;
  const float phase =
      ramp > 0.0f ? std::min(1.0f, t_in_shift_ / ramp) : 1.0f;
  // Inverts posture's clamp((1 - phase) / lead): the phase whose weight is one
  // minus the lifting rung's. Pinned at 1 until that weight starts to move.
  restoring_phase_ = std::min(1.0f, 2.0f - shift_lead_ - phase);

  std::map<std::string, LegOutput> out = emit_held();
  for (const auto& name : pair_order_[pair_idx_]) {
    out[name].phase = phase;
  }
  if (t_in_shift_ >= pre_lift_shift_time_) {
    t_in_shift_ = -1.0f;
    restoring_.clear();
  }
  return out;
}

}  // namespace hexa::gait
