#include "gait/reseat.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

#include "gait/gaits/base.hpp"
#include "gait/stand_transition.hpp"

namespace hexa::gait {

namespace {
// How close to its target a foot counts as already in place. A settle leaves the
// legs it got to exactly on nominal; a leg it did not get to is millimetres out,
// and a height change moves every foot further than that again.
constexpr float kInPlaceEpsilon = 1e-5f;
}  // namespace

ReseatGeometry default_geometry_from_pose(const JointAngles& standing_angles,
                                          const kin::LegSpec& leg_spec) {
  const float th_c = standing_angles[0];
  const float th_f = standing_angles[1];
  const float th_t = standing_angles[2];
  const Vec3 foot_leg = kin::forward_kinematics({th_c, th_f, th_t}, leg_spec);
  const float default_foot_depth = -foot_leg[2];
  // Angle of the tibia from straight down, positive toward +r: pi/2 - (th_f +
  // th_t).
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

    // Solved per leg: two legs reaching out different distances lean their
    // tibias differently, so they land on different radii at the same height.
    // One shared solve would drag the whole stance onto one group's radius.
    const ReseatGeometry& g = geometry[i];
    const float d_new = g.default_foot_depth + target_height_m;
    // arcsin argument: positive when the tibia's vertical projection exceeds the
    // foot depth (femur tilts up).
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

    // Keep the leg pointing where it already points: the standing splay (and
    // anything else that swivelled the foot) is preserved, and only the radius
    // and depth follow the new height. The azimuth does not depend on z, so the
    // stored stance's height compensation is irrelevant here.
    const Vec3 cur_leg = kin::body_to_leg(current_stance.at(name), it->second);
    const float az = (std::hypot(cur_leg[0], cur_leg[1]) > 1e-6f)
                         ? std::atan2(cur_leg[1], cur_leg[0])
                         : 0.0f;
    const Vec3 body_xyz = kin::leg_to_body(
        Vec3(r_new * std::cos(az), r_new * std::sin(az), -d_new), it->second);
    // Add target_height so apply_body_pose's z-subtraction lands the foot in the
    // leg frame at -d_new.
    out[name] = Vec3(body_xyz[0], body_xyz[1], body_xyz[2] + target_height_m);
  }
  return out;
}

ReseatController::ReseatController(std::map<std::string, Vec3> current_stance,
                                   std::map<std::string, Vec3> target_stance,
                                   float pair_swing_time, float pair_dwell_time,
                                   const SwingProfile& swing,
                                   float controller_dt)
    : pair_swing_time_(pair_swing_time),
      pair_dwell_time_(pair_dwell_time),
      swing_(swing),
      controller_dt_(controller_dt) {
  // A reseat travels the same chord whatever direction the body is facing, so
  // the lateral arch is dropped; the rise/descent split and the touchdown probe
  // are the gait's, so a foot re-plants as gently as it lands mid-walk.
  swing_.width = 0.0f;
  require_all_legs(current_stance, "current_stance");
  require_all_legs(target_stance, "target_stance");
  if (pair_swing_time <= 0.0f) {
    throw std::invalid_argument("pair_swing_time must be positive");
  }
  if (pair_dwell_time < 0.0f) {
    throw std::invalid_argument("pair_dwell_time must be non-negative");
  }
  for (const auto& name : LEG_NAMES) {
    target_[name] = target_stance.at(name);
    positions_[name] = current_stance.at(name);
  }
  seed_pair_origin();
}

std::map<std::string, LegOutput> ReseatController::update(float dt) {
  if (done_) {
    std::map<std::string, LegOutput> out;
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{target_[name], 0.0f, true};
    }
    return out;
  }

  if (dwell_remaining_ > 0.0f) {
    // Held between two pair swings: every foot stays put. Seed the next pair's
    // origins on the tick the dwell expires.
    dwell_remaining_ -= dt;
    if (dwell_remaining_ <= 0.0f) {
      dwell_remaining_ = 0.0f;
      seed_pair_origin();
    }
    std::map<std::string, LegOutput> out;
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{positions_[name], 0.0f, true};
    }
    return out;
  }

  t_in_pair_ += dt;
  const float phase = t_in_pair_ / pair_swing_time_;
  const std::array<std::string, 2>& active = PAIR_ORDER[pair_idx_];

  std::map<std::string, LegOutput> out;
  if (phase >= 1.0f) {
    // Snap both active legs to their targets simultaneously and advance.
    for (const auto& name : active) {
      positions_[name] = target_[name];
    }
    pair_idx_ += 1;
    t_in_pair_ = 0.0f;
    // A dwell exists to let this pair land before the next one lifts, so it is
    // only owed if a next one is actually going to lift. seed_pair_origin()
    // decides that (and sets done_ when nothing is left).
    if (pair_dwell_time_ > 0.0f && remaining_pair_needs_moving()) {
      dwell_remaining_ = pair_dwell_time_;
    } else {
      seed_pair_origin();
    }
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{positions_[name], 0.0f, true};
    }
    return out;
  }

  // Mid-pair: the active legs follow a rest-to-rest swing arc from their
  // pair-start origin to their target, shaped by the gait's swing profile
  // (vertical lift over a linear XY chord — direction-agnostic). One of the two
  // may already be standing on its target, in which case it stays down — the
  // pair is mirrored to keep the body balanced, and lifting fewer feet only ever
  // helps that.
  for (const auto& name : LEG_NAMES) {
    const bool is_active = (name == active[0] || name == active[1]);
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
      out[name] = LegOutput{positions_[name], 0.0f, true};
    }
  }
  return out;
}

bool ReseatController::remaining_pair_needs_moving() const {
  for (std::size_t i = pair_idx_; i < PAIR_ORDER.size(); ++i) {
    if (pair_needs_moving(i)) {
      return true;
    }
  }
  return false;
}

bool ReseatController::pair_needs_moving(std::size_t idx) const {
  for (const auto& name : PAIR_ORDER[idx]) {
    if ((positions_.at(name) - target_.at(name)).norm() > kInPlaceEpsilon) {
      return true;
    }
  }
  return false;
}

void ReseatController::seed_pair_origin() {
  while (pair_idx_ < PAIR_ORDER.size() && !pair_needs_moving(pair_idx_)) {
    // Already standing where this pair is being sent. Snap out the float dust
    // and move on — no swing, and no dwell to cover a swing that never happens.
    for (const auto& name : PAIR_ORDER[pair_idx_]) {
      positions_[name] = target_[name];
    }
    pair_idx_ += 1;
  }
  if (pair_idx_ >= PAIR_ORDER.size()) {
    done_ = true;
    return;
  }
  const std::array<std::string, 2>& active = PAIR_ORDER[pair_idx_];
  pair_origin_.clear();
  for (const auto& name : active) {
    pair_origin_[name] = positions_[name];
  }
}

}  // namespace hexa::gait
