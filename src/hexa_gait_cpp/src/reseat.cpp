#include "hexa_gait_cpp/reseat.hpp"

#include <cmath>
#include <stdexcept>

#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/initialize.hpp"
#include "hexa_gait_cpp/validation.hpp"

namespace hexa_gait {

ReseatGeometry default_geometry_from_pose(const JointAngles& standing_angles,
                                          const kin::LegSpec& leg_spec) {
  const double th_c = standing_angles[0];
  const double th_f = standing_angles[1];
  const double th_t = standing_angles[2];
  const Vec3 foot_leg =
      kin::forward_kinematics({th_c, th_f, th_t}, leg_spec);
  const double default_foot_depth = -foot_leg[2];
  // Angle of the tibia from straight down, positive toward +r: pi/2 - (th_f + th_t).
  const double tibia_from_vertical = M_PI / 2.0 - (th_f + th_t);
  ReseatGeometry g;
  g.coxa_len = leg_spec.coxa_len;
  g.femur_len = leg_spec.femur_len;
  g.tibia_len = leg_spec.tibia_len;
  g.tibia_from_vertical = tibia_from_vertical;
  g.default_foot_depth = default_foot_depth;
  return g;
}

std::map<std::string, Vec3> reseat_nominal_stance(
    double target_height_m, const ReseatGeometry& geometry,
    const std::map<std::string, kin::LegSpec>& leg_specs) {
  const double d_new = geometry.default_foot_depth + target_height_m;
  // arcsin argument: positive when the tibia's vertical projection exceeds the
  // foot depth (femur tilts up).
  const double arg =
      (geometry.tibia_len * std::cos(geometry.tibia_from_vertical) - d_new) /
      geometry.femur_len;
  if (arg < -1.0 || arg > 1.0) {
    throw std::invalid_argument(
        "target_height_m is outside the geometrically feasible reseat range "
        "(arcsin arg not in [-1, 1])");
  }
  const double alpha = std::asin(arg);
  const double r_new = geometry.coxa_len + geometry.femur_len * std::cos(alpha) +
                       geometry.tibia_len * std::sin(geometry.tibia_from_vertical);

  std::map<std::string, Vec3> out;
  for (const auto& name : LEG_NAMES) {
    auto it = leg_specs.find(name);
    if (it == leg_specs.end()) {
      throw std::invalid_argument("leg_specs missing " + name);
    }
    const Vec3 body_xyz = kin::leg_to_body(Vec3(r_new, 0.0, -d_new), it->second);
    // Add target_height so apply_body_pose's z-subtraction lands the foot in the
    // leg frame at -d_new.
    out[name] = Vec3(body_xyz[0], body_xyz[1], body_xyz[2] + target_height_m);
  }
  return out;
}

ReseatController::ReseatController(std::map<std::string, Vec3> current_stance,
                                   std::map<std::string, Vec3> target_stance,
                                   double pair_swing_time,
                                   double pair_dwell_time,
                                   double swing_clearance, double controller_dt)
    : pair_swing_time_(pair_swing_time),
      pair_dwell_time_(pair_dwell_time),
      swing_clearance_(swing_clearance),
      controller_dt_(controller_dt) {
  require_all_legs(current_stance, "current_stance");
  require_all_legs(target_stance, "target_stance");
  if (pair_swing_time <= 0.0) {
    throw std::invalid_argument("pair_swing_time must be positive");
  }
  if (pair_dwell_time < 0.0) {
    throw std::invalid_argument("pair_dwell_time must be non-negative");
  }
  for (const auto& name : LEG_NAMES) {
    target_[name] = target_stance.at(name);
    positions_[name] = current_stance.at(name);
  }
  seed_pair_origin();
}

std::map<std::string, LegOutput> ReseatController::update(double dt) {
  if (done_) {
    std::map<std::string, LegOutput> out;
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{target_[name], 0.0, true};
    }
    return out;
  }

  if (dwell_remaining_ > 0.0) {
    // Held between two pair swings: every foot stays put. Seed the next pair's
    // origins on the tick the dwell expires.
    dwell_remaining_ -= dt;
    if (dwell_remaining_ <= 0.0) {
      dwell_remaining_ = 0.0;
      seed_pair_origin();
    }
    std::map<std::string, LegOutput> out;
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{positions_[name], 0.0, true};
    }
    return out;
  }

  t_in_pair_ += dt;
  const double phase = t_in_pair_ / pair_swing_time_;
  const std::array<std::string, 2>& active = PAIR_ORDER[pair_idx_];

  std::map<std::string, LegOutput> out;
  if (phase >= 1.0) {
    // Snap both active legs to their targets simultaneously and advance.
    for (const auto& name : active) {
      positions_[name] = target_[name];
    }
    pair_idx_ += 1;
    t_in_pair_ = 0.0;
    if (pair_idx_ >= PAIR_ORDER.size()) {
      done_ = true;
    } else if (pair_dwell_time_ > 0.0) {
      // Hold before the next pair lifts; seeding deferred to dwell expiry.
      dwell_remaining_ = pair_dwell_time_;
    } else {
      seed_pair_origin();
    }
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{positions_[name], 0.0, true};
    }
    return out;
  }

  // Mid-pair: both active legs follow a rest-to-rest swing arc from their
  // pair-start origin to their target. swing_width is zero (vertical lift over
  // a linear XY chord — direction-agnostic).
  for (const auto& name : LEG_NAMES) {
    if (name == active[0] || name == active[1]) {
      const Vec3 origin = pair_origin_[name];
      const Vec3 target = target_[name];
      const Vec3 point = swing_arc(phase, origin, target, swing_clearance_,
                                   /*swing_width=*/0.0, identity_y_sign(target),
                                   pair_swing_time_, controller_dt_,
                                   Vec3::Zero(), Vec3::Zero());
      positions_[name] = point;
      out[name] = LegOutput{point, phase, false};
    } else {
      out[name] = LegOutput{positions_[name], 0.0, true};
    }
  }
  return out;
}

void ReseatController::seed_pair_origin() {
  if (pair_idx_ >= PAIR_ORDER.size()) {
    return;
  }
  const std::array<std::string, 2>& active = PAIR_ORDER[pair_idx_];
  pair_origin_.clear();
  for (const auto& name : active) {
    pair_origin_[name] = positions_[name];
  }
}

}  // namespace hexa_gait
