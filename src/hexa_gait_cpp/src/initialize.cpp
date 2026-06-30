#include "hexa_gait_cpp/initialize.hpp"

#include <stdexcept>

#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/validation.hpp"

namespace hexa_gait {

InitializeController::InitializeController(
    std::map<std::string, Vec3> initial_stance,
    std::map<std::string, Vec3> nominal_stance, double coxa_to_bottom,
    double pair_swing_time, double lift_body_time, double swing_clearance,
    double place_feet_clearance, double swing_width, double controller_dt)
    : coxa_to_bottom_(coxa_to_bottom),
      place_feet_clearance_(place_feet_clearance),
      pair_swing_time_(pair_swing_time),
      lift_body_time_(lift_body_time),
      swing_clearance_(swing_clearance),
      swing_width_(swing_width),
      controller_dt_(controller_dt) {
  require_all_legs(initial_stance, "initial_stance");
  require_all_legs(nominal_stance, "nominal_stance");
  if (pair_swing_time <= 0.0) {
    throw std::invalid_argument("pair_swing_time must be positive");
  }
  if (lift_body_time <= 0.0) {
    throw std::invalid_argument("lift_body_time must be positive");
  }
  for (const auto& name : LEG_NAMES) {
    initial_[name] = initial_stance.at(name);
    nominal_[name] = nominal_stance.at(name);
  }
  // Per-leg target at the end of PLACE_FEET: standing XY, IK target at
  // body-frame z = -coxa_to_bottom + place_feet_clearance.
  lift_start_z_ = -coxa_to_bottom + place_feet_clearance;
  for (const auto& name : LEG_NAMES) {
    ground_targets_[name] =
        Vec3(nominal_[name][0], nominal_[name][1], lift_start_z_);
  }
  positions_ = initial_;
}

std::map<std::string, LegOutput> InitializeController::update(double dt) {
  if (state_ == InitializeState::PLACE_FEET) {
    return tick_place_feet(dt);
  }
  if (state_ == InitializeState::LIFT_BODY) {
    return tick_lift_body(dt);
  }
  return emit_nominal();
}

std::map<std::string, LegOutput> InitializeController::tick_place_feet(
    double dt) {
  t_in_pair_ += dt;
  const double phase = t_in_pair_ / pair_swing_time_;
  const std::array<std::string, 2>& active = PAIR_ORDER[pair_idx_];

  std::map<std::string, LegOutput> out;
  if (phase >= 1.0) {
    // Snap the active pair to their ground targets and advance.
    for (const auto& name : active) {
      positions_[name] = ground_targets_[name];
    }
    pair_idx_ += 1;
    t_in_pair_ = 0.0;
    if (pair_idx_ >= PAIR_ORDER.size()) {
      state_ = InitializeState::LIFT_BODY;
    }
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{positions_[name], 0.0, true};
    }
    return out;
  }

  // Mid-pair: active legs follow a rest-to-rest swing arc from initial_stance
  // to the ground target. Endpoint velocities pinned to zero.
  for (const auto& name : LEG_NAMES) {
    if (name == active[0] || name == active[1]) {
      const Vec3 origin = initial_[name];
      const Vec3 target = ground_targets_[name];
      const Vec3 point = swing_arc(phase, origin, target, swing_clearance_,
                                   swing_width_, identity_y_sign(target),
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

std::map<std::string, LegOutput> InitializeController::tick_lift_body(
    double dt) {
  // All six feet stay at standing XY; body-frame z ramps via smoothstep from
  // the PLACE_FEET endpoint down to nominal_stance.z.
  t_in_lift_ += dt;
  const double tau = t_in_lift_ / lift_body_time_;
  const double s = smoothstep(tau);
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    const Vec3& nom = nominal_[name];
    const double z = lift_start_z_ + s * (nom[2] - lift_start_z_);
    const Vec3 point(nom[0], nom[1], z);
    positions_[name] = point;
    out[name] = LegOutput{point, tau, true};
  }
  if (tau >= 1.0) {
    // Snap to nominal so downstream sees no drift and advance to DONE.
    for (const auto& name : LEG_NAMES) {
      positions_[name] = nominal_[name];
      out[name] = LegOutput{nominal_[name], 1.0, true};
    }
    state_ = InitializeState::DONE;
  }
  return out;
}

std::map<std::string, LegOutput> InitializeController::emit_nominal() const {
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = LegOutput{nominal_.at(name), 0.0, true};
  }
  return out;
}

}  // namespace hexa_gait
