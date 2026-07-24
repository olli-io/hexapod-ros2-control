#include "gait/pause.hpp"

#include <stdexcept>

#include "gait/gaits/base.hpp"

namespace hexa::gait {

PauseController::PauseController(std::map<std::string, Vec3> nominal_stance,
                                 float swing_clearance, float swing_width,
                                 float controller_dt, float descent_speed,
                                 float min_reset_time, float max_reset_time)
    : swing_clearance_(swing_clearance),
      swing_width_(swing_width),
      controller_dt_(controller_dt),
      descent_speed_(descent_speed),
      min_reset_time_(min_reset_time),
      max_reset_time_(max_reset_time) {
  require_all_legs(nominal_stance, "nominal_stance");
  if (descent_speed <= 0.0f) {
    throw std::invalid_argument("descent_speed must be positive");
  }
  if (min_reset_time <= 0.0f) {
    throw std::invalid_argument("min_reset_time must be positive");
  }
  if (max_reset_time < min_reset_time) {
    throw std::invalid_argument("max_reset_time < min_reset_time");
  }
  for (const auto& name : LEG_NAMES) {
    nominal_[name] = nominal_stance.at(name);
  }
  positions_ = nominal_;
}

void PauseController::begin(const std::map<std::string, Vec3>& last_targets,
                            const std::map<std::string, bool>& swing_flags) {
  require_all_legs(last_targets, "last_targets");

  positions_.clear();
  for (const auto& name : LEG_NAMES) {
    positions_[name] = last_targets.at(name);
  }
  descents_.clear();

  for (const auto& name : LEG_NAMES) {
    auto flag = swing_flags.find(name);
    if (flag == swing_flags.end() || !flag->second) {
      continue;
    }
    const Vec3& pos = positions_[name];
    const float x = pos[0];
    const float y = pos[1];
    const float z_high = pos[2];
    const float z_low = nominal_[name][2];
    // Already at or below nominal Z — no descent needed; treat as landed.
    if (z_high <= z_low + 1e-9f) {
      positions_[name] = Vec3(x, y, z_low);
      continue;
    }
    LegDescent descent;
    descent.origin = Vec3(x, y, z_high);
    descent.target = Vec3(x, y, z_low);
    descent.duration = adaptive_descent_time(z_high - z_low);
    descents_[name] = descent;
  }

  state_ = descents_.empty() ? PauseState::PAUSED : PauseState::LOWERING;
}

std::map<std::string, LegOutput> PauseController::update(float dt) {
  if (state_ == PauseState::PAUSED) {
    return emit_held();
  }
  return tick(dt);
}

std::map<std::string, LegOutput> PauseController::tick(float dt) {
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    auto it = descents_.find(name);
    if (it == descents_.end()) {
      out[name] = LegOutput{positions_[name], 0.0f, true};
      continue;
    }
    LegDescent& descent = it->second;
    descent.elapsed += dt;
    if (descent.elapsed >= descent.duration) {
      positions_[name] = descent.target;
      descents_.erase(it);
      out[name] = LegOutput{positions_[name], 0.0f, true};
    } else {
      const Vec3 point = descent_point(descent);
      positions_[name] = point;
      out[name] = LegOutput{point, descent.elapsed / descent.duration, false};
    }
  }
  if (descents_.empty()) {
    state_ = PauseState::PAUSED;
  }
  return out;
}

std::map<std::string, LegOutput> PauseController::emit_held() const {
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = LegOutput{positions_.at(name), 0.0f, true};
  }
  return out;
}

float PauseController::adaptive_descent_time(float distance_z) const {
  const float raw = distance_z / descent_speed_;
  if (raw < min_reset_time_) {
    return min_reset_time_;
  }
  if (raw > max_reset_time_) {
    return max_reset_time_;
  }
  return raw;
}

Vec3 PauseController::descent_point(const LegDescent& descent) const {
  // XY stays at the origin; only Z evolves. swing_arc with zero endpoint
  // velocities and zero clearance degenerates to a rest-to-rest interpolation
  // along the stride vector (here, purely -z).
  const float phase = descent.elapsed / descent.duration;
  return swing_arc(phase, descent.origin, descent.target,
                   /*swing_clearance=*/0.0f, swing_width_,
                   identity_y_sign(descent.target), descent.duration,
                   Vec3::Zero(), Vec3::Zero());
}

}  // namespace hexa::gait
