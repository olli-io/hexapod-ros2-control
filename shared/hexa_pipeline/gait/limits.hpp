// Velocity caps derived from the baked gait config — single source of truth.
// Float fork of limits.hpp (plan part 06). Linear cap is per-gait (depends on
// the strategy's duty factor); angular cap is the raw angular_z_max.
// scale_to_envelope cuts (v_x, v_y, omega_z) so the implied per-leg planar speed
// never exceeds linear_max.
#pragma once

#include <map>
#include <string>
#include <tuple>

#include "gait/types.hpp"

namespace hexa::gait {

// Per-gait linear caps, per-gait yaw bias, and a shared angular cap. Lookups by
// gait name throw std::out_of_range on unknown names (fail fast).
struct VelocityCaps {
  std::map<std::string, float> linear_max_by_gait;
  std::map<std::string, float> yaw_bias_by_gait;
  float angular_max = 0.0f;

  float linear_max(const std::string& gait) const {
    return linear_max_by_gait.at(gait);
  }
  float yaw_bias(const std::string& gait) const {
    return yaw_bias_by_gait.at(gait);
  }
};

// Build per-gait caps from the baked config (config::kGaits + kAngularMax),
// mirroring the double load_velocity_caps that reads tuning.yaml.
VelocityCaps load_velocity_caps_from_config();

// Clamp omega_z and cut the velocity triple to fit the gait envelope. Returns
// (v_x, v_y, omega_z). leg_mounts maps leg name to (r_x, r_y, r_z); r_z ignored.
std::tuple<float, float, float> scale_to_envelope(
    float v_x, float v_y, float omega_z,
    const std::map<std::string, Vec3>& leg_mounts, float linear_max,
    float angular_max, float yaw_bias);

}  // namespace hexa::gait
