// Velocity caps derived from gait.yaml — single source of truth. Port of
// limits.py. Linear cap is per-gait (depends on the strategy's duty factor);
// angular cap is the raw angular_z_max. scale_to_envelope cuts (v_x, v_y,
// omega_z) so the implied per-leg planar speed never exceeds linear_max.
#pragma once

#include <map>
#include <string>
#include <tuple>

#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

// Per-gait linear caps, per-gait yaw bias, and a shared angular cap. Lookups by
// gait name throw std::out_of_range on unknown names (fail fast).
struct VelocityCaps {
  std::map<std::string, double> linear_max_by_gait;
  std::map<std::string, double> yaw_bias_by_gait;
  double angular_max = 0.0;

  double linear_max(const std::string& gait) const {
    return linear_max_by_gait.at(gait);
  }
  double yaw_bias(const std::string& gait) const {
    return yaw_bias_by_gait.at(gait);
  }
};

// Build per-gait caps from gait.yaml and the strategy registry.
VelocityCaps load_velocity_caps(const std::string& gait_yaml);

// Clamp omega_z and cut the velocity triple to fit the gait envelope. Returns
// (v_x, v_y, omega_z). leg_mounts maps leg name to (r_x, r_y, r_z); r_z ignored.
std::tuple<double, double, double> scale_to_envelope(
    double v_x, double v_y, double omega_z,
    const std::map<std::string, Vec3>& leg_mounts, double linear_max,
    double angular_max, double yaw_bias);

}  // namespace hexa_gait
