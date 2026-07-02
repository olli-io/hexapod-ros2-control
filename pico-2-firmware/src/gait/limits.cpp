#include "gait/limits.hpp"

#include <cmath>
#include <string>

#include "config_generated.hpp"  // hexa::config::kGaits, kAngularMax

namespace hexa::gait {

VelocityCaps load_velocity_caps_from_config() {
  VelocityCaps caps;
  caps.angular_max = ::hexa::config::kAngularMax;
  // The generator (gen_config.py) ports limits.cpp's derivation, so the per-gait
  // linear_max / yaw_bias are already baked into kGaits — mirror them out keyed
  // by gait name.
  for (const auto& g : ::hexa::config::kGaits) {
    const std::string name(g.name.data());
    caps.linear_max_by_gait[name] = g.linear_max;
    caps.yaw_bias_by_gait[name] = g.yaw_bias;
  }
  return caps;
}

std::tuple<float, float, float> scale_to_envelope(
    float v_x, float v_y, float omega_z,
    const std::map<std::string, Vec3>& leg_mounts, float linear_max,
    float angular_max, float yaw_bias) {
  if (omega_z > angular_max) {
    omega_z = angular_max;
  } else if (omega_z < -angular_max) {
    omega_z = -angular_max;
  }

  const float cap_sq = linear_max * linear_max;
  float max_leg_v_sq = 0.0f;
  for (const auto& [name, r] : leg_mounts) {
    (void)name;
    const float vlx = v_x - omega_z * r[1];
    const float vly = v_y + omega_z * r[0];
    const float v_sq = vlx * vlx + vly * vly;
    if (v_sq > max_leg_v_sq) {
      max_leg_v_sq = v_sq;
    }
  }

  if (max_leg_v_sq <= cap_sq) {
    return {v_x, v_y, omega_z};
  }

  const float rho = yaw_bias / (1.0f - yaw_bias);

  float t_required = 0.0f;
  bool feasible = true;
  for (const auto& [name, r] : leg_mounts) {
    (void)name;
    const float r_x = r[0];
    const float r_y = r[1];
    const float a0 = v_x - omega_z * r_y;
    const float a1 = rho * v_x - omega_z * r_y;
    const float b0 = v_y + omega_z * r_x;
    const float b1 = rho * v_y + omega_z * r_x;
    const float c = a0 * a0 + b0 * b0 - cap_sq;
    if (c <= 0.0f) {
      continue;
    }
    const float a = a1 * a1 + b1 * b1;
    if (a <= 0.0f) {
      feasible = false;
      break;
    }
    const float b = -2.0f * (a0 * a1 + b0 * b1);
    const float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) {
      feasible = false;
      break;
    }
    const float t_leg = (-b - std::sqrt(disc)) / (2.0f * a);
    if (t_leg > t_required) {
      t_required = t_leg;
    }
  }

  if (!feasible || rho * t_required >= 1.0f) {
    float max_r = 0.0f;
    for (const auto& [name, r] : leg_mounts) {
      (void)name;
      const float rr = std::hypot(r[0], r[1]);
      if (rr > max_r) {
        max_r = rr;
      }
    }
    const float omega_v_outer = std::fabs(omega_z) * max_r;
    if (omega_v_outer > linear_max) {
      return {0.0f, 0.0f, omega_z * (linear_max / omega_v_outer)};
    }
    return {0.0f, 0.0f, omega_z};
  }

  const float s_v = 1.0f - rho * t_required;
  const float s_w = 1.0f - t_required;
  return {v_x * s_v, v_y * s_v, omega_z * s_w};
}

}  // namespace hexa::gait
