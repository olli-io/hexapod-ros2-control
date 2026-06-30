#include "hexa_gait_cpp/limits.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

#include "hexa_gait_cpp/gaits/registry.hpp"

namespace hexa_gait {

VelocityCaps load_velocity_caps(const std::string& gait_yaml) {
  const YAML::Node raw = YAML::LoadFile(gait_yaml);

  const double stride_length = raw["stride_length"].as<double>();
  const double min_swing_time = raw["min_swing_time"].as<double>();
  const double angular_max = raw["angular_z_max"].as<double>();
  const double yaw_bias = raw["yaw_bias"].as<double>();

  VelocityCaps caps;
  caps.angular_max = angular_max;
  // Duty factor is not in YAML — it lives on each strategy class. Enumerate the
  // registry so a new gait shows up in the caps map as soon as it is registered.
  for (const auto& [name, factory] : strategies()) {
    const double duty = factory()->duty_factor();
    caps.linear_max_by_gait[name] =
        stride_length * (1.0 - duty) / (min_swing_time * duty);
    caps.yaw_bias_by_gait[name] = 0.5 + (yaw_bias - 0.5) * (1.5 - duty);
  }
  return caps;
}

std::tuple<double, double, double> scale_to_envelope(
    double v_x, double v_y, double omega_z,
    const std::map<std::string, Vec3>& leg_mounts, double linear_max,
    double angular_max, double yaw_bias) {
  if (omega_z > angular_max) {
    omega_z = angular_max;
  } else if (omega_z < -angular_max) {
    omega_z = -angular_max;
  }

  const double cap_sq = linear_max * linear_max;
  double max_leg_v_sq = 0.0;
  for (const auto& [name, r] : leg_mounts) {
    (void)name;
    const double vlx = v_x - omega_z * r[1];
    const double vly = v_y + omega_z * r[0];
    const double v_sq = vlx * vlx + vly * vly;
    if (v_sq > max_leg_v_sq) {
      max_leg_v_sq = v_sq;
    }
  }

  if (max_leg_v_sq <= cap_sq) {
    return {v_x, v_y, omega_z};
  }

  const double rho = yaw_bias / (1.0 - yaw_bias);

  double t_required = 0.0;
  bool feasible = true;
  for (const auto& [name, r] : leg_mounts) {
    (void)name;
    const double r_x = r[0];
    const double r_y = r[1];
    const double a0 = v_x - omega_z * r_y;
    const double a1 = rho * v_x - omega_z * r_y;
    const double b0 = v_y + omega_z * r_x;
    const double b1 = rho * v_y + omega_z * r_x;
    const double c = a0 * a0 + b0 * b0 - cap_sq;
    if (c <= 0.0) {
      continue;
    }
    const double a = a1 * a1 + b1 * b1;
    if (a <= 0.0) {
      feasible = false;
      break;
    }
    const double b = -2.0 * (a0 * a1 + b0 * b1);
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) {
      feasible = false;
      break;
    }
    const double t_leg = (-b - std::sqrt(disc)) / (2.0 * a);
    if (t_leg > t_required) {
      t_required = t_leg;
    }
  }

  if (!feasible || rho * t_required >= 1.0) {
    double max_r = 0.0;
    for (const auto& [name, r] : leg_mounts) {
      (void)name;
      const double rr = std::hypot(r[0], r[1]);
      if (rr > max_r) {
        max_r = rr;
      }
    }
    const double omega_v_outer = std::abs(omega_z) * max_r;
    if (omega_v_outer > linear_max) {
      return {0.0, 0.0, omega_z * (linear_max / omega_v_outer)};
    }
    return {0.0, 0.0, omega_z};
  }

  const double s_v = 1.0 - rho * t_required;
  const double s_w = 1.0 - t_required;
  return {v_x * s_v, v_y * s_v, omega_z * s_w};
}

}  // namespace hexa_gait
