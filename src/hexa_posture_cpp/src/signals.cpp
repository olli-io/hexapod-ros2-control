// Port of the pure free functions and constants in hexa_posture/posture_node.py.
#include "hexa_posture_cpp/signals.hpp"

#include <algorithm>
#include <cmath>

namespace hexa_posture {

bool is_posture_active(const std::string& state) {
  return POSTURE_ACTIVE_STATES.find(state) != POSTURE_ACTIVE_STATES.end();
}

bool is_gait_engaged(const std::string& state) {
  return GAIT_ENGAGED_STATES.find(state) != GAIT_ENGAGED_STATES.end();
}

double slew_toward(double current, double target, double rate_per_s,
                   double dt) {
  if (rate_per_s <= 0.0 || dt <= 0.0) {
    return current;
  }
  const double step = rate_per_s * dt;
  if (target > current) {
    return std::min(target, current + step);
  }
  return std::max(target, current - step);
}

bool twist_is_zero(double linear_x, double linear_y, double linear_z,
                   double angular_x, double angular_y, double angular_z) {
  return std::abs(linear_x) < CMD_VEL_ZERO_TOL &&
         std::abs(linear_y) < CMD_VEL_ZERO_TOL &&
         std::abs(linear_z) < CMD_VEL_ZERO_TOL &&
         std::abs(angular_x) < CMD_VEL_ZERO_TOL &&
         std::abs(angular_y) < CMD_VEL_ZERO_TOL &&
         std::abs(angular_z) < CMD_VEL_ZERO_TOL;
}

std::optional<std::pair<double, double>> stance_centroid_xy(
    const std::array<Vec3, LEG_COUNT>& feet,
    const std::array<bool, LEG_COUNT>& stance) {
  double sum_x = 0.0;
  double sum_y = 0.0;
  int n = 0;
  for (std::size_t i = 0; i < LEG_COUNT; ++i) {
    if (stance[i]) {
      sum_x += feet[i][0];
      sum_y += feet[i][1];
      ++n;
    }
  }
  if (n < MIN_STANCE_FOR_CENTROID) {
    return std::nullopt;
  }
  const double count = static_cast<double>(n);
  return std::make_pair(sum_x / count, sum_y / count);
}

std::optional<double> max_swing_lift_z(
    const std::array<Vec3, LEG_COUNT>& feet,
    const std::array<bool, LEG_COUNT>& stance) {
  double stance_sum_z = 0.0;
  int stance_n = 0;
  bool any_swing = false;
  double max_swing_z = 0.0;
  for (std::size_t i = 0; i < LEG_COUNT; ++i) {
    if (stance[i]) {
      stance_sum_z += feet[i][2];
      ++stance_n;
    } else {
      if (!any_swing || feet[i][2] > max_swing_z) {
        max_swing_z = feet[i][2];
      }
      any_swing = true;
    }
  }
  if (stance_n < MIN_STANCE_FOR_CENTROID) {
    return std::nullopt;
  }
  if (!any_swing) {
    return 0.0;
  }
  const double ground = stance_sum_z / static_cast<double>(stance_n);
  const double lift = max_swing_z - ground;
  return lift > 0.0 ? lift : 0.0;
}

std::optional<double> lpf_step_scalar(std::optional<double> prev,
                                      std::optional<double> raw, double tau,
                                      double dt) {
  if (!raw.has_value()) {
    return prev;
  }
  if (!prev.has_value()) {
    return raw;
  }
  const double denom = tau + dt;
  const double alpha = denom > 0.0 ? dt / denom : 1.0;
  return *prev + alpha * (*raw - *prev);
}

std::optional<std::pair<double, double>> lpf_step_xy(
    std::optional<std::pair<double, double>> prev,
    std::optional<std::pair<double, double>> raw, double tau, double dt) {
  if (!raw.has_value()) {
    return prev;
  }
  if (!prev.has_value()) {
    return raw;
  }
  const double denom = tau + dt;
  const double alpha = denom > 0.0 ? dt / denom : 1.0;
  const auto [px, py] = *prev;
  const auto [rx, ry] = *raw;
  return std::make_pair(px + alpha * (rx - px), py + alpha * (ry - py));
}

}  // namespace hexa_posture
