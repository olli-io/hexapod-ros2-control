#include "attitude_filter.hpp"

#include <cmath>

namespace hexa::imu {

namespace {
constexpr double kGravity = 9.80665;
}

Attitude accel_tilt(const std::array<double, 3>& a) {
  Attitude t;
  t.roll = std::atan2(a[1], a[2]);
  t.pitch = std::atan2(-a[0], std::hypot(a[1], a[2]));
  return t;
}

const Attitude& AttitudeFilter::update(const std::array<double, 3>& accel,
                                       const std::array<double, 3>& gyro,
                                       double dt) {
  if (!state_.has_value()) {
    state_ = accel_tilt(accel);
    return *state_;
  }
  Attitude& s = *state_;
  // Body rates to Euler rates (ZYX).
  const double sr = std::sin(s.roll), cr = std::cos(s.roll);
  const double cp = std::cos(s.pitch), tp = std::tan(s.pitch);
  s.roll += dt * (gyro[0] + sr * tp * gyro[1] + cr * tp * gyro[2]);
  s.pitch += dt * (cr * gyro[1] - sr * gyro[2]);
  if (std::fabs(cp) > 1e-6) {
    s.yaw += dt * (sr * gyro[1] + cr * gyro[2]) / cp;
  }
  s.yaw = std::remainder(s.yaw, 2.0 * M_PI);

  // A body accelerating hard is not reading gravity; trust the gyro then.
  const double norm =
      std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
  if (std::fabs(norm - kGravity) <= cfg_.accel_reject * kGravity) {
    const Attitude t = accel_tilt(accel);
    const double k = cfg_.tau > 0.0 ? dt / (cfg_.tau + dt) : 1.0;
    s.roll += k * std::remainder(t.roll - s.roll, 2.0 * M_PI);
    s.pitch += k * (t.pitch - s.pitch);
  }
  return s;
}

std::array<double, 3> rotate_rpy(const std::array<double, 3>& v, double roll,
                                 double pitch, double yaw) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  // Rx
  const double x1 = v[0];
  const double y1 = cr * v[1] - sr * v[2];
  const double z1 = sr * v[1] + cr * v[2];
  // Ry
  const double x2 = cp * x1 + sp * z1;
  const double y2 = y1;
  const double z2 = -sp * x1 + cp * z1;
  // Rz
  return {cy * x2 - sy * y2, sy * x2 + cy * y2, z2};
}

std::array<double, 4> quaternion_from_rpy(double roll, double pitch,
                                          double yaw) {
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
}

}  // namespace hexa::imu
