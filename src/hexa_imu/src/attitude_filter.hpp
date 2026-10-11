// Complementary attitude filter: gyro integration for the fast part, the
// accelerometer's gravity direction for the slow part. Yaw is gyro-only and
// drifts. Pure: no clock, the caller passes dt.
#pragma once

#include <array>
#include <optional>

namespace hexa::imu {

struct AttitudeFilterConfig {
  double tau = 0.5;           // s, crossover; the accel trims below 1/tau
  double accel_reject = 0.15; // share of g; skip the accel trim beyond it
};

struct Attitude {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

class AttitudeFilter {
 public:
  explicit AttitudeFilter(AttitudeFilterConfig cfg) : cfg_(cfg) {}

  // accel in m/s^2, gyro in rad/s, body frame (REP-103). The first call seeds
  // roll/pitch from the accelerometer.
  const Attitude& update(const std::array<double, 3>& accel,
                         const std::array<double, 3>& gyro, double dt);

  void reset() { state_.reset(); }
  std::optional<Attitude> attitude() const { return state_; }

 private:
  AttitudeFilterConfig cfg_;
  std::optional<Attitude> state_;
};

// Roll/pitch from the gravity direction alone.
Attitude accel_tilt(const std::array<double, 3>& accel);

// Rotate a vector by intrinsic roll-pitch-yaw (R = Rz * Ry * Rx).
std::array<double, 3> rotate_rpy(const std::array<double, 3>& v, double roll,
                                 double pitch, double yaw);

// Quaternion (x, y, z, w) of R = Rz(yaw) * Ry(pitch) * Rx(roll).
std::array<double, 4> quaternion_from_rpy(double roll, double pitch, double yaw);

}  // namespace hexa::imu
