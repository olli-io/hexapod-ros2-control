// Headless: register decode, mount rotation and the attitude filter. No device.

#include <array>
#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "attitude_filter.hpp"
#include "mpu6500.hpp"

namespace {

using hexa::imu::AttitudeFilter;
using hexa::imu::AttitudeFilterConfig;

constexpr double kG = 9.80665;

std::array<double, 3> gravity_at(double roll, double pitch) {
  // Gravity in the body frame of a body at (roll, pitch): R^T * (0, 0, g).
  return {-kG * std::sin(pitch), kG * std::sin(roll) * std::cos(pitch),
          kG * std::cos(roll) * std::cos(pitch)};
}

TEST(Mpu6500, DecodesBigEndianBurst) {
  const auto s = hexa::imu::scales_for(4, 500);
  std::array<std::uint8_t, 14> b{};
  b[4] = 0x20;  // accel z = 8192 LSB = 1 g at +-4 g
  b[0] = 0xF0;  // accel x = -4096 LSB = -0.5 g
  b[8] = 0x00;
  b[9] = 0x83;  // gyro x = 131 LSB = 2 dps at +-500 dps (65.5 LSB/dps)
  const auto r = hexa::imu::decode_burst(b, s);
  EXPECT_NEAR(r.accel[2], kG, 1e-9);
  EXPECT_NEAR(r.accel[0], -0.5 * kG, 1e-9);
  EXPECT_NEAR(r.gyro[0], 2.0 * M_PI / 180.0, 1e-9);
}

TEST(Mpu6500, RejectsRangesTheChipLacks) {
  EXPECT_THROW(hexa::imu::scales_for(3, 500), std::invalid_argument);
  EXPECT_THROW(hexa::imu::dlpf_code(50), std::invalid_argument);
  EXPECT_EQ(hexa::imu::dlpf_code(41), 3);
}

TEST(AttitudeFilter, SeedsFromGravity) {
  AttitudeFilter f(AttitudeFilterConfig{});
  const auto& a = f.update(gravity_at(0.1, -0.2), {0, 0, 0}, 0.005);
  EXPECT_NEAR(a.roll, 0.1, 1e-9);
  EXPECT_NEAR(a.pitch, -0.2, 1e-9);
}

TEST(AttitudeFilter, IntegratesTheGyroAndTrimsDrift) {
  AttitudeFilter f(AttitudeFilterConfig{0.5, 0.15});
  f.update(gravity_at(0, 0), {0, 0, 0}, 0.005);
  // A 0.2 rad/s roll for 1 s, gravity following.
  double roll = 0.0;
  for (int i = 0; i < 200; ++i) {
    roll += 0.2 * 0.005;
    f.update(gravity_at(roll, 0), {0.2, 0, 0}, 0.005);
  }
  EXPECT_NEAR(f.attitude()->roll, 0.2, 5e-3);
  // A residual gyro bias is pulled back by the accelerometer.
  for (int i = 0; i < 2000; ++i) {
    f.update(gravity_at(0.2, 0), {0.01, 0, 0}, 0.005);
  }
  EXPECT_NEAR(f.attitude()->roll, 0.2, 0.01);
}

TEST(AttitudeFilter, IgnoresAccelUnderHardAcceleration) {
  AttitudeFilter f(AttitudeFilterConfig{0.5, 0.15});
  f.update(gravity_at(0, 0), {0, 0, 0}, 0.005);
  for (int i = 0; i < 200; ++i) {
    f.update({0.0, 5.0, kG * 1.4}, {0, 0, 0}, 0.005);
  }
  EXPECT_NEAR(f.attitude()->roll, 0.0, 1e-9);
}

TEST(Mount, RotatesSensorAxesIntoTheBody) {
  // Sensor yawed 90 deg: its +x is the body's +y.
  const auto v = hexa::imu::rotate_rpy({1, 0, 0}, 0, 0, M_PI / 2);
  EXPECT_NEAR(v[0], 0.0, 1e-12);
  EXPECT_NEAR(v[1], 1.0, 1e-12);
}

TEST(Quaternion, MatchesRollPitch) {
  const auto q = hexa::imu::quaternion_from_rpy(0.1, -0.2, 0.0);
  const double roll = std::atan2(2 * (q[3] * q[0] + q[1] * q[2]),
                                 1 - 2 * (q[0] * q[0] + q[1] * q[1]));
  const double pitch = std::asin(2 * (q[3] * q[1] - q[2] * q[0]));
  EXPECT_NEAR(roll, 0.1, 1e-12);
  EXPECT_NEAR(pitch, -0.2, 1e-12);
}

}  // namespace
