// MPU6500 over Linux i2c-dev: raw ioctl, no external library. The register
// decode is a free function so the host test runs without a device.
#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace hexa::imu {

struct Mpu6500Config {
  std::string bus = "/dev/i2c-1";
  int address = 0x68;
  int accel_range_g = 4;       // 2, 4, 8, 16
  int gyro_range_dps = 500;    // 250, 500, 1000, 2000
  int dlpf_hz = 41;            // 184, 92, 41, 20, 10, 5
  int sample_rate_hz = 200;    // internal rate, 1 kHz / (1 + SMPLRT_DIV)
};

// SI units, sensor frame.
struct RawSample {
  std::array<double, 3> accel{};  // m/s^2
  std::array<double, 3> gyro{};   // rad/s
};

// Full-scale ranges -> LSB scales. Throws std::invalid_argument on a range
// the chip does not have.
struct Scales {
  double accel_lsb_per_g;
  double gyro_lsb_per_dps;
};
Scales scales_for(int accel_range_g, int gyro_range_dps);

// The 14-byte burst from ACCEL_XOUT_H: accel xyz, temperature, gyro xyz, each
// big-endian int16.
RawSample decode_burst(const std::array<std::uint8_t, 14>& b, const Scales& s);

// DLPF_CFG / A_DLPF_CFG code for a bandwidth. Throws std::invalid_argument.
std::uint8_t dlpf_code(int dlpf_hz);

class Mpu6500 {
 public:
  explicit Mpu6500(Mpu6500Config cfg);
  ~Mpu6500();
  Mpu6500(const Mpu6500&) = delete;
  Mpu6500& operator=(const Mpu6500&) = delete;

  // Opens the bus, checks WHO_AM_I and configures the chip. Throws
  // std::runtime_error with the reason.
  void open();
  void close();
  bool is_open() const { return fd_ >= 0; }

  // One burst read. Throws std::runtime_error on a bus error.
  RawSample read();

 private:
  void write_reg(std::uint8_t reg, std::uint8_t value);
  void read_regs(std::uint8_t reg, std::uint8_t* out, std::size_t n);

  Mpu6500Config cfg_;
  Scales scales_;
  int fd_ = -1;
};

}  // namespace hexa::imu
