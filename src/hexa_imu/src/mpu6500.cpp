#include "mpu6500.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace hexa::imu {

namespace {

constexpr std::uint8_t kSmplrtDiv = 0x19;
constexpr std::uint8_t kConfig = 0x1A;
constexpr std::uint8_t kGyroConfig = 0x1B;
constexpr std::uint8_t kAccelConfig = 0x1C;
constexpr std::uint8_t kAccelConfig2 = 0x1D;
constexpr std::uint8_t kAccelXoutH = 0x3B;
constexpr std::uint8_t kPwrMgmt1 = 0x6B;
constexpr std::uint8_t kWhoAmI = 0x75;

constexpr std::uint8_t kDeviceReset = 0x80;
constexpr std::uint8_t kClockPll = 0x01;
// MPU6500; MPU9250 is the same die plus a magnetometer.
constexpr std::uint8_t kWhoAmI6500 = 0x70;
constexpr std::uint8_t kWhoAmI9250 = 0x71;

constexpr double kGravity = 9.80665;
constexpr double kDegToRad = M_PI / 180.0;
constexpr int kInternalRateHz = 1000;

std::string errno_text(const std::string& what) {
  return what + ": " + std::strerror(errno);
}

std::int16_t be16(std::uint8_t hi, std::uint8_t lo) {
  return static_cast<std::int16_t>((static_cast<std::uint16_t>(hi) << 8) | lo);
}

std::uint8_t range_code(int value, std::initializer_list<int> table,
                        const char* what) {
  std::uint8_t code = 0;
  for (int entry : table) {
    if (entry == value) {
      return code;
    }
    ++code;
  }
  throw std::invalid_argument(std::string("mpu6500: unsupported ") + what +
                              " " + std::to_string(value));
}

}  // namespace

Scales scales_for(int accel_range_g, int gyro_range_dps) {
  const int a = range_code(accel_range_g, {2, 4, 8, 16}, "accel_range_g");
  const int g = range_code(gyro_range_dps, {250, 500, 1000, 2000},
                           "gyro_range_dps");
  return {16384.0 / (1 << a), 131.0 / (1 << g)};
}

std::uint8_t dlpf_code(int dlpf_hz) {
  // Code 0 (250 Hz) is left out: it bypasses the accel filter differently.
  return static_cast<std::uint8_t>(
      1 + range_code(dlpf_hz, {184, 92, 41, 20, 10, 5}, "dlpf_hz"));
}

RawSample decode_burst(const std::array<std::uint8_t, 14>& b, const Scales& s) {
  RawSample out;
  for (std::size_t i = 0; i < 3; ++i) {
    out.accel[i] = be16(b[2 * i], b[2 * i + 1]) / s.accel_lsb_per_g * kGravity;
    out.gyro[i] =
        be16(b[8 + 2 * i], b[9 + 2 * i]) / s.gyro_lsb_per_dps * kDegToRad;
  }
  return out;
}

Mpu6500::Mpu6500(Mpu6500Config cfg)
    : cfg_(std::move(cfg)),
      scales_(scales_for(cfg_.accel_range_g, cfg_.gyro_range_dps)) {}

Mpu6500::~Mpu6500() { close(); }

void Mpu6500::open() {
  close();
  fd_ = ::open(cfg_.bus.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error(errno_text("open " + cfg_.bus));
  }
  if (ioctl(fd_, I2C_SLAVE, cfg_.address) < 0) {
    const std::string err = errno_text("I2C_SLAVE");
    close();
    throw std::runtime_error(err);
  }
  try {
    std::uint8_t who = 0;
    read_regs(kWhoAmI, &who, 1);
    if (who != kWhoAmI6500 && who != kWhoAmI9250) {
      throw std::runtime_error("mpu6500: unexpected WHO_AM_I " +
                               std::to_string(who));
    }
    write_reg(kPwrMgmt1, kDeviceReset);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_reg(kPwrMgmt1, kClockPll);
    const std::uint8_t dlpf = dlpf_code(cfg_.dlpf_hz);
    write_reg(kConfig, dlpf);
    write_reg(kAccelConfig2, dlpf);
    write_reg(kSmplrtDiv,
              static_cast<std::uint8_t>(kInternalRateHz / cfg_.sample_rate_hz - 1));
    write_reg(kGyroConfig,
              static_cast<std::uint8_t>(range_code(cfg_.gyro_range_dps,
                                                   {250, 500, 1000, 2000},
                                                   "gyro_range_dps")
                                        << 3));
    write_reg(kAccelConfig,
              static_cast<std::uint8_t>(
                  range_code(cfg_.accel_range_g, {2, 4, 8, 16}, "accel_range_g")
                  << 3));
  } catch (...) {
    close();
    throw;
  }
}

void Mpu6500::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

RawSample Mpu6500::read() {
  std::array<std::uint8_t, 14> b{};
  read_regs(kAccelXoutH, b.data(), b.size());
  return decode_burst(b, scales_);
}

void Mpu6500::write_reg(std::uint8_t reg, std::uint8_t value) {
  const std::uint8_t buf[2] = {reg, value};
  if (::write(fd_, buf, 2) != 2) {
    throw std::runtime_error(errno_text("mpu6500 write"));
  }
}

void Mpu6500::read_regs(std::uint8_t reg, std::uint8_t* out, std::size_t n) {
  // One combined transaction (repeated start), so no other master can slip in
  // between the register pointer and the burst.
  i2c_msg msgs[2] = {
      {static_cast<__u16>(cfg_.address), 0, 1, &reg},
      {static_cast<__u16>(cfg_.address), I2C_M_RD, static_cast<__u16>(n), out},
  };
  i2c_rdwr_ioctl_data xfer{msgs, 2};
  if (ioctl(fd_, I2C_RDWR, &xfer) < 0) {
    throw std::runtime_error(errno_text("mpu6500 read"));
  }
}

}  // namespace hexa::imu
