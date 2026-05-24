#pragma once

#include <string>

#include "hexa_hardware/transport.hpp"

namespace hexa_hardware {

// POSIX serial transport (raw 8N1, blocking with VTIME=1; timing driven
// by poll()). Works for true UARTs and for USB-CDC bridges like the
// Servo 2040, which ignores the baud rate but accepts the device.
class UartTransport final : public Transport {
 public:
  UartTransport(std::string device, int baud);
  ~UartTransport() override;

  void open() override;
  void close() override;
  bool is_open() const override { return fd_ >= 0; }
  void write(std::span<const std::uint8_t> data) override;
  std::size_t read(std::span<std::uint8_t> buf, int timeout_ms) override;

 private:
  std::string device_;
  int baud_;
  int fd_ = -1;
};

}  // namespace hexa_hardware
