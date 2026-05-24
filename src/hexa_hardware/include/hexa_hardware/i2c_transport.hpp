#pragma once

#include <cstdint>
#include <string>

#include "hexa_hardware/transport.hpp"

namespace hexa_hardware {

// Placeholder for an I2C transport. Wired through the factory so
// `connection.type: i2c` parses cleanly; open() throws to flag that
// the body has not been implemented yet.
class I2cTransport final : public Transport {
 public:
  I2cTransport(std::string bus_path, std::uint8_t address);
  ~I2cTransport() override = default;

  void open() override;
  void close() override;
  bool is_open() const override { return false; }
  void write(std::span<const std::uint8_t> data) override;
  std::size_t read(std::span<std::uint8_t> buf, int timeout_ms) override;

 private:
  std::string bus_path_;
  std::uint8_t address_;
};

}  // namespace hexa_hardware
