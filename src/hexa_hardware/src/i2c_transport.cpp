#include "hexa_hardware/i2c_transport.hpp"

#include <stdexcept>
#include <utility>

// THIS IS A PLACEHOLDER, NOT A COMPLETE MODULE

namespace hexa_hardware {

I2cTransport::I2cTransport(std::string bus_path, std::uint8_t address)
    : bus_path_(std::move(bus_path)), address_(address) {}

void I2cTransport::open() {
  // TODO: implement using <linux/i2c-dev.h> + ioctl(I2C_SLAVE).
  // bus_path_ is the device node (e.g. /dev/i2c-1) and address_ is
  // the 7-bit slave address of the servo board.
  throw std::runtime_error(
      "hexa_hardware: I2cTransport(" + bus_path_ + ") not yet implemented");
}

void I2cTransport::close() {}

void I2cTransport::write(std::span<const std::uint8_t> /*data*/) {
  throw std::runtime_error("hexa_hardware: I2cTransport::write not implemented");
}

std::size_t I2cTransport::read(std::span<std::uint8_t> /*buf*/, int /*timeout_ms*/) {
  throw std::runtime_error("hexa_hardware: I2cTransport::read not implemented");
}

}  // namespace hexa_hardware
