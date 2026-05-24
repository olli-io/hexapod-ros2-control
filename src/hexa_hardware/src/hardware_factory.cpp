#include "hexa_hardware/hardware_factory.hpp"

#include <stdexcept>

#include "hexa_hardware/i2c_transport.hpp"
#include "hexa_hardware/servo2040_protocol.hpp"
#include "hexa_hardware/uart_transport.hpp"
#include "hexa_hardware/usb_transport.hpp"

namespace hexa_hardware {

std::unique_ptr<Transport> make_transport(const HardwareConfig& cfg) {
  const auto& type = cfg.connection.type;
  if (type == "uart") {
    return std::make_unique<UartTransport>(cfg.connection.device,
                                           cfg.connection.baud);
  }
  if (type == "i2c") {
    // `baud` field is repurposed as the 7-bit slave address when type=i2c.
    return std::make_unique<I2cTransport>(
        cfg.connection.device, static_cast<std::uint8_t>(cfg.connection.baud));
  }
  if (type == "usb") {
    return std::make_unique<UsbTransport>(cfg.connection.device);
  }
  throw std::runtime_error(
      "hexa_hardware: unknown connection.type '" + type +
      "' (known: uart, i2c, usb)");
}

std::unique_ptr<BoardProtocol> make_board_protocol(const HardwareConfig& cfg,
                                                   Transport& transport) {
  const auto& type = cfg.parser.type;
  if (type == "servo2040") {
    return std::make_unique<Servo2040Protocol>(transport);
  }
  throw std::runtime_error(
      "hexa_hardware: unknown parser.type '" + type +
      "' (known: servo2040)");
}

}  // namespace hexa_hardware
