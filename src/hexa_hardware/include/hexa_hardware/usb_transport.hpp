#pragma once

#include <string>

#include "hexa_hardware/transport.hpp"

namespace hexa_hardware {

// Placeholder for a USB (HID / bulk) transport — distinct from the
// USB-CDC ACM case, which already works through UartTransport.
// Wired through the factory so `connection.type: usb` parses cleanly;
// open() throws to flag that the body has not been implemented yet.
class UsbTransport final : public Transport {
 public:
  explicit UsbTransport(std::string device_path);
  ~UsbTransport() override = default;

  void open() override;
  void close() override;
  bool is_open() const override { return false; }
  void write(std::span<const std::uint8_t> data) override;
  std::size_t read(std::span<std::uint8_t> buf, int timeout_ms) override;

 private:
  std::string device_path_;
};

}  // namespace hexa_hardware
