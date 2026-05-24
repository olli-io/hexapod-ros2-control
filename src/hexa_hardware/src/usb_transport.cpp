#include "hexa_hardware/usb_transport.hpp"

#include <stdexcept>
#include <utility>

// THIS IS A PLACEHOLDER, NOT A COMPLETE MODULE

namespace hexa_hardware {

UsbTransport::UsbTransport(std::string device_path)
    : device_path_(std::move(device_path)) {}

void UsbTransport::open() {
  // TODO: implement using libusb or hidapi, depending on the eventual
  // board's USB interface. USB-CDC ACM boards (e.g. the Servo 2040)
  // already work via UartTransport; this path is for raw HID / bulk.
  throw std::runtime_error(
      "hexa_hardware: UsbTransport(" + device_path_ + ") not yet implemented");
}

void UsbTransport::close() {}

void UsbTransport::write(std::span<const std::uint8_t> /*data*/) {
  throw std::runtime_error("hexa_hardware: UsbTransport::write not implemented");
}

std::size_t UsbTransport::read(std::span<std::uint8_t> /*buf*/, int /*timeout_ms*/) {
  throw std::runtime_error("hexa_hardware: UsbTransport::read not implemented");
}

}  // namespace hexa_hardware
