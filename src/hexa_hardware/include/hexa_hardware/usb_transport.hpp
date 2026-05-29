#pragma once

#include <cstdint>
#include <string>

#include "hexa_hardware/transport.hpp"

namespace hexa_hardware {

// Raw USB transport (libusb bulk endpoints) — distinct from the USB-CDC
// ACM case, which goes through UartTransport. Use this for boards that
// expose a custom vendor-class interface rather than a serial endpoint.
//
// device_path encodes the device + endpoints in lsusb-style form:
//   "vid:pid[/iface[/ep_in/ep_out]]"
// VID/PID are hex; iface/endpoints accept decimal or 0x-prefixed hex.
// Defaults: iface=0, ep_in=0x81, ep_out=0x01 (EP1 IN/OUT — what most
// single-interface vendor-class firmware exposes).
class UsbTransport final : public Transport {
 public:
  explicit UsbTransport(std::string device_path);
  ~UsbTransport() override;

  void open() override;
  void close() override;
  bool is_open() const override { return handle_ != nullptr; }
  void write(std::span<const std::uint8_t> data) override;
  std::size_t read(std::span<std::uint8_t> buf, int timeout_ms) override;

 private:
  std::string device_path_;
  // Opaque libusb pointers; void* keeps libusb headers out of the public
  // include surface (only the .cpp links against libusb).
  void* ctx_ = nullptr;
  void* handle_ = nullptr;
  int interface_ = 0;
  std::uint8_t ep_in_ = 0x81;
  std::uint8_t ep_out_ = 0x01;
};

}  // namespace hexa_hardware
