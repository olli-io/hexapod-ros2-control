#include "hexa_hardware/usb_transport.hpp"

#include <libusb-1.0/libusb.h>

#include <charconv>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace hexa_hardware {

namespace {

constexpr int kWriteTimeoutMs = 1000;

struct UsbAddress {
  std::uint16_t vid = 0;
  std::uint16_t pid = 0;
  int interface = 0;
  std::uint8_t ep_in = 0x81;
  std::uint8_t ep_out = 0x01;
};

int parse_int(std::string_view s, int base) {
  if (s.empty()) {
    throw std::runtime_error("hexa_hardware: empty USB token");
  }
  const char* first = s.data();
  const char* last = s.data() + s.size();
  if (base == 0) {
    if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
      first += 2;
      base = 16;
    } else {
      base = 10;
    }
  }
  int value = 0;
  const auto res = std::from_chars(first, last, value, base);
  if (res.ec != std::errc{} || res.ptr != last) {
    throw std::runtime_error("hexa_hardware: bad USB integer '" +
                             std::string(s) + "'");
  }
  return value;
}

UsbAddress parse_address(const std::string& path) {
  const auto colon = path.find(':');
  if (colon == std::string::npos) {
    throw std::runtime_error(
        "hexa_hardware: UsbTransport device '" + path +
        "' must be 'vid:pid[/iface[/ep_in/ep_out]]'");
  }
  const auto rest = path.find('/', colon);
  const auto pid_end = (rest == std::string::npos) ? path.size() : rest;

  UsbAddress addr;
  addr.vid = static_cast<std::uint16_t>(
      parse_int(std::string_view(path.data(), colon), 16));
  addr.pid = static_cast<std::uint16_t>(
      parse_int(std::string_view(path.data() + colon + 1, pid_end - colon - 1),
                16));

  if (rest == std::string::npos) return addr;

  std::vector<std::string_view> tokens;
  std::size_t i = rest + 1;
  while (i <= path.size()) {
    const auto j = path.find('/', i);
    const auto end = (j == std::string::npos) ? path.size() : j;
    tokens.emplace_back(path.data() + i, end - i);
    if (j == std::string::npos) break;
    i = j + 1;
  }
  if (tokens.size() > 3) {
    throw std::runtime_error(
        "hexa_hardware: too many fields in USB device '" + path + "'");
  }
  if (!tokens.empty()) addr.interface = parse_int(tokens[0], 0);
  if (tokens.size() >= 2) {
    addr.ep_in = static_cast<std::uint8_t>(parse_int(tokens[1], 0));
  }
  if (tokens.size() >= 3) {
    addr.ep_out = static_cast<std::uint8_t>(parse_int(tokens[2], 0));
  }
  return addr;
}

libusb_context* as_ctx(void* p) { return static_cast<libusb_context*>(p); }
libusb_device_handle* as_handle(void* p) {
  return static_cast<libusb_device_handle*>(p);
}

}  // namespace

UsbTransport::UsbTransport(std::string device_path)
    : device_path_(std::move(device_path)) {}

UsbTransport::~UsbTransport() {
  close();
}

void UsbTransport::open() {
  close();
  const UsbAddress addr = parse_address(device_path_);

  libusb_context* ctx = nullptr;
  int rc = libusb_init(&ctx);
  if (rc != 0) {
    throw std::runtime_error(
        std::string("hexa_hardware: libusb_init failed: ") +
        libusb_error_name(rc));
  }

  libusb_device_handle* handle =
      libusb_open_device_with_vid_pid(ctx, addr.vid, addr.pid);
  if (handle == nullptr) {
    libusb_exit(ctx);
    throw std::runtime_error(
        "hexa_hardware: USB device " + device_path_ +
        " not found or not openable (check permissions / udev rules)");
  }

  // Some kernels auto-bind a driver to vendor-class interfaces; detach so
  // we can claim. Ignore the result — it's only an error if a driver was
  // actually present and refused to release.
  if (libusb_kernel_driver_active(handle, addr.interface) == 1) {
    libusb_detach_kernel_driver(handle, addr.interface);
  }

  rc = libusb_claim_interface(handle, addr.interface);
  if (rc != 0) {
    libusb_close(handle);
    libusb_exit(ctx);
    throw std::runtime_error(
        "hexa_hardware: libusb_claim_interface(" +
        std::to_string(addr.interface) +
        ") failed: " + libusb_error_name(rc));
  }

  ctx_ = ctx;
  handle_ = handle;
  interface_ = addr.interface;
  ep_in_ = addr.ep_in;
  ep_out_ = addr.ep_out;
}

void UsbTransport::close() {
  if (handle_ != nullptr) {
    libusb_release_interface(as_handle(handle_), interface_);
    libusb_close(as_handle(handle_));
    handle_ = nullptr;
  }
  if (ctx_ != nullptr) {
    libusb_exit(as_ctx(ctx_));
    ctx_ = nullptr;
  }
}

void UsbTransport::write(std::span<const std::uint8_t> data) {
  if (handle_ == nullptr) {
    throw std::runtime_error("hexa_hardware: write on closed USB transport");
  }
  int sent = 0;
  const int rc = libusb_bulk_transfer(
      as_handle(handle_), ep_out_,
      const_cast<unsigned char*>(data.data()),
      static_cast<int>(data.size()), &sent, kWriteTimeoutMs);
  if (rc != 0 || static_cast<std::size_t>(sent) != data.size()) {
    throw std::runtime_error(
        std::string("hexa_hardware: USB bulk write failed: ") +
        libusb_error_name(rc));
  }
}

std::size_t UsbTransport::read(std::span<std::uint8_t> buf, int timeout_ms) {
  if (handle_ == nullptr) return 0;
  int got = 0;
  const int rc = libusb_bulk_transfer(
      as_handle(handle_), ep_in_, buf.data(),
      static_cast<int>(buf.size()), &got, timeout_ms);
  // Mirror UartTransport: timeouts and short reads return what we got;
  // hard errors also surface as a short/zero read so the protocol layer
  // can decide whether to retry or escalate.
  if (rc != 0 && rc != LIBUSB_ERROR_TIMEOUT) {
    return static_cast<std::size_t>(got);
  }
  return static_cast<std::size_t>(got);
}

}  // namespace hexa_hardware
