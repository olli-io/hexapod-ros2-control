// Byte-pipe abstraction for the link between the host and a servo
// controller board. Concrete implementations cover physical layers
// (UART, I2C, USB). Framing, command semantics, and request/response
// orchestration are the BoardProtocol's job — Transport only moves bytes.

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace hexa_hardware {

class Transport {
 public:
  virtual ~Transport() = default;

  Transport(const Transport&) = delete;
  Transport& operator=(const Transport&) = delete;

  virtual void open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;

  // Blocking write of the entire buffer. Throws std::runtime_error on
  // I/O failure or if the transport is not open.
  virtual void write(std::span<const std::uint8_t> data) = 0;

  // Read up to buf.size() bytes within timeout_ms total. Returns bytes
  // actually read; may be less than requested on timeout.
  virtual std::size_t read(std::span<std::uint8_t> buf, int timeout_ms) = 0;

 protected:
  Transport() = default;
};

}  // namespace hexa_hardware
