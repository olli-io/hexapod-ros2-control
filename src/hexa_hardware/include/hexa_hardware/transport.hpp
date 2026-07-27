// Byte-pipe abstraction for the link between the host and a servo
// controller board. Concrete implementations cover physical layers
// (UART, I2C, USB). Framing, command semantics, and request/response
// orchestration are the BoardProtocol's job — Transport only moves bytes.

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace hexa_hardware {

// Threading contract — one reader, many writers.
//
// hexa_hardware drives the link from two threads: the controller-manager
// thread emits SET/SETALL frames every control cycle, and the aux thread owns
// the GET request/response round trips (see hexa_hardware.hpp). An
// implementation must therefore make `write()` atomic with respect to other
// `write()` calls, so two frames can never interleave their bytes on the wire.
//
// `read()` is single-reader by construction — only the aux thread ever calls it
// — so it needs no locking, and must NOT share a lock with `write()`: a reader
// blocked on a slow board would then stall the control cycle's SET, which is the
// exact coupling this split exists to remove. Serial links are full duplex, so a
// concurrent write cannot corrupt an in-flight read.
//
// `open()` / `close()` are lifecycle-only and are called with no other thread
// running (on_configure, and on_cleanup after the aux thread is joined).
class Transport {
 public:
  virtual ~Transport() = default;

  Transport(const Transport&) = delete;
  Transport& operator=(const Transport&) = delete;

  virtual void open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;

  // Blocking write of the entire buffer, atomic against concurrent writes.
  // Throws std::runtime_error on I/O failure or if the transport is not open.
  virtual void write(std::span<const std::uint8_t> data) = 0;

  // Read up to buf.size() bytes within timeout_ms total. Returns bytes
  // actually read; may be less than requested on timeout. Single-reader.
  virtual std::size_t read(std::span<std::uint8_t> buf, int timeout_ms) = 0;

 protected:
  Transport() = default;
};

}  // namespace hexa_hardware
