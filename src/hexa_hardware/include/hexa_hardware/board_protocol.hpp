// Semantic operations a servo controller board exposes to the
// hardware interface. Concrete subclasses own a Transport reference,
// frame/parse the board's wire protocol, and translate the calls
// below into bytes on the link.

#pragma once

#include <cstdint>
#include <span>
#include <vector>

namespace hexa_hardware {

class BoardProtocol {
 public:
  virtual ~BoardProtocol() = default;

  BoardProtocol(const BoardProtocol&) = delete;
  BoardProtocol& operator=(const BoardProtocol&) = delete;

  // Drive `values.size()` consecutive servo pins starting at start_pin
  // to the given raw pulse-width values. Interpretation of the value
  // (µs, ticks, …) is board-specific.
  virtual void send_servo_positions(std::uint8_t start_pin,
                                    std::span<const std::uint16_t> values) = 0;

  // Drive a single digital output (e.g. the relay rail).
  virtual void send_digital(std::uint8_t pin, bool value) = 0;

  // Request `count` raw values from `count` consecutive pins (ADC,
  // touch, etc.). Returns true and populates `out` on a complete reply
  // within timeout_ms; false on timeout or framing error.
  virtual bool read_aux(std::uint8_t start_pin, std::uint8_t count,
                        std::vector<std::uint16_t>& out, int timeout_ms) = 0;

 protected:
  BoardProtocol() = default;
};

}  // namespace hexa_hardware
