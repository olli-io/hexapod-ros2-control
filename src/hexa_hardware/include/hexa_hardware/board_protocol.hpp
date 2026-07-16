// Semantic operations a servo controller board exposes to the
// hardware interface. Concrete subclasses own a Transport reference,
// frame/parse the board's wire protocol, and translate the calls
// below into bytes on the link.

#pragma once

#include <cstdint>
#include <span>

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

  // Energise (true) or de-energise (false) the servo power rail. The relay
  // pin is chosen entirely on the board — the host addresses this by intent,
  // not by pin number. Note: while an over-current trip is latched some boards
  // refuse re-enable until a disable clears the latch.
  virtual void set_servo_power(bool on) = 0;

  // Read the battery bus in engineering units: `voltage_v` in volts,
  // `current_a` in amps. The board reports fixed-point units on the wire and
  // this call applies the fixed protocol conversion, so callers get real units
  // and carry no scaling. Returns true on a complete reply within timeout_ms;
  // false on timeout or framing error (leaving the outputs unspecified).
  virtual bool read_battery(float& voltage_v, float& current_a,
                            int timeout_ms) = 0;

  // Read the board's fault/status register. `tripped` is true while an
  // over-current latch is active (the board has already disabled the servo
  // cluster and dropped the relay); `trip_amps` is the current captured at the
  // trip. The latch is sticky — it keeps reporting until a disable
  // (set_servo_power(false)) clears it. Returns true on a complete reply within
  // timeout_ms; false on timeout or framing error (outputs then unspecified).
  virtual bool read_status(bool& tripped, float& trip_amps, int timeout_ms) = 0;

 protected:
  BoardProtocol() = default;
};

}  // namespace hexa_hardware
