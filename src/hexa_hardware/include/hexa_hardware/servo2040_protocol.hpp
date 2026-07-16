// "Chica" binary protocol used by the Pimoroni Servo 2040 board.
//
// Framing rule (used for resync): command bytes have MSB=1, data bytes
// have MSB=0. A 14-bit value `v` packs into two data bytes as
// `lo = v & 0x7F; hi = (v >> 7) & 0x7F`.

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "hexa_hardware/board_protocol.hpp"
#include "hexa_hardware/transport.hpp"

namespace hexa_hardware {

constexpr std::uint8_t kCmdSet = 'S' | 0x80;
constexpr std::uint8_t kCmdGet = 'G' | 0x80;
constexpr std::uint16_t kValueMax = 0x3FFF;  // 14-bit
constexpr std::size_t kMaxBatch = 64;        // 128 pin space; one frame must fit comfortably

// Fixed command indices in the board's cmdPins space (see firmware main.h).
// CURR and VOLT are consecutive, so one GET(kCurrIndex, 2) returns both.
constexpr std::uint8_t kCurrIndex = 24;
constexpr std::uint8_t kVoltIndex = 25;
constexpr std::uint8_t kRelayIndex = 26;
// Read-only latched fault register (no pin). GET(kStatusIndex, 1) returns a
// 14-bit word: bit0 = TRIPPED (over-current latch active), bits1-10 =
// TRIP_CURRENT at 0.1 A/count. 0 = clean.
constexpr std::uint8_t kStatusIndex = 27;
constexpr std::uint16_t kStatusTrippedBit = 0x1;
constexpr float kTripAmpsPerCount = 0.1f;

// Battery telemetry wire units: the board sends fixed-point centi-units
// (count = round(value * 100)), so one count is 0.01 V / 0.01 A. Must match the
// firmware TELEMETRY_COUNTS_PER_UNIT. This is the only telemetry conversion the
// host performs — a fixed protocol constant, not per-board calibration.
constexpr float kVoltsPerCount = 0.01f;
constexpr float kAmpsPerCount = 0.01f;

// Encode a SET frame into `out` (cleared first). Values are clamped to 14 bits.
void encode_set(std::uint8_t start, std::span<const std::uint16_t> values,
                std::vector<std::uint8_t>& out);

// Encode a GET request frame (3 bytes) into `out`.
void encode_get(std::uint8_t start, std::uint8_t count,
                std::vector<std::uint8_t>& out);

// Decode a GET reply payload. `payload` must start *after* the GET command
// byte (i.e. start with [start_idx][count][val_lo][val_hi]...). Returns true
// on success and fills `start` / `values`. Returns false if the buffer is
// too short or count doesn't match.
bool decode_get_payload(std::span<const std::uint8_t> payload,
                        std::uint8_t& start,
                        std::vector<std::uint16_t>& values);

// BoardProtocol implementation for the Servo 2040 / Chica protocol.
// Holds a Transport& (not owned); the controller-manager calls in from
// a single thread, so no synchronisation is required.
class Servo2040Protocol final : public BoardProtocol {
 public:
  explicit Servo2040Protocol(Transport& transport) : transport_(transport) {}

  void send_servo_positions(std::uint8_t start_pin,
                            std::span<const std::uint16_t> values) override;
  void set_servo_power(bool on) override;
  bool read_battery(float& voltage_v, float& current_a,
                    int timeout_ms) override;
  bool read_status(bool& tripped, float& trip_amps, int timeout_ms) override;

 private:
  // Issue a GET for `count` raw values from `count` consecutive indices and
  // decode the reply. Returns true and fills `out` on a complete reply within
  // timeout_ms; false on timeout or framing error.
  bool get_raw(std::uint8_t start, std::uint8_t count,
               std::vector<std::uint16_t>& out, int timeout_ms);

  Transport& transport_;
  // Reusable buffers to avoid per-call allocation on the hot path.
  std::vector<std::uint8_t> encode_buf_;
  std::vector<std::uint16_t> decode_buf_;
};

}  // namespace hexa_hardware
