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

// SETALL — the compact all-servos fast path (firmware main.h SETALL_*).
//
// A SET addressing all 18 servos is [S][0][18] + 36 data bytes = 39 bytes, which
// exceeds the RP2040's 32-byte UART RX FIFO: if the firmware's main loop stalls
// while the frame streams in, the FIFO overruns and the *tail* of the frame — the
// last servo — is lost. SETALL packs the same pose into 30 bytes, which fits
// entirely inside the FIFO, so the loop can be busy for a whole frame and lose
// nothing. It is servo-only: the relay and all telemetry keep using SET/GET.
constexpr std::uint8_t kCmdSetAll = 0x55 | 0x80;      // 0xD5
constexpr std::size_t kSetAllServoCount = 18;
constexpr int kSetAllValueBits = 11;
constexpr std::uint16_t kSetAllPulseBaseUs = 500;     // value = pulse_us - base
constexpr std::uint16_t kSetAllValueMax = 2000;       // base + 2000 = 2500 us
constexpr std::size_t kSetAllPayloadBytes = 29;       // ceil(18 * 11 / 7)
constexpr std::size_t kSetAllFrameBytes = 1 + kSetAllPayloadBytes;

// Fixed command indices in the board's cmdPins space (see firmware main.h).
// CURR and VOLT are consecutive, so one GET(kCurrIndex, 2) returns both.
constexpr std::uint8_t kCurrIndex = 24;
constexpr std::uint8_t kVoltIndex = 25;
constexpr std::uint8_t kRelayIndex = 26;
// Read-only latched fault register (no pin). GET(kStatusIndex, 1) returns a
// 14-bit word: bit0 = TRIPPED (over-current latch active), bits1-3 = TIER (which
// OVERCURRENT_TIERS entry fired), bits4-13 = TRIP_CURRENT at 0.1 A/count. 0 =
// clean. Layout mirrors the firmware STATUS_* masks in main.h.
constexpr std::uint8_t kStatusIndex = 27;
constexpr std::uint16_t kStatusTrippedBit = 0x1;             // bit 0
constexpr int kStatusTierShift = 1;                          // bits 1..3
constexpr std::uint16_t kStatusTierMask = 0x7;
constexpr int kStatusCurrentShift = 4;                       // bits 4..13
constexpr std::uint16_t kStatusCurrentMask = 0x3FF;
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

// Encode a SETALL frame into `out` (cleared first). `pulses_us` must hold
// exactly kSetAllServoCount pulse widths in board index order (0..17); each is
// clamped to [500, 2500] µs, matching the firmware. The 18 values go out as one
// MSB-first bitstream of 11-bit fields, emitted 7 bits at a time into the low 7
// bits of each payload byte, so every payload byte keeps its MSB clear and
// resync still works. The final byte's low 5 bits are zero padding.
// Throws std::invalid_argument if `pulses_us` is the wrong length.
void encode_setall(std::span<const std::uint16_t> pulses_us,
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
  void send_all_servo_positions(
      std::span<const std::uint16_t> pulses_us) override;
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
