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
  void send_digital(std::uint8_t pin, bool value) override;
  bool read_aux(std::uint8_t start_pin, std::uint8_t count,
                std::vector<std::uint16_t>& out, int timeout_ms) override;

 private:
  Transport& transport_;
  // Reusable encode buffer to avoid per-call allocation on the hot path.
  std::vector<std::uint8_t> encode_buf_;
};

}  // namespace hexa_hardware
