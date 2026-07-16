#include "hexa_hardware/servo2040_protocol.hpp"

namespace hexa_hardware {

void encode_set(std::uint8_t start, std::span<const std::uint16_t> values,
                std::vector<std::uint8_t>& out) {
  out.clear();
  out.reserve(3 + values.size() * 2);
  out.push_back(kCmdSet);
  out.push_back(start & 0x7F);
  out.push_back(static_cast<std::uint8_t>(values.size()) & 0x7F);
  for (std::uint16_t v : values) {
    std::uint16_t c = v > kValueMax ? kValueMax : v;
    out.push_back(static_cast<std::uint8_t>(c & 0x7F));
    out.push_back(static_cast<std::uint8_t>((c >> 7) & 0x7F));
  }
}

void encode_get(std::uint8_t start, std::uint8_t count,
                std::vector<std::uint8_t>& out) {
  out.clear();
  out.reserve(3);
  out.push_back(kCmdGet);
  out.push_back(start & 0x7F);
  out.push_back(count & 0x7F);
}

bool decode_get_payload(std::span<const std::uint8_t> payload,
                        std::uint8_t& start,
                        std::vector<std::uint16_t>& values) {
  if (payload.size() < 2) {
    return false;
  }
  start = payload[0] & 0x7F;
  const std::uint8_t count = payload[1] & 0x7F;
  if (payload.size() < static_cast<std::size_t>(2 + count * 2)) {
    return false;
  }
  values.clear();
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint8_t lo = payload[2 + i * 2] & 0x7F;
    const std::uint8_t hi = payload[3 + i * 2] & 0x7F;
    values.push_back(static_cast<std::uint16_t>(lo) |
                     (static_cast<std::uint16_t>(hi) << 7));
  }
  return true;
}

void Servo2040Protocol::send_servo_positions(
    std::uint8_t start_pin, std::span<const std::uint16_t> values) {
  encode_set(start_pin, values, encode_buf_);
  transport_.write(encode_buf_);
}

void Servo2040Protocol::set_servo_power(bool on) {
  // A SET of 1/0 to the board's relay index; the board owns the actual GPIO.
  const std::uint16_t v = on ? 1u : 0u;
  send_servo_positions(kRelayIndex, std::span<const std::uint16_t>(&v, 1));
}

bool Servo2040Protocol::read_battery(float& voltage_v, float& current_a,
                                     int timeout_ms) {
  // CURR (24) and VOLT (25) are consecutive, so one GET fetches both:
  // values[0] = current, values[1] = voltage.
  if (!get_raw(kCurrIndex, 2, decode_buf_, timeout_ms) ||
      decode_buf_.size() != 2) {
    return false;
  }
  current_a = static_cast<float>(decode_buf_[0]) * kAmpsPerCount;
  voltage_v = static_cast<float>(decode_buf_[1]) * kVoltsPerCount;
  return true;
}

bool Servo2040Protocol::read_status(bool& tripped, float& trip_amps,
                                    int timeout_ms) {
  // STATUS (27) is a single 14-bit word: bit0 = TRIPPED, bits1-10 = trip
  // current at 0.1 A/count.
  if (!get_raw(kStatusIndex, 1, decode_buf_, timeout_ms) ||
      decode_buf_.size() != 1) {
    return false;
  }
  const std::uint16_t word = decode_buf_[0];
  tripped = (word & kStatusTrippedBit) != 0;
  trip_amps = static_cast<float>((word >> 1) & 0x3FF) * kTripAmpsPerCount;
  return true;
}

bool Servo2040Protocol::get_raw(std::uint8_t start, std::uint8_t count,
                                std::vector<std::uint16_t>& out, int timeout_ms) {
  if (!transport_.is_open()) return false;

  encode_get(start, count, encode_buf_);
  transport_.write(encode_buf_);

  // Resync: drop bytes until we see a command byte (MSB set). Discard any
  // command that isn't G (e.g. a stray S echo).
  std::uint8_t b = 0;
  while (true) {
    if (transport_.read(std::span<std::uint8_t>(&b, 1), timeout_ms) != 1) {
      return false;
    }
    if ((b & 0x80) == 0) continue;
    if (b == kCmdGet) break;
  }
  // Read payload: [start][count][2*count value bytes].
  const std::size_t payload_len = 2 + static_cast<std::size_t>(count) * 2;
  std::vector<std::uint8_t> payload(payload_len);
  if (transport_.read(payload, timeout_ms) != payload_len) {
    return false;
  }
  std::uint8_t reply_start = 0;
  if (!decode_get_payload(payload, reply_start, out)) return false;
  return reply_start == start && out.size() == count;
}

}  // namespace hexa_hardware
