#include "hexa_hardware/servo2040_protocol.hpp"

#include <stdexcept>
#include <string>

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

void encode_setall(std::span<const std::uint16_t> pulses_us,
                   std::vector<std::uint8_t>& out) {
  if (pulses_us.size() != kSetAllServoCount) {
    throw std::invalid_argument(
        "encode_setall: expected " + std::to_string(kSetAllServoCount) +
        " pulses, got " + std::to_string(pulses_us.size()));
  }
  out.clear();
  out.reserve(kSetAllFrameBytes);
  out.push_back(kCmdSetAll);
  // Mirror of the firmware unpacker: an MSB-first bitstream of 11-bit fields,
  // drained 7 bits at a time so every payload byte keeps its MSB clear. `acc`
  // never holds more than 6 + 11 bits before the drain, so 32 bits is ample.
  std::uint32_t acc = 0;
  int nbits = 0;
  for (const std::uint16_t pulse : pulses_us) {
    std::uint16_t v = pulse < kSetAllPulseBaseUs
                          ? 0
                          : static_cast<std::uint16_t>(pulse - kSetAllPulseBaseUs);
    if (v > kSetAllValueMax) v = kSetAllValueMax;
    acc = (acc << kSetAllValueBits) | v;
    nbits += kSetAllValueBits;
    while (nbits >= 7) {
      out.push_back(static_cast<std::uint8_t>((acc >> (nbits - 7)) & 0x7F));
      nbits -= 7;
    }
  }
  // 18 * 11 = 198 bits leaves 2 over; pad the tail byte's low 5 bits with zeros.
  if (nbits > 0) {
    out.push_back(static_cast<std::uint8_t>((acc << (7 - nbits)) & 0x7F));
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
  encode_set(start_pin, values, send_buf_);
  transport_.write(send_buf_);
}

void Servo2040Protocol::send_all_servo_positions(
    std::span<const std::uint16_t> pulses_us) {
  encode_setall(pulses_us, send_buf_);
  transport_.write(send_buf_);
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
  // STATUS (27) is a single 14-bit word: bit0 = TRIPPED, bits1-3 = TIER,
  // bits4-13 = trip current at 0.1 A/count (see firmware main.h STATUS_* masks).
  if (!get_raw(kStatusIndex, 1, decode_buf_, timeout_ms) ||
      decode_buf_.size() != 1) {
    return false;
  }
  const std::uint16_t word = decode_buf_[0];
  tripped = (word & kStatusTrippedBit) != 0;
  trip_amps = static_cast<float>((word >> kStatusCurrentShift) &
                                 kStatusCurrentMask) *
              kTripAmpsPerCount;
  return true;
}

bool Servo2040Protocol::get_raw(std::uint8_t start, std::uint8_t count,
                                std::vector<std::uint16_t>& out, int timeout_ms) {
  if (!transport_.is_open()) return false;

  encode_get(start, count, get_buf_);
  transport_.write(get_buf_);

  // Resync: drop bytes until we see a command byte (MSB set). Discard any
  // command that isn't G (e.g. a stray S echo).
  //
  // Bounded, because each iteration renews the read timeout: a board streaming
  // junk faster than the timeout would otherwise hold this loop forever and
  // starve every later poll — battery telemetry and, worse, the over-current
  // STATUS read. Failing the poll instead just drops one sample; the caller
  // retries on the next aux period. The bound is generous — the board sends
  // nothing unsolicited, so the only legitimate leading garbage is one stale
  // GET reply (at most 3 + 2 * kMaxBatch bytes).
  constexpr int kResyncMaxBytes = 3 + 2 * static_cast<int>(kMaxBatch);
  std::uint8_t b = 0;
  bool synced = false;
  for (int i = 0; i < kResyncMaxBytes; ++i) {
    if (transport_.read(std::span<std::uint8_t>(&b, 1), timeout_ms) != 1) {
      return false;
    }
    if ((b & 0x80) == 0) continue;
    if (b == kCmdGet) {
      synced = true;
      break;
    }
  }
  if (!synced) return false;

  // Read payload: [start][count][2*count value bytes].
  const std::size_t payload_len = 2 + static_cast<std::size_t>(count) * 2;
  get_payload_buf_.assign(payload_len, 0);
  if (transport_.read(get_payload_buf_, timeout_ms) != payload_len) {
    return false;
  }
  std::uint8_t reply_start = 0;
  if (!decode_get_payload(get_payload_buf_, reply_start, out)) return false;
  return reply_start == start && out.size() == count;
}

}  // namespace hexa_hardware
