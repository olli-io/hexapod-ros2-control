#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>
#include <span>
#include <stdexcept>
#include <vector>

#include "hexa_hardware/servo2040_protocol.hpp"
#include "hexa_hardware/transport.hpp"

namespace hh = hexa_hardware;

namespace {

// In-memory Transport: captures written bytes and replays a canned reply.
class FakeTransport : public hh::Transport {
 public:
  void open() override { open_ = true; }
  void close() override { open_ = false; }
  bool is_open() const override { return open_; }

  void write(std::span<const std::uint8_t> data) override {
    written.insert(written.end(), data.begin(), data.end());
  }

  std::size_t read(std::span<std::uint8_t> buf, int /*timeout_ms*/) override {
    std::size_t n = 0;
    while (n < buf.size() && read_pos < to_read.size()) buf[n++] = to_read[read_pos++];
    return n;
  }

  bool open_ = true;
  std::vector<std::uint8_t> written;
  std::vector<std::uint8_t> to_read;
  std::size_t read_pos = 0;
};

void push_value(std::vector<std::uint8_t>& b, std::uint16_t v) {
  b.push_back(v & 0x7F);
  b.push_back((v >> 7) & 0x7F);
}

}  // namespace

// Spec example #1: set pin 4 to 1500 → S, 4, 1, lo(1500), hi(1500).
TEST(EncodeSet, SinglePin) {
  std::vector<std::uint16_t> v{1500};
  std::vector<std::uint8_t> out;
  hh::encode_set(4, v, out);
  ASSERT_EQ(out.size(), 5u);
  EXPECT_EQ(out[0], hh::kCmdSet);
  EXPECT_EQ(out[1], 4);
  EXPECT_EQ(out[2], 1);
  EXPECT_EQ(out[3], 1500 & 0x7F);
  EXPECT_EQ(out[4], (1500 >> 7) & 0x7F);
}

// Spec example #2: set pins 7..12 to 1007..1012.
TEST(EncodeSet, ConsecutiveBatch) {
  std::vector<std::uint16_t> v{1007, 1008, 1009, 1010, 1011, 1012};
  std::vector<std::uint8_t> out;
  hh::encode_set(7, v, out);
  ASSERT_EQ(out.size(), 3u + v.size() * 2);
  EXPECT_EQ(out[0], hh::kCmdSet);
  EXPECT_EQ(out[1], 7);
  EXPECT_EQ(out[2], 6);
  for (std::size_t i = 0; i < v.size(); ++i) {
    const std::uint16_t expect = v[i];
    EXPECT_EQ(out[3 + i * 2], expect & 0x7F);
    EXPECT_EQ(out[4 + i * 2], (expect >> 7) & 0x7F);
  }
}

TEST(EncodeSet, ClampsTo14Bit) {
  std::vector<std::uint16_t> v{0xFFFF};
  std::vector<std::uint8_t> out;
  hh::encode_set(0, v, out);
  // 0x3FFF round-trip → lo=0x7F, hi=0x7F.
  EXPECT_EQ(out[3], 0x7F);
  EXPECT_EQ(out[4], 0x7F);
}

TEST(EncodeSet, AllBytesAreData) {
  std::vector<std::uint16_t> v{0x3FFF, 0x0, 0x1234};
  std::vector<std::uint8_t> out;
  hh::encode_set(0x12, v, out);
  EXPECT_EQ(out[0] & 0x80, 0x80);  // command byte
  for (std::size_t i = 1; i < out.size(); ++i) {
    EXPECT_EQ(out[i] & 0x80, 0x00) << "data byte " << i << " has MSB set";
  }
}

// ── SETALL — the compact all-servos frame ──────────────────────────────────
//
// The 39-byte whole-robot SET overruns the board's 32-byte UART RX FIFO when
// the firmware's main loop stalls mid-arrival, and what gets lost is the frame's
// tail — the last servo. SETALL packs the same pose into 30 bytes, so these
// tests pin the wire format hard: the encoder must stay a bit-exact mirror of
// the firmware unpacker or the whole robot goes to the wrong pose.

namespace {

// A representative pose spanning the encodable range, and the exact bytes the
// spec's reference packer (protocol.md) emits for it. A golden vector, so a
// future refactor cannot quietly change the format.
const std::vector<std::uint16_t> kGoldenPulses = {
    500,  617,  734,  851,  968,  1085, 1202, 1319, 1436,
    1553, 1670, 1787, 1904, 2021, 2138, 2255, 2372, 2489};

const std::vector<std::uint8_t> kGoldenFrame = {
    0xD5, 0x00, 0x00, 0x3A, 0x47, 0x28, 0x57, 0x67, 0x28, 0x49,
    0x15, 0x3E, 0x33, 0x1B, 0x54, 0x20, 0x76, 0x24, 0x54, 0x0F,
    0x2F, 0x4B, 0x71, 0x66, 0x36, 0x6D, 0x7A, 0x43, 0x71, 0x20};

// The firmware's unpack loop (chica-servo2040.cpp, SETALL_CMD branch),
// reimplemented so the round-trip test checks against the board's own reading of
// the bytes rather than against the encoder restated.
std::vector<std::uint16_t> firmware_unpack(std::span<const std::uint8_t> frame) {
  EXPECT_EQ(frame.size(), hh::kSetAllFrameBytes);
  EXPECT_EQ(frame[0], hh::kCmdSetAll);
  const auto raw = frame.subspan(1);
  std::vector<std::uint16_t> out;
  std::uint32_t acc = 0;
  unsigned nbits = 0, bi = 0;
  for (std::size_t s = 0; s < hh::kSetAllServoCount; ++s) {
    while (nbits < static_cast<unsigned>(hh::kSetAllValueBits)) {
      acc = (acc << 7) | (raw[bi++] & 0x7F);
      nbits += 7;
    }
    unsigned value = (acc >> (nbits - hh::kSetAllValueBits)) &
                     ((1u << hh::kSetAllValueBits) - 1);
    nbits -= hh::kSetAllValueBits;
    if (value > hh::kSetAllValueMax) value = hh::kSetAllValueMax;
    out.push_back(static_cast<std::uint16_t>(hh::kSetAllPulseBaseUs + value));
  }
  return out;
}

}  // namespace

TEST(EncodeSetAll, MatchesGoldenFrame) {
  std::vector<std::uint8_t> out;
  hh::encode_setall(kGoldenPulses, out);
  EXPECT_EQ(out, kGoldenFrame);
}

// 30 bytes is the point of the exercise: it fits the board's 32-byte RX FIFO
// whole, where the 39-byte SET did not.
TEST(EncodeSetAll, FitsTheFifoAndKeepsPayloadMsbClear) {
  std::vector<std::uint8_t> out;
  hh::encode_setall(kGoldenPulses, out);
  ASSERT_EQ(out.size(), 30u);
  ASSERT_EQ(out.size(), hh::kSetAllFrameBytes);
  EXPECT_LE(out.size(), 32u);
  EXPECT_EQ(out[0], hh::kCmdSetAll);
  for (std::size_t i = 1; i < out.size(); ++i) {
    EXPECT_EQ(out[i] & 0x80, 0x00) << "payload byte " << i << " has MSB set";
  }
}

TEST(EncodeSetAll, RoundTripsThroughTheFirmwareUnpacker) {
  std::vector<std::uint8_t> out;
  hh::encode_setall(kGoldenPulses, out);
  EXPECT_EQ(firmware_unpack(out), kGoldenPulses);

  // Endpoints and a flat pose, exactly.
  const std::vector<std::vector<std::uint16_t>> poses = {
      std::vector<std::uint16_t>(18, 500),
      std::vector<std::uint16_t>(18, 2500),
      std::vector<std::uint16_t>(18, 1500),
  };
  for (const auto& pose : poses) {
    hh::encode_setall(pose, out);
    EXPECT_EQ(firmware_unpack(out), pose);
  }

  // Randomised poses: a packing bug that only bites on a particular bit
  // alignment shows up here and not in a hand-picked vector.
  std::mt19937 rng(20260726);
  std::uniform_int_distribution<int> us(500, 2500);
  std::vector<std::uint16_t> pose(18);
  for (int iter = 0; iter < 2000; ++iter) {
    for (auto& p : pose) p = static_cast<std::uint16_t>(us(rng));
    hh::encode_setall(pose, out);
    ASSERT_EQ(firmware_unpack(out), pose) << "iteration " << iter;
  }
}

// The firmware clamps out-of-range pulses into [500, 2500] and drives them; the
// host must land on the same value rather than wrap or truncate.
TEST(EncodeSetAll, ClampsOutOfRangePulses) {
  std::vector<std::uint16_t> pose(18, 1500);
  pose[0] = 400;    // below base
  pose[1] = 0;      // far below
  pose[17] = 3000;  // above max
  std::vector<std::uint8_t> out;
  hh::encode_setall(pose, out);
  const auto decoded = firmware_unpack(out);
  EXPECT_EQ(decoded[0], 500);
  EXPECT_EQ(decoded[1], 500);
  EXPECT_EQ(decoded[17], 2500);
}

// 18 * 11 = 198 bits leaves 2 bits over in the 29th byte; the rest is zero pad.
TEST(EncodeSetAll, PadsTheTailByteWithZeros) {
  std::vector<std::uint8_t> out;
  hh::encode_setall(std::vector<std::uint16_t>(18, 2500), out);
  EXPECT_EQ(out.back() & 0x1F, 0x00);
}

TEST(EncodeSetAll, RejectsWrongServoCount) {
  std::vector<std::uint8_t> out;
  EXPECT_THROW(hh::encode_setall(std::vector<std::uint16_t>(17, 1500), out),
               std::invalid_argument);
  EXPECT_THROW(hh::encode_setall(std::vector<std::uint16_t>(19, 1500), out),
               std::invalid_argument);
  EXPECT_THROW(hh::encode_setall(std::vector<std::uint16_t>{}, out),
               std::invalid_argument);
}

TEST(SendAllServoPositions, WritesOneSetAllFrame) {
  FakeTransport t;
  hh::Servo2040Protocol proto(t);
  proto.send_all_servo_positions(kGoldenPulses);
  EXPECT_EQ(t.written, kGoldenFrame);
}

TEST(EncodeGet, ThreeByteFrame) {
  std::vector<std::uint8_t> out;
  hh::encode_get(20, 6, out);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0], hh::kCmdGet);
  EXPECT_EQ(out[1], 20);
  EXPECT_EQ(out[2], 6);
}

TEST(DecodeGet, RoundTripPayload) {
  std::vector<std::uint16_t> values_in{0, 1, 1500, 0x3FFF};
  // Hand-build a reply payload: [start][count][values…].
  std::vector<std::uint8_t> payload;
  payload.push_back(10);
  payload.push_back(static_cast<std::uint8_t>(values_in.size()));
  for (auto v : values_in) {
    payload.push_back(v & 0x7F);
    payload.push_back((v >> 7) & 0x7F);
  }
  std::uint8_t start = 0;
  std::vector<std::uint16_t> out;
  EXPECT_TRUE(hh::decode_get_payload(payload, start, out));
  EXPECT_EQ(start, 10);
  EXPECT_EQ(out, values_in);
}

TEST(DecodeGet, RejectsShortBuffer) {
  std::array<std::uint8_t, 3> payload{5, 2, 0x7F};  // count=2 but only 1 value byte
  std::uint8_t start = 0;
  std::vector<std::uint16_t> out;
  EXPECT_FALSE(hh::decode_get_payload(payload, start, out));
}

// read_battery issues one GET(24,2) and decodes centi-units into volts/amps.
TEST(ReadBattery, DecodesCentiUnits) {
  FakeTransport t;
  // Canned reply: [G][24][2][current=150][voltage=830][checksum]. The checksum
  // trails the payload and is not consumed by the host.
  t.to_read = {hh::kCmdGet, hh::kCurrIndex, 2};
  push_value(t.to_read, 150);  // 1.50 A
  push_value(t.to_read, 830);  // 8.30 V
  t.to_read.push_back(0xFF);   // trailing checksum|0x80, ignored

  hh::Servo2040Protocol proto(t);
  float voltage = 0.0f, current = 0.0f;
  ASSERT_TRUE(proto.read_battery(voltage, current, 50));
  EXPECT_FLOAT_EQ(current, 1.50f);
  EXPECT_FLOAT_EQ(voltage, 8.30f);

  // The request is a single GET starting at CURR for 2 values.
  ASSERT_EQ(t.written.size(), 3u);
  EXPECT_EQ(t.written[0], hh::kCmdGet);
  EXPECT_EQ(t.written[1], hh::kCurrIndex);
  EXPECT_EQ(t.written[2], 2);
}

TEST(ReadBattery, FailsOnEmptyReply) {
  FakeTransport t;  // to_read empty → resync read returns 0
  hh::Servo2040Protocol proto(t);
  float voltage = -1.0f, current = -1.0f;
  EXPECT_FALSE(proto.read_battery(voltage, current, 50));
}

// read_status issues one GET(27,1) and decodes the latched fault word:
// bit0 = TRIPPED, bits1-3 = TIER, bits4-13 = trip current at 0.1 A/count.
TEST(ReadStatus, DecodesTrippedWord) {
  FakeTransport t;
  // A real 11.5 A trip on tier 2: current count = 115 (0x073), placed in
  // bits4-13; tier 2 in bits1-3; TRIPPED in bit0.
  const std::uint16_t word =
      hh::kStatusTrippedBit |
      (2u << hh::kStatusTierShift) |
      (115u << hh::kStatusCurrentShift);
  t.to_read = {hh::kCmdGet, hh::kStatusIndex, 1};
  push_value(t.to_read, word);
  t.to_read.push_back(0xFF);  // trailing checksum|0x80, ignored

  hh::Servo2040Protocol proto(t);
  bool tripped = false;
  float trip_amps = -1.0f;
  ASSERT_TRUE(proto.read_status(tripped, trip_amps, 50));
  EXPECT_TRUE(tripped);
  EXPECT_FLOAT_EQ(trip_amps, 11.5f);

  ASSERT_EQ(t.written.size(), 3u);
  EXPECT_EQ(t.written[0], hh::kCmdGet);
  EXPECT_EQ(t.written[1], hh::kStatusIndex);
  EXPECT_EQ(t.written[2], 1);
}

// A clean word (0) decodes to not-tripped, 0 A — the tier bits do not leak into
// the current field.
TEST(ReadStatus, DecodesCleanWord) {
  FakeTransport t;
  t.to_read = {hh::kCmdGet, hh::kStatusIndex, 1};
  push_value(t.to_read, 0);
  t.to_read.push_back(0xFF);

  hh::Servo2040Protocol proto(t);
  bool tripped = true;
  float trip_amps = -1.0f;
  ASSERT_TRUE(proto.read_status(tripped, trip_amps, 50));
  EXPECT_FALSE(tripped);
  EXPECT_FLOAT_EQ(trip_amps, 0.0f);
}

// set_servo_power sends a SET of 1/0 to the board-owned relay index.
TEST(SetServoPower, WritesRelaySet) {
  FakeTransport t;
  hh::Servo2040Protocol proto(t);
  proto.set_servo_power(true);
  // [S][26][1][lo(1)=1][hi(1)=0]
  ASSERT_EQ(t.written.size(), 5u);
  EXPECT_EQ(t.written[0], hh::kCmdSet);
  EXPECT_EQ(t.written[1], hh::kRelayIndex);
  EXPECT_EQ(t.written[2], 1);
  EXPECT_EQ(t.written[3], 1);
  EXPECT_EQ(t.written[4], 0);

  t.written.clear();
  proto.set_servo_power(false);
  ASSERT_EQ(t.written.size(), 5u);
  EXPECT_EQ(t.written[3], 0);  // value low byte = 0
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
