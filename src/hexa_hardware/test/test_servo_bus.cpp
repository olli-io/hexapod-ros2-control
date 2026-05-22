#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "hexa_hardware/servo_bus.hpp"

namespace hh = hexa_hardware;

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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
