// Dirty-band arithmetic: byte range -> full-width tile-row band.
//
// The band is what a panel flush actually pushes, so an off-by-one here shows up
// as a half-drawn eye. Geometry under test is the SH1122 256x64 full buffer:
// tiles_w = 32, so one tile row is 256 bytes and covers 8 pixel rows, and the
// whole buffer is 2048 bytes / 8 tile rows.

#include <gtest/gtest.h>

#include <array>

#include "dirty_band.hpp"

namespace {

constexpr unsigned kTilesW = 32;
constexpr std::size_t kRowBytes = kTilesW * 8;   // 256
constexpr std::size_t kBufBytes = 256 * 64 / 8;  // 2048
constexpr std::size_t kRows = kBufBytes / kRowBytes;  // 8

struct Buffers {
    std::array<std::uint8_t, kBufBytes> cur{};
    std::array<std::uint8_t, kBufBytes> last{};

    eyes::DirtyBand band() const {
        return eyes::dirtyTileRows(cur.data(), last.data(), kBufBytes, kTilesW);
    }
};

TEST(DirtyBand, IdenticalBuffersAreNotDirty) {
    Buffers b;
    b.cur.fill(0xA5);
    b.last.fill(0xA5);
    const auto band = b.band();
    EXPECT_FALSE(band.dirty);
    EXPECT_EQ(band.th, 0);
}

TEST(DirtyBand, SingleByteChangeYieldsOneTileRow) {
    Buffers b;
    b.cur[kRowBytes * 3 + 10] = 0x01;  // somewhere inside tile row 3
    const auto band = b.band();
    EXPECT_TRUE(band.dirty);
    EXPECT_EQ(band.ty, 3);
    EXPECT_EQ(band.th, 1);
}

TEST(DirtyBand, FirstByteOfARowBelongsToThatRow) {
    Buffers b;
    b.cur[kRowBytes * 5] = 0x01;
    const auto band = b.band();
    EXPECT_EQ(band.ty, 5);
    EXPECT_EQ(band.th, 1);
}

TEST(DirtyBand, LastByteOfARowBelongsToThatRow) {
    Buffers b;
    b.cur[kRowBytes * 5 + kRowBytes - 1] = 0x01;
    const auto band = b.band();
    EXPECT_EQ(band.ty, 5);
    EXPECT_EQ(band.th, 1);
}

// The boundary case the arithmetic most easily gets wrong: the last byte of one
// row and the first of the next must widen the band to two rows, not one.
TEST(DirtyBand, ChangeStraddlingARowBoundarySpansTwoRows) {
    Buffers b;
    b.cur[kRowBytes * 2 + kRowBytes - 1] = 0x01;
    b.cur[kRowBytes * 3] = 0x01;
    const auto band = b.band();
    EXPECT_TRUE(band.dirty);
    EXPECT_EQ(band.ty, 2);
    EXPECT_EQ(band.th, 2);
}

// Two far-apart changes widen to the band that contains both — the flush is
// contiguous full-width rows, it cannot skip the clean rows between them.
TEST(DirtyBand, DisjointChangesWidenToTheEnclosingBand) {
    Buffers b;
    b.cur[kRowBytes * 1 + 4] = 0x01;
    b.cur[kRowBytes * 6 + 4] = 0x01;
    const auto band = b.band();
    EXPECT_EQ(band.ty, 1);
    EXPECT_EQ(band.th, 6);
}

TEST(DirtyBand, WholeBufferChangedCoversEveryRow) {
    Buffers b;
    b.cur.fill(0xFF);
    const auto band = b.band();
    EXPECT_TRUE(band.dirty);
    EXPECT_EQ(band.ty, 0);
    EXPECT_EQ(band.th, kRows);
}

TEST(DirtyBand, VeryFirstAndVeryLastBytes) {
    Buffers first;
    first.cur[0] = 0x01;
    EXPECT_EQ(first.band().ty, 0);
    EXPECT_EQ(first.band().th, 1);

    Buffers last;
    last.cur[kBufBytes - 1] = 0x01;
    EXPECT_EQ(last.band().ty, kRows - 1);
    EXPECT_EQ(last.band().th, 1);
}

TEST(DirtyBand, ZeroLengthBufferIsNotDirty) {
    const std::uint8_t dummy = 0;
    const auto band = eyes::dirtyTileRows(&dummy, &dummy, 0, kTilesW);
    EXPECT_FALSE(band.dirty);
}

}  // namespace
