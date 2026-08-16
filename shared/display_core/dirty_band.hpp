#pragma once

// Dirty-region arithmetic for a u8g2 full-buffer panel flush.
//
// A u8g2 frame buffer is tile-row major: `tiles_w` tiles across, 8 bytes each,
// and one tile row covers 8 pixel rows. Comparing the new buffer against the
// last flushed one gives a byte range; this maps that range to the full-width
// tile-row band that contains it, which is what u8g2_UpdateDisplayArea takes.
//
// Pure and header-only on purpose: no u8g2, no SDK, no I/O, so both the Linux
// and Pico panel drivers can share it and it can be unit-tested on the host.
// Repo-owned — not part of the vendored core/ or u8g2/ trees.

#include <cstddef>
#include <cstdint>

namespace eyes {

struct DirtyBand {
    std::uint8_t ty = 0;    // first dirty tile row
    std::uint8_t th = 0;    // tile-row count (0 when !dirty)
    bool dirty = false;     // false => buffers identical, skip the flush
};

// `cur` and `last` must both be `n` bytes. `tiles_w` is the buffer's tile width
// (u8g2_GetBufferTileWidth), so one tile row is `tiles_w * 8` bytes.
inline DirtyBand dirtyTileRows(const std::uint8_t* cur, const std::uint8_t* last,
                               std::size_t n, unsigned tiles_w) {
    DirtyBand band;
    const std::size_t bytes_per_tile_row = static_cast<std::size_t>(tiles_w) * 8u;
    if (n == 0 || bytes_per_tile_row == 0) return band;

    std::size_t first = 0;
    while (first < n && cur[first] == last[first]) ++first;
    if (first == n) return band;  // unchanged

    std::size_t end = n;
    while (end > first && cur[end - 1] == last[end - 1]) --end;

    const std::size_t ty = first / bytes_per_tile_row;
    const std::size_t ty_end = (end - 1) / bytes_per_tile_row;
    band.ty = static_cast<std::uint8_t>(ty);
    band.th = static_cast<std::uint8_t>(ty_end - ty + 1);
    band.dirty = true;
    return band;
}

}  // namespace eyes
