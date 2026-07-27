#pragma once

// Text screen — the display's alternative to the face. Renders a UTF-8
// message (Nerd Font icons included) onto the 256x64 panel through the u8g2 C
// API. Pure: no rclcpp, no I/O, no clocks; the caller owns the u8g2 handle and
// decides when the text mode is active.

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

typedef struct u8g2_struct u8g2_t;

namespace hexa::display {

struct TextScreenConfig {
    int width = 256;
    int height = 64;
    // Baseline-to-baseline distance. 16 px fits four lines of the bundled
    // Pixel Operator font (16 px native em) on the 64 px panel.
    int line_spacing_px = 16;
    // Left/right margin used by the word wrap and by left-aligned lines.
    int margin_px = 2;
    // Center each line horizontally (false = left-align at margin_px).
    bool hcenter = true;
    // Font override; nullptr uses the bundled Pixel Operator font
    // (u8g2_font_hexa_text_16 from fonts/hexa_text_font.h).
    const uint8_t* font = nullptr;
};

// Split `text` into drawable lines: hard-break on '\n', then greedy word-wrap
// each paragraph to the panel width (pixel-measured with the active font; a
// word longer than a full line hard-breaks mid-word). Sets the font on `g`.
// Lines beyond what fits vertically are dropped.
std::vector<std::string> layoutTextScreen(u8g2_t* g, std::string_view text,
                                          const TextScreenConfig& cfg);

// Draw `text` centered on the panel (see layoutTextScreen for the line
// layout). Does not clear the buffer or flush the panel — the caller owns the
// frame lifecycle. Returns the number of lines drawn.
int drawTextScreen(u8g2_t* g, std::string_view text, const TextScreenConfig& cfg);

// Read one pixel back out of a full-buffer SH1122 u8g2 framebuffer (horizontal
// byte layout, MSB = leftmost pixel). u8g2 has no readback API; tests and the
// terminal mirror need one. Coordinates are buffer coordinates — a U8G2_MIRROR
// setup (the physical panel) stores x flipped.
bool textScreenPixel(const u8g2_t* g, int x, int y);

}  // namespace hexa::display
