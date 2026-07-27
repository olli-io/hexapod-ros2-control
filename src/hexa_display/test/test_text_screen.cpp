// Unit tests for the text screen (display text mode). Headless: rendering goes
// into a Sh1122Panel buffer, read back through textScreenPixel — no ROS graph,
// no hardware.

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "Sh1122Panel.h"
#include "text_screen.hpp"
#include "u8g2.h"

using hexa::display::drawTextScreen;
using hexa::display::layoutTextScreen;
using hexa::display::TextScreenConfig;
using hexa::display::textScreenPixel;

namespace {

face::PanelConfig headlessCfg() {
    face::PanelConfig cfg;
    cfg.headless = true;
    return cfg;
}

int litPixels(u8g2_t* g, int y_min = 0, int y_max = 64) {
    int n = 0;
    for (int y = y_min; y < y_max; ++y)
        for (int x = 0; x < 256; ++x)
            if (textScreenPixel(g, x, y)) ++n;
    return n;
}

int lineWidthPx(u8g2_t* g, const std::string& line) {
    return u8g2_GetUTF8Width(g, line.c_str());
}

}  // namespace

TEST(TextScreen, DrawSetsPixelsAndEmptyDrawsNothing) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();
    TextScreenConfig cfg;

    panel.clearBuffer();
    EXPECT_EQ(drawTextScreen(g, "Hello hexapod", cfg), 1);
    EXPECT_GT(litPixels(g), 0);

    panel.clearBuffer();
    drawTextScreen(g, "", cfg);
    EXPECT_EQ(litPixels(g), 0);
}

TEST(TextScreen, SingleLineIsVerticallyCentered) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();
    TextScreenConfig cfg;

    panel.clearBuffer();
    drawTextScreen(g, "Xg", cfg);
    // One 16 px line box centered on the 64 px panel: every glyph pixel
    // (ascender through descender) stays within the middle box, y in [24, 40).
    EXPECT_GT(litPixels(g, 24, 40), 0);
    EXPECT_EQ(litPixels(g, 0, 24), 0);
    EXPECT_EQ(litPixels(g, 40, 64), 0);
}

TEST(TextScreen, NewlineSplitsIntoStackedBands) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();
    TextScreenConfig cfg;

    panel.clearBuffer();
    EXPECT_EQ(drawTextScreen(g, "AAA\nBBB", cfg), 2);
    // Two line boxes centered: y in [16, 32) and [32, 48).
    EXPECT_GT(litPixels(g, 16, 32), 0);
    EXPECT_GT(litPixels(g, 32, 48), 0);
    EXPECT_EQ(litPixels(g, 0, 16), 0);
    EXPECT_EQ(litPixels(g, 48, 64), 0);
}

TEST(TextScreen, BlankLinePreserved) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    TextScreenConfig cfg;
    const auto lines = layoutTextScreen(panel.u8g2(), "top\n\nbottom", cfg);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_EQ(lines[1], "");
}

TEST(TextScreen, WordWrapKeepsLinesInsideMargins) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();
    TextScreenConfig cfg;

    const std::string msg =
        "a fairly long status message that cannot possibly fit on one panel line";
    const auto lines = layoutTextScreen(g, msg, cfg);
    ASSERT_GT(lines.size(), 1u);
    std::string rejoined;
    for (const auto& line : lines) {
        EXPECT_LE(lineWidthPx(g, line), cfg.width - 2 * cfg.margin_px);
        rejoined += (rejoined.empty() ? "" : " ") + line;
    }
    EXPECT_EQ(rejoined, msg);  // wrap re-breaks only at spaces, loses nothing
}

TEST(TextScreen, OverlongWordHardBreaks) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();
    TextScreenConfig cfg;

    const std::string word(60, 'W');  // ~9 px/glyph — far wider than the panel
    const auto lines = layoutTextScreen(g, word, cfg);
    ASSERT_GT(lines.size(), 1u);
    std::string rejoined;
    for (const auto& line : lines) {
        EXPECT_LE(lineWidthPx(g, line), cfg.width - 2 * cfg.margin_px);
        rejoined += line;
    }
    EXPECT_EQ(rejoined, word);
}

TEST(TextScreen, ClipsToFourLines) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    TextScreenConfig cfg;
    const auto lines = layoutTextScreen(panel.u8g2(), "1\n2\n3\n4\n5\n6", cfg);
    EXPECT_EQ(lines.size(), 4u);  // 64 px / 16 px line spacing
}

TEST(TextScreen, Latin1Renders) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();
    TextScreenConfig cfg;

    panel.clearBuffer();
    drawTextScreen(g, "\xc3\xa4\xc3\xb6\xc3\xa5 25\xc2\xb0", cfg);  // "äöå 25°"
    EXPECT_GT(litPixels(g), 0);
}

TEST(TextScreen, PixelReadbackBoundsChecked) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();
    EXPECT_FALSE(textScreenPixel(g, -1, 0));
    EXPECT_FALSE(textScreenPixel(g, 0, -1));
    EXPECT_FALSE(textScreenPixel(g, 256, 0));
    EXPECT_FALSE(textScreenPixel(g, 0, 64));
}

// The budget hexa_buttons writes its screens against (info_text.LINE_BUDGET).
// Pinned here, on the side that actually owns the font metrics, because the
// failure mode is silent: an over-budget line does not truncate, it *wraps*,
// and the extra line pushes the last one off a four-line panel.
TEST(TextScreen, ThirtyCharactersOfRealTextFitOneLine) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    TextScreenConfig cfg;
    // The widest content hexa_buttons actually emits: the address line at a
    // full-length IPv4, and a hotspot password line. Both exactly at budget.
    for (const std::string& line : {std::string("Control -> 192.168.172.42:8080"),
                                    std::string("Password -> hexahexatwelve1234")}) {
        ASSERT_EQ(line.size(), 30u) << line;
        const auto lines = layoutTextScreen(panel.u8g2(), line, cfg);
        EXPECT_EQ(lines.size(), 1u) << line << " wrapped, budget is too generous";
        EXPECT_LE(lineWidthPx(panel.u8g2(), line), 256 - 2 * cfg.margin_px) << line;
    }
}

// The budget is characters of *mixed* text, not a pixel guarantee: this font is
// proportional, and its widest glyphs advance 9 px against an average nearer 8.
// 28 is therefore the floor that holds whatever the characters are — worth
// knowing before anyone puts a configurable SSID on a line already near 30.
TEST(TextScreen, TwentyEightWideGlyphsAreTheGuaranteedFloor) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    TextScreenConfig cfg;
    // layoutTextScreen is what installs the font on the u8g2 handle; measuring
    // before it has run reads a null font.
    layoutTextScreen(panel.u8g2(), "x", cfg);
    const int max_w = 256 - 2 * cfg.margin_px;
    EXPECT_LE(lineWidthPx(panel.u8g2(), std::string(28, 'M')), max_w);
    EXPECT_GT(lineWidthPx(panel.u8g2(), std::string(30, 'M')), max_w);
}
