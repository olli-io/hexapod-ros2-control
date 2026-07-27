#include "text_screen.hpp"

#include <algorithm>

#include "u8g2.h"

#include "fonts/hexa_text_font.h"

namespace hexa::display {
namespace {

// Byte length of the UTF-8 code point starting at s[i].
size_t cpLen(std::string_view s, size_t i) {
    const auto b = static_cast<unsigned char>(s[i]);
    if (b < 0x80) return 1;
    if (b < 0xE0) return 2;
    if (b < 0xF0) return 3;
    return 4;
}

int textWidth(u8g2_t* g, const std::string& s) {
    return u8g2_GetUTF8Width(g, s.c_str());
}

// Hard-break `word` (no spaces) into chunks of at most `max_w` px, splitting
// only at code-point boundaries.
void breakWord(u8g2_t* g, std::string_view word, int max_w,
               std::vector<std::string>& out) {
    std::string chunk;
    size_t i = 0;
    while (i < word.size()) {
        const size_t n = std::min(cpLen(word, i), word.size() - i);
        const std::string next = chunk + std::string(word.substr(i, n));
        if (!chunk.empty() && textWidth(g, next) > max_w) {
            out.push_back(chunk);
            chunk.clear();
        } else {
            chunk = next;
            i += n;
        }
    }
    if (!chunk.empty()) out.push_back(chunk);
}

void wrapParagraph(u8g2_t* g, std::string_view para, int max_w,
                   std::vector<std::string>& out) {
    std::string line;
    size_t pos = 0;
    while (pos < para.size()) {
        const size_t sp = para.find(' ', pos);
        const std::string_view word =
            para.substr(pos, (sp == std::string_view::npos ? para.size() : sp) - pos);
        pos = (sp == std::string_view::npos) ? para.size() : sp + 1;
        if (word.empty()) continue;
        const std::string joined =
            line.empty() ? std::string(word) : line + ' ' + std::string(word);
        if (textWidth(g, joined) <= max_w) {
            line = joined;
        } else {
            if (!line.empty()) out.push_back(line);
            line.clear();
            if (textWidth(g, std::string(word)) > max_w) {
                breakWord(g, word, max_w, out);
                if (!out.empty()) {
                    line = out.back();  // last chunk keeps accepting words
                    out.pop_back();
                }
            } else {
                line = std::string(word);
            }
        }
    }
    // An empty paragraph ("\n\n" in the message) stays as a blank line.
    if (!line.empty() || para.empty()) out.push_back(line);
}

}  // namespace

std::vector<std::string> layoutTextScreen(u8g2_t* g, std::string_view text,
                                          const TextScreenConfig& cfg) {
    u8g2_SetFont(g, cfg.font ? cfg.font : u8g2_font_hexa_text_16);
    u8g2_SetFontMode(g, 1);

    const int max_w = cfg.width - 2 * cfg.margin_px;
    std::vector<std::string> lines;
    size_t start = 0;
    while (start <= text.size()) {
        const size_t nl = text.find('\n', start);
        const size_t end = (nl == std::string_view::npos) ? text.size() : nl;
        wrapParagraph(g, text.substr(start, end - start), max_w, lines);
        if (nl == std::string_view::npos) break;
        start = nl + 1;
    }

    const int max_lines = std::max(1, cfg.height / cfg.line_spacing_px);
    if (static_cast<int>(lines.size()) > max_lines) lines.resize(max_lines);
    return lines;
}

int drawTextScreen(u8g2_t* g, std::string_view text, const TextScreenConfig& cfg) {
    const std::vector<std::string> lines = layoutTextScreen(g, text, cfg);

    const int ascent = u8g2_GetAscent(g);
    const int descent = u8g2_GetDescent(g);  // negative below the baseline
    const int block_h = static_cast<int>(lines.size()) * cfg.line_spacing_px;
    const int top = (cfg.height - block_h) / 2;
    // Center the glyph extent (ascent..descent) inside each line box.
    const int baseline_off = ascent + (cfg.line_spacing_px - (ascent - descent)) / 2;

    for (size_t i = 0; i < lines.size(); ++i) {
        const int w = textWidth(g, lines[i]);
        const int x = cfg.hcenter ? std::max(0, (cfg.width - w) / 2) : cfg.margin_px;
        const int y = top + static_cast<int>(i) * cfg.line_spacing_px + baseline_off;
        u8g2_DrawUTF8(g, static_cast<u8g2_uint_t>(x), static_cast<u8g2_uint_t>(y),
                      lines[i].c_str());
    }
    return static_cast<int>(lines.size());
}

bool textScreenPixel(const u8g2_t* g, int x, int y) {
    auto* mg = const_cast<u8g2_t*>(g);
    const int w = u8g2_GetBufferTileWidth(mg) * 8;
    const int h = u8g2_GetBufferTileHeight(mg) * 8;
    if (x < 0 || y < 0 || x >= w || y >= h) return false;
    const uint8_t* buf = u8g2_GetBufferPtr(mg);
    return (buf[y * (w / 8) + x / 8] >> (7 - x % 8)) & 1;
}

}  // namespace hexa::display
