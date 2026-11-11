#include "eyes_face.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "eyes.hpp"
#include "text_screen.hpp"

namespace hexa::eyes_cpp {

bool isMood(const std::string& name) {
    return std::find(kMoods.begin(), kMoods.end(), name) != kMoods.end() ||
           std::find(kScanning.begin(), kScanning.end(), name) != kScanning.end();
}

std::string moodNames() {
    std::string out;
    for (const auto& m : kMoods) out += (out.empty() ? "" : ", ") + m;
    for (const auto& m : kScanning)
        if (m != "scanning") out += ", " + m;
    return out;
}

EyesFace::EyesFace(std::array<std::string, kExpressionCount> mood_for)
    : _mood_for(std::move(mood_for)) {
    for (std::size_t i = 0; i < kExpressionCount; ++i) {
        if (!isMood(_mood_for[i]))
            throw std::invalid_argument("unknown mood '" + _mood_for[i] + "' for " +
                                        expressionName(static_cast<Expression>(i)) +
                                        "; valid: " + moodNames());
    }
}

const std::string& EyesFace::moodFor(Expression e) const {
    return _mood_for[static_cast<std::size_t>(e)];
}

Image EyesFace::render(const eyes::AnimFrame& f, double now_s) {
    const std::string& mood = moodFor(f.expr);
    if (mood != _shown) {
        _shown = mood;
        _since = now_s;
    }
    const double t = now_s - _since;
    const double phase = std::fmod(t / 1.2, 1.0);   // one lap of the heartbeat / spinner
    return draw_eyes(f.gx, f.gy, f.lid, mood, phase, t);
}

Image EyesFace::fromU8g2(const u8g2_t* g) {
    Image im(kW, kH);
    for (int y = 0; y < kH; ++y)
        for (int x = 0; x < kW; ++x)
            if (hexa::display::textScreenPixel(g, x, y)) im.at(x, y) = 255;
    return im;
}

}  // namespace hexa::eyes_cpp
