#pragma once

// The greyscale face: turns one frame of the shared eye animation into pixels.
//
// Pure (no rclcpp, no I/O, no clocks): the node owns the clock and the panel.
// The policy and `eyes::EyeAnim` from shared/display_core decide *what* the face
// is doing (expression, blink, gaze); this maps each expression to a mood of the
// shared/eyes_core renderer and draws it.

#include <array>
#include <cstddef>
#include <string>

#include "EyeAnim.h"
#include "Expression.h"
#include "image.hpp"

typedef struct u8g2_struct u8g2_t;

namespace hexa::eyes_cpp {

constexpr std::size_t kExpressionCount = static_cast<std::size_t>(Expression::_COUNT);

// True if `name` is a mood shared/eyes_core can draw.
bool isMood(const std::string& name);
// Every valid mood name, comma separated, for error messages.
std::string moodNames();

class EyesFace {
public:
    // mood_for[i] is the mood shown for Expression i. Throws
    // std::invalid_argument naming the first unknown mood.
    explicit EyesFace(std::array<std::string, kExpressionCount> mood_for);

    const std::string& moodFor(Expression e) const;

    // Draw `f` at `now_s` seconds. Animated moods (scanning, thug-life, ...) run
    // on a clock that restarts whenever the mood shown changes, so thug-life
    // always opens with the eyes bare and then drops the glasses.
    Image render(const eyes::AnimFrame& f, double now_s);

    // The 256x64 u8g2 framebuffer (as drawn by the shared text screen) as an
    // 8-bit image: lit pixels 255, the rest 0.
    static Image fromU8g2(const u8g2_t* g);

private:
    std::array<std::string, kExpressionCount> _mood_for;
    std::string _shown;
    double _since = 0.0;
};

}  // namespace hexa::eyes_cpp
