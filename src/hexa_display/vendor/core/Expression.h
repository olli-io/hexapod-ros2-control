#pragma once

#include <stdint.h>

enum class Expression : uint8_t {
    NEUTRAL = 0,  // 0 0
    HAPPY   = 1,  // ^ ^
    SLEEPY  = 2,  // - -
    DEAD    = 3,  // x x
    GREEDY  = 4,  // $ $
    WOOZY   = 5,  // ~ ~
    ANGRY   = 6,  // > <
    LOVE    = 7,  // ♥ ♥
    _COUNT
};

enum class GazeDirection : uint8_t {
    CENTER     = 0,
    UP         = 1,
    DOWN       = 2,
    LEFT       = 3,
    RIGHT      = 4,
    UP_LEFT    = 5,
    UP_RIGHT   = 6,
    DOWN_LEFT  = 7,
    DOWN_RIGHT = 8,
    _COUNT
};

inline bool isValid(Expression e) {
    return static_cast<uint8_t>(e) < static_cast<uint8_t>(Expression::_COUNT);
}
inline bool isValid(GazeDirection g) {
    return static_cast<uint8_t>(g) < static_cast<uint8_t>(GazeDirection::_COUNT);
}

const char* expressionName(Expression e);
const char* gazeName(GazeDirection g);
