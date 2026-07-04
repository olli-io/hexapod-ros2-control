#pragma once

#include <cctype>
#include <optional>
#include <string>

#include "Expression.h"

// Case-insensitive name <-> enum parsing for the ROS topic interface. The
// strings come straight from the firmware's expressionName()/gazeName(), so the
// topic vocabulary can never drift from the on-wire enum names in commands.md.
namespace face {

inline std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

inline std::optional<Expression> parseExpression(const std::string& name) {
    const std::string want = upper(name);
    for (int i = 0; i < static_cast<int>(Expression::_COUNT); ++i) {
        const auto e = static_cast<Expression>(i);
        if (want == expressionName(e)) return e;
    }
    return std::nullopt;
}

inline std::optional<GazeDirection> parseGaze(const std::string& name) {
    const std::string want = upper(name);
    for (int i = 0; i < static_cast<int>(GazeDirection::_COUNT); ++i) {
        const auto g = static_cast<GazeDirection>(i);
        if (want == gazeName(g)) return g;
    }
    return std::nullopt;
}

}  // namespace face
