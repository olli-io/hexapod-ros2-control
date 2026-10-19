// gestures.yaml `expression:` keys -> PolicyConfig::gesture_expression_map.
// gen_config.py bakes the same table for the Pico.
#pragma once

#include <string>
#include <unordered_map>

#include "Expression.h"

namespace hexa::display {

// Throws std::runtime_error on an unknown expression name.
std::unordered_map<std::string, Expression> parseGestureExpressions(
    const std::string& yaml_text);

std::unordered_map<std::string, Expression> loadGestureExpressions(
    const std::string& path);

}  // namespace hexa::display
