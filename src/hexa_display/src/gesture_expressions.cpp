#include "gesture_expressions.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "FaceNames.h"

namespace hexa::display {
namespace {

std::unordered_map<std::string, Expression> fromNode(const YAML::Node& doc) {
    std::unordered_map<std::string, Expression> out;
    for (const auto& entry : doc["gestures"]) {
        if (!entry["expression"]) {
            continue;
        }
        const std::string id = entry["id"].as<std::string>();
        const std::string name = entry["expression"].as<std::string>();
        const auto expression = face::parseExpression(name);
        if (!expression) {
            throw std::runtime_error("gestures.yaml " + id + ": unknown expression " +
                                     name);
        }
        out[id] = *expression;
    }
    return out;
}

}  // namespace

std::unordered_map<std::string, Expression> parseGestureExpressions(
    const std::string& yaml_text) {
    return fromNode(YAML::Load(yaml_text));
}

std::unordered_map<std::string, Expression> loadGestureExpressions(
    const std::string& path) {
    return fromNode(YAML::LoadFile(path));
}

}  // namespace hexa::display
