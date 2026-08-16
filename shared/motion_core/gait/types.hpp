// Shared foundational types for the gait engine.
#pragma once

#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include "vec3.hpp"

namespace hexa::gait {

using Vec3 = ::hexa::Vec3;
using JointAngles = ::hexa::JointAngles;

// Modulo with the sign of the divisor: non-negative for the [0, 1) phase wrap.
inline float pymod(float a, float b) {
  float r = std::fmod(a, b);
  if (r != 0.0f && ((r < 0.0f) != (b < 0.0f))) {
    r += b;
  }
  return r;
}

// Same strings/order as leg_index.hpp LEG_NAMES, but std::string so they can key
// the engine's std::map state.
inline const std::array<std::string, 6> LEG_NAMES = {
    "l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear",
};

// One leg's contribution to a LegTargets message.
struct LegOutput {
  Vec3 foot_target = Vec3::Zero();
  float phase = 0.0f;
  bool stance = true;
};

template <typename Value>
void require_all_legs(const std::map<std::string, Value>& m,
                      const std::string& what) {
  std::string missing;
  for (const auto& name : LEG_NAMES) {
    if (m.find(name) == m.end()) {
      if (!missing.empty()) {
        missing += ", ";
      }
      missing += name;
    }
  }
  if (!missing.empty()) {
    throw std::invalid_argument(what + " missing legs: " + missing);
  }
}

}  // namespace hexa::gait
