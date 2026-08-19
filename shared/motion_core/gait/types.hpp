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

// Which legs a strategy walks. A property of the strategy, so selecting a
// quadruped gait is what enters quadruped mode — no second command channel.
enum class LegSet {
  HEXAPOD,    // all six
  QUADRUPED,  // the four corners; the middle pair is parked
};

// The pair the quadruped set drops. Named once so no predicate has to spell out
// which legs "the middles" are.
inline const std::array<std::string, 2> PARKED_LEGS = {"l_middle", "r_middle"};

inline bool leg_is_parked(LegSet set, const std::string& name) {
  if (set != LegSet::QUADRUPED) {
    return false;
  }
  for (const auto& parked : PARKED_LEGS) {
    if (parked == name) {
      return true;
    }
  }
  return false;
}

// One leg's contribution to a LegTargets message.
struct LegOutput {
  Vec3 foot_target = Vec3::Zero();
  float phase = 0.0f;
  bool stance = true;
  // Lifted out of the walk entirely: neither stance nor swing, carrying no
  // weight and taking no phase. `stance` is meaningless while this is set, so
  // every collective predicate tests this one first.
  bool parked = false;
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
