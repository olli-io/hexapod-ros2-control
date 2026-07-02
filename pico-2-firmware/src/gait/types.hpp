// Shared foundational types for the gait engine — float fork of
// hexa_gait_cpp/include/hexa_gait_cpp/types.hpp (plan part 06).
//
// double->float for the RP2350 single-precision FPU. The firmware's Vec3 /
// JointAngles (src/vec3.hpp) replace Eigen; leg keys stay std::string in
// std::map (the map->array optimization is a later part, per overview 00).
//
// The whole port lives in namespace hexa::gait so it can be compiled alongside
// the untouched double hexa_gait_cpp (namespace hexa_gait) in one host binary
// without an ODR clash (see test/host/test_gait.cpp).
#pragma once

#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include "vec3.hpp"  // hexa::Vec3, hexa::JointAngles

namespace hexa::gait {

using Vec3 = ::hexa::Vec3;
using JointAngles = ::hexa::JointAngles;

// Floating-point modulo matching Python's % for a positive divisor: the result
// has the sign of the divisor (always non-negative for the [0, 1) phase wrap).
inline float pymod(float a, float b) {
  float r = std::fmod(a, b);
  if (r != 0.0f && ((r < 0.0f) != (b < 0.0f))) {
    r += b;
  }
  return r;
}

// Canonical leg order. Fixed at six legs (see CLAUDE.md). Same strings/order as
// leg_index.hpp LEG_NAMES and hexa_gait::LEG_NAMES, but std::string so it can key
// the engine's std::map<std::string, ...> state.
inline const std::array<std::string, 6> LEG_NAMES = {
    "l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear",
};

// One leg's contribution to a LegTargets message — the shared currency every
// trajectory controller in this package emits. stance=true means the foot is on
// the ground this tick.
struct LegOutput {
  Vec3 foot_target = Vec3::Zero();
  float phase = 0.0f;
  bool stance = true;
};

// Every controller checks that a per-leg map covers all six legs and raises with
// the missing names listed.
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
