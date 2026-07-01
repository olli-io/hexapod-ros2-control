// Shared foundational types and invariants for the gait engine.
//
// Vec3 is the body-frame 3-vector currency used throughout (foot targets,
// stride vectors, mount positions). LEG_NAMES is the canonical leg ordering;
// every per-leg loop iterates it so behaviour is deterministic and matches the
// Python package (hexa_kinematics.leg_specs.LEG_NAMES, re-exported via clock).
// LegOutput is the per-leg currency every trajectory controller emits, and
// require_all_legs is the LEG_NAMES-coverage check they all run at construction.
#pragma once

#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

namespace hexa_gait {

using Vec3 = Eigen::Vector3d;

// Floating-point modulo matching Python's % for a positive divisor: the result
// has the sign of the divisor (always non-negative for the [0, 1) phase wrap).
inline double pymod(double a, double b) {
  double r = std::fmod(a, b);
  if (r != 0.0 && ((r < 0.0) != (b < 0.0))) {
    r += b;
  }
  return r;
}

// IK-convention joint angle triple (theta_coxa, theta_femur, theta_tibia).
using JointAngles = std::array<double, 3>;

// Canonical leg order. Fixed at six legs (see CLAUDE.md: leg count is not
// parameterised). Mirrors hexa_kinematics.leg_specs.LEG_NAMES.
inline const std::array<std::string, 6> LEG_NAMES = {
    "l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear",
};

// One leg's contribution to a LegTargets message — the shared currency every
// trajectory controller in this package emits. stance=true means the foot is on
// the ground this tick. During a descent/swing the phase value is informational
// (fractional progress through the curve).
struct LegOutput {
  Vec3 foot_target = Vec3::Zero();
  double phase = 0.0;
  bool stance = true;
};

// Every controller checks that a per-leg map covers all six legs and raises with
// the missing names listed (mirrors the `set(LEG_NAMES) - set(...)` checks
// throughout the Python package).
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

}  // namespace hexa_gait
