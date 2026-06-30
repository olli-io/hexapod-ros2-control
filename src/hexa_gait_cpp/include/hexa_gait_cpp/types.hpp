// Shared foundational types for the gait engine.
//
// Vec3 is the body-frame 3-vector currency used throughout (foot targets,
// stride vectors, mount positions). LEG_NAMES is the canonical leg ordering;
// every per-leg loop iterates it so behaviour is deterministic and matches the
// Python package (hexa_kinematics.leg_specs.LEG_NAMES, re-exported via clock).
#pragma once

#include <array>
#include <cmath>
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

}  // namespace hexa_gait
