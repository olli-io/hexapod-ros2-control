// Shared foundational types for the C++ posture library.
//
// The posture package does no kinematics of its own; it only needs a 3-vector
// to carry per-leg foot targets from /legs/targets into the support-signal
// helpers (see signals.hpp). Vec3 uses the same underlying type as
// hexa_kinematics_cpp / hexa_gait_cpp (Eigen::Vector3d) for consistency, even
// though this package does not depend on those libraries.
#pragma once

#include <cstddef>

#include <Eigen/Core>

namespace hexa_posture {

// Body-frame 3-vector currency (x, y, z) in metres.
using Vec3 = Eigen::Vector3d;

// Leg count is fixed at 6 across the whole robot.
constexpr std::size_t LEG_COUNT = 6;

}  // namespace hexa_posture
