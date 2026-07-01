// Adapter onto the real C++ kinematics library (hexa_kinematics_cpp).
//
// Replaces the former compile-only kinematics_stub.hpp. The gait engine refers
// to the kinematics surface as hexa_gait::kin::*; this header includes the real
// headers and aliases that namespace onto ::hexa_kinematics. Because both
// packages share the same underlying types (Point3 = Eigen::Vector3d,
// JointAngles = std::array<double, 3>), the swap is just this alias — no engine
// code changed. Nominal / initial / reseat stance values are now real geometry.
#pragma once

#include "hexa_kinematics_cpp/body_transform.hpp"
#include "hexa_kinematics_cpp/joint_config.hpp"
#include "hexa_kinematics_cpp/leg_geometry.hpp"
#include "hexa_kinematics_cpp/leg_ik.hpp"
#include "hexa_kinematics_cpp/leg_specs.hpp"

namespace hexa_gait {
namespace kin = ::hexa_kinematics;
}  // namespace hexa_gait
