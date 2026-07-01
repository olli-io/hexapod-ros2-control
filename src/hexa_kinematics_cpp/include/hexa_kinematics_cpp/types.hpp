// Shared foundational types for the C++ kinematics library.
//
// Port of the type aliases in hexa_kinematics/leg_geometry.py and the
// UnreachableTarget exception from hexa_kinematics/leg_ik.py. Point3 and
// JointAngles use the same underlying types as hexa_gait_cpp (Eigen::Vector3d
// and std::array<double, 3>) so the gait engine can consume this library
// through a one-line namespace alias (see hexa_gait_cpp/kinematics.hpp).
#pragma once

#include <array>
#include <stdexcept>

#include <Eigen/Core>

namespace hexa_kinematics {

// Body / leg-frame 3-vector currency (x, y, z) in metres.
using Point3 = Eigen::Vector3d;

// IK-convention joint-angle triple (theta_coxa, theta_femur, theta_tibia) in
// radians. See leg_geometry.hpp for the joint-zero conventions.
using JointAngles = std::array<double, 3>;

// Foot target lies outside the leg's reachable workspace. Mirrors
// hexa_kinematics.leg_ik.UnreachableTarget (a ValueError in Python).
class UnreachableTarget : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

}  // namespace hexa_kinematics
