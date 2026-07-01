// Shared foundational types for the C++ kinematics library.
//
// Port of the type aliases and LegSpec dataclass in
// hexa_kinematics/leg_geometry.py and the UnreachableTarget exception from
// hexa_kinematics/leg_ik.py. Point3 and JointAngles use the same underlying
// types as hexa_gait_cpp (Eigen::Vector3d and std::array<double, 3>) so the gait
// engine can consume this library through a one-line namespace alias (see
// hexa_gait_cpp/kinematics.hpp).
//
// A leg is described in its coxa-mount frame — a frame rigidly attached to the
// body at the coxa pivot. The frame is aligned (REP-103, body frame):
//
//   +x — radially outward from the body in the leg's neutral direction.
//   +y — in the body's horizontal plane, completing a right-handed frame.
//   +z — up (parallel to body +z).
//
// The body->coxa-mount mapping is fully captured by mount_xyz (translation) and
// mount_yaw (rotation about body +z). No roll/pitch is supported: all coxa axes
// are vertical, which matches typical hexapod hardware.
//
// Joint angles (radians) follow the chain coxa -> femur -> tibia:
//
//   theta_coxa  — rotation about coxa-mount +z. 0 puts the femur along +x;
//                 positive rotates toward +y (right-hand rule).
//   theta_femur — rotation about the post-coxa +y axis. 0 keeps the femur
//                 horizontal; positive tilts the foot toward -z.
//   theta_tibia — bend at the knee, same axis as theta_femur. 0 keeps the
//                 tibia colinear with the femur (leg fully extended); positive
//                 bends the knee toward -z.
#pragma once

#include <array>
#include <stdexcept>

#include <Eigen/Core>

namespace hexa_kinematics {

// Body / leg-frame 3-vector currency (x, y, z) in metres.
using Point3 = Eigen::Vector3d;

// IK-convention joint-angle triple (theta_coxa, theta_femur, theta_tibia) in
// radians. See the joint-zero conventions above.
using JointAngles = std::array<double, 3>;

// Foot target lies outside the leg's reachable workspace. Mirrors
// hexa_kinematics.leg_ik.UnreachableTarget (a ValueError in Python).
class UnreachableTarget : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Mirror of hexa_kinematics.leg_geometry.LegSpec (frozen dataclass).
struct LegSpec {
  Point3 mount_xyz = Point3::Zero();  // coxa pivot position in body frame (m)
  double mount_yaw = 0.0;             // rotation about body +z (rad)
  double coxa_len = 0.0;
  double femur_len = 0.0;
  double tibia_len = 0.0;
};

}  // namespace hexa_kinematics
