// Geometric description of hexapod legs. Port of leg_geometry.py.
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

#include "hexa_kinematics_cpp/types.hpp"

namespace hexa_kinematics {

// Mirror of hexa_kinematics.leg_geometry.LegSpec (frozen dataclass).
struct LegSpec {
  Point3 mount_xyz = Point3::Zero();  // coxa pivot position in body frame (m)
  double mount_yaw = 0.0;             // rotation about body +z (rad)
  double coxa_len = 0.0;
  double femur_len = 0.0;
  double tibia_len = 0.0;
};

}  // namespace hexa_kinematics
