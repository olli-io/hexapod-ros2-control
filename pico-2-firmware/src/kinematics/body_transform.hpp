// Body frame <-> leg coxa-mount frame transforms, and body-pose composition
// (plan part 05).
//
// Fork of hexa_kinematics_cpp/src/body_transform.cpp, double->float. Two layers
// of body-frame manipulation:
//
//   - body_to_leg / leg_to_body map between the (nominal) body frame and a
//     single leg's coxa-mount frame. Geometry only — no body-pose offset.
//   - BodyPose + apply_body_pose represent a 6-DOF offset of the body from its
//     nominal pose, and re-express a nominal-frame point as seen from the
//     offset body frame.
//
// Mirrors hexa_interfaces/msg/BodyPose.msg. Rotation convention is intrinsic
// XYZ (roll about body +x, then pitch about body +y, then yaw about body +z).
#pragma once

#include "config_generated.hpp"  // hexa::config::LegSpec
#include "vec3.hpp"              // hexa::Vec3

namespace hexa {

// Same generated geometry type the IK uses (redeclaring the identical alias in
// this namespace is well-formed; leg_ik.hpp declares it too).
using LegSpec = config::LegSpec;

// Map a point from the body frame into the leg's coxa-mount frame.
Vec3 body_to_leg(const Vec3& p_body, const LegSpec& leg);

// Map a point from the leg's coxa-mount frame back into the body frame.
Vec3 leg_to_body(const Vec3& p_leg, const LegSpec& leg);

// 6-DOF offset of the body from its nominal walking pose. Mirrors
// hexa_interfaces/msg/BodyPose.msg. Library-side type so the pure kinematics
// code stays free of ROS message dependencies.
struct BodyPose {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
};

inline constexpr BodyPose IDENTITY_BODY_POSE{};

// Re-express a foot target given in the nominal body frame as it appears in the
// body frame offset by pose: p_offset = R(pose)^T * (p_nominal - t(pose)), with
// R(pose) = Rz(yaw) * Ry(pitch) * Rx(roll). Pure function; no state.
Vec3 apply_body_pose(const Vec3& p_nominal, const BodyPose& pose);

}  // namespace hexa
