// Body frame <-> leg coxa-mount frame transforms, and body-pose composition.
// Two layers: body_to_leg / leg_to_body are geometry only, with no body-pose
// offset; BodyPose + apply_body_pose re-express a nominal-frame point as seen
// from the offset body frame. Rotation convention is intrinsic XYZ (roll about
// body +x, then pitch about +y, then yaw about +z).
#pragma once

#include "config_generated.hpp"
#include "vec3.hpp"

namespace hexa {

// The identical alias in leg_ik.hpp is well-formed alongside this one.
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
