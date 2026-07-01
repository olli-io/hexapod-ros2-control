// Body frame <-> leg coxa-mount frame transforms, and body-pose composition.
// Port of body_transform.py.
//
// Two layers of body-frame manipulation live here:
//
//   - body_to_leg / leg_to_body map between the (nominal) body frame and a
//     single leg's coxa-mount frame. Geometry only — no notion of body pose
//     offset.
//   - BodyPose + apply_body_pose represent a 6-DOF offset of the body from its
//     nominal pose, and re-express a point given in the nominal frame as seen
//     from the offset body frame.
//
// Mirrors hexa_interfaces/msg/BodyPose.msg. The rotation convention is intrinsic
// XYZ (roll about body +x, then pitch about body +y, then yaw about body +z).
#pragma once

#include "hexa_kinematics_cpp/leg_geometry.hpp"
#include "hexa_kinematics_cpp/types.hpp"

namespace hexa_kinematics {

// Map a point from the body frame into the leg's coxa-mount frame.
Point3 body_to_leg(const Point3& p_body, const LegSpec& leg);

// Map a point from the leg's coxa-mount frame back into the body frame.
Point3 leg_to_body(const Point3& p_leg, const LegSpec& leg);

// 6-DOF offset of the body from its nominal walking pose. Mirrors
// hexa_interfaces/msg/BodyPose.msg. Library-side type so the pure kinematics
// code stays free of ROS message dependencies.
struct BodyPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

inline const BodyPose IDENTITY_BODY_POSE{};

// Re-express a foot target given in the nominal body frame as it appears in the
// body frame offset by pose: p_offset = R(pose)^T * (p_nominal - t(pose)), with
// R(pose) = Rz(yaw) * Ry(pitch) * Rx(roll). Pure function; no state.
Point3 apply_body_pose(const Point3& p_nominal, const BodyPose& pose);

}  // namespace hexa_kinematics
