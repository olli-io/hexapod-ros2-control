// Body<->leg transforms + body-pose composition — float fork of
// hexa_kinematics_cpp/src/body_transform.cpp (part 05). double->float
// (cosf/sinf); the math is otherwise line-for-line the double source.

#include "kinematics/body_transform.hpp"

#include <cmath>

namespace hexa {

Vec3 body_to_leg(const Vec3& p_body, const LegSpec& leg) {
  const float dx = p_body[0] - leg.mount_xyz[0];
  const float dy = p_body[1] - leg.mount_xyz[1];
  const float dz = p_body[2] - leg.mount_xyz[2];
  const float c = cosf(leg.mount_yaw);
  const float s = sinf(leg.mount_yaw);
  return Vec3(c * dx + s * dy, -s * dx + c * dy, dz);
}

Vec3 leg_to_body(const Vec3& p_leg, const LegSpec& leg) {
  const float c = cosf(leg.mount_yaw);
  const float s = sinf(leg.mount_yaw);
  const float x = c * p_leg[0] - s * p_leg[1];
  const float y = s * p_leg[0] + c * p_leg[1];
  return Vec3(x + leg.mount_xyz[0], y + leg.mount_xyz[1],
              p_leg[2] + leg.mount_xyz[2]);
}

Vec3 apply_body_pose(const Vec3& p_nominal, const BodyPose& pose) {
  const float dx = p_nominal[0] - pose.x;
  const float dy = p_nominal[1] - pose.y;
  const float dz = p_nominal[2] - pose.z;

  const float cr = cosf(pose.roll);
  const float sr = sinf(pose.roll);
  const float cp = cosf(pose.pitch);
  const float sp = sinf(pose.pitch);
  const float cy = cosf(pose.yaw);
  const float sy = sinf(pose.yaw);

  // R(pose) = Rz(yaw) * Ry(pitch) * Rx(roll); apply
  // R^T = Rx(-roll) * Ry(-pitch) * Rz(-yaw).
  // Rz(-yaw):
  const float x1 = cy * dx + sy * dy;
  const float y1 = -sy * dx + cy * dy;
  const float z1 = dz;
  // Ry(-pitch):
  const float x2 = cp * x1 - sp * z1;
  const float y2 = y1;
  const float z2 = sp * x1 + cp * z1;
  // Rx(-roll):
  const float x3 = x2;
  const float y3 = cr * y2 + sr * z2;
  const float z3 = -sr * y2 + cr * z2;
  return Vec3(x3, y3, z3);
}

}  // namespace hexa
