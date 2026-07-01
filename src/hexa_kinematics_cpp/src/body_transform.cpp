#include "hexa_kinematics_cpp/body_transform.hpp"

#include <cmath>

namespace hexa_kinematics {

Point3 body_to_leg(const Point3& p_body, const LegSpec& leg) {
  const double dx = p_body[0] - leg.mount_xyz[0];
  const double dy = p_body[1] - leg.mount_xyz[1];
  const double dz = p_body[2] - leg.mount_xyz[2];
  const double c = std::cos(leg.mount_yaw);
  const double s = std::sin(leg.mount_yaw);
  return Point3(c * dx + s * dy, -s * dx + c * dy, dz);
}

Point3 leg_to_body(const Point3& p_leg, const LegSpec& leg) {
  const double c = std::cos(leg.mount_yaw);
  const double s = std::sin(leg.mount_yaw);
  const double x = c * p_leg[0] - s * p_leg[1];
  const double y = s * p_leg[0] + c * p_leg[1];
  return Point3(x + leg.mount_xyz[0], y + leg.mount_xyz[1],
                p_leg[2] + leg.mount_xyz[2]);
}

Point3 apply_body_pose(const Point3& p_nominal, const BodyPose& pose) {
  const double dx = p_nominal[0] - pose.x;
  const double dy = p_nominal[1] - pose.y;
  const double dz = p_nominal[2] - pose.z;

  const double cr = std::cos(pose.roll);
  const double sr = std::sin(pose.roll);
  const double cp = std::cos(pose.pitch);
  const double sp = std::sin(pose.pitch);
  const double cy = std::cos(pose.yaw);
  const double sy = std::sin(pose.yaw);

  // R(pose) = Rz(yaw) * Ry(pitch) * Rx(roll); apply
  // R^T = Rx(-roll) * Ry(-pitch) * Rz(-yaw).
  // Rz(-yaw):
  const double x1 = cy * dx + sy * dy;
  const double y1 = -sy * dx + cy * dy;
  const double z1 = dz;
  // Ry(-pitch):
  const double x2 = cp * x1 - sp * z1;
  const double y2 = y1;
  const double z2 = sp * x1 + cp * z1;
  // Rx(-roll):
  const double x3 = x2;
  const double y3 = cr * y2 + sr * z2;
  const double z3 = -sr * y2 + cr * z2;
  return Point3(x3, y3, z3);
}

}  // namespace hexa_kinematics
