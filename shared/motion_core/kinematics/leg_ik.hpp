// Per-leg forward and inverse kinematics, both in the leg's coxa-mount frame.
//
// inverse_kinematics returns the knee-up branch — the standard spider stance,
// knee on the upper-z side of the femur-joint-to-foot chord. It is the
// unconstrained mathematical IK and does NOT honour servo joint limits; callers
// validate against hexa_description limits before commanding hardware.
#pragma once

#include <stdexcept>

#include "config_generated.hpp"
#include "vec3.hpp"

namespace hexa {

using LegSpec = config::LegSpec;

// Foot target outside the leg's reachable workspace annulus.
class UnreachableTarget : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Foot position in the coxa-mount frame, given joint angles.
Vec3 forward_kinematics(const JointAngles& angles, const LegSpec& spec);

// Joint angles placing the foot at target. Throws UnreachableTarget if target
// lies outside the workspace annulus around the femur joint.
JointAngles inverse_kinematics(const Vec3& target, const LegSpec& spec);

// IK target z that puts the ground contact at contact_z. Both functions above
// terminate at the *centre* of the spherical foot tip, and a sphere on a flat
// floor touches directly below its centre whatever the tibia's lean, so the
// offset is exactly foot_radius at every joint angle. Apply it in the nominal
// body frame, before apply_body_pose, so it stays exact under a tilt.
inline float ik_z_for_contact(float contact_z, float foot_radius) {
  return contact_z + foot_radius;
}

}  // namespace hexa
