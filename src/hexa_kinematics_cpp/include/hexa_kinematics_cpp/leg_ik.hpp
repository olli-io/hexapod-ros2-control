// Per-leg forward and inverse kinematics. Port of leg_ik.py.
//
// Both functions operate in the leg's coxa-mount frame (see leg_geometry.hpp).
// inverse_kinematics returns the knee-up branch — the standard hexapod spider
// stance, where the knee sits on the upper-z side of the chord from femur joint
// to foot.
#pragma once

#include "hexa_kinematics_cpp/leg_geometry.hpp"
#include "hexa_kinematics_cpp/types.hpp"

namespace hexa_kinematics {

// Foot position in the coxa-mount frame, given joint angles. Pure FK.
Point3 forward_kinematics(const JointAngles& angles, const LegSpec& spec);

// Joint angles placing the foot at target in the coxa-mount frame. Returns the
// knee-up branch. Throws UnreachableTarget if target lies outside the workspace
// annulus around the femur joint.
//
// This is the unconstrained mathematical IK — it does not honour servo joint
// limits. Callers must validate the returned angles against hexa_description
// joint limits before commanding hardware.
JointAngles inverse_kinematics(const Point3& target, const LegSpec& spec);

}  // namespace hexa_kinematics
