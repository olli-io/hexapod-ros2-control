// Per-leg forward and inverse kinematics (plan part 05).
//
// Fork of hexa_kinematics_cpp/src/leg_ik.cpp (namespace hexa_kinematics),
// double->float for the RP2350 single-precision FPU. Both functions operate in
// the leg's coxa-mount frame (see hexa_kinematics_cpp/types.hpp). The geometry
// currency is the firmware's Vec3 (src/vec3.hpp); the leg description is the
// generated config::LegSpec (src/config_generated.hpp) — the same fields as the
// double LegSpec, so no geometry values are duplicated here.
//
// inverse_kinematics returns the knee-up branch — the standard hexapod spider
// stance, where the knee sits on the upper-z side of the chord from femur joint
// to foot. It is the unconstrained mathematical IK; it does NOT honour servo
// joint limits. Callers validate against hexa_description limits before
// commanding hardware.
#pragma once

#include <stdexcept>

#include "config_generated.hpp"  // hexa::config::LegSpec
#include "vec3.hpp"              // hexa::Vec3, hexa::JointAngles

namespace hexa {

// The forked pure libs speak the generated geometry type directly.
using LegSpec = config::LegSpec;

// Foot target lies outside the leg's reachable workspace annulus. Mirrors
// hexa_kinematics::UnreachableTarget (a ValueError in the Python source).
class UnreachableTarget : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Foot position in the coxa-mount frame, given joint angles. Pure FK.
Vec3 forward_kinematics(const JointAngles& angles, const LegSpec& spec);

// Joint angles placing the foot at target in the coxa-mount frame. Returns the
// knee-up branch. Throws UnreachableTarget if target lies outside the workspace
// annulus around the femur joint.
JointAngles inverse_kinematics(const Vec3& target, const LegSpec& spec);

}  // namespace hexa
