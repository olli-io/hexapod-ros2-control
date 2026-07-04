// Adapter onto the firmware's float kinematics port (src/kinematics), mirroring
// hexa_gait_cpp/include/hexa_gait_cpp/kinematics.hpp.
//
// The gait engine refers to the kinematics surface as hexa::gait::kin::*; this
// header includes the real float headers and aliases that namespace onto
// ::hexa. Because the firmware kinematics already speaks config::LegSpec / Vec3 /
// JointAngles, the swap is just this alias — no engine code changed.
#pragma once

#include "kinematics/body_transform.hpp"
#include "kinematics/leg_ik.hpp"

namespace hexa::gait {
namespace kin = ::hexa;
}  // namespace hexa::gait
