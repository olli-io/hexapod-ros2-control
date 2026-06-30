// TEMPORARY placeholder for the hexa_kinematics surface consumed by the gait
// engine. hexa_kinematics is still a Python (ament_python) package; until it is
// ported to C++ (and leg_specs moves to hexa_description), this header provides
// compile-only stubs with signatures identical to the eventual real API so the
// engine builds and links.
//
// TODO(kinematics-port): replace this entire header with includes of the real
// ported hexa_kinematics C++ library. The swap should be a one-line include
// change plus a namespace alias — keep these signatures in lockstep with:
//   - hexa_kinematics/leg_geometry.py   (LegSpec)
//   - hexa_kinematics/leg_specs.py      (load_leg_specs)
//   - hexa_kinematics/body_transform.py (leg_to_body)
//   - hexa_kinematics/leg_ik.py         (forward_kinematics)
//   - hexa_kinematics/joint_config.py   (load_standing_pose, load_initial_pose)
//
// WARNING: every function below returns zeros / degenerate geometry. The engine
// COMPILES and the state machine RUNS, but any value derived from these
// (nominal stance, initial stance, reseat geometry) is PLACEHOLDER and wrong
// until the real kinematics port lands. Message flow and state transitions can
// be exercised; absolute foot positions cannot.
#pragma once

#include <map>
#include <string>

#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait::kin {

// Mirror of hexa_kinematics.leg_geometry.LegSpec (frozen dataclass).
struct LegSpec {
  Vec3 mount_xyz = Vec3::Zero();
  double mount_yaw = 0.0;
  double coxa_len = 0.0;
  double femur_len = 0.0;
  double tibia_len = 0.0;
};

// Mirror of hexa_kinematics.leg_specs.load_leg_specs: one LegSpec per leg.
// STUB: returns a zeroed spec per leg.
inline std::map<std::string, LegSpec> load_leg_specs(
    const std::string& /*geometry_yaml_path*/) {
  std::map<std::string, LegSpec> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = LegSpec{};
  }
  return out;
}

// Mirror of hexa_kinematics.body_transform.leg_to_body: map a point from the
// leg's coxa-mount frame back into the body frame.
// STUB: returns zero.
inline Vec3 leg_to_body(const Vec3& /*p_leg*/, const LegSpec& /*leg*/) {
  return Vec3::Zero();
}

// Mirror of hexa_kinematics.leg_ik.forward_kinematics: foot position in the
// leg's coxa-mount frame for a joint-angle triple.
// STUB: returns zero.
inline Vec3 forward_kinematics(const JointAngles& /*angles*/,
                               const LegSpec& /*leg*/) {
  return Vec3::Zero();
}

// Mirror of hexa_kinematics.joint_config.load_standing_pose:
// (theta_coxa, theta_femur, theta_tibia) in IK-convention radians.
// STUB: returns zeros.
inline JointAngles load_standing_pose(const std::string& /*standing_pose_path*/,
                                      const std::string& /*geometry_path*/) {
  return JointAngles{0.0, 0.0, 0.0};
}

// Mirror of hexa_kinematics.joint_config.load_initial_pose: per-leg folded
// initial-pose joint angles.
// STUB: returns zeros for every leg.
inline std::map<std::string, JointAngles> load_initial_pose(
    const std::string& /*geometry_path*/) {
  std::map<std::string, JointAngles> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = JointAngles{0.0, 0.0, 0.0};
  }
  return out;
}

}  // namespace hexa_gait::kin
