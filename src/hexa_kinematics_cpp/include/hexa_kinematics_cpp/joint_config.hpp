// Per-joint servo configuration and default standing pose. Port of
// joint_config.py.
//
// Loads the two YAMLs that live in hexa_description/config/:
//
//   - geometry.yaml — under joints:, per-joint-type servo center (URDF angle at
//     the servo's physical zero) plus absolute lower / upper travel limits, all
//     in intuitive per-joint degrees. Also the initial_pose: block.
//   - standing_pose.yaml — per-joint-type default at-rest angle.
//
// Both files express angles in degrees, in each joint's intuitive sense. This
// module is the single source of truth for converting those intuitive degrees
// into the IK-convention radians used by hexa_kinematics (see leg_geometry.hpp):
//
//   coxa  — theta_coxa  =  radians(deg).
//   femur — theta_femur = -radians(above_horizontal_deg).
//   tibia — theta_tibia =  pi - radians(interior_deg).
//
// femur and tibia conversions are monotonically decreasing, so an intuitive
// upper_limit_deg maps to a smaller URDF-rad value than lower_limit_deg. The
// loader reconciles this with min/max after conversion.
#pragma once

#include <map>
#include <string>

#include "hexa_kinematics_cpp/types.hpp"

namespace hexa_kinematics {

// Servo configuration for one joint type, in IK-convention radians.
struct JointLimits {
  double center = 0.0;    // rad — URDF angle at the servo's physical zero
  double lower = 0.0;     // rad — URDF lower bound (always <= upper)
  double upper = 0.0;     // rad — URDF upper bound
  double effort = 0.0;    // Nm
  double velocity = 0.0;  // rad/s
};

// Parse geometry.yaml's joints: block into {joint_type: JointLimits}. joint_type
// is one of "coxa", "femur", "tibia"; center/lower/upper are in IK-convention
// radians, with lower <= center <= upper.
std::map<std::string, JointLimits> load_joint_limits(const std::string& geometry_path);

// Parse standing_pose.yaml into (theta_coxa, theta_femur, theta_tibia) in
// IK-convention radians. Each angle is validated against geometry.yaml's
// [lower, upper] window; a value outside it throws.
JointAngles load_standing_pose(const std::string& standing_pose_path,
                               const std::string& geometry_path);

// Parse geometry.yaml's initial_pose: block into per-leg JointAngles. The YAML
// stores only the two reference coxa values (l_front_deg, l_middle_deg); the
// other four legs derive by the same mirror rules as load_leg_specs (rear
// negates coxa; right negates after the front/rear mirror). Femur and tibia are
// uniform across all six legs. Each per-leg angle is validated against the
// [lower, upper] window.
std::map<std::string, JointAngles> load_initial_pose(const std::string& geometry_path);

}  // namespace hexa_kinematics
