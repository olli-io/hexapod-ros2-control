// Loaders that expand hexa_description's YAML into typed kinematics inputs.
// Ports leg_specs.py and joint_config.py.
//
// Everything here parses config that lives in hexa_description/config/:
//
//   - geometry.yaml — leg segment lengths (leg:), the two reference coxa mounts
//     (mounts.l_front / mounts.l_middle), per-joint-type servo config (joints:),
//     and the initial_pose: block.
//   - standing_pose.yaml — per-joint-type default at-rest angle.
//
// Two families of output share this file because they share one concern (turn
// hexa_description YAML into typed values) and one mechanism: the six-leg
// expansion by the URDF symmetry rules
//
//   rear  mirrors front about the body y-axis: x -> -x, yaw -> pi - yaw.
//   right mirrors left  about the body x-axis: y -> -y, yaw -> -yaw.
//
// which drives both load_leg_specs (mount positions) and load_initial_pose
// (coxa angles). These match the mount_leg macro in hexapod.urdf.xacro: the YAML
// stays the single source of truth, and the URDF and this loader expand it the
// same way.
//
// Angles in the YAML are in each joint's intuitive degrees; this file is the
// single source of truth for converting them into the IK-convention radians used
// by the rest of hexa_kinematics (see types.hpp):
//
//   coxa  — theta_coxa  =  radians(deg).
//   femur — theta_femur = -radians(above_horizontal_deg).
//   tibia — theta_tibia =  pi - radians(interior_deg).
//
// femur and tibia conversions are monotonically decreasing, so an intuitive
// upper_limit_deg maps to a smaller URDF-rad value than lower_limit_deg. The
// loaders reconcile this with min/max after conversion.
#pragma once

#include <array>
#include <map>
#include <string>

#include "hexa_kinematics_cpp/types.hpp"

namespace hexa_kinematics {

// Canonical leg order. Fixed at six legs (see CLAUDE.md: leg count is not
// parameterised). Mirror of hexa_kinematics.leg_specs.LEG_NAMES.
inline const std::array<std::string, 6> LEG_NAMES = {
    "l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear",
};

// Parse geometry.yaml and return one LegSpec per leg, by name. Segment lengths
// come from leg.*; mount positions come from mounts.l_front / mounts.l_middle
// and are mirrored for the rear and right-side legs.
std::map<std::string, LegSpec> load_leg_specs(const std::string& geometry_yaml_path);

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
