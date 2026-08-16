// Runtime-supplied geometry + tuning for the control brain. The Pico has no
// filesystem and reads the baked config_generated.hpp constants, which baked()
// reconstructs exactly; hexa_locomotion fills the same struct from geometry.yaml
// / tuning.yaml at startup.
//
// Scope is deliberately geometry + tuning only: joystick bindings, servo
// calibration and supervisor safety all stay baked.
#pragma once

#include <array>
#include <string>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gait/limits.hpp"
#include "leg_index.hpp"
#include "vec3.hpp"

namespace hexa::pipeline {

struct PipelineConfig {
  // geometry.yaml
  std::array<hexa::config::LegSpec, hexa::kNumLegs> leg_specs;
  float coxa_to_bottom = 0.0f;
  float foot_radius = 0.0f;
  // tuning.yaml gait_node.default_standing_pose: a body height plus a per-group
  // reach and splay, not joint angles — gait::standing_pose_from solves the
  // per-leg triple from these and the leg specs.
  hexa::config::StandingPose standing_pose{};
  // The two belly-rest poses: folded is where the robot energizes and where a
  // fold ends, initialized is the unfold ladder's endpoint.
  std::array<hexa::JointAngles, hexa::kNumLegs> folded_pose{};
  std::array<hexa::JointAngles, hexa::kNumLegs> initialized_pose{};

  // tuning.yaml
  hexa::gait::EngineConfig engine{};
  hexa::gait::VelocityCaps caps;          // per-gait linear_max/yaw_bias + angular_max
  std::string default_gait;
  hexa::config::ControlConfig control{};  // vmax_ramp_time_* + snap_tol_*
  hexa::config::PostureConfig posture{};  // animation amplitudes, taus, limits, slew

  // The Pico's config, and the parity anchor for the ROS loader.
  static PipelineConfig baked();
};

}  // namespace hexa::pipeline
