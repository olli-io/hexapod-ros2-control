// Runtime-supplied geometry + tuning config for the control brain (part 10).
//
// The pipeline's builders normally read the baked config_generated.hpp constants
// directly (the Pico has no filesystem). To let the ROS hexa_locomotion node
// honor the repo's runtime-YAML tuning, the Pipeline ctor instead accepts a
// PipelineConfig carrying the geometry.yaml + tuning.yaml subset. PipelineConfig::
// baked() reconstructs the exact constexpr values, so the Pico (and every current
// caller) keeps identical behavior; hexa_locomotion fills the same struct from
// geometry.yaml / tuning.yaml at startup (reusing the double hexa_*_cpp loaders).
//
// Scope is deliberately geometry + tuning only. Joystick-binding tables and
// servo calibration stay baked (the ROS node builds a CommandIntent from topics
// and publishes joint radians, so it uses neither); supervisor safety
// (kInputTimeoutS / kBattery, from webteleop/display YAML) also stays baked.
#pragma once

#include <array>
#include <string>

#include "config_generated.hpp"  // config::LegSpec / ControlConfig / PostureConfig
#include "gait/engine.hpp"       // gait::EngineConfig
#include "gait/limits.hpp"       // gait::VelocityCaps
#include "leg_index.hpp"         // hexa::kNumLegs
#include "vec3.hpp"              // hexa::JointAngles

namespace hexa::pipeline {

struct PipelineConfig {
  // ── geometry.yaml ──
  std::array<hexa::config::LegSpec, hexa::kNumLegs> leg_specs;
  float coxa_to_bottom = 0.0f;
  hexa::JointAngles standing_pose{};
  std::array<hexa::JointAngles, hexa::kNumLegs> initial_pose{};

  // ── tuning.yaml ──
  hexa::gait::EngineConfig engine{};      // already converted from config::kEngine
  hexa::gait::VelocityCaps caps;          // per-gait linear_max/yaw_bias + angular_max
  std::string default_gait;
  hexa::config::ControlConfig control{};  // vmax_ramp_time_* + snap_tol_*
  hexa::config::PostureConfig posture{};  // animation amplitudes, taus, reserves, slew

  // Reconstruct the baked constexpr config (config_generated.hpp). Used by the
  // Pico / any default construction; also the parity anchor for the ROS loader.
  static PipelineConfig baked();
};

}  // namespace hexa::pipeline
