// Runtime-supplied geometry + tuning for the control brain. The Pico has no
// filesystem and reads the baked config_generated.hpp constants, which baked()
// reconstructs exactly; hexa_locomotion fills the same struct from geometry.yaml
// / tuning.yaml at startup.
//
// Scope is deliberately geometry + tuning only: joystick bindings, servo
// calibration and supervisor safety all stay baked.
#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

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
  // tuning.yaml gait_node.presets: the bundle the operator selects as one
  // thing. Each carries the legs it stands on, a body height plus a per-group
  // reach and splay (not joint angles — gait::standing_pose_from solves the
  // per-leg triple from these and the leg specs), and its own stride and swing
  // times. In declaration order, which is the order /gait/preset reports.
  std::vector<hexa::gait::PresetSpec> presets;
  // Index into `presets` of the one the robot boots on. Must stand on all six:
  // the cold-start baseline is folded on six, and it is what FAULT recovers to.
  std::size_t default_preset = 0;
  // The two belly-rest poses: folded is where the robot energizes, where a fold
  // ends and where quadruped mode parks the middle pair; initialized is the
  // unfold ladder's endpoint, and the rung the middles climb through either way.
  std::array<hexa::JointAngles, hexa::kNumLegs> folded_pose{};
  std::array<hexa::JointAngles, hexa::kNumLegs> initialized_pose{};

  // tuning.yaml
  hexa::gait::EngineConfig engine{};
  // Per preset, keyed by its id, then per gait. Three of the four inputs to a
  // cap ride the preset — the stride it lays down, the swing time it lays it
  // down in, and the stance the angular cap divides by — so the table cannot be
  // per-gait alone.
  std::map<std::string, hexa::gait::VelocityCaps> caps_by_preset;
  std::string default_gait;
  hexa::config::ControlConfig control{};  // vmax_ramp_time_* + snap_tol_*
  hexa::config::PostureConfig posture{};  // animation amplitudes, taus, limits, slew

  // The Pico's config, and the parity anchor for the ROS loader.
  static PipelineConfig baked();
};

}  // namespace hexa::pipeline
