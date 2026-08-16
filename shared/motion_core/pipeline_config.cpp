#include "pipeline_config.hpp"

#include <string>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gait/limits.hpp"

namespace hexa::pipeline {

PipelineConfig PipelineConfig::baked() {
  namespace cfg = ::hexa::config;
  PipelineConfig c;
  // geometry.yaml
  c.leg_specs = cfg::kLegSpecs;
  c.coxa_to_bottom = cfg::kCoxaToBottom;
  c.foot_radius = cfg::kFootRadius;
  c.standing_pose = cfg::kStandingPose;
  c.folded_pose = cfg::kFoldedPose;
  c.initialized_pose = cfg::kInitializedPose;
  // tuning.yaml
  c.engine = ::hexa::gait::engine_config_from_config();
  // The angular cap needs the standing foot positions. Solved here rather than
  // inside load_velocity_caps_from_config, to keep gait/limits.cpp free of the
  // IK/engine translation units.
  c.caps = ::hexa::gait::load_velocity_caps_from_config(
      ::hexa::gait::outer_stance_radius(
          ::hexa::gait::nominal_stance_from_config()));
  c.default_gait = std::string(cfg::kDefaultGait);
  c.control = cfg::kControl;
  c.posture = cfg::kPosture;
  return c;
}

}  // namespace hexa::pipeline
