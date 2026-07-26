#include "pipeline_config.hpp"

#include <string>

#include "config_generated.hpp"
#include "gait/engine.hpp"   // engine_config_from_config
#include "gait/limits.hpp"   // load_velocity_caps_from_config

namespace hexa::pipeline {

PipelineConfig PipelineConfig::baked() {
  namespace cfg = ::hexa::config;
  PipelineConfig c;
  // geometry.yaml
  c.leg_specs = cfg::kLegSpecs;
  c.coxa_to_bottom = cfg::kCoxaToBottom;
  c.standing_pose = cfg::kStandingPose;
  c.initial_pose = cfg::kInitialPose;
  // tuning.yaml
  c.engine = ::hexa::gait::engine_config_from_config();  // config::kEngine -> gait::EngineConfig
  // The angular cap is the linear one over the lever arm a yaw rate acts
  // through, so the caps need the standing foot positions. Solved here (rather
  // than inside load_velocity_caps_from_config) to keep gait/limits.cpp free of
  // the IK/engine translation units — test_control and test_joy_mapping link it
  // on its own.
  c.caps = ::hexa::gait::load_velocity_caps_from_config(
      ::hexa::gait::outer_stance_radius(
          ::hexa::gait::nominal_stance_from_config()));
  c.default_gait = std::string(cfg::kDefaultGait);
  c.control = cfg::kControl;
  c.posture = cfg::kPosture;
  return c;
}

}  // namespace hexa::pipeline
