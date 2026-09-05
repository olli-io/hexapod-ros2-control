#include "pipeline_config.hpp"

#include <cstddef>
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
  c.presets = ::hexa::gait::preset_specs_from_config();
  c.default_preset = cfg::kDefaultPreset;
  c.folded_pose = cfg::kFoldedPose;
  c.initialized_pose = cfg::kInitializedPose;
  // tuning.yaml
  c.engine = ::hexa::gait::engine_config_from_config();
  // The angular cap needs each preset's standing foot positions, and the linear
  // one its stride and swing time. Solved here rather than inside
  // load_velocity_caps_from_config, to keep gait/limits.cpp free of the
  // IK/engine translation units.
  const auto setups = ::hexa::gait::preset_setups_from_config();
  for (std::size_t i = 0; i < setups.size(); ++i) {
    c.caps_by_preset[setups[i].id] = ::hexa::gait::load_velocity_caps_from_config(
        i, ::hexa::gait::outer_stance_radius(setups[i].nominal_stance));
  }
  c.default_gait = std::string(cfg::kDefaultGait);
  c.control = cfg::kControl;
  c.posture = cfg::kPosture;
  return c;
}

}  // namespace hexa::pipeline
