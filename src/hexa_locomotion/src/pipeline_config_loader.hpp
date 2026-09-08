// Runtime PipelineConfig loader: geometry.yaml + tuning.yaml -> PipelineConfig.
// Must stay field-for-field identical to tools/gen_config.py; the parity test
// in test/test_config_loader.cpp enforces it.
#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "pipeline_config.hpp"

namespace hexa::locomotion {

// No ROS dependency. Throws on a missing/mistyped key or unparseable file.
hexa::pipeline::PipelineConfig load_pipeline_config_from_yaml(
    const std::string& geometry_path, const std::string& tuning_path);

// Reads hexa_description's installed YAMLs; on any error logs and falls back to
// PipelineConfig::baked() rather than crashing the controller.
hexa::pipeline::PipelineConfig load_pipeline_config(rclcpp::Node& node);

}  // namespace hexa::locomotion
