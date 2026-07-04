// Runtime geometry + tuning loader for the consolidated locomotion node.
//
// Builds a hexa::pipeline::PipelineConfig at node startup so editing
// geometry.yaml / tuning.yaml and restarting retunes the robot without a
// rebuild (the repo's runtime-tuning contract). Reads both files with yaml-cpp
// directly, reproducing the six-leg symmetry expansion + deg->IK-radian joint
// conventions and the per-gait velocity-caps derivation (the same math the
// build-time gen_config.py bakes), converting to the float PipelineConfig. The
// per-gait duty factors come from the linked firmware gait registry
// (hexa::gait::strategies()), so nothing is re-hardcoded here.
#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "pipeline_config.hpp"

namespace hexa::locomotion {

// Pure loader: build the PipelineConfig from explicit geometry.yaml / tuning.yaml
// paths, with no ROS dependency. Throws std::exception on a missing/mistyped key
// or unparseable file. The parity test drives this directly; the node wrapper
// below resolves the installed hexa_description paths and adds logging.
hexa::pipeline::PipelineConfig load_pipeline_config_from_yaml(
    const std::string& geometry_path, const std::string& tuning_path);

// Load the PipelineConfig from hexa_description's installed geometry.yaml /
// tuning.yaml. `node` supplies the logger. On any load error it logs and falls
// back to the baked defaults so a bad YAML edit degrades to the last-built
// config rather than crashing the controller.
hexa::pipeline::PipelineConfig load_pipeline_config(rclcpp::Node& node);

}  // namespace hexa::locomotion
