// Runtime geometry + tuning loader for the consolidated locomotion node.
//
// Builds a hexa::pipeline::PipelineConfig at node startup so editing
// geometry.yaml / tuning.yaml and restarting retunes the robot without a
// rebuild (the repo's runtime-tuning contract). Reuses the double C++ loaders
// (hexa_kinematics::load_leg_specs / load_standing_pose / load_initial_pose,
// hexa_gait::load_velocity_caps) and reads the engine/control/posture tuning
// blocks of tuning.yaml directly, converting double -> the float PipelineConfig.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "pipeline_config.hpp"

namespace hexa::locomotion {

// Load the PipelineConfig from hexa_description's installed geometry.yaml /
// tuning.yaml. `node` supplies the logger and any path-override parameters.
hexa::pipeline::PipelineConfig load_pipeline_config(rclcpp::Node& node);

}  // namespace hexa::locomotion
