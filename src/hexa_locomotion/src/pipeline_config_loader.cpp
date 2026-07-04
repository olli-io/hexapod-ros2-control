#include "pipeline_config_loader.hpp"

namespace hexa::locomotion {

hexa::pipeline::PipelineConfig load_pipeline_config(rclcpp::Node& node) {
  // TODO(phase-3b): read geometry.yaml + tuning.yaml at runtime and override the
  // baked values, so editing the YAML + a restart retunes without a rebuild:
  //   - geometry: hexa_kinematics::load_leg_specs / load_standing_pose /
  //     load_initial_pose (+ coxa_to_bottom) from geometry.yaml,
  //   - caps: hexa_gait::load_velocity_caps(tuning.yaml),
  //   - engine / control / posture: read the tuning.yaml gait_node / control_node
  //     / posture_node blocks with yaml-cpp (mapping mirrors gen_config.py),
  //   converting double -> the float PipelineConfig.
  //
  // Until then the node uses the build-time baked config (gen_config.py bakes it
  // from the same YAMLs, exactly like hexa_pico_bridge), so the node is already
  // correct as of the last build.
  RCLCPP_INFO(node.get_logger(),
              "hexa_locomotion: using baked geometry+tuning config "
              "(runtime YAML load pending, phase 3b)");
  return hexa::pipeline::PipelineConfig::baked();
}

}  // namespace hexa::locomotion
