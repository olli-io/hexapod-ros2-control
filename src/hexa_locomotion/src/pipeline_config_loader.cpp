#include "pipeline_config_loader.hpp"

#include <exception>
#include <map>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <hexa_gait_cpp/limits.hpp>            // hexa_gait::load_velocity_caps
#include <hexa_kinematics_cpp/description_loader.hpp>

#include "gait/types.hpp"  // hexa::gait::LEG_NAMES
#include "vec3.hpp"        // hexa::Vec3

namespace hexa::locomotion {

namespace {

namespace kin = hexa_kinematics;

float f(const YAML::Node& n) { return n.as<float>(); }

// A node's `ros__parameters:` block (tuning.yaml is a standard ROS params file).
YAML::Node params(const YAML::Node& root, const char* node_name) {
  return root[node_name]["ros__parameters"];
}

}  // namespace

hexa::pipeline::PipelineConfig load_pipeline_config(rclcpp::Node& node) {
  const std::string share =
      ament_index_cpp::get_package_share_directory("hexa_description");
  const std::string geometry_path = share + "/config/geometry.yaml";
  const std::string tuning_path = share + "/config/tuning.yaml";

  // Start from the baked defaults; overlay the runtime YAML. On any parse error
  // fall back entirely to baked so a bad edit degrades to the last-built config
  // rather than crashing the controller.
  hexa::pipeline::PipelineConfig cfg = hexa::pipeline::PipelineConfig::baked();

  try {
    const YAML::Node geo = YAML::LoadFile(geometry_path);
    const YAML::Node tun = YAML::LoadFile(tuning_path);
    const YAML::Node g = params(tun, "gait_node");
    const YAML::Node c = params(tun, "control_node");
    const YAML::Node p = params(tun, "posture_node");

    // ── geometry.yaml (double loaders → float), keys per gen_config.py ──
    const auto specs = kin::load_leg_specs(geometry_path);
    kin::StandingPoseDeg spose;
    spose.coxa_deg = g["standing_pose"]["coxa_deg"].as<double>();
    spose.femur_above_horizontal_deg =
        g["standing_pose"]["femur_above_horizontal_deg"].as<double>();
    spose.tibia_interior_deg =
        g["standing_pose"]["tibia_interior_deg"].as<double>();
    const auto standing = kin::load_standing_pose(spose, geometry_path);
    const auto initial = kin::load_initial_pose(geometry_path);

    for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
      const std::string name(hexa::gait::LEG_NAMES[i]);
      const auto& s = specs.at(name);
      cfg.leg_specs[i].mount_xyz = hexa::Vec3(static_cast<float>(s.mount_xyz[0]),
                                              static_cast<float>(s.mount_xyz[1]),
                                              static_cast<float>(s.mount_xyz[2]));
      cfg.leg_specs[i].mount_yaw = static_cast<float>(s.mount_yaw);
      cfg.leg_specs[i].coxa_len = static_cast<float>(s.coxa_len);
      cfg.leg_specs[i].femur_len = static_cast<float>(s.femur_len);
      cfg.leg_specs[i].tibia_len = static_cast<float>(s.tibia_len);
      const auto& ip = initial.at(name);
      cfg.initial_pose[i] = {static_cast<float>(ip[0]),
                             static_cast<float>(ip[1]),
                             static_cast<float>(ip[2])};
    }
    cfg.standing_pose = {static_cast<float>(standing[0]),
                         static_cast<float>(standing[1]),
                         static_cast<float>(standing[2])};
    cfg.coxa_to_bottom = geo["body"]["coxa_to_bottom"].as<float>();

    // ── tuning.yaml gait_node → gait::EngineConfig ──
    auto& e = cfg.engine;
    e.stride_length = f(g["stride_length"]);
    e.min_swing_time = f(g["min_swing_time"]);
    e.max_swing_time = f(g["max_swing_time"]);
    e.step_height = f(g["step_height"]);
    e.swing_width = f(g["swing_width"]);
    e.controller_dt = f(g["controller_dt"]);
    e.cmd_zero_tol = f(g["cmd_zero_tol"]);
    e.pause_debounce_delay = f(g["pause_debounce_delay"]);
    e.pause_to_reseat_delay = f(g["pause_to_reseat_delay"]);
    e.gait_change_pause_to_reseat_delay =
        f(g["gait_change_pause_to_reseat_delay"]);
    e.max_reset_time = f(g["max_reset_time"]);
    e.init_pair_swing_time = f(g["initialize"]["pair_swing_time"]);
    e.init_lift_body_time = f(g["initialize"]["lift_body_time"]);
    e.init_swing_clearance = f(g["initialize"]["swing_clearance"]);
    e.init_place_feet_clearance = f(g["initialize"]["place_feet_clearance"]);
    e.reseat_pose_settle_delay = f(g["reseat"]["pose_settle_delay"]);
    e.reseat_height_change_threshold = f(g["reseat"]["height_change_threshold"]);
    e.reseat_pair_swing_time = f(g["reseat"]["pair_swing_time"]);
    e.reseat_pair_dwell_time = f(g["reseat"]["pair_dwell_time"]);
    e.reseat_swing_clearance = f(g["reseat"]["swing_clearance"]);

    // ── velocity caps: reuse the double loader (tuning.yaml derivation) → float ──
    const auto dcaps = hexa_gait::load_velocity_caps(tuning_path);
    cfg.caps.angular_max = static_cast<float>(dcaps.angular_max);
    cfg.caps.linear_max_by_gait.clear();
    cfg.caps.yaw_bias_by_gait.clear();
    for (const auto& [k, v] : dcaps.linear_max_by_gait) {
      cfg.caps.linear_max_by_gait[k] = static_cast<float>(v);
    }
    for (const auto& [k, v] : dcaps.yaw_bias_by_gait) {
      cfg.caps.yaw_bias_by_gait[k] = static_cast<float>(v);
    }
    cfg.default_gait = g["default_gait"].as<std::string>();

    // ── tuning.yaml control_node → config::ControlConfig ──
    cfg.control.vmax_ramp_time_linear = f(c["vmax_ramp_time_linear"]);
    cfg.control.vmax_ramp_time_angular = f(c["vmax_ramp_time_angular"]);
    cfg.control.snap_tol_linear = f(c["snap_tol_linear"]);
    cfg.control.snap_tol_angular = f(c["snap_tol_angular"]);

    // ── tuning.yaml posture_node → config::PostureConfig ──
    auto& ps = cfg.posture;
    ps.gait_sway_gain = f(p["gait_sway_gain"]);
    ps.gait_sway_strength = f(p["gait_sway_strength"]);
    ps.vertical_body_roll_z_amplitude = f(p["vertical_body_roll_z_amplitude"]);
    ps.vertical_body_roll_pitch_amplitude_deg =
        f(p["vertical_body_roll_pitch_amplitude_deg"]);
    ps.vertical_body_roll_phase_offset =
        f(p["vertical_body_roll_phase_offset"]);
    ps.horizontal_body_roll_y_amplitude =
        f(p["horizontal_body_roll_y_amplitude"]);
    ps.horizontal_body_roll_yaw_amplitude_deg =
        f(p["horizontal_body_roll_yaw_amplitude_deg"]);
    ps.horizontal_body_roll_phase_offset =
        f(p["horizontal_body_roll_phase_offset"]);
    ps.body_roll_3d_z_amplitude = f(p["body_roll_3d_z_amplitude"]);
    ps.body_roll_3d_pitch_amplitude_deg =
        f(p["body_roll_3d_pitch_amplitude_deg"]);
    ps.body_roll_3d_y_amplitude = f(p["body_roll_3d_y_amplitude"]);
    ps.body_roll_3d_yaw_amplitude_deg = f(p["body_roll_3d_yaw_amplitude_deg"]);
    ps.body_roll_3d_horizontal_phase_offset =
        f(p["body_roll_3d_horizontal_phase_offset"]);
    ps.body_roll_3d_pitch_phase_offset =
        f(p["body_roll_3d_pitch_phase_offset"]);
    ps.body_roll_3d_yaw_phase_offset = f(p["body_roll_3d_yaw_phase_offset"]);
    ps.gait_bounce_arc_height = f(p["gait_bounce_arc_height"]);
    ps.gait_bounce_step_height_ref = f(p["gait_bounce_step_height_ref"]);
    ps.support_centroid_tau = f(p["support_centroid_tau"]);
    ps.swing_lift_tau = f(p["swing_lift_tau"]);
    ps.gait_activation_slew_rate = f(p["gait_activation_slew_rate"]);
    ps.animation_reserve_x = f(p["animation_reserve_x"]);
    ps.animation_reserve_y = f(p["animation_reserve_y"]);
    ps.animation_reserve_z = f(p["animation_reserve_z"]);
    ps.animation_reserve_roll = f(p["animation_reserve_roll"]);
    ps.animation_reserve_pitch = f(p["animation_reserve_pitch"]);
    ps.animation_reserve_yaw = f(p["animation_reserve_yaw"]);

    RCLCPP_INFO(node.get_logger(),
                "hexa_locomotion: loaded runtime config (gait=%s) from %s + %s",
                cfg.default_gait.c_str(), geometry_path.c_str(),
                tuning_path.c_str());
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node.get_logger(),
                 "hexa_locomotion: runtime config load failed (%s); using baked "
                 "defaults",
                 ex.what());
    return hexa::pipeline::PipelineConfig::baked();
  }
  return cfg;
}

}  // namespace hexa::locomotion
