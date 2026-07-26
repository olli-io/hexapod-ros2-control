// Parity test: the runtime yaml-cpp loader reproduces the baked config.
//
// hexa_locomotion loads its PipelineConfig from geometry.yaml / tuning.yaml at
// startup (load_pipeline_config_from_yaml). PipelineConfig::baked() reconstructs
// the same config from config_generated.hpp, which tools/gen_config.py bakes from
// those same YAMLs at build time. This test drives the runtime loader over the
// exact source YAMLs the codegen used (GEOMETRY_YAML / TUNING_YAML, injected by
// CMake) and asserts every field matches baked() within a float tolerance —
// proving the hand-ported symmetry/pose/caps math never drifts from the codegen.
#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "pipeline_config.hpp"
#include "pipeline_config_loader.hpp"

namespace {

// Direct-copy scalars round the same double to the same float, so they match
// exactly; derived caps (float vs double arithmetic in the codegen) can differ
// by a couple of ULPs. One modest absolute tolerance covers both.
constexpr float kTol = 1e-5f;

using hexa::locomotion::load_pipeline_config_from_yaml;
using hexa::pipeline::PipelineConfig;

void expect_vec3_near(const hexa::Vec3& a, const hexa::Vec3& b,
                      const std::string& what) {
  EXPECT_NEAR(a.x, b.x, kTol) << what << ".x";
  EXPECT_NEAR(a.y, b.y, kTol) << what << ".y";
  EXPECT_NEAR(a.z, b.z, kTol) << what << ".z";
}

TEST(ConfigLoaderParity, RuntimeLoaderMatchesBaked) {
  const PipelineConfig baked = PipelineConfig::baked();
  const PipelineConfig loaded =
      load_pipeline_config_from_yaml(GEOMETRY_YAML, TUNING_YAML);

  // ── geometry: leg specs, coxa_to_bottom, poses ──
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const std::string leg = "leg[" + std::to_string(i) + "]";
    expect_vec3_near(loaded.leg_specs[i].mount_xyz, baked.leg_specs[i].mount_xyz,
                     leg + ".mount_xyz");
    EXPECT_NEAR(loaded.leg_specs[i].mount_yaw, baked.leg_specs[i].mount_yaw, kTol)
        << leg << ".mount_yaw";
    EXPECT_NEAR(loaded.leg_specs[i].coxa_len, baked.leg_specs[i].coxa_len, kTol)
        << leg << ".coxa_len";
    EXPECT_NEAR(loaded.leg_specs[i].femur_len, baked.leg_specs[i].femur_len, kTol)
        << leg << ".femur_len";
    EXPECT_NEAR(loaded.leg_specs[i].tibia_len, baked.leg_specs[i].tibia_len, kTol)
        << leg << ".tibia_len";
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(loaded.initial_pose[i][j], baked.initial_pose[i][j], kTol)
          << leg << ".initial_pose[" << j << "]";
    }
  }
  EXPECT_NEAR(loaded.coxa_to_bottom, baked.coxa_to_bottom, kTol);
  EXPECT_NEAR(loaded.standing_pose.tip_radius, baked.standing_pose.tip_radius,
              kTol);
  EXPECT_NEAR(loaded.standing_pose.body_height, baked.standing_pose.body_height,
              kTol);
  EXPECT_NEAR(loaded.standing_pose.corner_leg_coxa,
              baked.standing_pose.corner_leg_coxa, kTol);

  // ── gait engine ──
  const auto& le = loaded.engine;
  const auto& be = baked.engine;
  EXPECT_NEAR(le.stride_length, be.stride_length, kTol);
  EXPECT_NEAR(le.min_swing_time, be.min_swing_time, kTol);
  EXPECT_NEAR(le.max_swing_time, be.max_swing_time, kTol);
  EXPECT_NEAR(le.step_height, be.step_height, kTol);
  EXPECT_NEAR(le.swing_width, be.swing_width, kTol);
  EXPECT_NEAR(le.swing_apex_fraction, be.swing_apex_fraction, kTol);
  EXPECT_NEAR(le.touchdown_velocity, be.touchdown_velocity, kTol);
  EXPECT_NEAR(le.swing_phase_margin, be.swing_phase_margin, kTol);
  EXPECT_NEAR(le.controller_dt, be.controller_dt, kTol);
  EXPECT_NEAR(le.cmd_zero_tol, be.cmd_zero_tol, kTol);
  EXPECT_NEAR(le.pause_debounce_delay, be.pause_debounce_delay, kTol);
  EXPECT_NEAR(le.pause_to_reseat_delay, be.pause_to_reseat_delay, kTol);
  EXPECT_NEAR(le.gait_change_pause_to_reseat_delay,
              be.gait_change_pause_to_reseat_delay, kTol);
  EXPECT_NEAR(le.max_reset_time, be.max_reset_time, kTol);
  EXPECT_NEAR(le.init_pair_swing_time, be.init_pair_swing_time, kTol);
  EXPECT_NEAR(le.init_lift_body_time, be.init_lift_body_time, kTol);
  EXPECT_NEAR(le.init_swing_clearance, be.init_swing_clearance, kTol);
  EXPECT_NEAR(le.init_place_feet_clearance, be.init_place_feet_clearance, kTol);
  EXPECT_NEAR(le.reseat_pose_settle_delay, be.reseat_pose_settle_delay, kTol);
  EXPECT_NEAR(le.reseat_height_change_threshold,
              be.reseat_height_change_threshold, kTol);
  EXPECT_NEAR(le.reseat_pair_swing_time, be.reseat_pair_swing_time, kTol);
  EXPECT_NEAR(le.reseat_pair_dwell_time, be.reseat_pair_dwell_time, kTol);
  EXPECT_NEAR(le.reseat_swing_clearance, be.reseat_swing_clearance, kTol);

  // ── velocity caps (per-gait, keyed by the registry names) ──
  // angular_max is derived, not read from YAML: the loader solves the standing
  // stance from geometry.yaml + tuning.yaml and divides each gait's linear cap
  // by the outermost foot's radius, so this leg of the parity check is what
  // catches a drift in that derivation.
  ASSERT_EQ(loaded.caps.linear_max_by_gait.size(),
            baked.caps.linear_max_by_gait.size());
  ASSERT_EQ(loaded.caps.angular_max_by_gait.size(),
            baked.caps.angular_max_by_gait.size());
  for (const auto& [name, v] : baked.caps.linear_max_by_gait) {
    ASSERT_TRUE(loaded.caps.linear_max_by_gait.count(name)) << name;
    EXPECT_NEAR(loaded.caps.linear_max_by_gait.at(name), v, kTol)
        << "linear_max[" << name << "]";
    ASSERT_TRUE(loaded.caps.angular_max_by_gait.count(name)) << name;
    EXPECT_NEAR(loaded.caps.angular_max_by_gait.at(name),
                baked.caps.angular_max_by_gait.at(name), kTol)
        << "angular_max[" << name << "]";
    EXPECT_NEAR(loaded.caps.yaw_bias_by_gait.at(name),
                baked.caps.yaw_bias_by_gait.at(name), kTol)
        << "yaw_bias[" << name << "]";
  }
  EXPECT_EQ(loaded.default_gait, baked.default_gait);

  // ── control velocity shaping ──
  EXPECT_NEAR(loaded.control.vmax_ramp_time_linear,
              baked.control.vmax_ramp_time_linear, kTol);
  EXPECT_NEAR(loaded.control.vmax_ramp_time_angular,
              baked.control.vmax_ramp_time_angular, kTol);
  EXPECT_NEAR(loaded.control.snap_tol_linear, baked.control.snap_tol_linear,
              kTol);
  EXPECT_NEAR(loaded.control.snap_tol_angular, baked.control.snap_tol_angular,
              kTol);

  // ── posture animation stack ──
  const auto& lp = loaded.posture;
  const auto& bp = baked.posture;
  EXPECT_NEAR(lp.gait_sway_gain, bp.gait_sway_gain, kTol);
  EXPECT_NEAR(lp.gait_sway_strength, bp.gait_sway_strength, kTol);
  EXPECT_NEAR(lp.vertical_body_roll_z_amplitude,
              bp.vertical_body_roll_z_amplitude, kTol);
  EXPECT_NEAR(lp.vertical_body_roll_pitch_amplitude_deg,
              bp.vertical_body_roll_pitch_amplitude_deg, kTol);
  EXPECT_NEAR(lp.vertical_body_roll_phase_offset,
              bp.vertical_body_roll_phase_offset, kTol);
  EXPECT_NEAR(lp.horizontal_body_roll_y_amplitude,
              bp.horizontal_body_roll_y_amplitude, kTol);
  EXPECT_NEAR(lp.horizontal_body_roll_yaw_amplitude_deg,
              bp.horizontal_body_roll_yaw_amplitude_deg, kTol);
  EXPECT_NEAR(lp.horizontal_body_roll_phase_offset,
              bp.horizontal_body_roll_phase_offset, kTol);
  EXPECT_NEAR(lp.body_roll_3d_z_amplitude, bp.body_roll_3d_z_amplitude, kTol);
  EXPECT_NEAR(lp.body_roll_3d_pitch_amplitude_deg,
              bp.body_roll_3d_pitch_amplitude_deg, kTol);
  EXPECT_NEAR(lp.body_roll_3d_y_amplitude, bp.body_roll_3d_y_amplitude, kTol);
  EXPECT_NEAR(lp.body_roll_3d_yaw_amplitude_deg,
              bp.body_roll_3d_yaw_amplitude_deg, kTol);
  EXPECT_NEAR(lp.body_roll_3d_horizontal_phase_offset,
              bp.body_roll_3d_horizontal_phase_offset, kTol);
  EXPECT_NEAR(lp.body_roll_3d_pitch_phase_offset,
              bp.body_roll_3d_pitch_phase_offset, kTol);
  EXPECT_NEAR(lp.body_roll_3d_yaw_phase_offset,
              bp.body_roll_3d_yaw_phase_offset, kTol);
  EXPECT_NEAR(lp.gait_bounce_arc_height, bp.gait_bounce_arc_height, kTol);
  EXPECT_NEAR(lp.gait_bounce_step_height_ref, bp.gait_bounce_step_height_ref,
              kTol);
  EXPECT_NEAR(lp.support_centroid_tau, bp.support_centroid_tau, kTol);
  EXPECT_NEAR(lp.swing_lift_tau, bp.swing_lift_tau, kTol);
  EXPECT_NEAR(lp.gait_activation_slew_rate, bp.gait_activation_slew_rate, kTol);
  EXPECT_NEAR(lp.animation_reserve_x, bp.animation_reserve_x, kTol);
  EXPECT_NEAR(lp.animation_reserve_y, bp.animation_reserve_y, kTol);
  EXPECT_NEAR(lp.animation_reserve_z, bp.animation_reserve_z, kTol);
  EXPECT_NEAR(lp.animation_reserve_roll, bp.animation_reserve_roll, kTol);
  EXPECT_NEAR(lp.animation_reserve_pitch, bp.animation_reserve_pitch, kTol);
  EXPECT_NEAR(lp.animation_reserve_yaw, bp.animation_reserve_yaw, kTol);
  EXPECT_NEAR(lp.pose_limit_x, bp.pose_limit_x, kTol);
  EXPECT_NEAR(lp.pose_limit_y, bp.pose_limit_y, kTol);
  EXPECT_NEAR(lp.pose_limit_roll, bp.pose_limit_roll, kTol);
  EXPECT_NEAR(lp.pose_limit_pitch, bp.pose_limit_pitch, kTol);
  EXPECT_NEAR(lp.pose_limit_yaw, bp.pose_limit_yaw, kTol);
  EXPECT_NEAR(lp.body_height_max, bp.body_height_max, kTol);
  EXPECT_NEAR(lp.body_height_min, bp.body_height_min, kTol);
  // The loader carries the standing-pose height it already read rather than
  // re-sourcing it; codegen reads the same key. Both must equal the stance.
  EXPECT_NEAR(lp.nominal_body_height, bp.nominal_body_height, kTol);
  EXPECT_NEAR(lp.nominal_body_height, loaded.standing_pose.body_height, kTol);
  EXPECT_EQ(lp.gait_body_animations_enabled, bp.gait_body_animations_enabled);
}

}  // namespace
