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
      EXPECT_NEAR(loaded.folded_pose[i][j], baked.folded_pose[i][j], kTol)
          << leg << ".folded_pose[" << j << "]";
      EXPECT_NEAR(loaded.initialized_pose[i][j], baked.initialized_pose[i][j],
                  kTol)
          << leg << ".initialized_pose[" << j << "]";
    }
  }
  EXPECT_NEAR(loaded.coxa_to_bottom, baked.coxa_to_bottom, kTol);
  EXPECT_NEAR(loaded.foot_radius, baked.foot_radius, kTol);
  // ── the preset table, position by position ──
  // Order is load-bearing: the baked table is indexed and the YAML list is
  // read in declaration order, so a reordered `presets:` block has to show up
  // here rather than silently renumbering what the firmware boots on.
  ASSERT_EQ(loaded.presets.size(), baked.presets.size());
  EXPECT_EQ(loaded.default_preset, baked.default_preset);
  for (std::size_t pi = 0; pi < baked.presets.size(); ++pi) {
    const auto& lp = loaded.presets[pi];
    const auto& bp = baked.presets[pi];
    ASSERT_EQ(lp.id, bp.id) << "preset " << pi << " is out of order";
    EXPECT_EQ(lp.leg_set, bp.leg_set) << lp.id;
    EXPECT_NEAR(lp.stride_length, bp.stride_length, kTol) << lp.id;
    EXPECT_NEAR(lp.stride_length_radial, bp.stride_length_radial, kTol) << lp.id;
    EXPECT_NEAR(lp.min_swing_time, bp.min_swing_time, kTol) << lp.id;
    EXPECT_NEAR(lp.max_swing_time, bp.max_swing_time, kTol) << lp.id;
    EXPECT_NEAR(lp.step_height, bp.step_height, kTol) << lp.id;
    EXPECT_NEAR(lp.standing.body_height, bp.standing.body_height, kTol) << lp.id;
    // All three groups, including the placeholder a parked-pair preset carries:
    // the loader fills it from the front group and gen_config.py does the same,
    // so a drift in that rule is a drift here.
    for (std::size_t gi = 0; gi < hexa::kNumLegGroups; ++gi) {
      const auto group = hexa::LEG_GROUP_NAMES[gi];
      EXPECT_NEAR(lp.standing.groups[gi].tip_reach,
                  bp.standing.groups[gi].tip_reach, kTol)
          << lp.id << "." << group << ".tip_reach";
      EXPECT_NEAR(lp.standing.groups[gi].coxa, bp.standing.groups[gi].coxa,
                  kTol)
          << lp.id << "." << group << ".coxa";
    }
  }

  // ── gait engine ──
  const auto& le = loaded.engine;
  const auto& be = baked.engine;
  EXPECT_NEAR(le.stride_length, be.stride_length, kTol);
  EXPECT_NEAR(le.stride_length_radial, be.stride_length_radial, kTol);
  EXPECT_NEAR(le.min_swing_time, be.min_swing_time, kTol);
  EXPECT_NEAR(le.max_swing_time, be.max_swing_time, kTol);
  EXPECT_NEAR(le.step_height, be.step_height, kTol);
  EXPECT_NEAR(le.swing_width, be.swing_width, kTol);
  EXPECT_NEAR(le.touchdown_velocity, be.touchdown_velocity, kTol);
  EXPECT_NEAR(le.touchdown_probe_fraction, be.touchdown_probe_fraction, kTol);
  EXPECT_NEAR(le.swing_phase_margin, be.swing_phase_margin, kTol);
  EXPECT_NEAR(le.quadruped_swing_phase_margin, be.quadruped_swing_phase_margin,
              kTol);
  EXPECT_NEAR(le.controller_dt, be.controller_dt, kTol);
  EXPECT_NEAR(le.cmd_zero_tol, be.cmd_zero_tol, kTol);
  EXPECT_NEAR(le.settle_debounce_delay, be.settle_debounce_delay, kTol);
  EXPECT_NEAR(le.settle_swing_time, be.settle_swing_time, kTol);
  EXPECT_NEAR(le.init_unfold_time, be.init_unfold_time, kTol);
  EXPECT_NEAR(le.init_pair_swing_time, be.init_pair_swing_time, kTol);
  EXPECT_NEAR(le.init_lift_body_time, be.init_lift_body_time, kTol);
  EXPECT_NEAR(le.init_place_clearance, be.init_place_clearance, kTol);
  EXPECT_NEAR(le.init_swing_clearance, be.init_swing_clearance, kTol);
  EXPECT_NEAR(le.reseat_pose_settle_delay, be.reseat_pose_settle_delay, kTol);
  EXPECT_NEAR(le.reseat_height_change_threshold,
              be.reseat_height_change_threshold, kTol);
  EXPECT_NEAR(le.reseat_pair_swing_time, be.reseat_pair_swing_time, kTol);
  EXPECT_NEAR(le.reseat_pair_dwell_time, be.reseat_pair_dwell_time, kTol);
  EXPECT_NEAR(le.reseat_swing_clearance, be.reseat_swing_clearance, kTol);
  EXPECT_NEAR(le.reseat_plane_ramp_time, be.reseat_plane_ramp_time, kTol);
  EXPECT_NEAR(le.quadruped_shift_time, be.quadruped_shift_time, kTol);
  EXPECT_NEAR(le.pair_fold_swing_time, be.pair_fold_swing_time, kTol);
  EXPECT_NEAR(le.pair_fold_dwell_time, be.pair_fold_dwell_time, kTol);
  EXPECT_NEAR(le.support_shift_lead, be.support_shift_lead, kTol);

  // ── velocity caps (per-gait, keyed by the registry names) ──
  // angular_max is derived, not read from YAML: the loader solves the standing
  // stance from geometry.yaml + tuning.yaml and divides each gait's linear cap
  // by the outermost foot's radius, so this leg of the parity check is what
  // catches a drift in that derivation.
  // One table per preset: three of the four inputs to a cap ride the preset.
  ASSERT_EQ(loaded.caps_by_preset.size(), baked.caps_by_preset.size());
  for (const auto& [preset, bcaps] : baked.caps_by_preset) {
    ASSERT_TRUE(loaded.caps_by_preset.count(preset)) << preset;
    const auto& lcaps = loaded.caps_by_preset.at(preset);
    ASSERT_EQ(lcaps.linear_max_by_gait.size(), bcaps.linear_max_by_gait.size())
        << preset;
    for (const auto& [name, v] : bcaps.linear_max_by_gait) {
      ASSERT_TRUE(lcaps.linear_max_by_gait.count(name)) << preset << "/" << name;
      EXPECT_NEAR(lcaps.linear_max_by_gait.at(name), v, kTol)
          << "linear_max[" << preset << "/" << name << "]";
      EXPECT_NEAR(lcaps.angular_max_by_gait.at(name),
                  bcaps.angular_max_by_gait.at(name), kTol)
          << "angular_max[" << preset << "/" << name << "]";
      EXPECT_NEAR(lcaps.yaw_bias_by_gait.at(name),
                  bcaps.yaw_bias_by_gait.at(name), kTol)
          << "yaw_bias[" << preset << "/" << name << "]";
    }
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
  EXPECT_NEAR(lp.support_shift_gain, bp.support_shift_gain, kTol);
  EXPECT_NEAR(lp.support_shift_lead, bp.support_shift_lead, kTol);
  EXPECT_NEAR(lp.support_shift_tau, bp.support_shift_tau, kTol);
  EXPECT_NEAR(lp.gait_activation_slew_rate, bp.gait_activation_slew_rate, kTol);
  EXPECT_NEAR(lp.pose_filter_tau, bp.pose_filter_tau, kTol);
  EXPECT_NEAR(lp.pose_filter_damping_ratio, bp.pose_filter_damping_ratio, kTol);
  EXPECT_NEAR(lp.pose_filter_snap_tol_linear, bp.pose_filter_snap_tol_linear,
              kTol);
  EXPECT_NEAR(lp.pose_filter_snap_tol_angular, bp.pose_filter_snap_tol_angular,
              kTol);
  EXPECT_NEAR(lp.pose_limit_x, bp.pose_limit_x, kTol);
  EXPECT_NEAR(lp.pose_limit_y, bp.pose_limit_y, kTol);
  EXPECT_NEAR(lp.pose_limit_roll, bp.pose_limit_roll, kTol);
  EXPECT_NEAR(lp.pose_limit_pitch, bp.pose_limit_pitch, kTol);
  EXPECT_NEAR(lp.pose_limit_yaw, bp.pose_limit_yaw, kTol);
  EXPECT_NEAR(lp.body_height_max, bp.body_height_max, kTol);
  EXPECT_NEAR(lp.body_height_min, bp.body_height_min, kTol);
  // The loader carries the DEFAULT preset's standing height it already read
  // rather than re-sourcing it; codegen reads the same key. Both must equal
  // that preset's stance.
  EXPECT_NEAR(lp.nominal_body_height, bp.nominal_body_height, kTol);
  EXPECT_NEAR(lp.nominal_body_height,
              loaded.presets[loaded.default_preset].standing.body_height, kTol);
  EXPECT_EQ(lp.gait_body_animations_enabled, bp.gait_body_animations_enabled);
}

}  // namespace
