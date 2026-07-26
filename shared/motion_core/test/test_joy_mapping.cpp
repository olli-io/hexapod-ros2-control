// Golden-trace parity test for the float joy_mapping port (plan part 07).
//
// Replays the SAME axes/buttons trace through the firmware's float map_joy
// (src/joy_mapping.cpp, namespace hexa::teleop) that gen_joy_golden.py ran
// through the untouched Python hexa_teleop.joy_mapping reference, and asserts
// the JoyOutput matches frame-for-frame. The trace (kFrames) and the reference
// outputs (kExpected) are baked into joy_golden_generated.hpp at build time from
// the same YAMLs the firmware bakes, so any divergence is purely the
// double->float port of the mapping math — checked under a loose tolerance.
//
// The JoyState / JoyConfig are seeded from config_generated.hpp exactly as
// main.cpp does (initial mode, default-gait index, default-gait stick cap), so
// the port and the reference start from identical state.

#include <string>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "joy_mapping.hpp"

#include "joy_golden_generated.hpp"

namespace {

namespace tel = hexa::teleop;
namespace cfg = hexa::config;

// Float port vs float64 reference, over an eased/integrated trace: a few 1e-6
// steps accumulate, so allow a loose absolute band well under any physical
// scale (velocities ~0.3, poses ~0.03).
constexpr float kTol = 2e-4f;

tel::JoyConfig seed_config() {
  tel::JoyConfig c;
  // Taken from the golden header rather than re-derived, so the port and the
  // reference cannot disagree on the stick full-scale caps themselves — this
  // test is about the mapping math, not the cap derivation (test_gait_unit and
  // test_config_loader cover that).
  c.gait_linear_max = joy_golden::kGaitLinearMax;
  c.gait_angular_z_max = joy_golden::kGaitAngularZMax;
  return c;
}

tel::JoyState seed_state() {
  tel::JoyState s;
  s.mode = tel::mode_from_string(cfg::kInitialMode);
  for (std::size_t i = 0; i < cfg::kGaitCycle.size(); ++i) {
    if (cfg::kGaitCycle[i] == cfg::kDefaultGait) {
      s.current_gait_idx = static_cast<int>(i);
      break;
    }
  }
  return s;
}

TEST(JoyMappingGolden, MatchesPythonReference) {
  const tel::JoyConfig cfg_rt = seed_config();
  tel::JoyState state = seed_state();

  for (int i = 0; i < joy_golden::kNumFrames; ++i) {
    const joy_golden::Frame& f = joy_golden::kFrames[i];
    const joy_golden::Expected& e = joy_golden::kExpected[i];
    const tel::JoyOutput o =
        tel::map_joy(f.axes, f.buttons, cfg_rt, state, 0.02f);

    SCOPED_TRACE("frame " + std::to_string(i));
    EXPECT_NEAR(o.linear_x, e.linear_x, kTol);
    EXPECT_NEAR(o.linear_y, e.linear_y, kTol);
    EXPECT_NEAR(o.angular_z, e.angular_z, kTol);
    EXPECT_NEAR(o.pose_x, e.pose_x, kTol);
    EXPECT_NEAR(o.pose_y, e.pose_y, kTol);
    EXPECT_NEAR(o.pose_z, e.pose_z, kTol);
    EXPECT_NEAR(o.pose_yaw, e.pose_yaw, kTol);
    EXPECT_NEAR(o.pose_roll, e.pose_roll, kTol);
    EXPECT_NEAR(o.pose_pitch, e.pose_pitch, kTol);
    EXPECT_EQ(o.mode_changed, e.mode_changed);
    EXPECT_EQ(o.init_request, e.init_request);
    EXPECT_EQ(o.has_gait_select, e.has_gait_select);
    if (e.has_gait_select) {
      EXPECT_EQ(o.gait_select, e.gait_select);
    }
    EXPECT_EQ(o.has_animation_name, e.has_animation_name);
    if (e.has_animation_name) {
      EXPECT_EQ(o.animation_name, e.animation_name);
    }
  }
}

}  // namespace
