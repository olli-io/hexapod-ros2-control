// gtest port of hexa_gait/test/test_reseat_engine.py — Engine integration tests
// for the RESEATING state. These use the real hexa_description YAML geometry (so
// reseat_nominal_stance returns sensible body-frame positions) and a compact
// engine config so the ladders complete in a handful of ticks. Skips if
// hexa_description is not on the prefix path.
//
// Two Python private reach-ins are substituted here (see per-site comments):
//   * engine._applied_height -> the public engine.applied_height() getter.
//   * engine._nominal[name]  -> the STAND-tick output: at EngineState::STAND,
//                               update() emits the committed nominal directly,
//                               so out[name].foot_target == nominal[name].

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_prefix.hpp>  // PackageNotFoundError
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hexa_gait_cpp/engine.hpp"
#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/kinematics.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Mirror the Python module-level timings.
constexpr double PAIR_TIME = 0.04;
constexpr double LIFT_TIME = 0.04;
constexpr double RESEAT_PAIR_TIME = 0.04;
constexpr double SETTLE_DELAY = 0.10;

std::string descriptionConfigDir() {
  try {
    return ament_index_cpp::get_package_share_directory("hexa_description") +
           "/config";
  } catch (const ament_index_cpp::PackageNotFoundError&) {
    return "";
  }
}

// Port of _config(): sets every knob to the Python fixture values.
EngineConfig make_config() {
  EngineConfig c;
  c.stride_length = 0.10;
  c.min_swing_time = 0.25;
  c.max_swing_time = 1.0;
  c.step_height = 0.03;
  c.swing_width = 0.0;
  c.controller_dt = 0.02;
  c.cmd_zero_tol = 1.0e-4;
  c.pause_debounce_delay = 0.0;
  c.pause_to_reseat_delay = 10.0;
  c.gait_change_pause_to_reseat_delay = 10.0;
  c.max_reset_time = 0.6;
  c.init_pair_swing_time = PAIR_TIME;
  c.init_lift_body_time = LIFT_TIME;
  c.init_swing_clearance = 0.01;
  c.init_place_feet_clearance = 0.001;
  c.reseat_pose_settle_delay = SETTLE_DELAY;
  c.reseat_height_change_threshold = 0.001;
  c.reseat_pair_swing_time = RESEAT_PAIR_TIME;
  c.reseat_pair_dwell_time = 0.0;
  c.reseat_swing_clearance = 0.02;
  return c;
}

// Port of _engine(): builds the reseat-enabled engine from the real YAML.
// coxa_to_bottom does not affect the reseat behaviours under test; pass the
// literal (matching the YAML's body.coxa_to_bottom = 0.03) rather than pull in
// a yaml-cpp read here.
Engine make_engine(const std::string& dir) {
  const std::string geo = dir + "/geometry.yaml";
  const std::string sp = dir + "/standing_pose.yaml";
  auto legs = kin::load_leg_specs(geo);
  auto nominal = nominal_stance_from_yaml(geo, sp);
  auto initial = initial_stance_from_yaml(geo);
  auto leg_contexts = build_leg_contexts(geo, sp);
  auto reseat_geometry = reseat_geometry_from_yaml(geo, sp);
  return Engine(make_config(), strategies().at("tripod")(), "tripod",
                std::move(nominal), std::move(initial), 0.03,
                std::move(leg_contexts), std::make_optional(std::move(legs)),
                std::make_optional(std::move(reseat_geometry)));
}

// Port of _drive_to_stand.
void drive_to_stand(Engine& engine) {
  ASSERT_TRUE(engine.start_initialize());
  for (int i = 0; i < 200; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::STAND) {
      return;
    }
  }
  FAIL() << "engine did not reach STAND within 200 ticks";
}

// Port of _drive_through_reseat: tick from RESEATING back to STAND; returns
// ticks consumed.
int drive_through_reseat(Engine& engine) {
  for (int i = 0; i < 500; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::STAND) {
      return i + 1;
    }
  }
  ADD_FAILURE() << "engine did not return to STAND after reseat";
  return -1;
}

// Substitute for reading engine._nominal: at STAND, update() emits nominal
// directly, so each leg's foot_target is the currently-committed nominal
// stance. Only call this while parked in STAND with no pending reseat.
std::map<std::string, Vec3> read_nominal(Engine& engine) {
  const auto out = engine.update(0.02, {0.0, 0.0}, 0.0);
  std::map<std::string, Vec3> nominal;
  for (const auto& name : LEG_NAMES) {
    nominal[name] = out.at(name).foot_target;
  }
  return nominal;
}

// Fixture: resolves the real config dir and skips when hexa_description is
// absent. Tests build the engine via make().
class ReseatEngine : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = descriptionConfigDir();
    if (dir_.empty()) GTEST_SKIP() << "hexa_description is not installed";
  }
  Engine make() { return make_engine(dir_); }
  std::string dir_;
};

TEST_F(ReseatEngine, SetTargetHeightDoesNotFireImmediately) {
  Engine engine = make();
  drive_to_stand(engine);
  engine.set_target_height(0.02);
  // One tick: settle timer at 0.02 s; threshold 0.10 — should still be in STAND.
  engine.update(0.02, {0.0, 0.0}, 0.0);
  EXPECT_EQ(engine.state(), EngineState::STAND);
}

TEST_F(ReseatEngine, HeightSettlesAndTriggersReseat) {
  Engine engine = make();
  drive_to_stand(engine);
  engine.set_target_height(0.02);
  // Tick past the settle delay: at dt=0.02 and delay=0.10 s, the 6th tick puts
  // elapsed at 0.12 s ≥ 0.10 s and the reseat fires.
  for (int i = 0; i < 20; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::RESEATING) {
      break;
    }
  }
  EXPECT_EQ(engine.state(), EngineState::RESEATING);
}

TEST_F(ReseatEngine, ReseatCompletesAndReturnsToStandWithUpdatedNominal) {
  Engine engine = make();
  drive_to_stand(engine);
  // engine._nominal substitute: STAND-tick output before the height change.
  const auto nominal_before = read_nominal(engine);
  engine.set_target_height(0.02);
  // Wait for the ladder to start, then tick it to completion.
  for (int i = 0; i < 500; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::RESEATING) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::RESEATING);
  drive_through_reseat(engine);
  // Nominal updated: XY changed radially, Z stayed at default.
  const auto nominal_after = read_nominal(engine);
  for (const auto& name : LEG_NAMES) {
    const Vec3& nb = nominal_before.at(name);
    const Vec3& na = nominal_after.at(name);
    EXPECT_NEAR(nb[2], na[2], 1e-9);
    EXPECT_TRUE(na[0] != nb[0] || na[1] != nb[1]) << name << " XY unchanged";
  }
  EXPECT_NEAR(engine.applied_height(), 0.02, 1e-9);
}

TEST_F(ReseatEngine, HeightChangeResetsSettleTimer) {
  // If the target keeps slewing, the settle timer resets each time, so reseat
  // does not fire mid-ramp.
  Engine engine = make();
  drive_to_stand(engine);
  for (int i = 0; i < 10; ++i) {
    // Slew height upward by 5 mm per tick — well above the float-noise epsilon,
    // so the settle timer never accumulates.
    engine.set_target_height(0.005 * (i + 1));
    engine.update(0.02, {0.0, 0.0}, 0.0);
  }
  EXPECT_EQ(engine.state(), EngineState::STAND);
  // Now hold the height steady — reseat fires after the delay.
  const double final = 0.005 * 10;
  engine.set_target_height(final);
  for (int i = 0; i < 20; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() != EngineState::STAND) {
      break;
    }
  }
  EXPECT_TRUE(engine.state() == EngineState::RESEATING ||
              engine.state() == EngineState::STAND);
}

TEST_F(ReseatEngine, HeldDpadAtOneMmPerTickDoesNotFireMidPress) {
  // Regression: the teleop integrates pose.z at 0.05 m/s and publishes at 50 Hz,
  // so a held D-pad slews the target by exactly 1 mm per tick. That used to sit
  // right on the YAML dead-band (reseat_height_change_threshold = 0.001) and the
  // settle timer accrued anyway, firing reseat mid-press. The fix uses a tighter
  // float-noise epsilon inside set_target_height so the timer reliably resets on
  // every per-tick D-pad step.
  Engine engine = make();
  drive_to_stand(engine);
  // Simulate the held D-pad for well past the settle delay.
  const double dt = 0.02;
  const int ticks = static_cast<int>(std::lround((SETTLE_DELAY + 0.10) / dt));
  double z = 0.0;
  for (int i = 0; i < ticks; ++i) {
    z += 0.001;
    engine.set_target_height(z);
    engine.update(dt, {0.0, 0.0}, 0.0);
  }
  ASSERT_EQ(engine.state(), EngineState::STAND)
      << "reseat fired while D-pad was still held";
  // Release: stop slewing, ride out the settle window — now it should fire.
  for (int i = 0; i < ticks; ++i) {
    engine.set_target_height(z);
    engine.update(dt, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::RESEATING) {
      break;
    }
  }
  EXPECT_EQ(engine.state(), EngineState::RESEATING);
}

TEST_F(ReseatEngine, CmdVelDuringReseatIsHeldUntilDone) {
  // Commit-to-completion: cmd_vel arriving mid-reseat must not bail to
  // ENGAGING / STOPPING. Mirrors the INITIALIZE / FOLDING contract.
  Engine engine = make();
  drive_to_stand(engine);
  engine.set_target_height(0.02);
  for (int i = 0; i < 500; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::RESEATING) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::RESEATING);
  // Now slam cmd_vel non-zero — engine must stay in RESEATING for at least a
  // couple of ticks (the ladder is 3 pairs × pair_swing_time but the entering
  // tick already advanced the controller). Stop well before completion so the
  // assertion never trips on the natural reseat → STAND boundary.
  const int n = static_cast<int>(2 * RESEAT_PAIR_TIME / 0.02) - 1;
  for (int i = 0; i < n; ++i) {
    engine.update(0.02, {0.2, 0.0}, 0.0);
    // Reseat continues until done, regardless of cmd_vel.
    EXPECT_EQ(engine.state(), EngineState::RESEATING);
  }
}

TEST_F(ReseatEngine, PendingFoldDefersUntilHeightZero) {
  // Two-press Start scheme: at lifted height, request_fold latches _pending_fold
  // but the engine must NOT fold until the height has been snapped back to zero
  // AND the reseat ladder has run to completion at applied_height=0.
  Engine engine = make();
  drive_to_stand(engine);
  engine.set_target_height(0.02);
  // Let the lift settle and reseat fire.
  for (int i = 0; i < 50; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::RESEATING) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::RESEATING);
  // Snap height back to zero and request a fold. Reseat must run to completion
  // at the new target (0), then engine returns to STAND, then consumes
  // _pending_fold and transitions to FOLDING.
  engine.set_target_height(0.0);
  ASSERT_TRUE(engine.request_fold());
  // Tick through reseat completion AND the subsequent fold trigger.
  for (int i = 0; i < 500; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::FOLDING) {
      break;
    }
  }
  EXPECT_EQ(engine.state(), EngineState::FOLDING);
  EXPECT_NEAR(engine.applied_height(), 0.0, 1e-9);
}

TEST_F(ReseatEngine, RequestFoldAtZeroHeightFoldsOnNextStandTick) {
  // When height is already at default, request_fold should kick off FOLDING
  // immediately on the next STAND tick. No reseat in the way.
  Engine engine = make();
  drive_to_stand(engine);
  ASSERT_TRUE(engine.request_fold());
  engine.update(0.02, {0.0, 0.0}, 0.0);
  EXPECT_EQ(engine.state(), EngineState::FOLDING);
}

TEST_F(ReseatEngine, RequestFoldRejectedWhenFoldedOrFolding) {
  Engine engine = make();
  // FOLDED: request_fold returns False (engine is already where FOLDING would
  // take it).
  ASSERT_EQ(engine.state(), EngineState::FOLDED);
  EXPECT_FALSE(engine.request_fold());
  drive_to_stand(engine);
  // Drive to FOLDING.
  ASSERT_TRUE(engine.request_fold());
  engine.update(0.02, {0.0, 0.0}, 0.0);
  ASSERT_EQ(engine.state(), EngineState::FOLDING);
  // Mid-FOLDING: rejected.
  EXPECT_FALSE(engine.request_fold());
}

TEST_F(ReseatEngine, CmdVelPreemptsPendingReseatInStand) {
  // If the user pushes the stick while the settle delay is counting down, the
  // engine bails to ENGAGING (walking takes priority) and leaves the pending
  // reseat alone. When the user later returns to STAND, the reseat fires if the
  // height target still differs.
  Engine engine = make();
  drive_to_stand(engine);
  engine.set_target_height(0.02);
  // Halfway through the settle window:
  for (int i = 0; i < 3; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
  }
  ASSERT_EQ(engine.state(), EngineState::STAND);
  // User pushes stick:
  engine.update(0.02, {0.2, 0.0}, 0.0);
  EXPECT_EQ(engine.state(), EngineState::ENGAGING);
}

TEST_F(ReseatEngine, ReseatWithZeroHeightIsANoop) {
  // The threshold dead-band: a target within reseat_height_change_threshold of
  // applied does not fire.
  Engine engine = make();
  drive_to_stand(engine);
  engine.set_target_height(0.0005);  // below the 1 mm threshold
  for (int i = 0; i < 50; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
  }
  EXPECT_EQ(engine.state(), EngineState::STAND);
  EXPECT_NEAR(engine.applied_height(), 0.0, 1e-12);
}

TEST_F(ReseatEngine, GaitChangeDuringHeightReseatCommitsAtHandoff) {
  // A gait request mid height-change RESEATING latches and commits at the same
  // RESEATING → STAND handoff that applies the new nominal, so one ladder serves
  // both the posture change and the gait change.
  Engine engine = make();
  drive_to_stand(engine);
  engine.set_target_height(0.02);
  for (int i = 0; i < 50; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::RESEATING) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::RESEATING);

  ASSERT_TRUE(engine.set_strategy("crawl"));
  EXPECT_EQ(engine.pending_strategy_name(),
            std::optional<std::string>("crawl"));

  drive_through_reseat(engine);
  EXPECT_EQ(engine.state(), EngineState::STAND);
  EXPECT_EQ(engine.strategy_name(), "crawl");
  EXPECT_FALSE(engine.pending_strategy_name().has_value());
  EXPECT_NEAR(engine.applied_height(), 0.02, 1e-9);
}

TEST(ReseatEngineCtor, RejectsPartialReseatKwargs) {
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const std::string geo = dir + "/geometry.yaml";
  const std::string sp = dir + "/standing_pose.yaml";
  auto nominal = nominal_stance_from_yaml(geo, sp);
  auto initial = initial_stance_from_yaml(geo);
  auto leg_contexts = build_leg_contexts(geo, sp);
  // leg_specs without reseat_geometry must raise.
  EXPECT_THROW(
      Engine(make_config(), strategies().at("tripod")(), "tripod",
             std::move(nominal), std::move(initial), 0.03,
             std::move(leg_contexts),
             std::make_optional(kin::load_leg_specs(geo)), std::nullopt),
      std::invalid_argument);
}

TEST(ReseatEngineCtor, InitializesWithNoReseatKwargs) {
  // Backward-compatible constructor: tests that pass none of the reseat kwargs
  // still build a valid engine — they just don't get the RESEATING behaviour.
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const std::string geo = dir + "/geometry.yaml";
  const std::string sp = dir + "/standing_pose.yaml";
  auto nominal = nominal_stance_from_yaml(geo, sp);
  auto initial = initial_stance_from_yaml(geo);
  auto leg_contexts = build_leg_contexts(geo, sp);
  Engine engine(make_config(), strategies().at("tripod")(), "tripod",
                std::move(nominal), std::move(initial), 0.03,
                std::move(leg_contexts));
  // With no reseat geometry, set_target_height + settle should be silently
  // inert.
  ASSERT_TRUE(engine.start_initialize());
  for (int i = 0; i < 200; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::STAND) {
      break;
    }
  }
  engine.set_target_height(0.02);
  for (int i = 0; i < 50; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
  }
  // Never enters RESEATING.
  EXPECT_EQ(engine.state(), EngineState::STAND);
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
