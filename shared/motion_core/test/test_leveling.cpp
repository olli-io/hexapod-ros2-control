// Body stabilization: the Leveler loop on its own against a rigid slope, then
// in the pipeline, where the gate follows the switch, the preset, the IMU
// freshness and a waiting preset change.

#include "posture/leveling.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "config_generated.hpp"
#include "pipeline.hpp"

namespace {

namespace pl = hexa::pipeline;
using hexa::gait::EngineState;
using hexa::posture::Leveler;
using hexa::posture::LevelingGate;
using hexa::posture::TiltSample;

constexpr float kDt = pl::kDt;
constexpr float kSlopeRoll = 0.12f;
constexpr float kSlopePitch = -0.08f;

hexa::config::StabilizationConfig cfg() {
  hexa::config::StabilizationConfig c = hexa::config::kStabilization;
  c.ki_stand = 2.0f;
  c.ki_gait = 0.5f;
  c.max_correction = 0.2f;
  c.measurement_tau = 0.1f;
  c.decay_tau = 0.2f;
  return c;
}

// A rigid body on a slope: the IMU reads slope + commanded tilt.
TiltSample plant(float cmd_roll, float cmd_pitch) {
  return {kSlopeRoll + cmd_roll, kSlopePitch + cmd_pitch};
}

void run_loop(Leveler& lv, LevelingGate gate, float target_roll,
              float target_pitch, int ticks, bool walking = false) {
  for (int i = 0; i < ticks; ++i) {
    const TiltSample s =
        plant(target_roll + lv.roll(), target_pitch + lv.pitch());
    lv.step(gate, walking, target_roll, target_pitch, s, kDt);
  }
}

TEST(Leveler, CancelsTheSlope) {
  Leveler lv(cfg());
  run_loop(lv, LevelingGate::kIntegrate, 0.0f, 0.0f, 5 * 200);
  EXPECT_NEAR(lv.roll(), -kSlopeRoll, 1e-3f);
  EXPECT_NEAR(lv.pitch(), -kSlopePitch, 1e-3f);
}

TEST(Leveler, OperatorTiltReadsRelativeToGravity) {
  Leveler lv(cfg());
  run_loop(lv, LevelingGate::kIntegrate, 0.05f, 0.0f, 5 * 200);
  const TiltSample s = plant(0.05f + lv.roll(), lv.pitch());
  EXPECT_NEAR(s.roll, 0.05f, 1e-3f);
  EXPECT_NEAR(s.pitch, 0.0f, 1e-3f);
}

TEST(Leveler, LevelGroundNeedsNoCorrection) {
  Leveler lv(cfg());
  for (int i = 0; i < 400; ++i) {
    lv.step(LevelingGate::kIntegrate, false, 0.03f, 0.0f,
            TiltSample{0.03f + lv.roll(), lv.pitch()}, kDt);
  }
  EXPECT_NEAR(lv.roll(), 0.0f, 1e-5f);
  EXPECT_NEAR(lv.pitch(), 0.0f, 1e-5f);
}

TEST(Leveler, WalkingConvergesSlower) {
  Leveler stand(cfg());
  Leveler walk(cfg());
  run_loop(stand, LevelingGate::kIntegrate, 0.0f, 0.0f, 100, false);
  run_loop(walk, LevelingGate::kIntegrate, 0.0f, 0.0f, 100, true);
  EXPECT_GT(std::fabs(stand.roll()), std::fabs(walk.roll()));
}

TEST(Leveler, CorrectionStaysOnItsDisc) {
  hexa::config::StabilizationConfig c = cfg();
  c.max_correction = 0.05f;
  Leveler lv(c);
  run_loop(lv, LevelingGate::kIntegrate, 0.0f, 0.0f, 5 * 200);
  EXPECT_NEAR(std::hypot(lv.roll(), lv.pitch()), 0.05f, 1e-5f);
}

TEST(Leveler, ClampedShareIsNotWoundUp) {
  Leveler lv(cfg());
  run_loop(lv, LevelingGate::kIntegrate, 0.0f, 0.0f, 5 * 200);
  lv.limit_to(-0.02f, 0.0f);
  EXPECT_FLOAT_EQ(lv.roll(), -0.02f);
  EXPECT_FLOAT_EQ(lv.pitch(), 0.0f);
  // Never grows it.
  lv.limit_to(-0.5f, 0.5f);
  EXPECT_FLOAT_EQ(lv.roll(), -0.02f);
}

TEST(Leveler, HoldKeepsAndDecayClears) {
  Leveler lv(cfg());
  run_loop(lv, LevelingGate::kIntegrate, 0.0f, 0.0f, 5 * 200);
  const float held = lv.roll();
  lv.step(LevelingGate::kHold, false, 0.0f, 0.0f, TiltSample{0.3f, 0.3f}, kDt);
  EXPECT_FLOAT_EQ(lv.roll(), held);
  for (int i = 0; i < 3 * 200; ++i) {
    lv.step(LevelingGate::kDecay, false, 0.0f, 0.0f, std::nullopt, kDt);
  }
  EXPECT_EQ(lv.roll(), 0.0f);
  EXPECT_EQ(lv.pitch(), 0.0f);
}

TEST(Leveler, NoSampleNoIntegration) {
  Leveler lv(cfg());
  lv.step(LevelingGate::kIntegrate, false, 0.1f, 0.1f, std::nullopt, kDt);
  EXPECT_EQ(lv.roll(), 0.0f);
}

TEST(LevelingGate, FollowsSwitchPresetFreshnessAndState) {
  using hexa::posture::leveling_gate;
  EXPECT_EQ(leveling_gate(true, true, true, false, EngineState::STAND),
            LevelingGate::kIntegrate);
  EXPECT_EQ(leveling_gate(true, true, true, false, EngineState::GAIT),
            LevelingGate::kIntegrate);
  EXPECT_EQ(leveling_gate(true, true, true, false, EngineState::GESTURE),
            LevelingGate::kHold);
  EXPECT_EQ(leveling_gate(true, true, true, false, EngineState::RESEATING),
            LevelingGate::kHold);
  EXPECT_EQ(leveling_gate(false, true, true, false, EngineState::STAND),
            LevelingGate::kDecay);
  EXPECT_EQ(leveling_gate(true, false, true, false, EngineState::STAND),
            LevelingGate::kDecay);
  EXPECT_EQ(leveling_gate(true, true, false, false, EngineState::STAND),
            LevelingGate::kDecay);
  EXPECT_EQ(leveling_gate(true, true, true, true, EngineState::STAND),
            LevelingGate::kDecay);
}

// ── In the pipeline ──

struct Rig {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  hexa::posture::BodyPose last{};

  pl::TickResult tick(const pl::CommandIntent& cmd, bool imu = true) {
    pl::TickInput in;
    in.now_us = now_us;
    in.bt_connected = true;
    in.last_input_us = now_us;
    in.imu_valid = imu;
    in.imu_roll = kSlopeRoll + last.roll;
    in.imu_pitch = kSlopePitch + last.pitch;
    in.imu_stamp_us = now_us;
    in.dt = pl::kDt;
    now_us += pl::kTickPeriodUs;
    const pl::TickResult r = p.tick(cmd, in);
    last = r.body_pose;
    return r;
  }

  void stand() {
    pl::CommandIntent init;
    init.init_request = true;
    tick(init);
    for (int i = 0; i < 4000 && p.engine().state() != EngineState::STAND; ++i) {
      tick(pl::CommandIntent{});
    }
    ASSERT_EQ(p.engine().state(), EngineState::STAND);
  }
};

pl::CommandIntent on() {
  pl::CommandIntent c;
  c.stabilize = true;
  return c;
}

TEST(PipelineLeveling, LevelsOnTheDefaultPreset) {
  Rig rig;
  rig.stand();
  pl::TickResult r;
  for (int i = 0; i < 5 * 200; ++i) {
    r = rig.tick(on());
  }
  EXPECT_TRUE(r.stabilizing);
  EXPECT_NEAR(r.body_pose.roll, -kSlopeRoll, 2e-3f);
  EXPECT_NEAR(r.body_pose.pitch, -kSlopePitch, 2e-3f);
  EXPECT_NEAR(r.level_roll, -kSlopeRoll, 2e-3f);
}

TEST(PipelineLeveling, OffOrStaleLeavesThePoseAlone) {
  Rig rig;
  rig.stand();
  pl::TickResult r;
  for (int i = 0; i < 2 * 200; ++i) {
    r = rig.tick(pl::CommandIntent{});
  }
  EXPECT_FALSE(r.stabilizing);
  EXPECT_EQ(r.body_pose.roll, 0.0f);

  for (int i = 0; i < 2 * 200; ++i) {
    r = rig.tick(on(), /*imu=*/false);
  }
  EXPECT_FALSE(r.stabilizing);
  EXPECT_EQ(r.body_pose.roll, 0.0f);
}

TEST(PipelineLeveling, SwitchingOffEasesTheCorrectionOut) {
  Rig rig;
  rig.stand();
  for (int i = 0; i < 5 * 200; ++i) {
    rig.tick(on());
  }
  pl::TickResult r;
  for (int i = 0; i < 3 * 200; ++i) {
    r = rig.tick(pl::CommandIntent{});
  }
  EXPECT_EQ(r.level_roll, 0.0f);
  EXPECT_EQ(r.level_pitch, 0.0f);
}

TEST(PipelineLeveling, PresetChangeWaitsForTheDecayThenStopsLeveling) {
  Rig rig;
  rig.stand();
  for (int i = 0; i < 5 * 200; ++i) {
    rig.tick(on());
  }
  pl::CommandIntent ask = on();
  ask.has_preset_select = true;
  ask.preset_select = "fast";
  ASSERT_TRUE(rig.tick(ask).preset_accepted);

  pl::CommandIntent stay = on();
  stay.has_preset_select = true;
  stay.preset_select = "fast";
  bool arrived = false;
  bool blocked = false;
  pl::TickResult r;
  for (int i = 0; i < 6000 && !arrived; ++i) {
    r = rig.tick(stay);
    blocked = blocked || r.gait_blocked_by_posture;
    arrived = r.preset == "fast" && r.engine_state == EngineState::STAND;
  }
  EXPECT_FALSE(blocked) << "the correction never decayed inside the timeout";
  ASSERT_TRUE(arrived);
  for (int i = 0; i < 2 * 200; ++i) {
    r = rig.tick(stay);
  }
  EXPECT_FALSE(r.stabilizing);
  EXPECT_EQ(r.level_roll, 0.0f);
}

}  // namespace
