// Integration smoke/behaviour test for the target-agnostic control brain
// (plan part 10, Tier 2/3 seam — hexa::pipeline::Pipeline).
//
// Drives the WHOLE firmware pipeline — teleop mapping -> velocity shaping ->
// gait engine -> posture -> compose/IK, plus the failsafe supervisor — natively
// (x86, float, no Pico SDK, no ROS), exactly as it is compiled for the Pico
// firmware (main.cpp) and the Gazebo bridge (firmware_bridge_node.cpp). This is
// the off-target proof that the extraction composes and the full chain runs: it
// stands up from FOLDED on the init button, walks on a stick command (legs move,
// no unreachable targets), and safe-stops on a lost link. The per-stage numeric
// fidelity is covered by the golden suites (test_gait / test_kinematics /
// test_joy_mapping / test_posture); this suite pins the composition.

#include <array>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "bt_teleop.hpp"
#include "gait/engine.hpp"
#include "pipeline.hpp"

namespace {

namespace pl = hexa::pipeline;
using hexa::gait::EngineState;

// A gamepad snapshot in the raw int16 layout bt_teleop emits (map_joy applies
// the teleop_joy.yaml axis signs itself). Neutral = sticks/dpad centered,
// analog triggers released at +max, no buttons.
struct Pad {
  std::array<std::int16_t, bt_teleop::kNumAxes> axes{};
  std::uint32_t buttons = 0;

  Pad() { neutral(); }
  void neutral() {
    axes.fill(0);
    axes[bt_teleop::kL2] = bt_teleop::kAxisMax;  // trigger rest = +max
    axes[bt_teleop::kR2] = bt_teleop::kAxisMax;
    buttons = 0;
  }
};

// Advance the pipeline `n` ticks with a fixed pad. `connected` drives both the
// link flag and the input freshness (fresh when connected, "no frame" when not).
// now_us advances by the shared 20 ms tick. Returns the final TickResult.
pl::TickResult run(pl::Pipeline& p, const Pad& pad, int n, std::uint64_t& now_us,
                   bool connected = true) {
  pl::TickResult res;
  for (int i = 0; i < n; ++i) {
    pl::TickInput in;
    in.now_us = now_us;
    in.axes = pad.axes.data();
    in.buttons = pad.buttons;
    in.bt_connected = connected;
    in.last_input_us = connected ? now_us : 0;
    in.dt = pl::kDt;
    res = p.tick(in);
    now_us += pl::kTickPeriodUs;
  }
  return res;
}

// Press the init (start) button as a clean rising edge: a settle, one tick with
// the bit set, then release. map_joy fires init_request on the edge, which the
// pipeline routes to Engine::start_initialize().
pl::TickResult press_init(pl::Pipeline& p, std::uint64_t& now_us) {
  Pad neutral;
  run(p, neutral, 3, now_us);
  Pad start;
  start.buttons = 1u << bt_teleop::kStart;
  const pl::TickResult res = run(p, start, 1, now_us);
  run(p, neutral, 1, now_us);
  return res;
}

// Stand up from a cold FOLDED engine: press init and run the neutral INITIALIZE
// ladder until the engine settles at STAND. Fails the calling test if it never
// gets there. The ladder is a few seconds; 2000 ticks (40 s) is generous slack.
void stand_up(pl::Pipeline& p, std::uint64_t& now_us) {
  ASSERT_EQ(p.engine().state(), EngineState::FOLDED);
  const pl::TickResult init = press_init(p, now_us);
  ASSERT_EQ(init.init_action, pl::InitAction::kInitialized);

  Pad neutral;
  bool stood = false;
  for (int i = 0; i < 2000 && !stood; ++i) {
    const pl::TickResult r = run(p, neutral, 1, now_us);
    stood = (r.engine_state == EngineState::STAND);
  }
  ASSERT_TRUE(stood) << "engine never reached STAND from the init ladder";
}

// A cold pipeline holds FOLDED — an unpaired/idle controller never moves the
// legs and never energizes the servo rail.
TEST(Pipeline, HoldsFoldedUntilInit) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  Pad neutral;
  const pl::TickResult r = run(p, neutral, 20, now_us);
  EXPECT_EQ(r.engine_state, EngineState::FOLDED);
  EXPECT_FALSE(r.relay_energized);
  EXPECT_FALSE(r.walking);
  EXPECT_EQ(r.unreachable, 0);
}

// The init button drives FOLDED -> INITIALIZE -> STAND, and once stood on a live
// link the supervisor arms the servo-rail relay.
TEST(Pipeline, InitLadderStandsAndArmsRelay) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  stand_up(p, now_us);

  Pad neutral;
  const pl::TickResult r = run(p, neutral, 5, now_us);
  EXPECT_EQ(r.engine_state, EngineState::STAND);
  EXPECT_TRUE(r.relay_energized) << "relay should arm on link-up + stand";
  EXPECT_EQ(r.unreachable, 0);
}

// A stick command walks the robot: the engine engages into GAIT, the master
// phase advances, the legs actually move (coxa sweeps), and every foot stays
// reachable throughout.
TEST(Pipeline, ForwardCommandWalks) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  stand_up(p, now_us);

  // drive_x is bound to right_stick_y in gait mode (teleop_joy.yaml). Full
  // deflection is well past the 0.1 deadband, so |cmd_vel| > 0 regardless of the
  // axis sign — enough to leave STAND and engage the gait.
  Pad fwd;
  fwd.axes[bt_teleop::kRightStickY] = bt_teleop::kAxisMax;

  bool reached_gait = false;
  bool phase_advanced = false;
  float coxa_min = 1e9f, coxa_max = -1e9f;
  int max_unreachable = 0;
  for (int i = 0; i < 600; ++i) {
    const pl::TickResult r = run(p, fwd, 1, now_us);
    reached_gait = reached_gait || (r.engine_state == EngineState::GAIT);
    phase_advanced = phase_advanced || (r.master_phase > 1e-3f);
    max_unreachable = std::max(max_unreachable, r.unreachable);
    // l_front coxa is theta[0] (leg-major/segment-minor order).
    coxa_min = std::min(coxa_min, r.theta[0]);
    coxa_max = std::max(coxa_max, r.theta[0]);
  }

  EXPECT_TRUE(reached_gait) << "engine never engaged into GAIT under a command";
  EXPECT_TRUE(phase_advanced) << "master phase never advanced";
  EXPECT_GT(coxa_max - coxa_min, 0.02f) << "legs did not move while walking";
  EXPECT_EQ(max_unreachable, 0) << "a foot target went unreachable while walking";
}

// A lost link is a safe-stop: the watchdog fires, the command is force-zeroed,
// and the posture chain sees a non-walking body — the engine settles instead of
// latching the last velocity.
TEST(Pipeline, LostLinkForcesStop) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  stand_up(p, now_us);

  // Walk for a bit on a live link.
  Pad fwd;
  fwd.axes[bt_teleop::kRightStickY] = bt_teleop::kAxisMax;
  run(p, fwd, 100, now_us);

  // Link drops mid-walk: keep commanding forward, but disconnected.
  const pl::TickResult r = run(p, fwd, 5, now_us, /*connected=*/false);
  EXPECT_TRUE(r.decision.input_stale);
  EXPECT_TRUE(r.decision.force_zero);
  EXPECT_FALSE(r.walking) << "a lost link must gate the posture animations off";
}

}  // namespace
