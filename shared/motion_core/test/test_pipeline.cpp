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

// Like run(), but asserts a latched hardware over-current fault on every tick
// (the STATUS-register signal hexa_hardware feeds via /hardware/fault). Link is
// held up so only the fault path is exercised.
pl::TickResult run_faulted(pl::Pipeline& p, const Pad& pad, int n,
                           std::uint64_t& now_us) {
  pl::TickResult res;
  for (int i = 0; i < n; ++i) {
    pl::TickInput in;
    in.now_us = now_us;
    in.axes = pad.axes.data();
    in.buttons = pad.buttons;
    in.bt_connected = true;
    in.last_input_us = now_us;
    in.hardware_fault = true;
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

// A cold pipeline holds FOLDED — an idle controller never moves the legs — but
// the rail arms on the live link so the robot comes up energized in the folded
// pose. (The caller staggers that energize leg by leg; see hexa::EnergizeSweep.)
TEST(Pipeline, HoldsFoldedUntilInit) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  Pad neutral;
  const pl::TickResult r = run(p, neutral, 20, now_us);
  EXPECT_EQ(r.engine_state, EngineState::FOLDED);
  EXPECT_TRUE(r.relay_energized);
  EXPECT_FALSE(r.walking);
  EXPECT_EQ(r.unreachable, 0);
}

// With no link there is no pilot, so the rail stays open however long the
// pipeline idles in FOLDED.
TEST(Pipeline, DoesNotEnergizeWithoutALink) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  Pad neutral;
  const pl::TickResult r = run(p, neutral, 20, now_us, /*connected=*/false);
  EXPECT_EQ(r.engine_state, EngineState::FOLDED);
  EXPECT_FALSE(r.relay_energized);
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

// Flicking the strafe stick from one stop to the other turns the command under
// six planted feet. Composition check for that path: the whole chain — map_joy,
// Control::shape's envelope cut and slew, the engine, IK — has to keep running
// through a reversal at the real caps and ramp rate, with the gait never
// dropping out and no foot going unreachable.
//
// Deliberately weaker than the engine-level reversal suite in test_gait_unit,
// which samples at the 5 ms engine tick; this one runs at the 20 ms pipeline
// tick, where a few-tick excursion window is easy to step over. Trust that suite
// for the numbers and this one for the seam.
TEST(Pipeline, StrafeReversalKeepsEveryFootReachable) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  stand_up(p, now_us);

  // right_stick_x is drive_y in gait mode (teleop_joy.yaml).
  Pad left;
  left.axes[bt_teleop::kRightStickX] = bt_teleop::kAxisMax;
  Pad right;
  right.axes[bt_teleop::kRightStickX] = bt_teleop::kAxisMin;

  // A lateral engagement takes longer than a forward one; 600 ticks (12 s) is
  // generous slack on the ~5 s it needs.
  bool reached_gait = false;
  int max_unreachable = 0;
  for (int i = 0; i < 600; ++i) {
    const pl::TickResult r = run(p, left, 1, now_us);
    reached_gait = reached_gait || (r.engine_state == EngineState::GAIT);
  }
  ASSERT_TRUE(reached_gait) << "engine never engaged into GAIT on a strafe";

  for (int i = 0; i < 400; ++i) {
    const pl::TickResult r = run(p, right, 1, now_us);
    max_unreachable = std::max(max_unreachable, r.unreachable);
    EXPECT_EQ(r.engine_state, EngineState::GAIT)
        << "the gait dropped out mid-reversal at tick " << i;
  }
  EXPECT_EQ(max_unreachable, 0)
      << "a foot target went unreachable while reversing";
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

// The runtime PipelineConfig seam is actually consumed, not ignored: baked()
// reproduces the default construction, and an override threads through to the
// built engine. This is the off-target guard for hexa_locomotion's YAML path
// (the ROS node fills a PipelineConfig from geometry.yaml / tuning.yaml).
TEST(PipelineConfig, BakedMatchesDefaultAndOverrideThreads) {
  pl::PipelineConfig cfg = pl::PipelineConfig::baked();
  EXPECT_EQ(cfg.default_gait, "tripod") << "baked default gait";

  pl::Pipeline baked_default;  // no-arg ctor delegates to baked()
  EXPECT_EQ(baked_default.engine().strategy_name(), "tripod");

  // A non-default gait in the config must build the engine on that strategy.
  cfg.default_gait = "ripple";
  pl::Pipeline overridden(cfg);
  EXPECT_EQ(overridden.engine().strategy_name(), "ripple")
      << "PipelineConfig.default_gait must thread through to the engine";
}

// A hardware over-current fault mid-walk latches the engine into FAULT and
// disarms the servo-rail relay (servos limp for manual repositioning). The
// supervisor also raises its aggregate fault flag.
TEST(Pipeline, HardwareFaultLatchesAndDisarmsRelay) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  stand_up(p, now_us);

  Pad fwd;
  fwd.axes[bt_teleop::kRightStickY] = bt_teleop::kAxisMax;
  run(p, fwd, 50, now_us);  // walking, relay armed

  const pl::TickResult r = run_faulted(p, fwd, 5, now_us);
  EXPECT_EQ(r.engine_state, EngineState::FAULT);
  EXPECT_FALSE(r.relay_energized) << "rail must drop on a hardware fault";
  EXPECT_TRUE(r.decision.fault) << "supervisor must flag the fault";
}

// FAULT is latched: once the fault input clears, the engine stays in FAULT
// (holding the folded pose) until the operator presses Start, which recovers via
// the same INITIALIZE ladder and re-arms the relay at STAND.
TEST(Pipeline, FaultLatchesUntilInitRecovers) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  stand_up(p, now_us);
  run_faulted(p, Pad{}, 5, now_us);
  ASSERT_EQ(p.engine().state(), EngineState::FAULT);

  // Fault input cleared, but the engine stays latched in FAULT.
  const pl::TickResult held = run(p, Pad{}, 10, now_us);
  EXPECT_EQ(held.engine_state, EngineState::FAULT);
  EXPECT_FALSE(held.relay_energized);

  // Start recovers exactly like a cold boot: FAULT -> INITIALIZE -> STAND.
  const pl::TickResult init = press_init(p, now_us);
  EXPECT_EQ(init.init_action, pl::InitAction::kInitialized);
  Pad neutral;
  bool stood = false;
  for (int i = 0; i < 2000 && !stood; ++i) {
    stood = run(p, neutral, 1, now_us).engine_state == EngineState::STAND;
  }
  ASSERT_TRUE(stood) << "engine never re-stood after fault recovery";
  const pl::TickResult r = run(p, neutral, 5, now_us);
  EXPECT_TRUE(r.relay_energized) << "relay should re-arm once re-stood";
}

// A still-asserted fault wins over a same-window Start: you cannot re-initialize
// onto a board that is still tripped.
TEST(Pipeline, FaultWinsOverConcurrentInit) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  stand_up(p, now_us);
  run_faulted(p, Pad{}, 3, now_us);
  ASSERT_EQ(p.engine().state(), EngineState::FAULT);

  // Press Start while the fault is STILL asserted (fault held true this tick).
  Pad start;
  start.buttons = 1u << bt_teleop::kStart;
  run(p, Pad{}, 3, now_us);  // settle for a clean edge, fault cleared
  run_faulted(p, Pad{}, 1, now_us);        // re-assert to simulate persistence
  pl::TickInput in;
  in.now_us = now_us;
  in.axes = start.axes.data();
  in.buttons = start.buttons;
  in.bt_connected = true;
  in.last_input_us = now_us;
  in.hardware_fault = true;
  const pl::TickResult r = p.tick(in);
  EXPECT_EQ(r.engine_state, EngineState::FAULT)
      << "a still-tripped board must not re-initialize";
}

}  // namespace
