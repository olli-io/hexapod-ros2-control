// Integration test for the whole control brain: teleop mapping -> velocity
// shaping -> gait engine -> posture -> compose/IK, plus the supervisor. It
// stands up from FOLDED on the init button, walks on a stick command, and
// safe-stops on a lost link. Per-stage numeric fidelity belongs to the golden
// suites; this one pins the composition.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "bt_teleop.hpp"
#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "kinematics/body_transform.hpp"
#include "kinematics/leg_ik.hpp"
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

// A strafe reversal through the whole chain — map_joy, Control::shape's envelope
// cut and slew, the engine, IK — at the real caps and ramp rate, with the gait
// never dropping out and no foot unreachable. Deliberately weaker than the
// engine-level suite in test_gait_unit: this runs at the 20 ms pipeline tick,
// where a few-tick excursion window is easy to step over.
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

// The same reversal asked for before the walk has started — while the engine is
// still on the engagement ladder out of the stand. Control::shape slews the planar
// command as a vector, so the turn goes straight through the origin and sits inside
// cmd_zero_tol for several 20 ms ticks; read as a released stick that used to abort
// the engagement into the reseat ladder, stand the robot back up and make the
// operator engage all over again. It must now walk through the turn.
TEST(Pipeline, StrafeReversalMidEngagementKeepsWalking) {
  // Ticks after the stick goes over, spanning the engagement: the opening, where
  // the command has not reached the knee, and the middle, where it has.
  for (const int flip_after : {2, 10, 25, 60}) {
    pl::Pipeline p;
    std::uint64_t now_us = 0;
    stand_up(p, now_us);

    Pad left;
    left.axes[bt_teleop::kRightStickX] = bt_teleop::kAxisMax;
    Pad right;
    right.axes[bt_teleop::kRightStickX] = bt_teleop::kAxisMin;

    bool engaging = false;
    for (int i = 0; i < 200 && !engaging; ++i) {
      engaging = run(p, left, 1, now_us).engine_state == EngineState::ENGAGING;
    }
    ASSERT_TRUE(engaging) << "never reached the engagement";
    // TickResult reports the state the tick began in, so this is where the stick
    // actually goes over.
    EngineState at_flip = EngineState::ENGAGING;
    for (int i = 0; i < flip_after; ++i) {
      at_flip = run(p, left, 1, now_us).engine_state;
    }
    ASSERT_EQ(at_flip, EngineState::ENGAGING)
        << "flip_after " << flip_after << " overshot the engagement";

    bool reached_gait = false;
    int max_unreachable = 0;
    for (int i = 0; i < 600; ++i) {
      const pl::TickResult r = run(p, right, 1, now_us);
      max_unreachable = std::max(max_unreachable, r.unreachable);
      reached_gait = reached_gait || r.engine_state == EngineState::GAIT;
      ASSERT_TRUE(r.engine_state == EngineState::ENGAGING ||
                  r.engine_state == EngineState::GAIT)
          << "flip_after " << flip_after << ": the turn dropped the walk into "
          << hexa::gait::state_name(r.engine_state) << " at tick " << i;
    }
    EXPECT_TRUE(reached_gait)
        << "flip_after " << flip_after << ": never got to the walk";
    EXPECT_EQ(max_unreachable, 0)
        << "flip_after " << flip_after
        << ": a foot target went unreachable while reversing";
  }
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

// ── Quadruped mode ──
//
// The mode rides /cmd_gait, so these drive the core tick with a CommandIntent
// rather than the pad — the same seam hexa_locomotion uses.

namespace {

// One tick of the core (non-joy) seam.
pl::TickResult tick_cmd(pl::Pipeline& p, const pl::CommandIntent& cmd,
                        std::uint64_t& now_us) {
  pl::TickInput in;
  in.now_us = now_us;
  in.bt_connected = true;
  in.last_input_us = now_us;
  in.dt = pl::kDt;
  now_us += pl::kTickPeriodUs;
  return p.tick(cmd, in);
}

pl::CommandIntent drive(float vx, float vy, float wz) {
  pl::CommandIntent cmd;
  cmd.linear_x = vx;
  cmd.linear_y = vy;
  cmd.angular_z = wz;
  return cmd;
}

// Foot positions in the (posed) body frame, recomputed from the joint angles the
// pipeline actually commanded — so this reads what the servos were told, not
// what the engine intended.
std::array<hexa::Vec3, hexa::kNumLegs> feet_from_theta(
    const pl::TickResult& r) {
  std::array<hexa::Vec3, hexa::kNumLegs> out{};
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const hexa::JointAngles a = {r.theta[i * 3 + 0], r.theta[i * 3 + 1],
                                 r.theta[i * 3 + 2]};
    out[i] = hexa::leg_to_body(
        hexa::forward_kinematics(a, hexa::config::kLegSpecs[i]),
        hexa::config::kLegSpecs[i]);
  }
  return out;
}

// Shortest distance from the body origin to the edge of the polygon the loaded
// feet enclose; negative when the origin is outside it. Grounded means "within
// kContactBand of the lowest foot", so a parked leg a quarter-metre up and a
// swinging corner both drop out on the same test.
constexpr float kContactBand = 0.005f;

float support_margin(const std::array<hexa::Vec3, hexa::kNumLegs>& feet) {
  float lowest = feet[0].z;
  for (const auto& f : feet) {
    lowest = std::min(lowest, f.z);
  }
  // Traced round the chassis so the hull comes out in order: l_front, r_front,
  // r_rear, l_rear (leg_index.hpp order is l_front, l_middle, l_rear, r_front,
  // r_middle, r_rear).
  static constexpr std::array<std::size_t, 6> kRing = {0, 3, 4, 5, 2, 1};
  std::vector<std::pair<float, float>> hull;
  for (const std::size_t i : kRing) {
    if (feet[i].z <= lowest + kContactBand) {
      hull.push_back({feet[i].x, feet[i].y});
    }
  }
  if (hull.size() < 3) {
    return -1.0f;
  }
  float margin = 1.0f;
  float sign = 0.0f;
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];
    const float ex = b.first - a.first;
    const float ey = b.second - a.second;
    const float len = std::hypot(ex, ey);
    if (len <= 0.0f) {
      continue;
    }
    const float d = (ex * (0.0f - a.second) - ey * (0.0f - a.first)) / len;
    if (sign == 0.0f) {
      sign = d < 0.0f ? -1.0f : 1.0f;
    }
    margin = std::min(margin, sign * d);
  }
  return margin;
}

// How many of the four corners are loaded, on the same contact test. The middle
// pair is a quarter-metre up and never counts.
int grounded_corners(const std::array<hexa::Vec3, hexa::kNumLegs>& feet) {
  float lowest = feet[0].z;
  for (const auto& f : feet) {
    lowest = std::min(lowest, f.z);
  }
  int n = 0;
  for (const std::size_t i : {std::size_t{0}, std::size_t{2}, std::size_t{3},
                              std::size_t{5}}) {
    if (feet[i].z <= lowest + kContactBand) {
      ++n;
    }
  }
  return n;
}

// Stand up on four legs from the belly and run the ladder out. Both commands
// ride one tick, the way the ROS node publishes them: the gait is what carries
// the leg set, and the init that follows it is what climbs the ladder for it.
// Fails the calling test if the engine never reaches a parked stand.
void enter_quadruped(pl::Pipeline& p, std::uint64_t& now_us,
                     std::string_view gait = "quad_walk") {
  ASSERT_EQ(p.engine().state(), EngineState::FOLDED);
  pl::CommandIntent select;
  select.has_gait_select = true;
  select.gait_select = gait;
  select.init_request = true;
  select.init_quadruped = true;
  const pl::TickResult first = tick_cmd(p, select, now_us);
  ASSERT_TRUE(first.gait_accepted);
  ASSERT_EQ(first.init_action, pl::InitAction::kInitialized);

  for (int i = 0; i < 4000; ++i) {
    const pl::TickResult r = tick_cmd(p, drive(0.0f, 0.0f, 0.0f), now_us);
    if (r.engine_state == EngineState::STAND &&
        p.engine().leg_set() == hexa::gait::LegSet::QUADRUPED) {
      return;
    }
  }
  FAIL() << "never reached a parked stand";
}

}  // namespace

// The headline: with the middle pair parked, four feet down and one lifting at
// a time, the body has to be carried into the next triangle before the foot
// leaves it. Recomputed from the commanded joint angles every tick, on eight
// headings, for several cycles each.
TEST(Quadruped, CreepKeepsTheBodyInsideTheSupportTriangle) {
  const auto cfg = hexa::gait::engine_config_from_config();
  const float swing_end =
      hexa::gait::swing_end_phase(3.0f / 4.0f,
                                  cfg.quadruped_swing_phase_margin);
  const float speed = cfg.stride_length * swing_end /
                      (cfg.min_swing_time * (1.0f - swing_end));

  // Every footfall order in the rotation: they hand the body over differently,
  // so the margin is not one gait's property.
  for (const auto& gait : hexa::config::kQuadrupedGaitCycle) {
    for (int h = 0; h < 8; ++h) {
      const float theta = 2.0f * 3.14159265f * static_cast<float>(h) / 8.0f;
      pl::Pipeline p;
      std::uint64_t now_us = 0;
      ASSERT_NO_FATAL_FAILURE(enter_quadruped(p, now_us, gait));

      float worst = 1.0f;
      int walked = 0;
      for (int i = 0; i < 3000; ++i) {
        const pl::TickResult r = tick_cmd(
            p, drive(speed * std::cos(theta), speed * std::sin(theta), 0.0f),
            now_us);
        if (r.engine_state != EngineState::GAIT) {
          continue;
        }
        ++walked;
        worst = std::min(worst, support_margin(feet_from_theta(r)));
      }
      ASSERT_GT(walked, 500) << gait << " heading " << h << " never got walking";
      EXPECT_GT(worst, 0.008f)
          << gait << " heading " << h << " left only " << worst * 1000.0f
          << " mm of static margin";
    }
  }
}

// compose_gait holds last-good angles on an unreachable foot and says nothing,
// so this is the only tripwire on the extra reach the support shift spends.
TEST(Quadruped, CreepNeverAsksForAnUnreachableFoot) {
  const auto cfg = hexa::gait::engine_config_from_config();
  const float swing_end =
      hexa::gait::swing_end_phase(3.0f / 4.0f,
                                  cfg.quadruped_swing_phase_margin);
  const float speed = cfg.stride_length * swing_end /
                      (cfg.min_swing_time * (1.0f - swing_end));

  for (const auto& gait : hexa::config::kQuadrupedGaitCycle) {
    for (int h = 0; h < 8; ++h) {
      const float theta = 2.0f * 3.14159265f * static_cast<float>(h) / 8.0f;
      pl::Pipeline p;
      std::uint64_t now_us = 0;
      ASSERT_NO_FATAL_FAILURE(enter_quadruped(p, now_us, gait));
      for (int i = 0; i < 2000; ++i) {
        const pl::TickResult r = tick_cmd(
            p, drive(speed * std::cos(theta), speed * std::sin(theta), 0.0f),
            now_us);
        ASSERT_EQ(r.unreachable, 0) << gait << " heading " << h << " tick " << i;
      }
    }
  }
}

// Everything above only ever reads a tick the engine spent in GAIT. The ladders
// on either side of the walk carry the same body over the same four feet, and
// nothing here may lift a corner before the body has left the diagonal it stands
// on. The three below are those ladders.

// The engagement ladder's first lift-off comes off a standing start, where the
// body sits at the centre of the four-foot rectangle — exactly on the diagonal
// of the triangle the other three make. It has to be carried off it first.
TEST(Quadruped, EngagementKeepsTheBodyInsideTheSupportTriangle) {
  const auto cfg = hexa::gait::engine_config_from_config();
  const float swing_end = hexa::gait::swing_end_phase(
      3.0f / 4.0f, cfg.quadruped_swing_phase_margin);
  const float speed = cfg.stride_length * swing_end /
                      (cfg.min_swing_time * (1.0f - swing_end));

  for (int h = 0; h < 8; ++h) {
    const float theta = 2.0f * 3.14159265f * static_cast<float>(h) / 8.0f;
    pl::Pipeline p;
    std::uint64_t now_us = 0;
    ASSERT_NO_FATAL_FAILURE(enter_quadruped(p, now_us));

    float worst = 1.0f;
    int engaged = 0;
    for (int i = 0; i < 4000; ++i) {
      const pl::TickResult r = tick_cmd(
          p, drive(speed * std::cos(theta), speed * std::sin(theta), 0.0f),
          now_us);
      if (r.engine_state != EngineState::ENGAGING) {
        if (engaged > 0) {
          break;  // handed over to GAIT, which the tests above cover
        }
        continue;
      }
      ++engaged;
      worst = std::min(worst, support_margin(feet_from_theta(r)));
    }
    ASSERT_GT(engaged, 100) << "heading " << h << " never engaged";
    EXPECT_GT(worst, 0.008f)
        << "heading " << h << " left only " << worst * 1000.0f
        << " mm of static margin during the engagement";
  }
}

// The turn is the other thing that can arrive mid-engagement, and on four feet
// the reseat detour it used to take costs six seconds — four rungs of one corner,
// each waiting on the body. It must walk through instead, and the body must stay
// inside its support triangle while it does: the ladder's hold slows the walk to
// the knee under a support shift that is still steering off the same phases.
TEST(Quadruped, ReversingMidEngagementDoesNotReseat) {
  const auto cfg = hexa::gait::engine_config_from_config();
  const float swing_end = hexa::gait::swing_end_phase(
      3.0f / 4.0f, cfg.quadruped_swing_phase_margin);
  const float speed = cfg.stride_length * swing_end /
                      (cfg.min_swing_time * (1.0f - swing_end));

  pl::Pipeline p;
  std::uint64_t now_us = 0;
  ASSERT_NO_FATAL_FAILURE(enter_quadruped(p, now_us));

  // Until a corner is actually off the ground, so the turn lands on an engagement
  // with something committed to it rather than inside the shift hold.
  bool lifted = false;
  for (int i = 0; i < 4000 && !lifted; ++i) {
    const pl::TickResult r = tick_cmd(p, drive(speed, 0.0f, 0.0f), now_us);
    lifted = r.engine_state == EngineState::ENGAGING &&
             grounded_corners(feet_from_theta(r)) < 4;
  }
  ASSERT_TRUE(lifted) << "the engagement never lifted a corner";

  int worst_airborne = 0;
  float worst_margin = 1.0f;
  bool reached_gait = false;
  for (int i = 0; i < 6000; ++i) {
    const pl::TickResult r = tick_cmd(p, drive(-speed, 0.0f, 0.0f), now_us);
    ASSERT_TRUE(r.engine_state == EngineState::ENGAGING ||
                r.engine_state == EngineState::GAIT)
        << "the turn dropped the walk into "
        << hexa::gait::state_name(r.engine_state) << " at tick " << i;
    reached_gait = reached_gait || r.engine_state == EngineState::GAIT;
    const auto feet = feet_from_theta(r);
    worst_airborne = std::max(worst_airborne, 4 - grounded_corners(feet));
    worst_margin = std::min(worst_margin, support_margin(feet));
  }
  EXPECT_TRUE(reached_gait) << "never got to the walk";
  EXPECT_LE(worst_airborne, 1)
      << "the turn had " << worst_airborne << " corners off the ground";
  EXPECT_GT(worst_margin, 0.008f)
      << "the turn left only " << worst_margin * 1000.0f
      << " mm of static margin";
}

// The reseat ladder re-plants in mirrored pairs. On four feet a mirrored pair is
// half the robot's support, so it has to go one corner at a time.
TEST(Quadruped, ReseatLiftsOneCornerAtATime) {
  const auto cfg = hexa::gait::engine_config_from_config();
  const float swing_end = hexa::gait::swing_end_phase(
      3.0f / 4.0f, cfg.quadruped_swing_phase_margin);
  const float speed = cfg.stride_length * swing_end /
                      (cfg.min_swing_time * (1.0f - swing_end));

  pl::Pipeline p;
  std::uint64_t now_us = 0;
  ASSERT_NO_FATAL_FAILURE(enter_quadruped(p, now_us));

  // Withdraw the command mid-engagement: the one route that always ends on the
  // reseat ladder, since the engagement cannot re-plant its own feet at a zero.
  // Driven until a corner is actually off the ground rather than for a fixed
  // count — the engagement holds all four planted for quadruped_shift_time
  // first, and a withdrawal inside that hold has nothing to re-plant.
  bool lifted = false;
  for (int i = 0; i < 4000 && !lifted; ++i) {
    const pl::TickResult r = tick_cmd(p, drive(speed, 0.0f, 0.0f), now_us);
    lifted = r.engine_state == EngineState::ENGAGING &&
             grounded_corners(feet_from_theta(r)) < 4;
  }
  ASSERT_TRUE(lifted) << "the engagement never lifted a corner";
  ASSERT_EQ(p.engine().state(), EngineState::ENGAGING);

  int reseated = 0;
  int worst_airborne = 0;
  float worst_margin = 1.0f;
  for (int i = 0; i < 6000; ++i) {
    const pl::TickResult r = tick_cmd(p, drive(0.0f, 0.0f, 0.0f), now_us);
    if (r.engine_state != EngineState::RESEATING) {
      if (reseated > 0) {
        break;
      }
      continue;
    }
    ++reseated;
    const auto feet = feet_from_theta(r);
    worst_airborne = std::max(worst_airborne, 4 - grounded_corners(feet));
    worst_margin = std::min(worst_margin, support_margin(feet));
  }
  ASSERT_GT(reseated, 100) << "never reached the reseat ladder";
  EXPECT_LE(worst_airborne, 1)
      << "the ladder had " << worst_airborne << " corners off the ground";
  EXPECT_GT(worst_margin, 0.008f)
      << "the ladder left only " << worst_margin * 1000.0f
      << " mm of static margin";
}

// The settle walks the feet home on the gait's own clock rather than handing
// over to the ladder, so the support shift has to survive a zero command: the
// legs still lift one at a time all the way to the stand.
TEST(Quadruped, SettleKeepsTheBodyInsideTheSupportTriangle) {
  const auto cfg = hexa::gait::engine_config_from_config();
  const float swing_end = hexa::gait::swing_end_phase(
      3.0f / 4.0f, cfg.quadruped_swing_phase_margin);
  const float speed = cfg.stride_length * swing_end /
                      (cfg.min_swing_time * (1.0f - swing_end));

  pl::Pipeline p;
  std::uint64_t now_us = 0;
  ASSERT_NO_FATAL_FAILURE(enter_quadruped(p, now_us));

  bool walked = false;
  for (int i = 0; i < 4000 && !walked; ++i) {
    const pl::TickResult r = tick_cmd(p, drive(speed, 0.0f, 0.0f), now_us);
    walked = r.engine_state == EngineState::GAIT;
  }
  ASSERT_TRUE(walked) << "never got walking";
  for (int i = 0; i < 400; ++i) {
    tick_cmd(p, drive(speed, 0.0f, 0.0f), now_us);
  }

  float worst = 1.0f;
  int stopping = 0;
  for (int i = 0; i < 6000; ++i) {
    const pl::TickResult r = tick_cmd(p, drive(0.0f, 0.0f, 0.0f), now_us);
    if (r.engine_state == EngineState::STAND) {
      break;
    }
    ++stopping;
    worst = std::min(worst, support_margin(feet_from_theta(r)));
  }
  ASSERT_GT(stopping, 100) << "stopped without running a settle";
  EXPECT_GT(worst, 0.008f)
      << "the settle left only " << worst * 1000.0f << " mm of static margin";
}

// A parked leg is held at fixed joint angles, bypassing both the body pose and
// IK: a 50 mm support shift must not drag a tucked leg with it.
TEST(Quadruped, ParkedJointAnglesAreConstantUnderABodyPose) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  ASSERT_NO_FATAL_FAILURE(enter_quadruped(p, now_us));

  const pl::TickResult base = tick_cmd(p, drive(0.0f, 0.0f, 0.0f), now_us);
  // l_middle is leg 1, r_middle is leg 4.
  const std::array<std::size_t, 2> middles = {1, 4};
  std::array<float, 6> before{};
  for (std::size_t k = 0; k < middles.size(); ++k) {
    for (std::size_t j = 0; j < 3; ++j) {
      before[k * 3 + j] = base.theta[middles[k] * 3 + j];
    }
  }

  pl::CommandIntent posed = drive(0.0f, 0.0f, 0.0f);
  posed.pose_x = 0.06f;
  posed.pose_y = -0.06f;
  pl::TickResult r;
  for (int i = 0; i < 600; ++i) {
    r = tick_cmd(p, posed, now_us);
  }
  for (std::size_t k = 0; k < middles.size(); ++k) {
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_NEAR(r.theta[middles[k] * 3 + j], before[k * 3 + j], 1e-6f)
          << "middle leg " << middles[k] << " joint " << j << " moved";
    }
  }
}

// The gamepad path end to end: select from the belly stands the robot up on
// four legs, and off the belly it is the same fold start asks for. Everything
// between is the ladder the /cmd_gait tests drive, so this only has to prove
// the buttons reach it — and that the leg set survives the round trip.
TEST(Quadruped, SelectStandsUpOnFourLegsAndFoldsBack) {
  pl::Pipeline p;
  std::uint64_t now_us = 0;
  Pad neutral;
  run(p, neutral, 3, now_us);

  Pad select;
  select.buttons = 1u << bt_teleop::kSelect;
  const pl::TickResult pressed = run(p, select, 1, now_us);
  EXPECT_TRUE(pressed.has_gait_select);
  EXPECT_EQ(pressed.gait_select, hexa::config::kDefaultQuadrupedGait);
  EXPECT_TRUE(pressed.init_request);
  EXPECT_EQ(pressed.init_action, pl::InitAction::kInitialized);
  run(p, neutral, 1, now_us);

  bool parked = false;
  for (int i = 0; i < 4000 && !parked; ++i) {
    run(p, neutral, 1, now_us);
    parked = p.engine().state() == EngineState::STAND &&
             p.engine().leg_set() == hexa::gait::LegSet::QUADRUPED;
  }
  ASSERT_TRUE(parked) << "select never stood the robot up on four legs";

  // Off the belly the same button means what start means: fold. That is the
  // only way out, and it is what makes the next press's leg set free again.
  const pl::TickResult folding = run(p, select, 1, now_us);
  EXPECT_EQ(folding.init_action, pl::InitAction::kFoldRequested);
  run(p, neutral, 1, now_us);
  bool folded = false;
  for (int i = 0; i < 6000 && !folded; ++i) {
    run(p, neutral, 1, now_us);
    folded = p.engine().state() == EngineState::FOLDED;
  }
  ASSERT_TRUE(folded) << "select never folded the robot";
  EXPECT_EQ(p.engine().leg_set(), hexa::gait::LegSet::HEXAPOD);

  // And start from there stands all six back up.
  Pad start;
  start.buttons = 1u << bt_teleop::kStart;
  const pl::TickResult back = run(p, start, 1, now_us);
  EXPECT_TRUE(back.has_gait_select);
  EXPECT_NE(back.gait_select, "quad_walk");
  EXPECT_NE(back.gait_select, "quad_canter");
  EXPECT_EQ(back.init_action, pl::InitAction::kInitialized);
  run(p, neutral, 1, now_us);

  bool stood = false;
  for (int i = 0; i < 4000 && !stood; ++i) {
    run(p, neutral, 1, now_us);
    stood = p.engine().state() == EngineState::STAND;
  }
  EXPECT_TRUE(stood) << "start never brought all six legs back";
  EXPECT_EQ(p.engine().leg_set(), hexa::gait::LegSet::HEXAPOD);
}

}  // namespace
