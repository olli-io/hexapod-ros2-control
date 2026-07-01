// gtest port of hexa_gait/test/test_engine.py — exercises the pure C++ Engine
// (no ROS). Behavioural parity with the Python suite; the handful of Python
// assertions that reached into private engine state (_engagement, _clock,
// _strategy, _STANCE_SEAM_EPSILON) are rewritten against the observable public
// API (state(), master_phase(), strategy_name(), update() outputs). See the
// port plan and the per-test comments below for the exact substitutions.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "hexa_gait_cpp/clock.hpp"
#include "hexa_gait_cpp/engine.hpp"
#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Symmetric six-leg layout. Front/rear sit at 0.18 m from body centre (the
// outer legs in pure rotation); middle legs sit at 0.12 m. Chosen so the
// per-leg radii are easy to reason about in the mixed-motion tests below.
const std::map<std::string, Vec3>& mounts() {
  static const std::map<std::string, Vec3> m = {
      {"l_front", Vec3(0.15, 0.10, 0.0)},
      {"r_front", Vec3(0.15, -0.10, 0.0)},
      {"l_middle", Vec3(0.0, 0.12, 0.0)},
      {"r_middle", Vec3(0.0, -0.12, 0.0)},
      {"l_rear", Vec3(-0.15, 0.10, 0.0)},
      {"r_rear", Vec3(-0.15, -0.10, 0.0)},
  };
  return m;
}

// Foot directly under each hip at a fixed walk-plane Z.
std::map<std::string, Vec3> nominal_stance() {
  std::map<std::string, Vec3> out;
  for (const auto& [n, xyz] : mounts()) {
    out[n] = Vec3(xyz[0], xyz[1], -0.10);
  }
  return out;
}

// Folded feet sitting above each hip at body-frame z > 0. Exact value doesn't
// matter for the post-INITIALIZE behaviours these tests target; only matters
// that the engine starts in INITIALIZE and runs the ladder to STAND first.
std::map<std::string, Vec3> initial_stance() {
  std::map<std::string, Vec3> out;
  for (const auto& [n, xyz] : mounts()) {
    out[n] = Vec3(xyz[0], xyz[1], 0.05);
  }
  return out;
}

std::map<std::string, LegContext> leg_contexts() {
  const auto nominal = nominal_stance();
  std::map<std::string, LegContext> out;
  for (const auto& n : LEG_NAMES) {
    LegContext ctx;
    ctx.name = n;
    ctx.mount_xyz = mounts().at(n);
    ctx.mount_yaw = 0.0;
    ctx.nominal_stance = nominal.at(n);
    out[n] = ctx;
  }
  return out;
}

// min_swing_time=0.25, beta=0.5 (tripod) -> min_cycle_time = 0.5 s; same for
// max_swing_time=1.0 -> max_cycle_time = 2.0 s. Other gait factors derive their
// own bounds from swing_time / (1 - beta). The compact INITIALIZE timings keep
// the cold-start ladder short (3*0.04 + 0.04 = 0.16 s) so drive-past-initialize
// finishes in well under 20 ticks at dt=0.02.
EngineConfig make_config(double stride_length = 0.10,
                         double min_swing_time = 0.25,
                         double max_swing_time = 1.0,
                         double pause_debounce_delay = 0.0,
                         double pause_to_reseat_delay = 0.2,
                         double gait_change_pause_to_reseat_delay = 0.1) {
  EngineConfig c;
  c.stride_length = stride_length;
  c.min_swing_time = min_swing_time;
  c.max_swing_time = max_swing_time;
  c.step_height = 0.03;
  c.swing_width = 0.0;
  c.controller_dt = 0.02;
  c.cmd_zero_tol = 1.0e-4;
  c.pause_debounce_delay = pause_debounce_delay;
  c.pause_to_reseat_delay = pause_to_reseat_delay;
  c.gait_change_pause_to_reseat_delay = gait_change_pause_to_reseat_delay;
  c.max_reset_time = 0.6;
  c.init_pair_swing_time = 0.04;
  c.init_lift_body_time = 0.04;
  c.init_swing_clearance = 0.01;
  c.init_place_feet_clearance = 0.001;
  // Reseat knobs are inert: the tests below construct Engine without leg_specs /
  // reseat_geometry, so the internal reseat path stays disabled.
  c.reseat_pose_settle_delay = 0.1;
  c.reseat_height_change_threshold = 0.001;
  c.reseat_pair_swing_time = 0.04;
  c.reseat_pair_dwell_time = 0.0;
  c.reseat_swing_clearance = 0.01;
  return c;
}

// Tripod phase offsets (l_front/r_middle/l_rear lead, the other tripod trails by
// half a cycle) — the anonymous-namespace tripod_offsets() in registry.cpp is
// not exported, so mirror its values here for the spy strategy.
PhaseOffsets tripod_offsets() {
  return PhaseOffsets({
      {"l_front", 0.0},
      {"r_middle", 0.0},
      {"l_rear", 0.0},
      {"r_front", 0.5},
      {"l_middle", 0.5},
      {"r_rear", 0.5},
  });
}

// Records every (leg, phase, StrideParams) call from the engine, and returns
// each leg's nominal stance as the "foot target" so no motion is injected.
// Port of _SpyStrategy. foot_target is const, so calls is mutable.
class SpyStrategy : public Strategy {
 public:
  struct Call {
    std::string leg;
    double phase;
    StrideParams stride;
  };

  SpyStrategy() : offsets_(tripod_offsets()) {}

  const PhaseOffsets& phase_offsets() const override { return offsets_; }
  double duty_factor() const override { return 0.5; }
  bool unstable() const override { return false; }
  Vec3 foot_target(double phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    calls.push_back({leg.name, phase, stride});
    return leg.nominal_stance;
  }

  StrideParams last_stride(const std::string& leg_name) const {
    for (auto it = calls.rbegin(); it != calls.rend(); ++it) {
      if (it->leg == leg_name) {
        return it->stride;
      }
    }
    ADD_FAILURE() << "no recorded stride for " << leg_name;
    return StrideParams{};
  }

  double last_phase(const std::string& leg_name) const {
    for (auto it = calls.rbegin(); it != calls.rend(); ++it) {
      if (it->leg == leg_name) {
        return it->phase;
      }
    }
    ADD_FAILURE() << "no recorded phase for " << leg_name;
    return 0.0;
  }

  void clear() const { calls.clear(); }

  mutable std::vector<Call> calls;

 private:
  PhaseOffsets offsets_;
};

// Build an Engine driven by a fresh spy; the returned raw pointer stays valid
// only until a strategy swap (set_strategy that actually applies a new gait
// destroys the spy). Callers that swap must not touch the spy afterwards.
Engine make_spy_engine(SpyStrategy** spy_out,
                       EngineConfig config = make_config()) {
  auto spy = std::make_unique<SpyStrategy>();
  *spy_out = spy.get();
  return Engine(config, std::move(spy), "spy", nominal_stance(),
                initial_stance(), 0.02, leg_contexts());
}

Engine make_gait_engine(const std::string& name,
                        EngineConfig config = make_config()) {
  return Engine(config, strategies().at(name)(), name, nominal_stance(),
                initial_stance(), 0.02, leg_contexts());
}

// Trigger the cold-start ladder and tick until it reaches STAND. Existing tests
// were written before the FOLDED / INITIALIZE states existed and assume the
// engine begins at STAND; the compact init timings keep this loop short.
void drive_past_initialize(Engine& engine, double dt = 0.02) {
  engine.start_initialize();
  for (int i = 0; i < 200; ++i) {
    if (engine.state() == EngineState::STAND) {
      return;
    }
    engine.update(dt, {0.0, 0.0}, 0.0);
  }
  FAIL() << "engine did not reach STAND within 200 ticks";
}

// Run from FOLDED through INITIALIZE / STAND / ENGAGING into GAIT. Returns the
// tick count. Used by the cycle_time / stride tests that target steady-state
// GAIT and predate engagement (which freezes its snapshot at entry).
int drive_to_gait(Engine& engine, std::pair<double, double> v_body_xy,
                  double omega_z, double dt = 0.02) {
  engine.start_initialize();
  for (int i = 0; i < 200; ++i) {
    engine.update(dt, v_body_xy, omega_z);
    if (engine.state() == EngineState::GAIT) {
      return i + 1;
    }
  }
  ADD_FAILURE() << "engine did not reach GAIT within 200 ticks";
  return -1;
}

void drive_to_state(Engine& engine, std::pair<double, double> v_body_xy,
                    EngineState target, int max_ticks = 500, double dt = 0.02) {
  for (int i = 0; i < max_ticks; ++i) {
    engine.update(dt, v_body_xy, 0.0);
    if (engine.state() == target) {
      return;
    }
  }
  FAIL() << "engine did not reach " << state_name(target) << " within "
         << max_ticks << " ticks (ended in " << state_name(engine.state())
         << ")";
}

std::vector<EngineState> trace_states(Engine& engine,
                                      std::pair<double, double> v_body_xy,
                                      int ticks, double dt = 0.02) {
  std::vector<EngineState> trace;
  for (int i = 0; i < ticks; ++i) {
    engine.update(dt, v_body_xy, 0.0);
    trace.push_back(engine.state());
  }
  return trace;
}

void assert_ordered_subsequence(const std::vector<EngineState>& trace,
                                const std::vector<EngineState>& expected) {
  size_t idx = 0;
  for (const auto& want : expected) {
    bool found = false;
    while (idx < trace.size()) {
      if (trace[idx++] == want) {
        found = true;
        break;
      }
    }
    ASSERT_TRUE(found) << state_name(want) << " missing or out of order";
  }
}

bool trace_contains(const std::vector<EngineState>& trace, EngineState s) {
  return std::find(trace.begin(), trace.end(), s) != trace.end();
}

const std::pair<double, double> kCmd = {0.20, 0.0};

// ── Cycle-time derivation and stride ──────────────────────────────────────

TEST(Engine, BelowSaturationDerivesCycleTimeFromVelocity) {
  // v = 0.20 m/s straight forward, stride_length = 0.10, duty = 0.5
  // -> cycle_time_raw = 0.10 / (0.20 * 0.5) = 1.0 s, inside [min, max].
  // stance_time = 0.5 s. Per-leg stride = 0.20 * 0.5 = 0.10 m.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_to_gait(engine, {0.20, 0.0}, 0.0);
  spy->clear();
  engine.update(0.02, {0.20, 0.0}, 0.0);

  EXPECT_EQ(engine.state(), EngineState::GAIT);
  const StrideParams stride = spy->last_stride("l_front");
  EXPECT_NEAR(stride.cycle_time, 1.0, 1e-9);
  EXPECT_NEAR(stride.stride_vector[0], 0.10, 1e-9);
  EXPECT_NEAR(stride.stride_vector[1], 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(stride.stride_vector[2], 0.0);
}

TEST(Engine, SaturationClampsCycleTimeAndPerLegStride) {
  // v = 0.80 m/s; raw = 0.10 / 0.40 = 0.25 < min_cycle_time so cycle_time
  // clamps to 0.5 s. Raw stride 0.80 * 0.25 = 0.20 m, clamped to 0.10 m.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_to_gait(engine, {0.80, 0.0}, 0.0);
  spy->clear();
  engine.update(0.02, {0.80, 0.0}, 0.0);

  const StrideParams stride = spy->last_stride("l_front");
  EXPECT_NEAR(stride.cycle_time, 0.5, 1e-9);
  const double magnitude =
      std::hypot(stride.stride_vector[0], stride.stride_vector[1]);
  EXPECT_NEAR(magnitude, 0.10, 1e-9);
}

TEST(Engine, SlowCommandClampsCycleTimeToMaxAndStrideShrinksLinearly) {
  // v = 0.02 m/s; raw = 0.10 / 0.01 = 10 s > max_cycle_time so cycle_time
  // clamps to 2.0 s. Per-leg stride = 0.02 * 1.0 = 0.02 m — no further clamp.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_to_gait(engine, {0.02, 0.0}, 0.0);
  spy->clear();
  engine.update(0.02, {0.02, 0.0}, 0.0);

  const StrideParams stride = spy->last_stride("l_front");
  EXPECT_NEAR(stride.cycle_time, 2.0, 1e-9);
  EXPECT_NEAR(stride.stride_vector[0], 0.02, 1e-9);
}

TEST(Engine, PureRotationOuterLegDictatesCycleTime) {
  // Pure omega_z = 1.0 rad/s. Outer legs (front/rear) have radius
  // sqrt(0.15^2 + 0.10^2); middle legs have radius 0.12. cycle_time is set by
  // the outer-leg radius; outer stride saturates at stride_length, middle is
  // proportionally shorter.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_to_gait(engine, {0.0, 0.0}, 1.0);
  spy->clear();
  engine.update(0.02, {0.0, 0.0}, 1.0);

  const double outer_r = std::hypot(0.15, 0.10);
  const double inner_r = 0.12;
  const double expected_cycle = 0.10 / (outer_r * 0.5);

  const StrideParams outer = spy->last_stride("l_front");
  const StrideParams inner = spy->last_stride("l_middle");
  EXPECT_NEAR(outer.cycle_time, expected_cycle, 1e-9);
  EXPECT_NEAR(inner.cycle_time, expected_cycle, 1e-9);  // shared

  const double outer_mag =
      std::hypot(outer.stride_vector[0], outer.stride_vector[1]);
  const double inner_mag =
      std::hypot(inner.stride_vector[0], inner.stride_vector[1]);
  EXPECT_NEAR(outer_mag, 0.10, 1e-9);
  EXPECT_NEAR(inner_mag, 0.10 * (inner_r / outer_r), 1e-9);
}

TEST(Engine, ZeroCommandStaysInStandAndSkipsCycleTimeMath) {
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_past_initialize(engine);
  spy->clear();
  const auto out = engine.update(0.02, {0.0, 0.0}, 0.0);

  EXPECT_EQ(engine.state(), EngineState::STAND);
  // Strategy was never invoked: STAND emits nominal directly.
  EXPECT_TRUE(spy->calls.empty());
  for (const auto& name : LEG_NAMES) {
    EXPECT_TRUE(out.at(name).stance);
  }
}

TEST(Engine, PhaseAdvancesFasterAtHigherVelocity) {
  // Two engines identical but for commanded velocity: the faster command
  // accumulates phase faster across a fixed dt (cycle_time shrinks). Drive both
  // through engagement first so this isolates GAIT phase advance.
  SpyStrategy* spy_a = nullptr;
  SpyStrategy* spy_b = nullptr;
  Engine engine_a = make_spy_engine(&spy_a);
  Engine engine_b = make_spy_engine(&spy_b);

  drive_to_gait(engine_a, {0.10, 0.0}, 0.0);
  drive_to_gait(engine_b, {0.30, 0.0}, 0.0);
  spy_a->clear();
  spy_b->clear();

  // A handful of GAIT ticks — not enough for the faster engine to lap the
  // slower, so the direct phase comparison is well-defined.
  for (int i = 0; i < 3; ++i) {
    engine_a.update(0.02, {0.10, 0.0}, 0.0);
    engine_b.update(0.02, {0.30, 0.0}, 0.0);
  }

  const double last_phase_a = spy_a->last_phase("l_front");
  const double last_phase_b = spy_b->last_phase("l_front");
  EXPECT_GT(last_phase_b, last_phase_a);
}

// ── Engagement / STAND handoff ────────────────────────────────────────────

TEST(Engine, StandToFirstNonzeroCmdRoutesThroughEngaging) {
  // Python also asserted engine._engagement.state is ENGAGING; that private
  // reach-in is covered by the public state() == ENGAGING check.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_past_initialize(engine);
  engine.update(0.02, {0.20, 0.0}, 0.0);
  EXPECT_EQ(engine.state(), EngineState::ENGAGING);
}

TEST(Engine, NoFirstTickPositionJumpFromStand) {
  // The reported bug: at STAND -> first non-zero cmd tick, no foot should jump.
  // Every leg must still be near NOMINAL.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  const auto nominal = nominal_stance();
  drive_past_initialize(engine);
  const auto out = engine.update(0.02, {0.20, 0.0}, 0.0);
  for (const auto& name : LEG_NAMES) {
    const double dx = std::abs(out.at(name).foot_target[0] - nominal.at(name)[0]);
    const double dy = std::abs(out.at(name).foot_target[1] - nominal.at(name)[1]);
    EXPECT_LT(dx, 0.01) << name << " jumped on first tick";
    EXPECT_LT(dy, 0.01);
  }
}

TEST(Engine, EngagingToGaitAtExitMaster) {
  // After one full engagement cycle the engine reaches GAIT and the clock is
  // seeded at exit_master, which wraps to 0.0. master_phase() surfaces the
  // clock during GAIT, so it replaces the Python engine._clock.master reach-in.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_to_gait(engine, {0.20, 0.0}, 0.0);
  ASSERT_EQ(engine.state(), EngineState::GAIT);
  // The handoff tick resets the clock to exit_master = 0.0; drive_to_gait
  // returns immediately after, so no GAIT step has advanced it yet.
  EXPECT_NEAR(engine.master_phase(), 0.0, 1e-9);
  // First GAIT tick advances by dt / cycle_time = 0.02 / 1.0.
  engine.update(0.02, {0.20, 0.0}, 0.0);
  EXPECT_NEAR(engine.master_phase(), 0.02, 1e-9);
}

TEST(Engine, MasterPhaseTracksEngagementDuringEngaging) {
  // During ENGAGING the engine's clock is frozen and master_phase() mirrors the
  // engagement controller's exit_master. Python compared the two directly (a
  // tautology here since master_phase() *returns* exit_master while ENGAGING),
  // so we keep the substantive claim: master_phase advances across the cycle.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_past_initialize(engine);

  engine.update(0.02, {0.20, 0.0}, 0.0);
  ASSERT_EQ(engine.state(), EngineState::ENGAGING);

  double last_phase = engine.master_phase();
  bool saw_progress = false;
  for (int i = 0; i < 80; ++i) {
    engine.update(0.02, {0.20, 0.0}, 0.0);
    if (engine.state() != EngineState::ENGAGING) {
      break;
    }
    const double phase = engine.master_phase();
    if (phase > last_phase) {
      saw_progress = true;
    }
    last_phase = phase;
  }
  EXPECT_TRUE(saw_progress) << "master_phase did not advance during ENGAGING";
}

TEST(Engine, MasterPhaseContinuousAcrossEngagingToGaitHandoff) {
  // The engagement controller seeds the clock with exit_master at handoff, so
  // the master_phase on the last ENGAGING tick and the first GAIT tick differ
  // only by one advance step (modulo the [0,1) wrap).
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_past_initialize(engine);

  double prev_phase = 0.0;
  for (int i = 0; i < 200; ++i) {
    engine.update(0.02, {0.20, 0.0}, 0.0);
    const double phase = engine.master_phase();
    if (engine.state() == EngineState::GAIT) {
      const double step = pymod(phase - prev_phase, 1.0);
      EXPECT_LT(step, 0.05)
          << "master_phase stepped by " << step << " across ENGAGING -> GAIT";
      return;
    }
    prev_phase = phase;
  }
  FAIL() << "engine did not reach GAIT within 200 ticks";
}

TEST(Engine, MasterPhaseContinuousAcrossResumingToGaitHandoff) {
  // Same continuity requirement on the PAUSED -> RESUMING -> GAIT path.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy, make_config());
  drive_to_gait(engine, {0.20, 0.0}, 0.0);

  // Tick a few GAIT cycles so the paused master_phase is not trivially 0.
  for (int i = 0; i < 20; ++i) {
    engine.update(0.02, {0.20, 0.0}, 0.0);
  }
  // Park at zero cmd until PAUSED.
  for (int i = 0; i < 300; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::PAUSED) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::PAUSED);

  // First non-zero tick enters RESUMING and runs one engagement tick.
  engine.update(0.02, {0.20, 0.0}, 0.0);
  ASSERT_EQ(engine.state(), EngineState::RESUMING);
  double prev_phase = engine.master_phase();

  for (int i = 0; i < 400; ++i) {
    engine.update(0.02, {0.20, 0.0}, 0.0);
    const double phase = engine.master_phase();
    if (engine.state() == EngineState::GAIT) {
      const double step = pymod(phase - prev_phase, 1.0);
      EXPECT_LT(step, 0.05)
          << "master_phase stepped by " << step << " across RESUMING -> GAIT";
      return;
    }
    prev_phase = phase;
  }
  FAIL() << "engine did not reach GAIT within 400 RESUMING ticks";
}

// ── Pause / resume debounce ───────────────────────────────────────────────

TEST(Engine, EngagingToPausingOnZeroCmd) {
  // cmd zeros mid-engagement: bail to PAUSING on the very first zero tick,
  // regardless of pause_debounce_delay (the debounce is for GAIT zero-crossings;
  // ticking ENGAGING at zero cmd would snap mid-flight swing legs to NOMINAL).
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy, make_config(0.10, 0.25, 1.0, 0.8));
  drive_past_initialize(engine);
  engine.update(0.02, {0.20, 0.0}, 0.0);
  ASSERT_EQ(engine.state(), EngineState::ENGAGING);
  engine.update(0.02, {0.0, 0.0}, 0.0);
  EXPECT_EQ(engine.state(), EngineState::PAUSING);
}

TEST(Engine, BriefZeroCmdUnderDebounceStaysInGait) {
  // Right-joystick passing through center sends cmd_vel to zero for a handful of
  // ticks. With pause_debounce_delay set, those zero ticks must not trip the
  // pause transition.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy, make_config(0.10, 0.25, 1.0, 0.15));
  drive_to_gait(engine, {0.0, 0.0}, 0.5);
  ASSERT_EQ(engine.state(), EngineState::GAIT);

  // 5 zero ticks at dt=0.02 -> 0.10 s, well under the 0.15 s debounce.
  for (int i = 0; i < 5; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    EXPECT_EQ(engine.state(), EngineState::GAIT);
  }
  // Stick re-engages on the other side of center: still GAIT.
  engine.update(0.02, {0.0, 0.0}, -0.5);
  EXPECT_EQ(engine.state(), EngineState::GAIT);
}

TEST(Engine, SustainedZeroCmdPastDebounceEntersPausing) {
  // If cmd_vel really stays zero, the debounce expires and the engine commits.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy, make_config(0.10, 0.25, 1.0, 0.10));
  drive_to_gait(engine, {0.20, 0.0}, 0.0);
  ASSERT_EQ(engine.state(), EngineState::GAIT);

  // 4 ticks * 0.02 = 0.08 s < 0.10 s -> still GAIT.
  for (int i = 0; i < 4; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
  }
  EXPECT_EQ(engine.state(), EngineState::GAIT);

  // Past the window the engine leaves GAIT; with every leg already at nominal
  // the pause controller flips to PAUSED immediately. Accept either.
  for (int i = 0; i < 2; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
  }
  EXPECT_TRUE(engine.state() == EngineState::PAUSING ||
              engine.state() == EngineState::PAUSED);
}

TEST(Engine, DebounceResetsOnNonzeroCmd) {
  // A near-miss zero crossing must fully reset the timer so the next zero burst
  // gets its own full window.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy, make_config(0.10, 0.25, 1.0, 0.10));
  drive_to_gait(engine, {0.20, 0.0}, 0.0);

  for (int i = 0; i < 4; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
  }
  engine.update(0.02, {0.20, 0.0}, 0.0);
  ASSERT_EQ(engine.state(), EngineState::GAIT);

  // New zero burst: 4 ticks (0.08 s) must still be inside the window.
  for (int i = 0; i < 4; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
  }
  EXPECT_EQ(engine.state(), EngineState::GAIT);
}

TEST(Engine, RepeatEngagementAfterFullStop) {
  // Walk -> stop -> walk: the second walk must re-engage cleanly.
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  const auto nominal = nominal_stance();

  drive_to_gait(engine, {0.20, 0.0}, 0.0);
  for (int i = 0; i < 500; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::STAND) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::STAND);

  const auto out = engine.update(0.02, {0.20, 0.0}, 0.0);
  EXPECT_EQ(engine.state(), EngineState::ENGAGING);
  for (const auto& name : LEG_NAMES) {
    const double dx = std::abs(out.at(name).foot_target[0] - nominal.at(name)[0]);
    EXPECT_LT(dx, 0.01) << name << " jumped on second engagement";
  }
}

TEST(Engine, GaitSwingLiftoffVelocityMatchesBodyVelocity) {
  // Regression for the 2x swing-trajectory bug. Engagement covers one full
  // cycle and exits at master = 0, so l_front is at PEP and about to swing at
  // GAIT entry. Trace l_front to capture the swing lift-off velocity.
  Engine engine = make_gait_engine("tripod");
  engine.start_initialize();
  const double dt = 0.001;
  const double cmd_v = 0.20;

  std::vector<double> trace_gait;
  for (int i = 0; i < 1600; ++i) {
    const auto out = engine.update(dt, {cmd_v, 0.0}, 0.0);
    if (engine.state() == EngineState::GAIT) {
      trace_gait.push_back(out.at("l_front").foot_target[0]);
      if (trace_gait.size() >= 20) {
        break;
      }
    }
  }
  ASSERT_GE(trace_gait.size(), 20u);

  // Skip index 0 (the flip-to-GAIT tick still carries engagement's output);
  // measure from index 1 onward, where GAIT's swing_arc drives every tick.
  const double v_gait_liftoff = (trace_gait[11] - trace_gait[1]) / (10 * dt);
  // Sampled near the lift-off endpoint of the primary Bezier where dB/dt = -v_in.
  // With the trajectory fix v_in = -cmd_v; pre-fix it was -2*cmd_v ~ -0.40.
  EXPECT_NEAR(v_gait_liftoff, -cmd_v, 5e-3);
}

// ── set_strategy ──────────────────────────────────────────────────────────

TEST(Engine, SetStrategySwapInStandSucceeds) {
  // Python also asserted engine._strategy.duty_factor == 2/3; that private
  // reach-in is redundant with strategy_name() == "crawl" (the engine rebuilds
  // the strategy from the registry, so the name determines the duty factor).
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_past_initialize(engine);
  ASSERT_EQ(engine.state(), EngineState::STAND);
  EXPECT_TRUE(engine.set_strategy("crawl"));
  EXPECT_EQ(engine.strategy_name(), "crawl");
}

TEST(Engine, SetStrategyUnknownNameReturnsFalse) {
  SpyStrategy* spy = nullptr;
  Engine engine = make_spy_engine(&spy);
  drive_past_initialize(engine);
  EXPECT_FALSE(engine.set_strategy("gallop"));
  // Strategy must not change.
  EXPECT_EQ(engine.strategy_name(), "spy");
}

TEST(Engine, SetStrategySameNameIsNoOp) {
  // Python asserted the strategy *object* was unchanged (identity). C++ has no
  // strategy getter; the observable proxy is that set_strategy succeeds and the
  // name is unchanged (the STAND same-name branch skips apply_strategy).
  Engine engine = make_gait_engine("tripod");
  drive_past_initialize(engine);
  EXPECT_TRUE(engine.set_strategy("tripod"));
  EXPECT_EQ(engine.strategy_name(), "tripod");
}

TEST(Engine, DeriveCycleTimeReadsStrategyDutyFactor) {
  // The engine derives min/max cycle_time = swing_time / (1 - beta), then clamps
  // cycle_time = stride / (v * beta). Verify both the floor and the divisor
  // track the active strategy's duty factor, via the free derive_cycle_time.
  const std::vector<std::pair<std::string, double>> cases = {
      {"tripod", 0.5}, {"crawl", 2.0 / 3.0}, {"ripple", 5.0 / 6.0}};
  const EngineConfig cfg = make_config(0.10, 0.25);

  for (const auto& [name, expected_duty] : cases) {
    SpyStrategy* spy = nullptr;
    Engine engine = make_spy_engine(&spy, cfg);
    drive_past_initialize(engine);
    ASSERT_TRUE(engine.set_strategy(name)) << name;

    const double expected_min_cycle = 0.25 / (1.0 - expected_duty);
    const double expected_max_cycle = cfg.max_swing_time / (1.0 - expected_duty);

    auto cycle_time = [&](double max_leg_v) {
      return derive_cycle_time(max_leg_v, cfg.stride_length, expected_duty,
                               expected_min_cycle, expected_max_cycle);
    };

    // Slow command under saturation: raw quotient exceeds the per-gait max, so
    // the engine clamps to that max.
    const double v_small = 0.001;
    const double raw = 0.10 / (v_small * expected_duty);
    EXPECT_GT(raw, expected_max_cycle) << name;
    EXPECT_NEAR(cycle_time(v_small), expected_max_cycle, 1e-9) << name;

    // Fast command above saturation: clamped to min_cycle.
    EXPECT_NEAR(cycle_time(10.0), expected_min_cycle, 1e-9) << name;
  }
}

TEST(Engine, StanceClassificationAbsorbsTouchdownUlp) {
  // Regression for the one-tick stance-flag seam bug. 1 - 2/3 is not exactly
  // representable, so a foot at the touchdown point 1/3 computes one ULP below
  // swing_end. The engine classifies stance as phase >= (1 - duty) - eps; a bare
  // >= would misflag that just-landed foot as airborne for one tick. The seam
  // epsilon (kStanceSeamEpsilon in engine.cpp) is 1e-9; mirror its value here.
  constexpr double kStanceSeamEpsilon = 1e-9;
  const double swing_end = 1.0 - 2.0 / 3.0;
  const double touchdown_phase = 1.0 / 3.0;
  // The trap: the touched-down foot computes just below swing_end.
  EXPECT_LT(touchdown_phase, swing_end);
  // The guard: the engine's rule still classes it as stance.
  EXPECT_GE(touchdown_phase, swing_end - kStanceSeamEpsilon);
  // The epsilon stays far below one phase tick, so it never swallows a real
  // swing tick.
  EXPECT_GT(kStanceSeamEpsilon, 0.0);
  EXPECT_LT(kStanceSeamEpsilon, 1e-6);
}

// ── Gait change while walking ─────────────────────────────────────────────

TEST(Engine, SetStrategyDuringGaitRunsPauseReseatEngageSequence) {
  Engine engine = make_gait_engine("tripod");
  drive_to_gait(engine, kCmd, 0.0);

  ASSERT_TRUE(engine.set_strategy("crawl"));
  ASSERT_EQ(engine.pending_strategy_name(), std::optional<std::string>("crawl"));

  const auto trace = trace_states(engine, kCmd, 400);
  assert_ordered_subsequence(
      trace, {EngineState::PAUSING, EngineState::PAUSED, EngineState::RESEATING,
              EngineState::STAND, EngineState::ENGAGING});
  EXPECT_FALSE(trace_contains(trace, EngineState::RESUMING));
  EXPECT_EQ(engine.strategy_name(), "crawl");
  EXPECT_FALSE(engine.pending_strategy_name().has_value());
}

TEST(Engine, GaitChangeUsesShortPauseToReseatDwell) {
  const double dt = 0.02;
  const EngineConfig cfg = make_config(0.10, 0.25, 1.0, 0.0, 5.0, 0.1);

  // Pending gait change: PAUSED hands off to RESEATING after the short dwell.
  Engine engine = make_gait_engine("tripod", cfg);
  drive_to_gait(engine, kCmd, 0.0);
  engine.set_strategy("crawl");
  drive_to_state(engine, kCmd, EngineState::PAUSED);
  int paused_ticks = 0;
  for (int i = 0; i < 50; ++i) {
    engine.update(dt, kCmd, 0.0);
    if (engine.state() != EngineState::PAUSED) {
      break;
    }
    ++paused_ticks;
  }
  EXPECT_EQ(engine.state(), EngineState::RESEATING);
  EXPECT_LE(paused_ticks * dt, 0.1 + dt);

  // Control: a normal zero-cmd pause with no pending change stays PAUSED past
  // the short-dwell window.
  Engine control = make_gait_engine("tripod", cfg);
  drive_to_gait(control, kCmd, 0.0);
  drive_to_state(control, {0.0, 0.0}, EngineState::PAUSED);
  for (int i = 0; i < 25; ++i) {  // 0.5 s — 5x the short dwell
    control.update(dt, {0.0, 0.0}, 0.0);
    EXPECT_EQ(control.state(), EngineState::PAUSED);
  }
}

TEST(Engine, PendingGaitUpdatesMidSequence) {
  Engine engine = make_gait_engine("tripod");
  drive_to_gait(engine, kCmd, 0.0);

  ASSERT_TRUE(engine.set_strategy("crawl"));
  engine.update(0.02, kCmd, 0.0);
  ASSERT_EQ(engine.state(), EngineState::PAUSING);
  ASSERT_TRUE(engine.set_strategy("ripple"));
  EXPECT_EQ(engine.pending_strategy_name(), std::optional<std::string>("ripple"));

  drive_to_state(engine, kCmd, EngineState::PAUSED);
  EXPECT_TRUE(engine.set_strategy("ripple"));

  drive_to_state(engine, kCmd, EngineState::GAIT);
  EXPECT_EQ(engine.strategy_name(), "ripple");
}

TEST(Engine, CycleBackToOriginalGaitStillCompletesSequence) {
  Engine engine = make_gait_engine("tripod");
  drive_to_gait(engine, kCmd, 0.0);

  ASSERT_TRUE(engine.set_strategy("crawl"));
  engine.update(0.02, kCmd, 0.0);
  ASSERT_EQ(engine.state(), EngineState::PAUSING);
  // Cycle back to the originally-active gait: still latched, sequence completes.
  ASSERT_TRUE(engine.set_strategy("tripod"));
  EXPECT_EQ(engine.pending_strategy_name(), std::optional<std::string>("tripod"));

  const auto trace = trace_states(engine, kCmd, 400);
  assert_ordered_subsequence(
      trace, {EngineState::PAUSED, EngineState::RESEATING, EngineState::STAND,
              EngineState::ENGAGING});
  EXPECT_FALSE(trace_contains(trace, EngineState::RESUMING));
  EXPECT_EQ(engine.strategy_name(), "tripod");
}

TEST(Engine, SetStrategyLockedDuringEngaging) {
  Engine engine = make_gait_engine("tripod");
  drive_past_initialize(engine);
  engine.update(0.02, kCmd, 0.0);
  ASSERT_EQ(engine.state(), EngineState::ENGAGING);

  EXPECT_FALSE(engine.set_strategy("crawl"));
  EXPECT_FALSE(engine.pending_strategy_name().has_value());

  drive_to_state(engine, kCmd, EngineState::GAIT);
  EXPECT_EQ(engine.strategy_name(), "tripod");
}

TEST(Engine, SetStrategyLockedDuringResuming) {
  Engine engine = make_gait_engine("tripod");
  drive_to_gait(engine, kCmd, 0.0);
  drive_to_state(engine, {0.0, 0.0}, EngineState::PAUSED);
  engine.update(0.02, kCmd, 0.0);
  ASSERT_EQ(engine.state(), EngineState::RESUMING);

  EXPECT_FALSE(engine.set_strategy("crawl"));
  EXPECT_FALSE(engine.pending_strategy_name().has_value());

  drive_to_state(engine, kCmd, EngineState::GAIT);
  EXPECT_EQ(engine.strategy_name(), "tripod");
}

TEST(Engine, ResumeSuppressedWhileGaitChangePending) {
  Engine engine = make_gait_engine("tripod");
  drive_to_gait(engine, kCmd, 0.0);
  engine.set_strategy("crawl");

  // cmd held non-zero the whole way: without suppression the first PAUSING tick
  // would route straight back to RESUMING.
  for (int i = 0; i < 400; ++i) {
    engine.update(0.02, kCmd, 0.0);
    EXPECT_NE(engine.state(), EngineState::RESUMING);
    if (engine.state() == EngineState::GAIT) {
      break;
    }
  }
  EXPECT_EQ(engine.state(), EngineState::GAIT);
  EXPECT_EQ(engine.strategy_name(), "crawl");
}

TEST(Engine, PendingGaitCommitsWhenCmdReleasedMidSequence) {
  Engine engine = make_gait_engine("tripod");
  drive_to_gait(engine, kCmd, 0.0);
  engine.set_strategy("crawl");
  engine.update(0.02, kCmd, 0.0);
  ASSERT_EQ(engine.state(), EngineState::PAUSING);

  // Operator releases the stick mid-sequence: the sequence still completes and
  // commits, then settles in STAND.
  drive_to_state(engine, {0.0, 0.0}, EngineState::STAND);
  EXPECT_EQ(engine.strategy_name(), "crawl");
  EXPECT_FALSE(engine.pending_strategy_name().has_value());
  for (int i = 0; i < 25; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    EXPECT_EQ(engine.state(), EngineState::STAND);
  }
}

TEST(Engine, SetStrategyAcceptedDuringNormalPause) {
  // Python also asserted the new duty factor (2/3) after the walk re-engaged;
  // strategy_name() == "crawl" covers that (engine rebuilds from the registry).
  Engine engine = make_gait_engine("tripod");
  drive_to_gait(engine, kCmd, 0.0);
  drive_to_state(engine, {0.0, 0.0}, EngineState::PAUSED);

  EXPECT_TRUE(engine.set_strategy("crawl"));
  EXPECT_EQ(engine.pending_strategy_name(), std::optional<std::string>("crawl"));
  drive_to_state(engine, {0.0, 0.0}, EngineState::STAND);
  EXPECT_EQ(engine.strategy_name(), "crawl");

  // The next walk engages the new gait.
  engine.update(0.02, kCmd, 0.0);
  EXPECT_EQ(engine.state(), EngineState::ENGAGING);
}

// ── Stance integrator: world-frame foot invariance ────────────────────────
//
// Ripple (beta = 5/6) spreads its five stance legs across stance phases. A
// velocity change mid-stance must not shift any stance foot in the world frame.

TEST(Engine, RippleStanceWorldInvariantAtConstantVelocity) {
  Engine engine = make_gait_engine("ripple");
  const double v_x = 0.05;
  drive_to_gait(engine, {v_x, 0.0}, 0.0);

  const double dt = 0.005;
  double body_x = 0.0;
  std::map<std::string, std::optional<std::pair<double, double>>> last_world;
  std::map<std::string, bool> prev_stance;
  for (const auto& n : LEG_NAMES) {
    last_world[n] = std::nullopt;
    prev_stance[n] = false;
  }

  for (int i = 0; i < 800; ++i) {
    const auto out = engine.update(dt, {v_x, 0.0}, 0.0);
    body_x += v_x * dt;
    for (const auto& name : LEG_NAMES) {
      if (!out.at(name).stance) {
        last_world[name] = std::nullopt;
        prev_stance[name] = false;
        continue;
      }
      const double wx = body_x + out.at(name).foot_target[0];
      const double wy = out.at(name).foot_target[1];
      if (prev_stance[name] && last_world[name].has_value()) {
        EXPECT_LT(std::abs(wx - last_world[name]->first), 1e-6)
            << name << " world drift dx";
        EXPECT_LT(std::abs(wy - last_world[name]->second), 1e-6)
            << name << " world drift dy";
      }
      last_world[name] = std::make_pair(wx, wy);
      prev_stance[name] = true;
    }
  }
}

TEST(Engine, RippleMidStanceVelocityStepKeepsFeetPlanted) {
  // At a mixed stance-phase moment, step velocity from (0.10, 0) to
  // (0.05, 0.05). Every leg that stays in stance must hold its world position.
  Engine engine = make_gait_engine("ripple");
  const double v_x = 0.10;
  drive_to_gait(engine, {v_x, 0.0}, 0.0);

  const double dt = 0.005;
  double body_x = 0.0;
  double body_y = 0.0;
  std::map<std::string, LegOutput> out;
  for (int i = 0; i < 60; ++i) {
    out = engine.update(dt, {v_x, 0.0}, 0.0);
    body_x += v_x * dt;
  }

  std::map<std::string, std::pair<double, double>> before;
  for (const auto& name : LEG_NAMES) {
    if (out.at(name).stance) {
      before[name] = {body_x + out.at(name).foot_target[0],
                      body_y + out.at(name).foot_target[1]};
    }
  }
  ASSERT_GE(before.size(), 4u);

  const std::pair<double, double> v_new = {0.05, 0.05};
  out = engine.update(dt, v_new, 0.0);
  body_x += v_new.first * dt;
  body_y += v_new.second * dt;

  for (const auto& [name, wbefore] : before) {
    if (!out.at(name).stance) {
      continue;  // flipped to swing on this tick
    }
    const double wx_after = body_x + out.at(name).foot_target[0];
    const double wy_after = body_y + out.at(name).foot_target[1];
    EXPECT_LT(std::abs(wx_after - wbefore.first), 1e-3) << name << " world dx";
    EXPECT_LT(std::abs(wy_after - wbefore.second), 1e-3) << name << " world dy";
  }
}

TEST(Engine, EngagementToGaitSeedWorldInvariantUnderVelocityStep) {
  // Drive through engagement -> GAIT, then step velocity on the first GAIT cycle
  // while legs are still in their initial stance window. Proves the seed at
  // handoff carries the correct world-locked anchors.
  Engine engine = make_gait_engine("ripple");
  const double v_x = 0.10;
  drive_to_gait(engine, {v_x, 0.0}, 0.0);
  ASSERT_EQ(engine.state(), EngineState::GAIT);

  const double dt = 0.005;
  double body_x = 0.0;
  double body_y = 0.0;
  auto out = engine.update(dt, {v_x, 0.0}, 0.0);
  body_x += v_x * dt;

  std::map<std::string, std::pair<double, double>> before;
  for (const auto& name : LEG_NAMES) {
    if (out.at(name).stance) {
      before[name] = {body_x + out.at(name).foot_target[0],
                      body_y + out.at(name).foot_target[1]};
    }
  }
  ASSERT_GE(before.size(), 4u);

  const std::pair<double, double> v_new = {0.05, 0.05};
  out = engine.update(dt, v_new, 0.0);
  body_x += v_new.first * dt;
  body_y += v_new.second * dt;

  for (const auto& [name, wbefore] : before) {
    if (!out.at(name).stance) {
      continue;
    }
    const double wx_after = body_x + out.at(name).foot_target[0];
    const double wy_after = body_y + out.at(name).foot_target[1];
    EXPECT_LT(std::abs(wx_after - wbefore.first), 1e-3) << name << " world dx";
    EXPECT_LT(std::abs(wy_after - wbefore.second), 1e-3) << name << " world dy";
  }
}

// ── Swing planner: body-frame foot continuity under mid-swing velocity step ─

TEST(Engine, RippleMidSwingVelocityStepKeepsSwingFootContinuous) {
  Engine engine = make_gait_engine("ripple");
  const double v_x = 0.10;
  drive_to_gait(engine, {v_x, 0.0}, 0.0);

  const double dt = 0.005;
  // Tick into steady-state until a leg is mid-swing (phase inside (0.04, 0.12)
  // on ripple's [0, 1/6) swing — past lift-off, before touchdown).
  std::string swing_name;
  std::map<std::string, LegOutput> last_out;
  for (int i = 0; i < 400; ++i) {
    last_out = engine.update(dt, {v_x, 0.0}, 0.0);
    for (const auto& n : LEG_NAMES) {
      if (!last_out.at(n).stance && last_out.at(n).phase > 0.04 &&
          last_out.at(n).phase < 0.12) {
        swing_name = n;
        break;
      }
    }
    if (!swing_name.empty()) {
      break;
    }
  }
  ASSERT_FALSE(swing_name.empty()) << "no leg landed mid-swing inside scan window";

  const Vec3 pos_before = last_out.at(swing_name).foot_target;

  // Single-tick velocity step: add v_y and omega_z on top of steady v_x. The
  // body-frame swing foot must shift by at most the latched arc's own per-tick
  // progression (<< 1 mm at dt = 5 ms on ripple).
  const auto out = engine.update(dt, {v_x, 0.05}, 0.3);
  ASSERT_FALSE(out.at(swing_name).stance);

  const Vec3 pos_after = out.at(swing_name).foot_target;
  EXPECT_LT(std::abs(pos_after[0] - pos_before[0]), 3.0e-3) << "swing dx";
  EXPECT_LT(std::abs(pos_after[1] - pos_before[1]), 3.0e-3) << "swing dy";
  EXPECT_LT(std::abs(pos_after[2] - pos_before[2]), 3.0e-3) << "swing dz";
}

TEST(Engine, RippleSwingLiftoffVelocityMatchesBodyVelocity) {
  // Ripple's beta = 5/6: the SwingPlanner overrides the default lift-off
  // velocity with -v_leg so swing launches at the stance-frame velocity.
  Engine engine = make_gait_engine("ripple");
  const double cmd_v = 0.10;
  drive_to_gait(engine, {cmd_v, 0.0}, 0.0);
  const double dt = 0.001;

  std::map<std::string, bool> prev_stance;
  for (const auto& n : LEG_NAMES) {
    prev_stance[n] = true;
  }
  std::vector<double> trace;
  std::string tracked;
  // Let the engagement-mid-swing leg complete its swing first so subsequent
  // lift-offs are pure GAIT (the integrator's PEP as swing origin).
  const int ticks = static_cast<int>(2.0 / dt);
  for (int i = 0; i < ticks; ++i) {
    const auto out = engine.update(dt, {cmd_v, 0.0}, 0.0);
    if (tracked.empty()) {
      for (const auto& n : LEG_NAMES) {
        if (prev_stance[n] && !out.at(n).stance) {
          tracked = n;
          trace.push_back(out.at(n).foot_target[0]);
          break;
        }
      }
    } else if (!out.at(tracked).stance) {
      trace.push_back(out.at(tracked).foot_target[0]);
      if (trace.size() >= 20) {
        break;
      }
    } else {
      // Tracked leg already touched down — restart the search.
      tracked.clear();
      trace.clear();
    }
    for (const auto& n : LEG_NAMES) {
      prev_stance[n] = out.at(n).stance;
    }
  }

  ASSERT_FALSE(tracked.empty());
  ASSERT_GE(trace.size(), 20u);
  // Sample dB/dt at the lift-off endpoint of the primary Bezier. Pre-fix this
  // would be ~ -5*cmd_v = -0.50.
  const double v_liftoff = (trace[15] - trace[1]) / (14 * dt);
  EXPECT_NEAR(v_liftoff, -cmd_v, 2.0e-2);
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
