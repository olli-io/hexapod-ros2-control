// gtest port of hexa_gait/test/test_fold.py — exercises the pure C++
// FoldController (warm shutdown, standing -> initial_pose) plus the Engine's
// FOLDING / FOLDED integration path. Behavioural parity with the Python suite.
//
// The Python module split (hexa_gait.fold + hexa_gait.initialize) is merged
// into hexa_gait_cpp/stand_transition.hpp. PAIR_ORDER_REVERSED is not exported,
// so it is computed here as the reverse of PAIR_ORDER (which is exactly what
// test_pair_order_reversed_mirrors_initialize asserts).

#include <gtest/gtest.h>

#include <array>
#include <map>
#include <string>
#include <vector>

#include "hexa_gait_cpp/clock.hpp"
#include "hexa_gait_cpp/engine.hpp"
#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/stand_transition.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

constexpr double COXA_TO_BOTTOM = 0.02;
constexpr double PAIR_TIME = 0.1;
constexpr double LIFT_TIME = 0.1;
constexpr double SWING_CLEARANCE = 0.02;
constexpr double PLACE_FEET_CLEARANCE = 0.001;

// fold's LIFT_FEET reverses initialize's PLACE_FEET pair order. Computed
// locally (PAIR_ORDER_REVERSED is not exported from stand_transition.hpp).
const std::array<std::array<std::string, 2>, 3>& pair_order_reversed() {
  static const std::array<std::array<std::string, 2>, 3> r = {
      {PAIR_ORDER[2], PAIR_ORDER[1], PAIR_ORDER[0]}};
  return r;
}

std::map<std::string, Vec3> nominal_stance() {
  return {
      {"l_front", Vec3(0.15, 0.10, -0.10)},
      {"r_front", Vec3(0.15, -0.10, -0.10)},
      {"l_middle", Vec3(0.0, 0.12, -0.10)},
      {"r_middle", Vec3(0.0, -0.12, -0.10)},
      {"l_rear", Vec3(-0.15, 0.10, -0.10)},
      {"r_rear", Vec3(-0.15, -0.10, -0.10)},
  };
}

// Feet folded up well above the body, in an asymmetric layout — verifies the
// controller never accidentally relies on initial_stance matching
// nominal_stance in XY.
std::map<std::string, Vec3> initial_stance() {
  return {
      {"l_front", Vec3(0.05, 0.04, 0.08)},
      {"r_front", Vec3(0.05, -0.04, 0.08)},
      {"l_middle", Vec3(0.0, 0.05, 0.08)},
      {"r_middle", Vec3(0.0, -0.05, 0.08)},
      {"l_rear", Vec3(-0.05, 0.04, 0.08)},
      {"r_rear", Vec3(-0.05, -0.04, 0.08)},
  };
}

Vec3 ground_target(const std::string& name) {
  const Vec3 nom = nominal_stance().at(name);
  return Vec3(nom[0], nom[1], -COXA_TO_BOTTOM + PLACE_FEET_CLEARANCE);
}

struct ControllerArgs {
  std::map<std::string, Vec3> initial_stance = ::hexa_gait::initial_stance();
  std::map<std::string, Vec3> nominal_stance = ::hexa_gait::nominal_stance();
  double coxa_to_bottom = COXA_TO_BOTTOM;
  double pair_swing_time = PAIR_TIME;
  double lift_body_time = LIFT_TIME;
  double swing_clearance = SWING_CLEARANCE;
  double place_feet_clearance = PLACE_FEET_CLEARANCE;
  double swing_width = 0.0;
  double controller_dt = 0.02;
};

FoldController make_controller(const ControllerArgs& a = ControllerArgs{}) {
  return FoldController(a.initial_stance, a.nominal_stance, a.coxa_to_bottom,
                        a.pair_swing_time, a.lift_body_time, a.swing_clearance,
                        a.place_feet_clearance, a.swing_width, a.controller_dt);
}

std::map<std::string, LegContext> leg_contexts() {
  const auto nominal = nominal_stance();
  const std::map<std::string, Vec3> mounts = {
      {"l_front", Vec3(0.15, 0.10, 0.0)},
      {"r_front", Vec3(0.15, -0.10, 0.0)},
      {"l_middle", Vec3(0.0, 0.12, 0.0)},
      {"r_middle", Vec3(0.0, -0.12, 0.0)},
      {"l_rear", Vec3(-0.15, 0.10, 0.0)},
      {"r_rear", Vec3(-0.15, -0.10, 0.0)},
  };
  std::map<std::string, LegContext> out;
  for (const auto& n : LEG_NAMES) {
    LegContext ctx;
    ctx.name = n;
    ctx.mount_xyz = mounts.at(n);
    ctx.mount_yaw = 0.0;
    ctx.nominal_stance = nominal.at(n);
    out[n] = ctx;
  }
  return out;
}

EngineConfig engine_config() {
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
  c.init_swing_clearance = SWING_CLEARANCE;
  c.init_place_feet_clearance = PLACE_FEET_CLEARANCE;
  // Reseat knobs (unused — Engine is constructed without leg_specs/reseat).
  c.reseat_pose_settle_delay = 0.1;
  c.reseat_height_change_threshold = 0.001;
  c.reseat_pair_swing_time = 0.1;
  c.reseat_pair_dwell_time = 0.0;
  c.reseat_swing_clearance = 0.02;
  return c;
}

Engine make_engine() {
  return Engine(engine_config(), strategies().at("tripod")(), "tripod",
                nominal_stance(), initial_stance(), COXA_TO_BOTTOM,
                leg_contexts());
}

// Run a fresh engine through INITIALIZE until it parks in STAND.
void drive_to_stand(Engine& engine) {
  ASSERT_TRUE(engine.start_initialize());
  for (int i = 0; i < 200; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::STAND) {
      return;
    }
  }
  FAIL() << "engine never reached STAND";
}

void expect_vec_near(const Vec3& a, const Vec3& b, double tol,
                     const std::string& msg = "") {
  EXPECT_NEAR(a[0], b[0], tol) << msg;
  EXPECT_NEAR(a[1], b[1], tol) << msg;
  EXPECT_NEAR(a[2], b[2], tol) << msg;
}

// --- FoldController unit tests --------------------------------------------

TEST(Fold, PairOrderReversedMirrorsInitialize) {
  // Documented behaviour: fold's LIFT_FEET reverses initialize's PLACE_FEET
  // pair order so the two ladders feel symmetric.
  const auto& rev = pair_order_reversed();
  for (std::size_t i = 0; i < PAIR_ORDER.size(); ++i) {
    EXPECT_EQ(rev[i], PAIR_ORDER[PAIR_ORDER.size() - 1 - i]);
  }
}

TEST(Fold, ControllerStartsInLowerBody) {
  FoldController ctrl = make_controller();
  EXPECT_EQ(ctrl.state(), FoldState::LOWER_BODY);
  EXPECT_FALSE(ctrl.done());
}

TEST(Fold, LowerBodyHoldsXyAndMonotonicallyRaisesZInBodyFrame) {
  // LOWER_BODY ramps body-frame z from nominal.z up to
  // -coxa_to_bottom + place_feet_clearance (less negative => foot closer to
  // body => world-frame body lowers onto its belly).
  FoldController ctrl = make_controller();
  const auto nominal = nominal_stance();
  const double dt = 0.005;

  std::map<std::string, double> prev_z;
  for (const auto& n : LEG_NAMES) {
    prev_z[n] = nominal.at(n)[2];
  }

  while (ctrl.state() == FoldState::LOWER_BODY) {
    const auto out = ctrl.update(dt);
    for (const auto& name : LEG_NAMES) {
      const Vec3 ft = out.at(name).foot_target;
      EXPECT_NEAR(ft[0], nominal.at(name)[0], 1e-12);
      EXPECT_NEAR(ft[1], nominal.at(name)[1], 1e-12);
      // Body-frame z increases (less negative) as the body descends.
      EXPECT_GE(ft[2], prev_z[name] - 1e-12);
      EXPECT_TRUE(out.at(name).stance);
      prev_z[name] = ft[2];
    }
  }

  // Snap to the LOWER_BODY endpoint at the boundary tick.
  EXPECT_EQ(ctrl.state(), FoldState::LIFT_FEET);
  for (const auto& name : LEG_NAMES) {
    // LOWER_BODY endpoint == ground target.
    EXPECT_NEAR(prev_z[name], ground_target(name)[2], 1e-12);
  }
}

TEST(Fold, FirstLiftFeetTickOnlyFirstReversePairMoves) {
  FoldController ctrl = make_controller();
  const double dt = 0.005;

  // Drain LOWER_BODY.
  while (ctrl.state() == FoldState::LOWER_BODY) {
    ctrl.update(dt);
  }
  ASSERT_EQ(ctrl.state(), FoldState::LIFT_FEET);

  const auto out = ctrl.update(dt);
  // First active pair under the reversed order is the LAST entry of PAIR_ORDER
  // (i.e. {"r_front", "l_rear"}).
  const auto& active = pair_order_reversed()[0];
  for (const auto& name : active) {
    // Active legs are mid-arc — not yet at initial.
    EXPECT_FALSE(out.at(name).stance);
    EXPECT_NE(out.at(name).foot_target, ground_target(name));
  }
  for (const auto& name : LEG_NAMES) {
    if (name == active[0] || name == active[1]) {
      continue;
    }
    // All other legs sit at their ground target (LOWER_BODY endpoint).
    expect_vec_near(out.at(name).foot_target, ground_target(name), 1e-12, name);
    EXPECT_TRUE(out.at(name).stance);
  }
}

TEST(Fold, LiftFeetPairsCompleteInReversedOrderAndSnapToInitial) {
  FoldController ctrl = make_controller();
  const auto initial = initial_stance();
  const double dt = 0.02;

  // Drain LOWER_BODY.
  while (ctrl.state() == FoldState::LOWER_BODY) {
    ctrl.update(dt);
  }

  auto drain_pair = [&](const std::array<std::string, 2>& expected_active) {
    std::map<std::string, LegOutput> out;
    const int max_iter = static_cast<int>(PAIR_TIME / dt) + 5;
    for (int i = 0; i < max_iter; ++i) {
      out = ctrl.update(dt);
      if (ctrl.state() == FoldState::LIFT_FEET) {
        bool still_in_pair = false;
        for (const auto& n : expected_active) {
          if (out.at(n).foot_target != initial.at(n)) {
            still_in_pair = true;
          }
        }
        if (!still_in_pair) {
          break;
        }
      } else {
        break;
      }
    }
    std::map<std::string, Vec3> snap;
    for (const auto& n : LEG_NAMES) {
      snap[n] = out.at(n).foot_target;
    }
    return snap;
  };

  const auto& rev = pair_order_reversed();

  // Pair 1 — reverse[0] == {"r_front", "l_rear"}; others still at ground.
  auto snap = drain_pair(rev[0]);
  for (const auto& name : rev[0]) {
    expect_vec_near(snap[name], initial.at(name), 1e-9, name);
  }
  for (const auto& name : rev[1]) {
    expect_vec_near(snap[name], ground_target(name), 1e-12, name);
  }
  for (const auto& name : rev[2]) {
    expect_vec_near(snap[name], ground_target(name), 1e-12, name);
  }

  // Pair 2 — reverse[1] == {"l_front", "r_rear"}.
  snap = drain_pair(rev[1]);
  for (const auto& name : rev[0]) {
    expect_vec_near(snap[name], initial.at(name), 1e-9, name);
  }
  for (const auto& name : rev[1]) {
    expect_vec_near(snap[name], initial.at(name), 1e-9, name);
  }
  for (const auto& name : rev[2]) {
    expect_vec_near(snap[name], ground_target(name), 1e-12, name);
  }

  // Pair 3 — middle pair; controller advances to DONE.
  snap = drain_pair(rev[2]);
  for (const auto& name : LEG_NAMES) {
    expect_vec_near(snap[name], initial.at(name), 1e-9, name);
  }
  EXPECT_EQ(ctrl.state(), FoldState::DONE);
}

TEST(Fold, DoneStateEmitsInitialForever) {
  FoldController ctrl = make_controller();
  const auto initial = initial_stance();
  for (int i = 0; i < 500; ++i) {
    ctrl.update(0.02);
    if (ctrl.state() == FoldState::DONE) {
      break;
    }
  }
  ASSERT_EQ(ctrl.state(), FoldState::DONE);
  const auto out = ctrl.update(0.02);
  for (const auto& name : LEG_NAMES) {
    EXPECT_EQ(out.at(name).foot_target, initial.at(name));
    EXPECT_TRUE(out.at(name).stance);
    EXPECT_EQ(out.at(name).phase, 0.0);
  }
}

TEST(Fold, MissingInitialStanceRaises) {
  ControllerArgs args;
  args.initial_stance = initial_stance();
  args.initial_stance.erase("l_rear");
  EXPECT_THROW(make_controller(args), std::invalid_argument);
}

TEST(Fold, NonpositiveTimingsRaise) {
  {
    ControllerArgs args;
    args.pair_swing_time = 0.0;
    EXPECT_THROW(make_controller(args), std::invalid_argument);
  }
  {
    ControllerArgs args;
    args.lift_body_time = -0.1;
    EXPECT_THROW(make_controller(args), std::invalid_argument);
  }
}

// --- Engine integration tests ---------------------------------------------

TEST(Fold, StartFoldRejectedFromFolded) {
  Engine engine = make_engine();
  EXPECT_EQ(engine.state(), EngineState::FOLDED);
  EXPECT_FALSE(engine.start_fold());
  EXPECT_EQ(engine.state(), EngineState::FOLDED);
}

TEST(Fold, StartFoldRejectedMidInitialize) {
  Engine engine = make_engine();
  engine.start_initialize();
  EXPECT_EQ(engine.state(), EngineState::INITIALIZE);
  EXPECT_FALSE(engine.start_fold());
  EXPECT_EQ(engine.state(), EngineState::INITIALIZE);
}

TEST(Fold, StartFoldRejectedDuringGait) {
  // The user requirement: fold can only happen from STAND. A press mid-walk
  // must NOT abort the gait.
  Engine engine = make_engine();
  drive_to_stand(engine);
  // Step into GAIT with a non-zero cmd_vel.
  engine.update(0.02, {0.1, 0.0}, 0.0);
  EXPECT_TRUE(engine.state() == EngineState::ENGAGING ||
              engine.state() == EngineState::GAIT);
  EXPECT_FALSE(engine.start_fold());
  EXPECT_TRUE(engine.state() == EngineState::ENGAGING ||
              engine.state() == EngineState::GAIT);
}

TEST(Fold, StartFoldFromStandTransitionsToFolding) {
  Engine engine = make_engine();
  drive_to_stand(engine);
  EXPECT_TRUE(engine.start_fold());
  EXPECT_EQ(engine.state(), EngineState::FOLDING);
  // Re-trigger from FOLDING is a no-op.
  EXPECT_FALSE(engine.start_fold());
  EXPECT_EQ(engine.state(), EngineState::FOLDING);
}

TEST(Fold, EngineCompletesFoldThenSettlesInFolded) {
  Engine engine = make_engine();
  const auto initial = initial_stance();
  drive_to_stand(engine);
  ASSERT_TRUE(engine.start_fold());

  // Total ladder: LIFT_TIME + 3 * PAIR_TIME = 0.4 s; one extra tick to observe
  // the FOLDED transition. dt = 0.02 => <= 25 ticks.
  for (int i = 0; i < 60; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::FOLDED) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::FOLDED);
  const auto out = engine.update(0.02, {0.0, 0.0}, 0.0);
  for (const auto& name : LEG_NAMES) {
    EXPECT_EQ(out.at(name).foot_target, initial.at(name));
    EXPECT_TRUE(out.at(name).stance);
  }
}

TEST(Fold, CmdVelDuringFoldDoesNotShortCircuitTheLadder) {
  // Sending a non-zero cmd_vel during FOLDING must not bail to ENGAGING /
  // STOPPING — the warm-shutdown commits to completion, mirroring INITIALIZE's
  // behaviour.
  Engine baseline = make_engine();
  Engine with_cmd = make_engine();
  drive_to_stand(baseline);
  drive_to_stand(with_cmd);
  baseline.start_fold();
  with_cmd.start_fold();

  const double dt = 0.02;
  std::vector<std::map<std::string, Vec3>> baseline_out;
  std::vector<std::map<std::string, Vec3>> cmd_out;
  const int max_iter =
      static_cast<int>((LIFT_TIME + 3 * PAIR_TIME) / dt) + 1;
  for (int i = 0; i < max_iter; ++i) {
    const auto out_a = baseline.update(dt, {0.0, 0.0}, 0.0);
    const auto out_b = with_cmd.update(dt, {0.2, 0.0}, 0.0);
    std::map<std::string, Vec3> a;
    std::map<std::string, Vec3> b;
    for (const auto& n : LEG_NAMES) {
      a[n] = out_a.at(n).foot_target;
      b[n] = out_b.at(n).foot_target;
    }
    baseline_out.push_back(a);
    cmd_out.push_back(b);
    if (with_cmd.state() != EngineState::FOLDING) {
      EXPECT_EQ(with_cmd.state(), EngineState::FOLDED);
      break;
    }
    EXPECT_EQ(with_cmd.state(), EngineState::FOLDING);
  }

  ASSERT_EQ(baseline_out.size(), cmd_out.size());
  for (std::size_t i = 0; i < baseline_out.size(); ++i) {
    for (const auto& n : LEG_NAMES) {
      EXPECT_EQ(baseline_out[i].at(n), cmd_out[i].at(n))
          << "tick " << i << " leg " << n;
    }
  }
}

TEST(Fold, EngineCanReInitializeAfterFold) {
  // After a fold round-trip the engine should accept a second
  // start_initialize() and run a fresh ladder — verifies the controllers are
  // rebuilt rather than left half-consumed.
  Engine engine = make_engine();
  drive_to_stand(engine);
  engine.start_fold();
  for (int i = 0; i < 60; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::FOLDED) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::FOLDED);

  EXPECT_TRUE(engine.start_initialize());
  for (int i = 0; i < 200; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::STAND) {
      break;
    }
  }
  EXPECT_EQ(engine.state(), EngineState::STAND);
}

TEST(Fold, StartFoldFromPausingIsANoop) {
  // Mid-pause the engine is lowering the airborne legs — a fold press during
  // the transition must be ignored so the pause / paused flow finishes cleanly.
  Engine engine = make_engine();
  drive_to_stand(engine);
  // Drive into GAIT, then release the stick. pause_debounce_delay=0 in this
  // config so the first cmd_zero tick is enough to enter PAUSING.
  engine.update(0.02, {0.1, 0.0}, 0.0);
  // Walk several ticks of GAIT, then drop to zero.
  for (int i = 0; i < 5; ++i) {
    engine.update(0.02, {0.1, 0.0}, 0.0);
  }
  engine.update(0.02, {0.0, 0.0}, 0.0);
  // The engine should now be PAUSING (or PAUSED if every airborne leg landed in
  // one tick).
  if (engine.state() == EngineState::PAUSING) {
    EXPECT_FALSE(engine.start_fold());
    EXPECT_EQ(engine.state(), EngineState::PAUSING);
  }
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
