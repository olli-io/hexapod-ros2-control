// gtest port of hexa_gait/test/test_initialize.py — exercises the pure C++
// InitializeController (the cold-start PLACE_FEET / LIFT_BODY / DONE ladder,
// merged into stand_transition.hpp) plus the FOLDED -> INITIALIZE -> STAND
// Engine integration. Behavioural parity with the Python suite. The Python
// `hexa_gait.initialize` module lives in hexa_gait_cpp/stand_transition.hpp.
//
// Python kwargs -> positional C++ ctor args; Python dict[str, tuple] foot
// targets -> std::map<std::string, Vec3>; pytest.approx(x, abs=e) -> EXPECT_NEAR;
// Python ValueError -> std::invalid_argument. The engine is built via the
// strategies() registry with an explicit strategy_name (the C++ Engine ctor
// requires it), mirroring make_gait_engine in test_engine.cpp.

#include <gtest/gtest.h>

#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

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

// Exact / approximate Vec3 comparisons — Python compares foot-target tuples with
// == (exact) and pytest.approx (abs tolerance).
bool vec_eq(const Vec3& a, const Vec3& b) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

void expect_vec_near(const Vec3& a, const Vec3& b, double eps) {
  EXPECT_NEAR(a[0], b[0], eps);
  EXPECT_NEAR(a[1], b[1], eps);
  EXPECT_NEAR(a[2], b[2], eps);
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

std::map<std::string, Vec3> initial_stance() {
  // Feet folded up well above the body, in some asymmetric layout — verifies the
  // controller never accidentally relies on initial_stance matching
  // nominal_stance in XY.
  return {
      {"l_front", Vec3(0.05, 0.04, 0.08)},
      {"r_front", Vec3(0.05, -0.04, 0.08)},
      {"l_middle", Vec3(0.0, 0.05, 0.08)},
      {"r_middle", Vec3(0.0, -0.05, 0.08)},
      {"l_rear", Vec3(-0.05, 0.04, 0.08)},
      {"r_rear", Vec3(-0.05, -0.04, 0.08)},
  };
}

// PLACE_FEET targets land the foot 1 mm above the floor (world z =
// +PLACE_FEET_CLEARANCE) so the swing arc never scuffs the ground; body-frame z
// therefore = -coxa_to_bottom + place_feet_clearance.
Vec3 ground_target(const std::string& name) {
  const Vec3 nom = nominal_stance().at(name);
  return Vec3(nom[0], nom[1], -COXA_TO_BOTTOM + PLACE_FEET_CLEARANCE);
}

// Port of _controller(**overrides): swing_width=0.0, controller_dt=0.02. The
// three overridden kwargs used by the suite (initial_stance, pair_swing_time,
// lift_body_time) are exposed as optional args.
InitializeController make_controller(
    const std::map<std::string, Vec3>& initial = initial_stance(),
    double pair_swing_time = PAIR_TIME, double lift_body_time = LIFT_TIME) {
  return InitializeController(initial, nominal_stance(), COXA_TO_BOTTOM,
                              pair_swing_time, lift_body_time, SWING_CLEARANCE,
                              PLACE_FEET_CLEARANCE, 0.0, 0.02);
}

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
  // Reseat knobs (unused by these tests — Engine is constructed without
  // leg_specs / reseat_geometry).
  c.reseat_pose_settle_delay = 0.1;
  c.reseat_height_change_threshold = 0.001;
  c.reseat_pair_swing_time = 0.1;
  c.reseat_pair_dwell_time = 0.0;
  c.reseat_swing_clearance = 0.02;
  return c;
}

// Port of _engine(): Tripod strategy built from the registry, explicit
// strategy_name "tripod".
Engine make_engine() {
  return Engine(engine_config(), strategies().at("tripod")(), "tripod",
                nominal_stance(), initial_stance(), COXA_TO_BOTTOM,
                leg_contexts());
}

// --- InitializeController unit tests --------------------------------------

TEST(Initialize, PairOrderIsMiddleThenDiagonals) {
  // Documented behaviour for static-stability reasons (middle pair first to keep
  // CoM centred, then each diagonal). Regression guard against reordering.
  const std::array<std::array<std::string, 2>, 3> expected = {{
      {"l_middle", "r_middle"},
      {"l_front", "r_rear"},
      {"r_front", "l_rear"},
  }};
  EXPECT_EQ(PAIR_ORDER, expected);
}

TEST(Initialize, ControllerStartsInPlaceFeet) {
  InitializeController ctrl = make_controller();
  EXPECT_EQ(ctrl.state(), InitializeState::PLACE_FEET);
  EXPECT_FALSE(ctrl.done());
}

TEST(Initialize, FirstTickOnlyActivePairMoves) {
  InitializeController ctrl = make_controller();
  const auto initial = initial_stance();
  const auto out = ctrl.update(0.02);
  EXPECT_EQ(ctrl.state(), InitializeState::PLACE_FEET);
  // The first active pair is middle left/right.
  for (const std::string& name : {"l_middle", "r_middle"}) {
    EXPECT_FALSE(vec_eq(out.at(name).foot_target, initial.at(name)));
    EXPECT_FALSE(out.at(name).stance);
  }
  // All other legs sit at exactly their initial_stance entry.
  for (const auto& name : LEG_NAMES) {
    if (name == "l_middle" || name == "r_middle") {
      continue;
    }
    EXPECT_TRUE(vec_eq(out.at(name).foot_target, initial.at(name)));
    EXPECT_TRUE(out.at(name).stance);
  }
}

TEST(Initialize, PairsCompleteInOrderAndSnapToGroundTargets) {
  InitializeController ctrl = make_controller();
  const auto initial = initial_stance();
  const double dt = 0.02;

  auto drain_pair =
      [&](const std::array<std::string, 2>& expected_active)
      -> std::map<std::string, Vec3> {
    std::map<std::string, LegOutput> out;
    for (int i = 0; i < static_cast<int>(PAIR_TIME / dt) + 5; ++i) {
      out = ctrl.update(dt);
      // Active pair stays airborne while the arc plays out.
      if (ctrl.state() == InitializeState::PLACE_FEET) {
        bool still_in_pair = false;
        for (const auto& n : expected_active) {
          if (!vec_eq(out.at(n).foot_target, ground_target(n))) {
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

  // Pair 1 — middle pair lands on ground; everyone else still at initial.
  auto snap = drain_pair({"l_middle", "r_middle"});
  for (const std::string& name : {"l_middle", "r_middle"}) {
    expect_vec_near(snap.at(name), ground_target(name), 1e-9);
  }
  for (const std::string& name : {"l_front", "r_front", "l_rear", "r_rear"}) {
    EXPECT_TRUE(vec_eq(snap.at(name), initial.at(name)));
  }

  // Pair 2 — front-left + rear-right diagonal lands; other diagonal at initial.
  snap = drain_pair({"l_front", "r_rear"});
  for (const std::string& name : {"l_middle", "r_middle", "l_front", "r_rear"}) {
    expect_vec_near(snap.at(name), ground_target(name), 1e-9);
  }
  for (const std::string& name : {"r_front", "l_rear"}) {
    EXPECT_TRUE(vec_eq(snap.at(name), initial.at(name)));
  }

  // Pair 3 — other diagonal lands; controller advances to LIFT_BODY.
  snap = drain_pair({"r_front", "l_rear"});
  for (const auto& name : LEG_NAMES) {
    expect_vec_near(snap.at(name), ground_target(name), 1e-9);
  }
  EXPECT_EQ(ctrl.state(), InitializeState::LIFT_BODY);
}

TEST(Initialize, LiftBodyMonotonicDescentInBodyFrameWithXyHeld) {
  InitializeController ctrl = make_controller();
  const auto nominal = nominal_stance();
  const double dt = 0.005;

  // Drain PLACE_FEET first; that's covered separately above.
  while (ctrl.state() == InitializeState::PLACE_FEET) {
    ctrl.update(dt);
  }
  ASSERT_EQ(ctrl.state(), InitializeState::LIFT_BODY);

  // LIFT_BODY starts from the PLACE_FEET endpoint (1 mm above floor).
  std::map<std::string, double> prev_z;
  for (const auto& n : LEG_NAMES) {
    prev_z[n] = -COXA_TO_BOTTOM + PLACE_FEET_CLEARANCE;
  }
  while (ctrl.state() == InitializeState::LIFT_BODY) {
    const auto out = ctrl.update(dt);
    for (const auto& name : LEG_NAMES) {
      const double x = out.at(name).foot_target[0];
      const double y = out.at(name).foot_target[1];
      const double z = out.at(name).foot_target[2];
      // XY held exactly at standing positions throughout LIFT_BODY.
      EXPECT_NEAR(x, nominal.at(name)[0], 1e-12);
      EXPECT_NEAR(y, nominal.at(name)[1], 1e-12);
      // Body-frame z is monotonically more negative (foot pressed further away
      // from body => world-frame body rising).
      EXPECT_LE(z, prev_z.at(name) + 1e-12);
      EXPECT_TRUE(out.at(name).stance);
      prev_z[name] = z;
    }
  }

  // Final tick snaps to nominal exactly. (Python calls update() once per leg in
  // a loop; DONE is idempotent, so a single call is equivalent.)
  const auto final_out = ctrl.update(dt);
  for (const auto& name : LEG_NAMES) {
    expect_vec_near(final_out.at(name).foot_target, nominal.at(name), 1e-12);
  }
  EXPECT_EQ(ctrl.state(), InitializeState::DONE);
  EXPECT_TRUE(ctrl.done());
}

TEST(Initialize, DoneStateEmitsNominalForever) {
  InitializeController ctrl = make_controller();
  const auto nominal = nominal_stance();
  for (int i = 0; i < 500; ++i) {
    ctrl.update(0.02);
    if (ctrl.state() == InitializeState::DONE) {
      break;
    }
  }
  ASSERT_EQ(ctrl.state(), InitializeState::DONE);
  const auto out = ctrl.update(0.02);
  for (const auto& name : LEG_NAMES) {
    EXPECT_TRUE(vec_eq(out.at(name).foot_target, nominal.at(name)));
    EXPECT_TRUE(out.at(name).stance);
    EXPECT_EQ(out.at(name).phase, 0.0);
  }
}

TEST(Initialize, MissingInitialStanceRaises) {
  auto incomplete = initial_stance();
  incomplete.erase("l_rear");
  try {
    make_controller(incomplete);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("initial_stance missing legs"),
              std::string::npos)
        << "message was: " << e.what();
  }
}

TEST(Initialize, NonpositiveTimingsRaise) {
  try {
    make_controller(initial_stance(), /*pair_swing_time=*/0.0);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("pair_swing_time"), std::string::npos)
        << "message was: " << e.what();
  }
  try {
    make_controller(initial_stance(), PAIR_TIME, /*lift_body_time=*/-0.1);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("lift_body_time"), std::string::npos)
        << "message was: " << e.what();
  }
}

// --- Engine integration tests ---------------------------------------------

TEST(Initialize, EngineStartsInFoldedNotInitialize) {
  // Operator-gated cold start: the engine waits in FOLDED until
  // start_initialize() is called (typically from the joystick start button via
  // /gait/initialize), so power-on does not move the robot.
  Engine engine = make_engine();
  EXPECT_EQ(engine.state(), EngineState::FOLDED);
}

TEST(Initialize, FoldedEmitsInitialStanceAndIgnoresCmdVel) {
  Engine engine = make_engine();
  const auto initial = initial_stance();
  // Multiple ticks with non-zero cmd_vel must all return the folded foot
  // positions and leave the engine in FOLDED.
  for (int i = 0; i < 5; ++i) {
    const auto out = engine.update(0.02, {0.3, 0.2}, 0.4);
    EXPECT_EQ(engine.state(), EngineState::FOLDED);
    for (const auto& name : LEG_NAMES) {
      EXPECT_TRUE(vec_eq(out.at(name).foot_target, initial.at(name)));
      EXPECT_TRUE(out.at(name).stance);
    }
  }
}

TEST(Initialize, StartInitializeTransitionsFoldedToInitialize) {
  Engine engine = make_engine();
  EXPECT_TRUE(engine.start_initialize());
  EXPECT_EQ(engine.state(), EngineState::INITIALIZE);
  // Re-trigger from INITIALIZE is a no-op.
  EXPECT_FALSE(engine.start_initialize());
  EXPECT_EQ(engine.state(), EngineState::INITIALIZE);
}

TEST(Initialize, StartInitializeFromStandIsANoop) {
  // After the cold-start has run once and the engine settled at STAND,
  // re-pressing the start button must not kick off another initialize cycle —
  // the legs are already standing and re-running PLACE_FEET would lift them from
  // nominal back to ground.
  Engine engine = make_engine();
  engine.start_initialize();
  for (int i = 0; i < 60; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::STAND) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::STAND);
  EXPECT_FALSE(engine.start_initialize());
  EXPECT_EQ(engine.state(), EngineState::STAND);
}

TEST(Initialize, EngineCompletesInitializeThenEntersStand) {
  Engine engine = make_engine();
  const auto nominal = nominal_stance();
  engine.start_initialize();
  // Total ladder: 3 * PAIR_TIME + LIFT_TIME = 0.4 s; one extra tick to observe
  // the STAND transition. dt = 0.02 => <= 25 ticks.
  for (int i = 0; i < 60; ++i) {
    engine.update(0.02, {0.0, 0.0}, 0.0);
    if (engine.state() == EngineState::STAND) {
      break;
    }
  }
  ASSERT_EQ(engine.state(), EngineState::STAND);
  const auto out = engine.update(0.02, {0.0, 0.0}, 0.0);
  for (const auto& name : LEG_NAMES) {
    EXPECT_TRUE(vec_eq(out.at(name).foot_target, nominal.at(name)));
    EXPECT_TRUE(out.at(name).stance);
  }
}

TEST(Initialize, CmdVelDuringInitializeDoesNotShortCircuitTheLadder) {
  // Sending a non-zero cmd_vel during INITIALIZE must not bail to ENGAGING /
  // STOPPING — the cold-start ladder runs to completion. Mirrors STOPPING's
  // commit-to-completion contract.
  Engine engine_baseline = make_engine();
  Engine engine_with_cmd = make_engine();
  engine_baseline.start_initialize();
  engine_with_cmd.start_initialize();

  const double dt = 0.02;
  std::vector<std::map<std::string, Vec3>> baseline_out;
  std::vector<std::map<std::string, Vec3>> cmd_out;
  for (int i = 0; i < static_cast<int>((3 * PAIR_TIME + LIFT_TIME) / dt) + 1;
       ++i) {
    const auto out_a = engine_baseline.update(dt, {0.0, 0.0}, 0.0);
    const auto out_b = engine_with_cmd.update(dt, {0.2, 0.0}, 0.0);
    std::map<std::string, Vec3> a;
    std::map<std::string, Vec3> b;
    for (const auto& n : LEG_NAMES) {
      a[n] = out_a.at(n).foot_target;
      b[n] = out_b.at(n).foot_target;
    }
    baseline_out.push_back(a);
    cmd_out.push_back(b);
    // Mid-INITIALIZE, the engine ignores cmd_vel and stays in INITIALIZE —
    // never ENGAGING / STOPPING — until the ladder ends.
    if (engine_with_cmd.state() != EngineState::INITIALIZE) {
      EXPECT_EQ(engine_with_cmd.state(), EngineState::STAND);
      break;
    }
    EXPECT_EQ(engine_with_cmd.state(), EngineState::INITIALIZE);
  }

  // Foot targets emitted under the two cmd streams are identical tick-for-tick
  // — cmd_vel had no effect inside INITIALIZE.
  ASSERT_EQ(baseline_out.size(), cmd_out.size());
  for (std::size_t i = 0; i < baseline_out.size(); ++i) {
    for (const auto& n : LEG_NAMES) {
      EXPECT_TRUE(vec_eq(baseline_out[i].at(n), cmd_out[i].at(n)))
          << "tick " << i << " leg " << n;
    }
  }
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
