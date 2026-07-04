// gtest port of hexa_gait/test/test_reseat.py — exercises the reseat ladder.
//
// Two layers:
//
// 1. reseat_nominal_stance — pure geometric function. At Δz=0 it reproduces the
//    YAML standing nominal stance; positive Δz pulls the feet inward radially
//    (femur drops, so the leg's horizontal reach shrinks); negative Δz pushes
//    them outward. The body-frame z stays pinned to the default value regardless
//    of Δz — the lift lives in pose.z, the gait nominal only tracks XY.
// 2. ReseatController — pair-sequenced ladder. Confirms the pair order, the
//    per-pair completion timing, and the per-pair targets.
//
// The geometric tests resolve the real hexa_description YAML via ament_index and
// GTEST_SKIP() when the package is not installed (mirrors test_real_config.cpp).

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>  // PackageNotFoundError
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hexa_gait_cpp/engine.hpp"           // reseat_geometry_from_yaml
#include "hexa_gait_cpp/kinematics.hpp"       // kin::
#include "hexa_gait_cpp/reseat.hpp"
#include "hexa_gait_cpp/stand_transition.hpp"  // PAIR_ORDER
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Component-wise near comparison (Python's pytest.approx / == on tuples).
void expect_vec_near(const Vec3& a, const Vec3& b, double tol) {
  EXPECT_NEAR(a[0], b[0], tol);
  EXPECT_NEAR(a[1], b[1], tol);
  EXPECT_NEAR(a[2], b[2], tol);
}

// Bit-exact equality — mirrors the Python `==` on the held/snapped positions.
bool vec_exact(const Vec3& a, const Vec3& b) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

// ── reseat_nominal_stance real-config helpers ──────────────────────────────

std::string descriptionConfigDir() {
  try {
    return ament_index_cpp::get_package_share_directory("hexa_description") +
           "/config";
  } catch (const ament_index_cpp::PackageNotFoundError&) {
    return "";
  }
}

std::map<std::string, kin::LegSpec> legSpecs(const std::string& dir) {
  return kin::load_leg_specs(dir + "/geometry.yaml");
}

ReseatGeometry yamlGeometry(const std::string& dir) {
  return reseat_geometry_from_yaml(dir + "/geometry.yaml", kin::StandingPoseDeg{});
}

std::map<std::string, Vec3> yamlNominal(const std::string& dir) {
  const auto legs = legSpecs(dir);
  const auto angles =
      kin::load_standing_pose(kin::StandingPoseDeg{}, dir + "/geometry.yaml");
  std::map<std::string, Vec3> out;
  for (const auto& n : LEG_NAMES) {
    out[n] = kin::leg_to_body(kin::forward_kinematics(angles, legs.at(n)),
                              legs.at(n));
  }
  return out;
}

TEST(ReseatNominalStance, AtZeroHeightReturnsYamlNominalStance) {
  // Sanity check: at Δz = 0 the reseat function reproduces the YAML-derived
  // nominal stance to numerical precision.
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const auto out = reseat_nominal_stance(0.0, yamlGeometry(dir), legSpecs(dir));
  const auto nominal = yamlNominal(dir);
  for (const auto& name : LEG_NAMES) {
    expect_vec_near(out.at(name), nominal.at(name), 1e-9);
  }
}

TEST(ReseatNominalStance, PositiveHeightGrowsRadialDistance) {
  // Body lifted ⇒ feet need to be deeper relative to body ⇒ with the tibia held
  // at its default near-vertical lean, the femur drops toward horizontal, which
  // extends the foot's radial reach outward in the leg frame. The body-frame
  // foot therefore sits *further* from the hip's X/Y projection — legs splay out.
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const auto legs = legSpecs(dir);
  const auto geom = yamlGeometry(dir);
  const auto nominal_default = reseat_nominal_stance(0.0, geom, legs);
  const auto nominal_lifted = reseat_nominal_stance(0.03, geom, legs);
  for (const auto& name : LEG_NAMES) {
    const double mx = legs.at(name).mount_xyz[0];
    const double my = legs.at(name).mount_xyz[1];
    const double r_default = std::hypot(nominal_default.at(name)[0] - mx,
                                        nominal_default.at(name)[1] - my);
    const double r_lifted = std::hypot(nominal_lifted.at(name)[0] - mx,
                                       nominal_lifted.at(name)[1] - my);
    EXPECT_GT(r_lifted, r_default) << name;
  }
}

TEST(ReseatNominalStance, NegativeHeightShrinksRadialDistance) {
  // Inverse of the positive case: body lowered ⇒ feet closer to body in z ⇒
  // femur rises (more above horizontal) ⇒ horizontal reach shrinks ⇒ legs tuck.
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const auto legs = legSpecs(dir);
  const auto geom = yamlGeometry(dir);
  const auto nominal_default = reseat_nominal_stance(0.0, geom, legs);
  const auto nominal_dropped = reseat_nominal_stance(-0.03, geom, legs);
  for (const auto& name : LEG_NAMES) {
    const double mx = legs.at(name).mount_xyz[0];
    const double my = legs.at(name).mount_xyz[1];
    const double r_default = std::hypot(nominal_default.at(name)[0] - mx,
                                        nominal_default.at(name)[1] - my);
    const double r_dropped = std::hypot(nominal_dropped.at(name)[0] - mx,
                                        nominal_dropped.at(name)[1] - my);
    EXPECT_LT(r_dropped, r_default) << name;
  }
}

TEST(ReseatNominalStance, BodyFrameZStaysAtDefaultAtAnyHeight) {
  // Critical invariant — the body lift lives entirely in pose.z, NOT in the
  // gait's nominal_stance. The kinematics chain's apply_body_pose subtracts
  // pose.z; the gait nominal must match the at-zero-height value so the net
  // leg-frame foot depth ends up at default_foot_depth + Δz.
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const auto legs = legSpecs(dir);
  const auto geom = yamlGeometry(dir);
  const double base_z = reseat_nominal_stance(0.0, geom, legs).at(LEG_NAMES[0])[2];
  for (double dz : {-0.03, -0.01, 0.0, 0.01, 0.03}) {
    const auto nominal = reseat_nominal_stance(dz, geom, legs);
    for (const auto& name : LEG_NAMES) {
      EXPECT_NEAR(nominal.at(name)[2], base_z, 1e-12) << name;
    }
  }
}

TEST(ReseatNominalStance, InfeasibleHeightRaises) {
  // A target so far from default that arcsin would saturate. default_foot_depth
  // ≈ 0.085 m; the maximum is roughly tibia_len*cos(θ_t) + femur_len ≈ 0.13 +
  // 0.08 ≈ 0.21 m. So Δz of 0.20 m exceeds the limit.
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const auto legs = legSpecs(dir);
  const auto geom = yamlGeometry(dir);
  EXPECT_THROW(reseat_nominal_stance(0.20, geom, legs), std::invalid_argument);
}

TEST(ReseatNominalStance, DefaultGeometryFromPoseMatchesYamlHelper) {
  // The two ways to build the geometry — direct via the leaf function, or
  // through the engine-level YAML helper — must agree.
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const auto legs = legSpecs(dir);
  const auto angles =
      kin::load_standing_pose(kin::StandingPoseDeg{}, dir + "/geometry.yaml");
  const ReseatGeometry direct =
      default_geometry_from_pose(angles, legs.at(LEG_NAMES[0]));
  const ReseatGeometry yaml_helper =
      reseat_geometry_from_yaml(dir + "/geometry.yaml", kin::StandingPoseDeg{});
  EXPECT_EQ(direct.coxa_len, yaml_helper.coxa_len);
  EXPECT_EQ(direct.femur_len, yaml_helper.femur_len);
  EXPECT_EQ(direct.tibia_len, yaml_helper.tibia_len);
  EXPECT_EQ(direct.tibia_from_vertical, yaml_helper.tibia_from_vertical);
  EXPECT_EQ(direct.default_foot_depth, yaml_helper.default_foot_depth);
}

// ── ReseatController ───────────────────────────────────────────────────────

// Symmetric six-leg layout (matches test_engine.py geometry); the exact numbers
// don't matter — only that the controller ticks through pairs correctly.
std::map<std::string, Vec3> stance(double z = -0.10) {
  return {
      {"l_front", Vec3(0.15, 0.10, z)},
      {"r_front", Vec3(0.15, -0.10, z)},
      {"l_middle", Vec3(0.0, 0.12, z)},
      {"r_middle", Vec3(0.0, -0.12, z)},
      {"l_rear", Vec3(-0.15, 0.10, z)},
      {"r_rear", Vec3(-0.15, -0.10, z)},
  };
}

// Stance with each foot translated radially outward by dx in body frame.
std::map<std::string, Vec3> shifted_stance(double dx) {
  std::map<std::string, Vec3> out;
  for (const auto& [name, xyz] : stance()) {
    // Push each foot away from the body centre along its (x, y).
    const double nx = xyz[0];
    const double ny = xyz[1];
    const double nz = xyz[2];
    const double r = std::hypot(nx, ny);
    if (r > 0.0) {
      const double scale = (r + dx) / r;
      out[name] = Vec3(nx * scale, ny * scale, nz);
    } else {
      out[name] = xyz;
    }
  }
  return out;
}

struct CtrlArgs {
  std::map<std::string, Vec3> current_stance = stance();
  std::map<std::string, Vec3> target_stance = shifted_stance(0.02);
  double pair_swing_time = 0.1;
  double pair_dwell_time = 0.0;
  double swing_clearance = 0.02;
  double controller_dt = 0.02;
};

ReseatController makeController(const CtrlArgs& a = CtrlArgs{}) {
  return ReseatController(a.current_stance, a.target_stance, a.pair_swing_time,
                          a.pair_dwell_time, a.swing_clearance,
                          a.controller_dt);
}

TEST(ReseatController, StartsNotDone) {
  auto ctrl = makeController();
  EXPECT_FALSE(ctrl.done());
}

TEST(ReseatController, FirstTickOnlyFirstPairSwings) {
  auto ctrl = makeController();
  const auto current = stance();
  const auto out = ctrl.update(0.02);
  const auto& active = PAIR_ORDER[0];  // ("l_middle", "r_middle")
  for (const auto& name : active) {
    EXPECT_FALSE(vec_exact(out.at(name).foot_target, current.at(name))) << name;
    EXPECT_FALSE(out.at(name).stance) << name;
  }
  for (const auto& name : LEG_NAMES) {
    if (name == active[0] || name == active[1]) {
      continue;
    }
    EXPECT_TRUE(vec_exact(out.at(name).foot_target, current.at(name))) << name;
    EXPECT_TRUE(out.at(name).stance) << name;
  }
}

TEST(ReseatController, PairsCompleteInOrderAndSnapToTargets) {
  const auto target = shifted_stance(0.02);
  CtrlArgs a;
  a.target_stance = target;
  auto ctrl = makeController(a);
  const double dt = 0.02;
  const auto current = stance();

  auto drain =
      [&](const std::array<std::string, 2>& active) -> std::map<std::string, Vec3> {
    for (int i = 0; i < 20; ++i) {
      const auto out = ctrl.update(dt);
      bool done = true;
      for (const auto& n : active) {
        if (!vec_exact(out.at(n).foot_target, target.at(n))) {
          done = false;
        }
      }
      if (done) {
        std::map<std::string, Vec3> snap;
        for (const auto& n : LEG_NAMES) {
          snap[n] = out.at(n).foot_target;
        }
        return snap;
      }
    }
    ADD_FAILURE() << "pair did not complete";
    return {};
  };

  // Pair 1: middle pair lands on target; others still at current stance.
  auto snap = drain(PAIR_ORDER[0]);
  for (const auto& name : PAIR_ORDER[0]) {
    expect_vec_near(snap.at(name), target.at(name), 1e-9);
  }
  for (const auto& name : PAIR_ORDER[1]) {
    EXPECT_TRUE(vec_exact(snap.at(name), current.at(name))) << name;
  }
  for (const auto& name : PAIR_ORDER[2]) {
    EXPECT_TRUE(vec_exact(snap.at(name), current.at(name))) << name;
  }

  // Pair 2: first diagonal lands; other diagonal still at current.
  snap = drain(PAIR_ORDER[1]);
  for (const auto& name : PAIR_ORDER[0]) {
    expect_vec_near(snap.at(name), target.at(name), 1e-9);
  }
  for (const auto& name : PAIR_ORDER[1]) {
    expect_vec_near(snap.at(name), target.at(name), 1e-9);
  }
  for (const auto& name : PAIR_ORDER[2]) {
    EXPECT_TRUE(vec_exact(snap.at(name), current.at(name))) << name;
  }

  // Pair 3: other diagonal lands; ladder done.
  snap = drain(PAIR_ORDER[2]);
  for (const auto& name : LEG_NAMES) {
    expect_vec_near(snap.at(name), target.at(name), 1e-9);
  }
  EXPECT_TRUE(ctrl.done());
}

TEST(ReseatController, DoneStateEmitsTargetForever) {
  auto ctrl = makeController();
  const auto target = shifted_stance(0.02);
  for (int i = 0; i < 500; ++i) {
    ctrl.update(0.02);
    if (ctrl.done()) {
      break;
    }
  }
  EXPECT_TRUE(ctrl.done());
  const auto out = ctrl.update(0.02);
  for (const auto& name : LEG_NAMES) {
    EXPECT_TRUE(vec_exact(out.at(name).foot_target, target.at(name))) << name;
    EXPECT_TRUE(out.at(name).stance) << name;
  }
}

TEST(ReseatController, MissingLegsRaises) {
  auto incomplete_current = stance();
  incomplete_current.erase("l_rear");
  CtrlArgs a;
  a.current_stance = incomplete_current;
  EXPECT_THROW(makeController(a), std::invalid_argument);

  auto incomplete_target = shifted_stance(0.02);
  incomplete_target.erase("r_front");
  CtrlArgs b;
  b.target_stance = incomplete_target;
  EXPECT_THROW(makeController(b), std::invalid_argument);
}

TEST(ReseatController, NonpositivePairSwingTimeRaises) {
  CtrlArgs a;
  a.pair_swing_time = 0.0;
  EXPECT_THROW(makeController(a), std::invalid_argument);
}

TEST(ReseatController, NegativePairDwellTimeRaises) {
  CtrlArgs a;
  a.pair_dwell_time = -0.01;
  EXPECT_THROW(makeController(a), std::invalid_argument);
}

TEST(ReseatController, DwellBetweenPairsHoldsAllLegs) {
  // pair_swing_time=0.1 with dt=0.02 ⇒ pair completes after the 5th tick
  // (phase = 5 * 0.02 / 0.1 = 1.0). Then a 0.06 s dwell ⇒ 3 dwell ticks
  // (0.02 × 3 = 0.06, hitting zero on the 3rd).
  const double pair_swing_time = 0.1;
  const double pair_dwell_time = 0.06;
  const double dt = 0.02;
  const auto target = shifted_stance(0.02);
  CtrlArgs a;
  a.target_stance = target;
  a.pair_swing_time = pair_swing_time;
  a.pair_dwell_time = pair_dwell_time;
  auto ctrl = makeController(a);

  // Tick through pair 1 to completion.
  std::map<std::string, LegOutput> out;
  for (int i = 0; i < 5; ++i) {
    out = ctrl.update(dt);
  }
  const auto& first_pair = PAIR_ORDER[0];
  const auto& second_pair = PAIR_ORDER[1];
  for (const auto& name : first_pair) {
    expect_vec_near(out.at(name).foot_target, target.at(name), 1e-9);
  }

  // Dwell window: every leg holds, all stance, no progress on pair 2.
  std::map<std::string, Vec3> held_positions;
  for (const auto& n : LEG_NAMES) {
    held_positions[n] = out.at(n).foot_target;
  }
  for (int i = 0; i < 3; ++i) {
    out = ctrl.update(dt);
    EXPECT_FALSE(ctrl.done());
    for (const auto& name : LEG_NAMES) {
      expect_vec_near(out.at(name).foot_target, held_positions.at(name), 1e-12);
      EXPECT_TRUE(out.at(name).stance) << name;
      EXPECT_DOUBLE_EQ(out.at(name).phase, 0.0) << name;
    }
  }

  // Next tick: pair 2 lifts off — its legs start swinging.
  out = ctrl.update(dt);
  for (const auto& name : second_pair) {
    EXPECT_FALSE(out.at(name).stance) << name;
    // Mid-arc target should differ from both held position and final target.
    EXPECT_FALSE(vec_exact(out.at(name).foot_target, held_positions.at(name)))
        << name;
  }
  // Non-active legs (pair 1 + pair 3) still hold.
  for (const auto& name : LEG_NAMES) {
    if (name == second_pair[0] || name == second_pair[1]) {
      continue;
    }
    expect_vec_near(out.at(name).foot_target, held_positions.at(name), 1e-12);
    EXPECT_TRUE(out.at(name).stance) << name;
  }
}

TEST(ReseatController, NoDwellAfterFinalPair) {
  // The ladder must flip done on the same tick the last pair snaps — no trailing
  // dwell. With pair_swing_time=0.1, dt=0.02, and pair_dwell_time=0.06 the timing
  // per pair is 5 swing ticks + 3 dwell ticks. Three pairs: 5 + 3 + 5 + 3 + 5 =
  // 21 ticks to the final snap; done should be True on that tick, not the 24th.
  CtrlArgs a;
  a.pair_swing_time = 0.1;
  a.pair_dwell_time = 0.06;
  auto ctrl = makeController(a);
  for (int tick = 1; tick < 25; ++tick) {
    const auto out = ctrl.update(0.02);
    if (ctrl.done()) {
      EXPECT_EQ(tick, 21);
      // All legs must be at target the instant done flips.
      const auto target = shifted_stance(0.02);
      for (const auto& name : LEG_NAMES) {
        expect_vec_near(out.at(name).foot_target, target.at(name), 1e-9);
      }
      return;
    }
  }
  FAIL() << "ladder did not complete within 24 ticks";
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
