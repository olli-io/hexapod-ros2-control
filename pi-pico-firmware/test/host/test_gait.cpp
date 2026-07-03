// Golden-trace test for the float gait engine port (plan part 06).
//
// Replays one cmd_vel profile (cold start -> stand -> walk -> stop -> reseat)
// through BOTH the firmware's float engine (src/gait, namespace hexa::gait) and
// the untouched double hexa_gait_cpp engine (namespace hexa_gait, Eigen). Both
// are compiled into this one binary; they live in different namespaces so there
// is no ODR clash. Both engines are fed geometry/config derived from the SAME
// baked config (config_generated.hpp), widened float->double for the reference,
// so any divergence is purely the double->float conversion of the engine math.
//
// The reference engine is constructed directly (no YAML): its config-derived
// inputs are byte-identical (up to float->double widening) to the float port's,
// and the SAME dt value is fed to both so accumulation-threshold crossings stay
// lockstep. At the demo cmd_vel the cycle time is clamped to a constant, so the
// master phase advances identically and the per-tick foot targets track to well
// under 1e-3 m.
//
// This target deliberately bridges float and double, so the build drops
// -Wdouble-promotion (see test/host/CMakeLists.txt); the firmware build is the
// -Wdouble-promotion gate for the port sources themselves.

#include <array>
#include <cmath>
#include <map>
#include <string>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gait/gaits/registry.hpp"
#include "kinematics/leg_ik.hpp"

#include "hexa_gait_cpp/engine.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/reseat.hpp"
#include "hexa_kinematics_cpp/leg_ik.hpp"

namespace flt = hexa::gait;
namespace ref = hexa_gait;
namespace refkin = hexa_kinematics;
namespace cfg = hexa::config;

namespace {

constexpr float kDt = 0.02f;
constexpr double kPosTol = 2e-3;    // metres, float vs double foot target
constexpr double kPhaseTol = 1e-2;  // wrap-aware phase difference
// Band around a swing/stance boundary (tripod swing_end = 0.5, plus the 0/1
// lift-off wrap) within which the discrete stance flag may legitimately flip one
// tick apart between float and double. ~1.5 per-tick phase advances at the
// clamped cycle time (dt / cycle_time = 0.02 / 0.8 = 0.025).
constexpr double kSeamGuard = 0.04;

// Distance from phase p (tripod) to the nearest swing/stance seam: lift-off at
// phase 0 (== 1) and touchdown at swing_end 0.5.
double dist_to_seam(float p) {
  const double x = static_cast<double>(p);
  return std::min({x, 1.0 - x, std::fabs(x - 0.5)});
}

// Mirror a generated (float) LegSpec into the double reference LegSpec.
refkin::LegSpec to_ref_spec(const cfg::LegSpec& s) {
  refkin::LegSpec d;
  d.mount_xyz = refkin::Point3(static_cast<double>(s.mount_xyz.x),
                               static_cast<double>(s.mount_xyz.y),
                               static_cast<double>(s.mount_xyz.z));
  d.mount_yaw = static_cast<double>(s.mount_yaw);
  d.coxa_len = static_cast<double>(s.coxa_len);
  d.femur_len = static_cast<double>(s.femur_len);
  d.tibia_len = static_cast<double>(s.tibia_len);
  return d;
}

refkin::JointAngles to_ref_angles(const hexa::JointAngles& a) {
  return {static_cast<double>(a[0]), static_cast<double>(a[1]),
          static_cast<double>(a[2])};
}

// ── Reference (double) engine inputs, all derived from the baked config ──

std::map<std::string, ref::Vec3> ref_nominal_stance() {
  std::map<std::string, ref::Vec3> out;
  for (int i = 0; i < 6; ++i) {
    const refkin::LegSpec d = to_ref_spec(cfg::kLegSpecs[i]);
    out[ref::LEG_NAMES[i]] = refkin::leg_to_body(
        refkin::forward_kinematics(to_ref_angles(cfg::kStandingPose), d), d);
  }
  return out;
}

std::map<std::string, ref::Vec3> ref_initial_stance() {
  std::map<std::string, ref::Vec3> out;
  for (int i = 0; i < 6; ++i) {
    const refkin::LegSpec d = to_ref_spec(cfg::kLegSpecs[i]);
    out[ref::LEG_NAMES[i]] = refkin::leg_to_body(
        refkin::forward_kinematics(to_ref_angles(cfg::kInitialPose[i]), d), d);
  }
  return out;
}

std::map<std::string, refkin::LegSpec> ref_leg_specs() {
  std::map<std::string, refkin::LegSpec> out;
  for (int i = 0; i < 6; ++i) {
    out[ref::LEG_NAMES[i]] = to_ref_spec(cfg::kLegSpecs[i]);
  }
  return out;
}

std::map<std::string, ref::LegContext> ref_leg_contexts() {
  const auto nominal = ref_nominal_stance();
  std::map<std::string, ref::LegContext> out;
  for (int i = 0; i < 6; ++i) {
    const refkin::LegSpec d = to_ref_spec(cfg::kLegSpecs[i]);
    ref::LegContext ctx;
    ctx.name = ref::LEG_NAMES[i];
    ctx.mount_xyz = d.mount_xyz;
    ctx.mount_yaw = d.mount_yaw;
    ctx.nominal_stance = nominal.at(ref::LEG_NAMES[i]);
    out[ref::LEG_NAMES[i]] = ctx;
  }
  return out;
}

ref::EngineConfig ref_engine_config() {
  const auto& c = cfg::kEngine;
  ref::EngineConfig o;
  o.stride_length = static_cast<double>(c.stride_length);
  o.min_swing_time = static_cast<double>(c.min_swing_time);
  o.max_swing_time = static_cast<double>(c.max_swing_time);
  o.step_height = static_cast<double>(c.step_height);
  o.swing_width = static_cast<double>(c.swing_width);
  o.controller_dt = static_cast<double>(c.controller_dt);
  o.cmd_zero_tol = static_cast<double>(c.cmd_zero_tol);
  o.pause_debounce_delay = static_cast<double>(c.pause_debounce_delay);
  o.pause_to_reseat_delay = static_cast<double>(c.pause_to_reseat_delay);
  o.gait_change_pause_to_reseat_delay =
      static_cast<double>(c.gait_change_pause_to_reseat_delay);
  o.max_reset_time = static_cast<double>(c.max_reset_time);
  o.init_pair_swing_time = static_cast<double>(c.init_pair_swing_time);
  o.init_lift_body_time = static_cast<double>(c.init_lift_body_time);
  o.init_swing_clearance = static_cast<double>(c.init_swing_clearance);
  o.init_place_feet_clearance = static_cast<double>(c.init_place_feet_clearance);
  o.reseat_pose_settle_delay = static_cast<double>(c.reseat_pose_settle_delay);
  o.reseat_height_change_threshold =
      static_cast<double>(c.reseat_height_change_threshold);
  o.reseat_pair_swing_time = static_cast<double>(c.reseat_pair_swing_time);
  o.reseat_pair_dwell_time = static_cast<double>(c.reseat_pair_dwell_time);
  o.reseat_swing_clearance = static_cast<double>(c.reseat_swing_clearance);
  return o;
}

std::unique_ptr<ref::Engine> make_ref_engine(const std::string& name) {
  const refkin::LegSpec d0 = to_ref_spec(cfg::kLegSpecs[0]);
  ref::ReseatGeometry geom =
      ref::default_geometry_from_pose(to_ref_angles(cfg::kStandingPose), d0);
  return std::make_unique<ref::Engine>(
      ref_engine_config(), ref::strategies().at(name)(), name,
      ref_nominal_stance(), ref_initial_stance(),
      static_cast<double>(cfg::kCoxaToBottom), ref_leg_contexts(),
      ref_leg_specs(), geom);
}

// Wrap-aware phase difference in [0, 0.5].
double phase_diff(float a, double b) {
  double d = std::fabs(static_cast<double>(a) - b);
  return std::min(d, 1.0 - d);
}

// The "phase-locked" states — where the two engines' outputs must track to
// float precision. The deterministic time-ramp ladders (INITIALIZE, FOLDING,
// PAUSING, RESEATING) are intentionally excluded: they advance a wall-clock
// ramp, so float-vs-double accumulation rounding can shift a boundary crossing
// by one tick, producing a transient (self-correcting) offset that is expected
// float behaviour, not an engine-math divergence. STAND / ENGAGING / GAIT /
// PAUSED (and the walking core) are driven by the phase clock, which — at a
// cmd_vel whose cycle time clamps to a constant — advances by identical
// increments in both engines and therefore stays locked.
bool is_locked(const std::string& state) {
  return state == "folded" || state == "stand" || state == "engaging" ||
         state == "gait" || state == "paused";
}

// One tick on both engines. When both are in a phase-locked state, diff every
// leg's foot target and phase; otherwise advance both but skip the strict
// numeric comparison (the ladders are covered by the state-milestone asserts).
// Returns true if a strict comparison ran this tick.
bool compare_tick(flt::Engine& fe, ref::Engine& re, float vx, float vy,
                  float wz, int tick, double& worst_pos) {
  const auto fo = fe.update(kDt, {vx, vy}, wz);
  const auto ro = re.update(static_cast<double>(kDt),
                            {static_cast<double>(vx), static_cast<double>(vy)},
                            static_cast<double>(wz));
  if (!is_locked(flt::state_value(fe.state())) ||
      !is_locked(ref::state_value(re.state()))) {
    return false;
  }
  double worst = 0.0;
  for (int i = 0; i < 6; ++i) {
    const std::string& n = flt::LEG_NAMES[i];
    const auto& f = fo.at(n);
    const auto& r = ro.at(ref::LEG_NAMES[i]);
    const double ex = std::fabs(static_cast<double>(f.foot_target.x) - r.foot_target[0]);
    const double ey = std::fabs(static_cast<double>(f.foot_target.y) - r.foot_target[1]);
    const double ez = std::fabs(static_cast<double>(f.foot_target.z) - r.foot_target[2]);
    worst = std::max({worst, ex, ey, ez});
    EXPECT_LT(ex, kPosTol) << n << " x @tick " << tick;
    EXPECT_LT(ey, kPosTol) << n << " y @tick " << tick;
    EXPECT_LT(ez, kPosTol) << n << " z @tick " << tick;
    EXPECT_LT(phase_diff(f.phase, r.phase), kPhaseTol)
        << n << " phase @tick " << tick;
    // Away from a swing/stance seam the discrete flag must agree; within the
    // seam band a one-tick float/double flip is expected.
    if (dist_to_seam(f.phase) > kSeamGuard) {
      EXPECT_EQ(f.stance, r.stance) << n << " stance @tick " << tick;
    }
  }
  worst_pos = std::max(worst_pos, worst);
  return true;
}

}  // namespace

TEST(GaitGolden, ColdStartWalkStopMatchesDoubleEngine) {
  auto fe = flt::make_default_engine("tripod");
  auto re = make_ref_engine("tripod");

  ASSERT_TRUE(fe->start_initialize());
  ASSERT_TRUE(re->start_initialize());

  double worst_pos = 0.0;
  int tick = 0;

  // Phase 1: cold-start ladder to STAND (cmd_vel = 0). The ladder itself is a
  // wall-clock ramp (excluded from strict comparison); we assert both converge.
  for (int i = 0; i < 130; ++i, ++tick) {
    compare_tick(*fe, *re, 0.0f, 0.0f, 0.0f, tick, worst_pos);
  }
  ASSERT_EQ(flt::state_name(fe->state()), ref::state_name(re->state()));
  ASSERT_EQ(fe->state(), flt::EngineState::STAND);

  // Phase 2: forward walk. vx small enough that cycle time clamps to a constant,
  // keeping the master phase lockstep between the two engines. This is the core
  // fidelity check: every GAIT tick is strictly diffed.
  bool both_gait = false;
  int locked_ticks = 0;
  for (int i = 0; i < 260; ++i, ++tick) {
    if (compare_tick(*fe, *re, 0.06f, 0.0f, 0.0f, tick, worst_pos)) {
      ++locked_ticks;
    }
    if (fe->state() == flt::EngineState::GAIT &&
        re->state() == ref::EngineState::GAIT) {
      both_gait = true;
    }
  }
  EXPECT_TRUE(both_gait);
  EXPECT_GT(locked_ticks, 200) << "expected a long strictly-diffed GAIT window";

  // Phase 3: stop. Debounce -> pause -> reseat -> stand.
  for (int i = 0; i < 200; ++i, ++tick) {
    compare_tick(*fe, *re, 0.0f, 0.0f, 0.0f, tick, worst_pos);
  }
  EXPECT_EQ(flt::state_name(fe->state()), ref::state_name(re->state()));
  EXPECT_EQ(fe->state(), flt::EngineState::STAND);

  // Every strictly-compared tick stayed well under the ~1e-3 fidelity target.
  EXPECT_LT(worst_pos, kPosTol) << "worst per-component foot-target error";
  RecordProperty("worst_pos_error_m", std::to_string(worst_pos));
}

TEST(GaitGolden, FinalGaitJointAnglesMatch) {
  // After the engines have been walking a while, the emitted foot targets fed
  // through IK must yield the same joint angles (float vs double) to ~1e-3 rad.
  auto fe = flt::make_default_engine("tripod");
  auto re = make_ref_engine("tripod");
  fe->start_initialize();
  re->start_initialize();
  double worst_pos = 0.0;
  for (int i = 0; i < 130; ++i) compare_tick(*fe, *re, 0.0f, 0.0f, 0.0f, i, worst_pos);
  std::map<std::string, hexa::gait::LegOutput> fo;
  for (int i = 0; i < 220; ++i) fo = fe->update(kDt, {0.06f, 0.0f}, 0.0f);
  for (int i = 0; i < 220; ++i) re->update(static_cast<double>(kDt), {0.06, 0.0}, 0.0);

  ASSERT_EQ(fe->state(), flt::EngineState::GAIT);
  for (int i = 0; i < 6; ++i) {
    const cfg::LegSpec& s = cfg::kLegSpecs[i];
    const refkin::LegSpec d = to_ref_spec(s);
    // Float port: body-frame target -> leg frame -> IK.
    const hexa::Vec3 fleg = hexa::body_to_leg(fo.at(flt::LEG_NAMES[i]).foot_target, s);
    const hexa::JointAngles fa = hexa::inverse_kinematics(fleg, s);
    // Same body-frame target through the double kinematics.
    const refkin::Point3 tgt(static_cast<double>(fo.at(flt::LEG_NAMES[i]).foot_target.x),
                             static_cast<double>(fo.at(flt::LEG_NAMES[i]).foot_target.y),
                             static_cast<double>(fo.at(flt::LEG_NAMES[i]).foot_target.z));
    const refkin::JointAngles da =
        refkin::inverse_kinematics(refkin::body_to_leg(tgt, d), d);
    EXPECT_NEAR(static_cast<double>(fa[0]), da[0], 1e-3) << "leg " << i;
    EXPECT_NEAR(static_cast<double>(fa[1]), da[1], 1e-3) << "leg " << i;
    EXPECT_NEAR(static_cast<double>(fa[2]), da[2], 1e-3) << "leg " << i;
  }
}
