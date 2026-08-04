// Gait engine — orchestrates clock, strategy, and the engagement / reseat
// controllers. Float fork of engine.hpp (plan part 06). The engine is the only
// stateful component in the gait chain; strategies stay pure. update() routes
// between modes based on the commanded body velocity through the
// FOLDED/INITIALIZE/STAND/ENGAGING/GAIT/SETTLING/FOLDING/RESEATING state
// machine.
//
// The yaml builders of the double engine are replaced by *_from_config helpers
// that read the baked config_generated.hpp (no filesystem on the RP2350).
#pragma once

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "gait/clock.hpp"
#include "gait/engagement.hpp"
#include "gait/gaits/base.hpp"
#include "gait/kinematics.hpp"
#include "gait/reseat.hpp"
#include "gait/stand_transition.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

enum class EngineState {
  FOLDED,
  INITIALIZE,
  STAND,
  ENGAGING,
  GAIT,
  // Command has gone to zero (or a gait change is pending): the gait keeps
  // running at a hard-zero stride, so every touchdown lands on the leg's
  // nominal stance and the walk re-plants its own feet. Ends when all six have
  // touched down — at most one cycle.
  SETTLING,
  FOLDING,
  RESEATING,
  FAULT,
};

// Engine-internal knobs, sourced entirely from tuning.yaml's gait_node block
// (baked into config::kEngine). None are on the wire.
struct EngineConfig {
  float stride_length = 0.0f;
  float min_swing_time = 0.0f;
  float max_swing_time = 0.0f;
  float step_height = 0.0f;
  float swing_width = 0.0f;
  float swing_apex_fraction = 0.5f;
  float touchdown_velocity = 0.0f;
  float touchdown_probe_height = 0.0f;
  float liftoff_velocity = 0.0f;
  // Share of each gait's nominal swing window handed back to stance at the
  // touchdown end, so every handover has a stretch with all six feet planted.
  float swing_phase_margin = 0.0f;
  float controller_dt = 0.0f;
  float cmd_zero_tol = 0.0f;
  float settle_debounce_delay = 0.0f;
  float settle_swing_time = 0.0f;
  float init_pair_swing_time = 0.0f;
  float init_lift_body_time = 0.0f;
  float init_swing_clearance = 0.0f;
  float init_place_feet_clearance = 0.0f;
  float reseat_pose_settle_delay = 0.0f;
  float reseat_height_change_threshold = 0.0f;
  float reseat_pair_swing_time = 0.0f;
  float reseat_pair_dwell_time = 0.0f;
  float reseat_swing_clearance = 0.0f;

  // How a gait swing is shaped. Shared by the engine and the engagement
  // controller so a swing looks the same however the leg got airborne.
  SwingProfile swing_profile() const {
    SwingProfile p;
    p.clearance = step_height;
    p.width = swing_width;
    p.apex_fraction = swing_apex_fraction;
    p.touchdown_velocity = touchdown_velocity;
    p.touchdown_probe_height = touchdown_probe_height;
    p.liftoff_velocity = liftoff_velocity;
    return p;
  }

  // How a reseat swing is shaped. Its own clearance (a re-plant lifts far less
  // than a step), but the gait's rise/descent split and touchdown probe: a foot
  // re-planting has the same landing to make as one finishing a step, so it is
  // eased the same way instead of falling back on the unshaped defaults.
  SwingProfile reseat_profile() const {
    SwingProfile p;
    p.clearance = reseat_swing_clearance;
    p.apex_fraction = swing_apex_fraction;
    p.touchdown_velocity = touchdown_velocity;
    p.touchdown_probe_height = touchdown_probe_height;
    p.liftoff_velocity = liftoff_velocity;
    return p;
  }
};

// How far a stance anchor may drift from its leg's nominal stance.
//
// `band` is the excursion the gait designs for — half a stride, the AEP..PEP
// envelope — which a steady walk rides exactly. `ceiling` is the hard limit the
// anchor may never pass. Between the two the outward rate is eased to zero, so
// the foot target stays velocity-continuous where a hard clip would kink it.
struct StanceBand {
  Vec3 nominal = Vec3::Zero();
  float band = 0.0f;
  float ceiling = 0.0f;
};

// Per-leg body-frame stance target as an integral from touchdown. Removes
// foot-scrub under varying velocity; reproduces the closed-form stance Bezier
// under constant velocity.
class StanceIntegrator {
 public:
  StanceIntegrator();
  void seed(const std::map<std::string, Vec3>& last_targets,
            const std::map<std::string, bool>& last_stance);
  // Returns the integrated body-frame target if in stance, else nullopt. The
  // integral is bounded by `bound`: without it a command reversal mid-stance
  // walks the leg back past its own touchdown point with nothing to stop it.
  std::optional<Vec3> step(const std::string& name, bool in_stance,
                           const Vec3& swing_target,
                           std::pair<float, float> v_leg, float dt,
                           const StanceBand& bound);
  void reset();
  bool is_stance(const std::string& name) const { return is_stance_.at(name); }

 private:
  std::map<std::string, Vec3> anchor_;
  std::map<std::string, bool> is_stance_;
};

// Per-leg swing plan. The lift-off end (origin, its velocity, swing_time) is
// latched when the foot leaves the ground; the touchdown end is re-aimed every
// tick so the foot lands on the live AEP carrying the live stance velocity. A
// latched touchdown end would step the foot's velocity at the swing->stance seam
// whenever cmd_vel changed mid-swing.
class SwingPlanner {
 public:
  SwingPlanner();
  void liftoff(const std::string& name, const Vec3& origin, const Vec3& target,
               std::pair<float, float> v_leg, float swing_time,
               int identity_y_sign_val);
  // Re-aim the touchdown end at the live AEP / stance velocity, and refresh the
  // swing duration the curve is built against — a stale one would scale every
  // realized velocity by latched / live whenever cycle_time moved mid-swing.
  // No-op unless the leg is mid-swing.
  void retarget(const std::string& name, const Vec3& target,
                std::pair<float, float> v_leg, float swing_time);
  void touchdown(const std::string& name);
  Vec3 evaluate(const std::string& name, float phase_in_swing,
                const SwingProfile& profile) const;
  void reset();
  bool is_swing(const std::string& name) const { return is_swing_.at(name); }
  const Vec3& target(const std::string& name) const { return target_.at(name); }

 private:
  std::map<std::string, Vec3> origin_;
  std::map<std::string, Vec3> target_;
  // Latched at lift-off; sets the curve's departure tangent.
  std::map<std::string, std::pair<float, float>> v_origin_;
  // Refreshed every swing tick; sets the velocity carried into stance.
  std::map<std::string, std::pair<float, float>> v_target_;
  std::map<std::string, float> swing_time_;
  std::map<std::string, int> identity_y_sign_;
  std::map<std::string, bool> is_swing_;
};

class Engine {
 public:
  // leg_specs and reseat_geometry must be supplied together (both empty disables
  // reseat, both set enables it). strategy_name must match the registry key for
  // the supplied strategy.
  Engine(EngineConfig config, std::unique_ptr<Strategy> strategy,
         std::string strategy_name, std::map<std::string, Vec3> nominal_stance,
         std::map<std::string, Vec3> initial_stance, float coxa_to_bottom,
         std::map<std::string, LegContext> leg_contexts,
         std::optional<std::map<std::string, kin::LegSpec>> leg_specs =
             std::nullopt,
         std::optional<ReseatGeometryByLeg> reseat_geometry = std::nullopt);

  EngineState state() const { return state_; }
  float master_phase() const;
  const std::string& strategy_name() const { return strategy_name_; }
  std::optional<std::string> pending_strategy_name() const {
    return pending_strategy_name_;
  }
  // Committed body height, updated when a reseat ladder applies a new nominal.
  float applied_height() const { return applied_height_; }

  bool set_strategy(const std::string& name);
  bool start_initialize();
  bool start_fold();
  bool request_fold();
  // Latch into FAULT from any state (over-current trip / hardware fault). Servos
  // go limp on the real board; recovery is start_initialize() from FAULT, which
  // reuses the cold-start INITIALIZE ladder. Idempotent while already faulted.
  void enter_fault();
  void set_target_height(float target_height);

  std::map<std::string, LegOutput> update(float dt,
                                          std::pair<float, float> v_body_xy,
                                          float omega_z);

 private:
  void apply_strategy(const std::string& name);
  std::unique_ptr<InitializeController> build_initialize();
  std::unique_ptr<FoldController> build_fold();
  std::unique_ptr<EngagementController> build_engagement();
  std::unique_ptr<ReseatController> build_reseat(
      const std::map<std::string, Vec3>& target_stance);
  void commit_new_nominal(const std::map<std::string, Vec3>& new_nominal,
                          float applied_height);

  bool cmd_is_zero(std::pair<float, float> v_body_xy, float omega_z) const;
  std::map<std::string, LegOutput> emit_stand() const;
  std::map<std::string, LegOutput> emit_held() const;
  // `settling` overrides the command with a hard zero and runs the clock at the
  // settle cycle time; see EngineState::SETTLING.
  std::map<std::string, LegOutput> tick_gait(float dt,
                                             std::pair<float, float> v_body_xy,
                                             float omega_z, bool settling);
  // Every foot planted on its nominal stance — the standing pose, tested for
  // rather than sequenced towards.
  bool all_settled() const;
  // No foot in the air. The gait's phase margin guarantees this window at every
  // handover, so it is the earliest the reseat ladder can take over cleanly.
  bool all_planted() const;
  // Whether letting the gait finish the settle beats handing the remaining legs
  // to the reseat ladder. A gait that swings its legs in many small groups walks
  // them home one group at a time and takes a whole cycle over it; the ladder
  // does three mirrored pairs regardless. Tripod wins on its own, the longer
  // duty factors do not.
  bool settle_beats_reseat() const;
  // Forget every in-flight swing, including any leg the settle was holding down.
  void reset_swing_state();
  // One settle tick's exit test: stand if the feet are home, hand the rest to
  // the reseat ladder if the gait would be slower at it, else keep settling.
  void finish_or_hand_off_settle();
  // SETTLING -> STAND: commit any pending gait change and hand the feet, already
  // on nominal, to the standing hold.
  void finish_settling();
  // Give the feet, wherever they actually are, to the reseat ladder and enter
  // RESEATING. The ladder arcs each one home and skips the ones already there,
  // so it is the only correct way to reach a stand from a stance the gait did
  // not re-plant itself — assigning nominal_ instead teleports them.
  void hand_off_to_reseat();
  std::map<std::string, LegOutput> tick_reseat(float dt);
  std::map<std::string, LegOutput> tick_fold(float dt);
  std::map<std::string, LegOutput> tick_engagement(
      float dt, std::pair<float, float> v_body_xy, float omega_z);
  void capture_state(const std::map<std::string, LegOutput>& out);

  EngineConfig config_;
  std::unique_ptr<Strategy> strategy_;
  std::string strategy_name_;
  std::map<std::string, Vec3> nominal_;
  std::map<std::string, Vec3> initial_;
  float coxa_to_bottom_;
  std::map<std::string, LegContext> legs_;
  std::optional<std::map<std::string, kin::LegSpec>> leg_specs_;
  std::optional<ReseatGeometryByLeg> reseat_geometry_;

  std::optional<GaitClock> clock_;
  StanceIntegrator stance_;
  SwingPlanner swing_;
  std::unique_ptr<EngagementController> engagement_;
  std::unique_ptr<InitializeController> initialize_;
  std::unique_ptr<FoldController> fold_;
  std::unique_ptr<ReseatController> reseat_;

  EngineState state_ = EngineState::FOLDED;
  std::map<std::string, Vec3> last_targets_;
  std::map<std::string, bool> last_stance_;
  float cmd_zero_elapsed_ = 0.0f;
  // Gain on the commanded velocity, eased 1 -> 0 across the debounce once the
  // command is inside cmd_zero_tol (or a gait change is waiting), and eased back
  // the same way if it returns.
  //
  // A command that small is one the engine has decided to call zero, but
  // stride_vector still multiplies it by a whole stance — 3.5 s on a ripple — so
  // 3 mm/s of "zero" is a 12 mm stride and half of that is live AEP. Zeroing it
  // in one tick therefore snapped every airborne foot; a gait change, which
  // settles from a *full* command, snapped them further. The settle is entered
  // only once this reaches zero, so by then there is nothing left to step.
  float cmd_gain_ = 1.0f;
  // Per-leg: its lift-off came due while the settle was holding them, so it is
  // sitting this swing window out. See the hold in tick_gait.
  std::map<std::string, bool> held_down_;

  float applied_height_ = 0.0f;
  float target_height_ = 0.0f;
  float height_stable_elapsed_ = 0.0f;
  bool pending_fold_ = false;
  std::optional<std::string> pending_strategy_name_;

  // Snapshot of the reseat target, set when RESEATING is entered.
  std::map<std::string, Vec3> reseat_target_stance_;
  float reseat_target_height_ = 0.0f;
};

// ── Config builders (replace the double engine's YAML builders) ──
//
// All read the baked config_generated.hpp constants; no filesystem access.

EngineConfig engine_config_from_config();
std::array<JointAngles, kNumLegs> standing_pose_from_config();
std::map<std::string, Vec3> nominal_stance_from_config();
std::map<std::string, Vec3> initial_stance_from_config();
std::map<std::string, kin::LegSpec> leg_specs_from_config();
ReseatGeometryByLeg reseat_geometry_from_config();
std::map<std::string, LegContext> build_leg_contexts_from_config();

// Assemble a fully-wired engine (reseat enabled) from the baked config.
// strategy_name must be a registry key (default "tripod").
std::unique_ptr<Engine> make_default_engine(
    const std::string& strategy_name = "tripod");

// ── Parameterized builders (runtime geometry, e.g. hexa_locomotion's YAML) ──
//
// Same construction as the *_from_config() versions but from explicit leg specs,
// standing/initial pose, and engine config instead of the baked constexpr, so a
// ROS caller can supply runtime-loaded values. The no-arg versions above delegate
// to these with the config_generated.hpp constants (so the Pico is unchanged).

// Per-leg resting joint angles from the standing pose: one belly clearance for
// the body, plus a tip reach out from the leg's own coxa axis and a splay for
// each of the three leg groups. femur/tibia come out uniform within a group —
// the radial reach is the same for both its legs — and differ between groups
// only where their reaches do. The coxa carries the splay, negated for rear legs
// and again for right ones, so a positive value splays outward on every leg.
// Throws hexa::UnreachableTarget if a tip reach / body height pair is
// geometrically impossible, std::invalid_argument if a resulting angle falls
// outside config::kJointLimits.
std::array<JointAngles, kNumLegs> standing_pose_from(
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    const ::hexa::config::StandingPose& standing);

std::map<std::string, Vec3> nominal_stance_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& standing_pose);
std::map<std::string, Vec3> initial_stance_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& initial_pose);
std::map<std::string, kin::LegSpec> leg_specs_from(
    const std::array<kin::LegSpec, kNumLegs>& specs);
ReseatGeometryByLeg reseat_geometry_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& standing_pose);
std::map<std::string, LegContext> build_leg_contexts_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& standing_pose);

std::unique_ptr<Engine> make_default_engine(
    const std::string& strategy_name,
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const EngineConfig& engine_cfg,
    const std::array<JointAngles, kNumLegs>& standing_pose,
    const std::array<JointAngles, kNumLegs>& initial_pose, float coxa_to_bottom);

// Wire string for /gait/state (folded, initialize, stand, ...).
std::string state_value(EngineState s);
// Uppercase name for log messages (FOLDED, INITIALIZE, STAND, ...).
std::string state_name(EngineState s);

}  // namespace hexa::gait
