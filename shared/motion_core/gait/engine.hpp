// Gait engine — orchestrates clock, strategy, and the engagement / reseat
// controllers, and is the only stateful component in the gait chain. update()
// routes the commanded body velocity through the
// FOLDED/INITIALIZE/STAND/ENGAGING/GAIT/SETTLING/FOLDING/RESEATING machine.
#pragma once

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include "gait/clock.hpp"
#include "gait/engagement.hpp"
#include "gait/gaits/base.hpp"
#include "gait/kinematics.hpp"
#include "gait/reseat.hpp"
#include "gait/reversal.hpp"
#include "gait/stand_transition.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

enum class EngineState {
  FOLDED,
  INITIALIZE,
  STAND,
  ENGAGING,
  GAIT,
  // Command gone to zero (or a gait change pending): the gait runs on at a
  // hard-zero stride, so every touchdown lands on nominal and the walk re-plants
  // its own feet. Ends when all six are down — at most one cycle.
  SETTLING,
  FOLDING,
  RESEATING,
  FAULT,
};

// Engine-internal knobs, from tuning.yaml's gait_node block. None are on the wire.
struct EngineConfig {
  float stride_length = 0.0f;
  // Most a leg's coxa-to-foot reach may close over one half stride; the tick's
  // stride is cut to what the worst-placed leg affords. Equal to stride_length
  // disables it.
  float stride_length_radial = 0.0f;
  float min_swing_time = 0.0f;
  float max_swing_time = 0.0f;
  float step_height = 0.0f;
  float swing_width = 0.0f;
  float touchdown_velocity = 0.0f;
  float touchdown_probe_fraction = 0.0f;
  // Share of the nominal swing window handed back to stance at the touchdown
  // end, so every handover has a stretch with all six feet planted.
  float swing_phase_margin = 0.0f;
  float controller_dt = 0.0f;
  float cmd_zero_tol = 0.0f;
  float settle_debounce_delay = 0.0f;
  float settle_swing_time = 0.0f;
  float init_unfold_time = 0.0f;
  float init_lift_body_time = 0.0f;
  float init_pair_swing_time = 0.0f;
  float init_place_clearance = 0.0f;
  float init_swing_clearance = 0.0f;
  float reseat_pose_settle_delay = 0.0f;
  float reseat_height_change_threshold = 0.0f;
  float reseat_pair_swing_time = 0.0f;
  float reseat_pair_dwell_time = 0.0f;
  float reseat_swing_clearance = 0.0f;

  // Shared with the engagement controller so a swing looks the same however the
  // leg got airborne. effective_stride is the stride this tick actually lays
  // down, which the radial budget may have cut; the headroom tracks it so it
  // stays the same share of the half-stride in every direction.
  SwingProfile swing_profile(float effective_stride) const {
    SwingProfile p;
    p.clearance = step_height;
    p.width = swing_width;
    p.touchdown_velocity = touchdown_velocity;
    p.touchdown_probe_fraction = touchdown_probe_fraction;
    // Derived, not configured: a foot may park exactly as far past its AEP as a
    // stance anchor may drift past the band.
    p.ride_headroom = kStanceExcursionGrace * 0.5f * effective_stride;
    return p;
  }

  SwingProfile swing_profile() const { return swing_profile(stride_length); }

  // Its own clearance (a re-plant lifts far less than a step) but the gait's
  // touchdown probe, since the landing is the same. No ride_headroom: a reseat
  // lands on still ground.
  SwingProfile reseat_profile() const {
    SwingProfile p;
    p.clearance = reseat_swing_clearance;
    p.touchdown_velocity = touchdown_velocity;
    p.touchdown_probe_fraction = touchdown_probe_fraction;
    return p;
  }
};

// How far a stance anchor may drift from its leg's nominal stance. `band` is the
// AEP..PEP envelope a steady walk rides exactly; `ceiling` is the hard limit.
// Between them the outward rate eases to zero, where a hard clip would kink the
// foot target.
struct StanceBand {
  Vec3 nominal = Vec3::Zero();
  float band = 0.0f;
  float ceiling = 0.0f;
};

// Per-leg stance target as an integral from touchdown: removes foot-scrub under
// varying velocity, reproduces the stance Bezier under constant velocity.
class StanceIntegrator {
 public:
  StanceIntegrator();
  void seed(const std::map<std::string, Vec3>& last_targets,
            const std::map<std::string, bool>& last_stance);
  // The integrated target if in stance, else nullopt. `bound` is what stops a
  // command reversal mid-stance walking the leg back past its own touchdown.
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

// Per-leg swing plan. The lift-off end is latched when the foot leaves the
// ground; the touchdown end is re-aimed every tick onto the live AEP and stance
// velocity, since a latched one would step the foot's velocity at the
// swing->stance seam whenever cmd_vel changed mid-swing.
class SwingPlanner {
 public:
  SwingPlanner();
  void liftoff(const std::string& name, const Vec3& origin, const Vec3& target,
               std::pair<float, float> v_leg, float swing_time,
               int identity_y_sign_val);
  // Re-aim the touchdown end at the live AEP / stance velocity and refresh the
  // swing duration — a stale one would scale every realized velocity by
  // latched / live whenever cycle_time moved mid-swing. No-op outside a swing.
  //
  // Rate-bounded from the probe on, where the target's own motion *is* slip: the
  // arc has become the touchdown ground line, so the foot's ground velocity is
  // exactly d(target)/dt while it may be touching. A command ramping through low
  // speed slides the AEP faster than the robot walks. The budget is the foot's
  // descent speed, so integrated over the probe it comes to the probe band's
  // height — the drag can never exceed the height it may land within. What the
  // target gives up is aim, not softness.
  //
  // v_target_ stays live and unbounded: it sets the arc's slope, and its own
  // slip contribution carries a (1 - t) factor that vanishes at touchdown.
  void retarget(const std::string& name, const Vec3& target,
                std::pair<float, float> v_leg, float swing_time,
                float phase_in_swing, float dt, const SwingProfile& profile);
  void touchdown(const std::string& name);
  Vec3 evaluate(const std::string& name, float phase_in_swing,
                const SwingProfile& profile) const;
  void reset();
  bool is_swing(const std::string& name) const { return is_swing_.at(name); }
  const Vec3& target(const std::string& name) const { return target_.at(name); }

 private:
  std::map<std::string, Vec3> origin_;
  std::map<std::string, Vec3> target_;
  std::map<std::string, std::pair<float, float>> v_origin_;
  // Refreshed every tick; sets the velocity carried into stance.
  std::map<std::string, std::pair<float, float>> v_target_;
  std::map<std::string, float> swing_time_;
  std::map<std::string, int> identity_y_sign_;
  std::map<std::string, bool> is_swing_;
};

class Engine {
 public:
  // leg_specs and reseat_geometry must be supplied together (both empty disables
  // reseat). strategy_name must match the registry key for `strategy`.
  Engine(EngineConfig config, std::unique_ptr<Strategy> strategy,
         std::string strategy_name, std::map<std::string, Vec3> nominal_stance,
         std::map<std::string, Vec3> folded_stance,
         std::map<std::string, Vec3> initialized_stance, float coxa_to_bottom,
         float foot_radius, std::map<std::string, LegContext> leg_contexts,
         std::optional<std::map<std::string, kin::LegSpec>> leg_specs =
             std::nullopt,
         std::optional<ReseatGeometryByLeg> reseat_geometry = std::nullopt);

  EngineState state() const { return state_; }
  float master_phase() const;
  const std::string& strategy_name() const { return strategy_name_; }
  std::optional<std::string> pending_strategy_name() const {
    return pending_strategy_name_;
  }
  float applied_height() const { return applied_height_; }

  bool set_strategy(const std::string& name);
  bool start_initialize();
  bool start_fold();
  bool request_fold();
  // Latch into FAULT from any state; servos go limp on the real board. Recovery
  // is start_initialize(). Idempotent while already faulted.
  void enter_fault();
  void set_target_height(float target_height);

  std::map<std::string, LegOutput> update(float dt,
                                          std::pair<float, float> v_body_xy,
                                          float omega_z);

  // Shape a command that reverses the robot's travel (gait/reversal.hpp). Call
  // once per tick with the raw command *before* the velocity shaping whose output
  // goes to update(); the return is what to shape. Skipping it entirely gives the
  // old behaviour, not a broken one.
  std::tuple<float, float, float> shape_reversal(
      float dt, std::pair<float, float> v_body_xy, float omega_z);


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
  // `settling` forces a hard-zero command and the settle cycle time.
  std::map<std::string, LegOutput> tick_gait(float dt,
                                             std::pair<float, float> v_body_xy,
                                             float omega_z, bool settling);
  // Every foot planted on its nominal stance, tested for rather than sequenced
  // towards.
  bool all_settled() const;
  // No foot in the air; the phase margin guarantees this window at every
  // handover.
  bool all_planted() const;
  // Whether letting the gait finish the settle beats the reseat ladder's three
  // mirrored pairs. Tripod wins; the longer duty factors, walking their legs home
  // in many small groups, do not.
  bool settle_beats_reseat() const;
  void reset_swing_state();
  // Stand if the feet are home, hand over to the reseat ladder if the gait would
  // be slower at it, else keep settling.
  void finish_or_hand_off_settle();
  void finish_settling();
  // The only correct way to reach a stand from a stance the gait did not
  // re-plant itself: the ladder arcs each foot home from where it is, where
  // assigning nominal_ would teleport it.
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
  std::map<std::string, Vec3> folded_;
  std::map<std::string, Vec3> initialized_;
  float coxa_to_bottom_;
  float foot_radius_;
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
  // command is inside cmd_zero_tol (or a gait change is waiting), and back again
  // if it returns. Zeroing in one tick snapped every airborne foot: stride_vector
  // multiplies even 3 mm/s by a whole stance, and half that stride is live AEP.
  // The settle is entered only once this reaches zero.
  float cmd_gain_ = 1.0f;
  // Per-leg: lift-off came due while the settle was holding them, so the leg sits
  // this swing window out.
  std::map<std::string, bool> held_down_;

  ReversalGate reversal_;
  // The command update() was last given — what the robot is carrying rather than
  // what is asked of it, which is the reversal ladder's reference.
  std::pair<float, float> applied_xy_{0.0f, 0.0f};
  float applied_omega_ = 0.0f;

  float applied_height_ = 0.0f;
  float target_height_ = 0.0f;
  float height_stable_elapsed_ = 0.0f;
  bool pending_fold_ = false;
  std::optional<std::string> pending_strategy_name_;

  std::map<std::string, Vec3> reseat_target_stance_;
  float reseat_target_height_ = 0.0f;
};

// Config builders reading the baked config_generated.hpp; no filesystem access.

EngineConfig engine_config_from_config();
std::array<JointAngles, kNumLegs> standing_pose_from_config();
std::map<std::string, Vec3> nominal_stance_from_config();
std::map<std::string, Vec3> folded_stance_from_config();
std::map<std::string, Vec3> initialized_stance_from_config();
std::map<std::string, kin::LegSpec> leg_specs_from_config();
ReseatGeometryByLeg reseat_geometry_from_config();
std::map<std::string, LegContext> build_leg_contexts_from_config();

// Fully-wired engine (reseat enabled) from the baked config.
std::unique_ptr<Engine> make_default_engine(
    const std::string& strategy_name = "tripod");

// Parameterized builders: the same construction from explicit runtime-loaded
// values (hexa_locomotion's YAML). The no-arg versions above delegate to these
// with the baked constants.

// Per-leg resting joint angles: one belly clearance for the body, plus a per-
// group tip reach and splay. The coxa carries the splay, negated for rear legs
// and again for right ones, so a positive value splays outward everywhere.
// Throws hexa::UnreachableTarget on an impossible reach / height pair, and
// std::invalid_argument if an angle falls outside config::kJointLimits.
std::array<JointAngles, kNumLegs> standing_pose_from(
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius, const ::hexa::config::StandingPose& standing);

std::map<std::string, Vec3> nominal_stance_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& standing_pose);
// Both belly-rest poses go through the same FK; the two names keep callers from
// mixing them up.
std::map<std::string, Vec3> rest_stance_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& rest_pose);
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
    const std::array<JointAngles, kNumLegs>& folded_pose,
    const std::array<JointAngles, kNumLegs>& initialized_pose,
    float coxa_to_bottom, float foot_radius);

// Wire string for /gait/state.
std::string state_value(EngineState s);
// Uppercase name for log messages.
std::string state_name(EngineState s);

}  // namespace hexa::gait
