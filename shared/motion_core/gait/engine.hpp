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
  // The two halves of a leg-set change the reseat does not cover: the middle
  // pair on its way to the folded pose, and on its way back down. The leg set
  // is HEXAPOD throughout both — it flips only once the pair has arrived.
  FOLDING_PAIR,
  UNFOLDING_PAIR,
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
  // The same, for the quadruped leg set. Its own knob because the two leg sets
  // buy different things with it: on six feet the overlap is insurance and costs
  // top speed, on four it is the window the support shift has to carry the body
  // across into the next triangle. Selected on the APPLIED leg set — see
  // swing_phase_margin_for.
  float quadruped_swing_phase_margin = 0.0f;
  // Quadruped mode only: how long a ladder holds all four feet planted before it
  // lifts one, while the support shift carries the body into the triangle the
  // other three make. The walk buys the same thing with the phase margin; a
  // ladder has no phase circle to buy it from, so it waits instead.
  float quadruped_shift_time = 0.0f;
  // posture's support_shift_lead. The reseat ladder writes the phases the
  // support shift reads, and only the last `lead` of a phase moves a foot's
  // weight, so a handover between rungs has to be written inside that window.
  float support_shift_lead = 0.0f;
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
  // The middle pair between the folded pose and the ground, standing on the four
  // corners — the half of a leg-set change the reseat does not cover.
  float pair_fold_swing_time = 0.0f;
  // All six feet planted for this long before the pair moves, either way.
  float pair_fold_dwell_time = 0.0f;

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

  // The pair's move between the folded pose and the ground. Zero clearance, and
  // not because nobody has tuned it: swing_arc measures clearance from the
  // higher of the two ends, which here is always the folded one, whose femur
  // sits ON its lower joint limit — so any climb over that end is unreachable.
  // At zero the arc degenerates to a plain eased chord, which is what this move
  // wants anyway. Zero clearance also zeroes the granted probe time, so the
  // unfold's landing is a segment of its own rather than the profile's tail.
  SwingProfile pair_fold_profile() const {
    SwingProfile p;
    p.clearance = 0.0f;
    p.touchdown_velocity = touchdown_velocity;
    p.touchdown_probe_fraction = touchdown_probe_fraction;
    return p;
  }

  // How far above its target the unfold hands over to the braked descent.
  // Derived, not configured: the same expression the gait's own probe band uses,
  // so the pair lands at exactly the speed every other touchdown does.
  float pair_fold_probe_band() const {
    return touchdown_velocity * touchdown_probe_fraction * pair_fold_swing_time;
  }
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

// Everything the quadruped leg set needs, supplied together or not at all.
// Absent leaves the mode unavailable and set_strategy refuses a quadruped gait.
// The parked middle pair is not in here: it is held at the folded pose the
// engine already carries, which is also where it powered up.
struct QuadrupedSetup {
  // Where the four corners stand while the middle pair is parked, at a zero
  // height offset.
  std::map<std::string, Vec3> nominal_stance;
  // Solved from the QUADRUPED standing pose, not the hexapod one: the two
  // stances reach out different distances, so sharing a snapshot would re-solve
  // a raised body onto the wrong footprint.
  ReseatGeometryByLeg reseat_geometry{};
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
         std::optional<ReseatGeometryByLeg> reseat_geometry = std::nullopt,
         std::optional<QuadrupedSetup> quadruped = std::nullopt);

  EngineState state() const { return state_; }
  float master_phase() const;
  // The leg set the robot is standing on. Equal to strategy_->leg_set() off the
  // belly; while FOLDED the strategy may already name the set the next
  // start_initialize() will stand up on.
  LegSet leg_set() const { return leg_set_; }
  const std::string& strategy_name() const { return strategy_name_; }
  std::optional<std::string> pending_strategy_name() const {
    return pending_strategy_name_;
  }
  float applied_height() const { return applied_height_; }

  // Accepted from FOLDED / FAULT, where it also fixes the leg set the next
  // stand comes up on, and from a stand or a walk as long as the leg set does
  // not change. A strategy that DOES change it is accepted from STAND alone,
  // where it arms a leg-set change; from anywhere else it is refused, which is
  // what makes "refused while walking" total.
  bool set_strategy(const std::string& name);
  // Stands up on the leg set the applied strategy asks for: hexapod on all six,
  // quadruped on the four corners with the middle pair left folded where it is.
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
  // Swap the walking leg set alone, leaving every nominal where it is. Whoever
  // calls this owes the feet a stance that matches — either commit_new_nominal
  // in the same breath (apply_leg_set, below) or a ladder that walks them there.
  void set_leg_set(LegSet set);
  // set_leg_set plus the nominal stance that goes with it, committed on the
  // spot. A teleport of nominal_, so it is only safe where no foot is standing
  // on the old one: the two ends of the stand ladder, and the fault revert. A
  // leg-set change from a stand uses set_leg_set and lets its reseat commit.
  void apply_leg_set(LegSet set);
  std::unique_ptr<InitializeController> build_initialize();
  std::unique_ptr<FoldController> build_fold();
  std::unique_ptr<EngagementController> build_engagement();
  std::unique_ptr<ReseatController> build_reseat(
      const std::map<std::string, Vec3>& target_stance);
  void commit_new_nominal(const std::map<std::string, Vec3>& new_nominal,
                          float applied_height);
  bool quadruped_available() const { return quadruped_.has_value(); }
  // The frozen standing-pose snapshot the reseat solve reads. Per leg set: the
  // two stances lean their tibias differently, so one snapshot would drag a
  // height change onto the other footprint's radius.
  const ReseatGeometryByLeg& geometry_for(LegSet set) const {
    return (set == LegSet::QUADRUPED && quadruped_.has_value())
               ? quadruped_->reseat_geometry
               : *reseat_geometry_;
  }
  bool is_parked(const std::string& name) const {
    return leg_is_parked(leg_set_, name);
  }
  // Off the applied leg set: the swing window has to match the feet that are
  // actually walking.
  float swing_margin() const {
    return swing_phase_margin_for(leg_set_, config_.swing_phase_margin,
                                  config_.quadruped_swing_phase_margin);
  }
  // What a ladder waits before lifting, off the APPLIED leg set for the same
  // reason the margin is: on six feet a mirrored pair leaves the body inside
  // what is left, and there is nothing to wait for.
  float ladder_shift_time() const {
    return leg_set_ == LegSet::QUADRUPED ? config_.quadruped_shift_time : 0.0f;
  }
  // legs_ with the parked pair dropped: what every per-leg velocity, stride and
  // reversal computation is priced against. Rebuilt whenever leg_set_ or the
  // nominal stance changes.
  void refresh_active_legs();
  // The base stance for a leg set, re-solved to the current applied height. The
  // bases are stored at zero height offset, so a leg-set swap that happens while
  // a height offset is still applied lands on the right footprint.
  std::map<std::string, Vec3> stance_for(LegSet set) const;
  // Overwrite the parked pair's entries with their held position. Applied to
  // every walking sub-controller's output, so none of them has to know about
  // parking. The two stand ladders pin the pair themselves — they are the ones
  // that know it is folded rather than merely held.
  void overlay_parked(std::map<std::string, LegOutput>& out) const;

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
  // Every walking foot standing where its phase says. See on_schedule_.
  bool feet_on_schedule() const;
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
  // The one way into RESEATING: sets the ladder, the target it commits on
  // arrival, and the state, so the five callers cannot drift apart.
  void begin_reseat(const std::map<std::string, Vec3>& target_stance,
                    float target_height);
  std::map<std::string, LegOutput> tick_reseat(float dt);
  // The middle pair's own move. Refuses unless leg_set_ is HEXAPOD, which is
  // what makes "the reseat already ran on six feet" structural rather than a
  // property of how the states happen to be wired.
  std::unique_ptr<PairFoldController> build_pair_fold(
      PairFoldDirection direction);
  std::map<std::string, LegOutput> tick_pair_fold(float dt);
  // The single commit point for a leg-set change: the set and the strategy that
  // walks it move together, or the engagement is rebuilt on the wrong swing
  // margin for as long as the pair is in the air.
  void commit_leg_set_change();
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
  std::map<std::string, LegContext> active_legs_;
  std::optional<std::map<std::string, kin::LegSpec>> leg_specs_;
  std::optional<ReseatGeometryByLeg> reseat_geometry_;

  // QUADRUPED iff the middle pair is at the folded pose. That biconditional is
  // what a leg-set change is built around: the whole change — both reseats and
  // the pair's own move — runs HEXAPOD, and the flip happens on the one tick
  // where the pair is actually at the pose the flag claims for it.
  LegSet leg_set_ = LegSet::HEXAPOD;
  // The hexapod stance at a zero height offset; nominal_ tracks whichever leg
  // set is applied, at the applied height. The quadruped half lives in
  // quadruped_.
  std::map<std::string, Vec3> hexa_nominal_base_;
  std::optional<QuadrupedSetup> quadruped_;
  // The last hexapod strategy applied. FAULT recovery reverts to it, so the
  // pristine folded baseline is never left paired with a four-leg strategy.
  std::string fallback_strategy_name_;

  std::optional<GaitClock> clock_;
  StanceIntegrator stance_;
  SwingPlanner swing_;
  std::unique_ptr<EngagementController> engagement_;
  std::unique_ptr<InitializeController> initialize_;
  std::unique_ptr<FoldController> fold_;
  std::unique_ptr<ReseatController> reseat_;
  std::unique_ptr<PairFoldController> pair_fold_;

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
  // Per-leg: is the foot where its phase says it is? Written false only at the
  // ENGAGING -> GAIT handoff, from what the engagement reports of its own
  // velocity envelope, and back to true at the leg's next touchdown under the
  // walk, where the planner lands it on the live AEP and the integrator anchors
  // there. The reversal ladder's reflection is the only reader.
  std::map<std::string, bool> on_schedule_;

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
  // The leg set a change is moving TO, live for the whole transition. A pair
  // fold is owed exactly when it differs from leg_set_, which is true only in
  // the reseat that runs ahead of the fold — that one bit is what tells the two
  // leg-set reseats apart without a second flag.
  std::optional<LegSet> pending_leg_set_;

  std::map<std::string, Vec3> reseat_target_stance_;
  float reseat_target_height_ = 0.0f;
};

// Config builders reading the baked config_generated.hpp; no filesystem access.

EngineConfig engine_config_from_config();
std::array<JointAngles, kNumLegs> standing_pose_from_config();
std::array<JointAngles, kNumLegs> quad_standing_pose_from_config();
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

// The same for quadruped mode, which stands on the four corners alone. The
// middle rows are copied from `standing_pose` as a placeholder: that pair never
// stands in this mode — it is parked at the folded pose — but the six-leg
// helpers this feeds (nominal_stance_from / reseat_geometry_from) want a full
// array, and a row that is reachable at every body height keeps their per-leg
// solves from throwing on a leg nobody is standing on.
std::array<JointAngles, kNumLegs> quad_standing_pose_from(
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius, const ::hexa::config::CornerStandingPose& quad_standing,
    const std::array<JointAngles, kNumLegs>& standing_pose);

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
    float coxa_to_bottom, float foot_radius,
    // Quadruped mode's stance, solved by the caller from quad_standing_pose.
    // Its parked middle pair needs nothing here — that is folded_pose.
    const std::array<JointAngles, kNumLegs>& quad_standing_pose);

// Wire string for /gait/leg_set. The same two words hexa_common's gait catalog
// uses, so the topic and the catalog need no mapping table between them.
std::string leg_set_value(LegSet set);

// Wire string for /gait/state.
std::string state_value(EngineState s);
// Uppercase name for log messages.
std::string state_name(EngineState s);

}  // namespace hexa::gait
