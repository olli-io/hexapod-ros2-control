// Gait engine — the only stateful component in the gait chain: clock, strategy
// and the engagement / reseat controllers, driven by a command-velocity state
// machine.
#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

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
  // hard-zero stride, so the walk re-plants its own feet onto nominal.
  SETTLING,
  FOLDING,
  RESEATING,
  // The middle pair on its way to the folded pose, and on its way back down.
  // leg_set_ is HEXAPOD throughout both — it flips only once the pair arrives.
  FOLDING_PAIR,
  UNFOLDING_PAIR,
  FAULT,
};

// Engine-internal knobs, from tuning.yaml's gait_node block. The first five are
// the ACTIVE PRESET's — rewritten on every preset change; the rest are global.
struct EngineConfig {
  float stride_length = 0.0f;
  // Most a leg's coxa-to-foot reach may close over one half stride; the tick's
  // stride is cut to what the worst-placed leg affords. == stride_length is off.
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
  // The same for the quadruped leg set, where the overlap is the window the
  // support shift needs rather than mere insurance. Selected on the APPLIED leg
  // set — see swing_phase_margin_for.
  float quadruped_swing_phase_margin = 0.0f;
  // Quadruped only: how long a ladder holds all four feet planted before lifting
  // one, so the support shift can carry the body. A ladder has no phase circle
  // to buy that window from, so it waits instead.
  float quadruped_shift_time = 0.0f;
  // posture's support_shift_lead: the reseat ladder writes the phases the support
  // shift reads, and a handover between rungs must land inside that window.
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
  float reseat_plane_ramp_time = 0.0f;
  // The middle pair between the folded pose and the ground.
  float pair_fold_swing_time = 0.0f;
  // All six feet planted for this long before the pair moves, either way.
  float pair_fold_dwell_time = 0.0f;

  // Shared with the engagement controller so a swing looks the same however the
  // leg got airborne. effective_stride is what this tick actually lays down —
  // the radial budget may have cut it.
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

  // Its own clearance (a re-plant lifts far less than a step), the gait's own
  // touchdown probe. No ride_headroom: a reseat lands on still ground.
  SwingProfile reseat_profile() const {
    SwingProfile p;
    p.clearance = reseat_swing_clearance;
    p.touchdown_velocity = touchdown_velocity;
    p.touchdown_probe_fraction = touchdown_probe_fraction;
    return p;
  }

  // Zero clearance deliberately: swing_arc measures clearance from the higher
  // end, here always the folded one, whose femur sits ON its lower joint limit,
  // so any climb over it is unreachable. The arc degenerates to an eased chord,
  // and the zeroed probe time makes the unfold's landing a segment of its own.
  SwingProfile pair_fold_profile() const {
    SwingProfile p;
    p.clearance = 0.0f;
    p.touchdown_velocity = touchdown_velocity;
    p.touchdown_probe_fraction = touchdown_probe_fraction;
    return p;
  }

  // How far above its target the unfold hands over to the braked descent — the
  // gait's own probe-band expression, so the pair lands at the same speed.
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
  // swing duration. No-op outside a swing.
  //
  // Rate-bounded from the probe on, where the arc has become the touchdown ground
  // line and the target's own motion *is* slip. The budget is the foot's descent
  // speed, so the drag can never exceed the height it may land within. v_target_
  // stays unbounded: its slip contribution vanishes at touchdown.
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

// One preset as CONFIGURED: leg set, standing pose, and the stride/swing bundle
// the walk lays down on it. tuning.yaml's `presets:` list and the baked kPresets
// table both land here; solve_preset turns one into the PresetSetup the engine
// consumes. The operator half — label, gait rotation, entry gait — is the
// teleops' and never reaches here.
struct PresetSpec {
  std::string id;
  LegSet leg_set = LegSet::HEXAPOD;
  // Always three groups. A preset that parks the middle pair carries a
  // placeholder there — solve_preset replaces those rows outright.
  ::hexa::config::StandingPose standing{};
  float stride_length = 0.0f;
  float stride_length_radial = 0.0f;
  float min_swing_time = 0.0f;
  float max_swing_time = 0.0f;
  float step_height = 0.0f;
};

struct PresetSetup {
  std::string id;
  LegSet leg_set = LegSet::HEXAPOD;
  // Where the walking feet stand at a zero height offset.
  std::map<std::string, Vec3> nominal_stance;
  // Solved from THIS preset's standing pose: two presets lean their tibias
  // differently, so a shared snapshot would re-solve onto the wrong footprint.
  ReseatGeometryByLeg reseat_geometry{};
  // The active preset's copy of these five lands in EngineConfig on every
  // apply_preset.
  float stride_length = 0.0f;
  float stride_length_radial = 0.0f;
  float min_swing_time = 0.0f;
  float max_swing_time = 0.0f;
  float step_height = 0.0f;
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
         // The preset table, in declaration order. Empty synthesizes a single
         // unnamed hexapod preset from the arguments above.
         std::vector<PresetSetup> presets = {},
         std::size_t default_preset = 0);

  EngineState state() const { return state_; }
  float master_phase() const;
  // The leg set the robot is standing on. While FOLDED the strategy may already
  // name the set the next start_initialize() will stand up on.
  LegSet leg_set() const { return leg_set_; }
  // The preset the robot is standing on — what /gait/preset carries. Equal to
  // the requested one except while a preset change is still running.
  const std::string& preset_id() const { return presets_[preset_].id; }
  const std::string& strategy_name() const { return strategy_name_; }
  std::optional<std::string> pending_strategy_name() const {
    return pending_strategy_name_;
  }
  float applied_height() const { return applied_height_; }

  // Accepted from FOLDED / FAULT, and from a stand or a walk as long as the gait
  // walks the leg set in force — or the one an armed preset change is heading
  // for, since preset and gait arrive in the same tick. Otherwise refused: only
  // /cmd_preset moves the robot between leg sets.
  bool set_strategy(const std::string& name);
  // Select a preset by id. Accepted from FOLDED / FAULT, where it fixes what the
  // next stand comes up on, and from a stand, where it arms a preset change.
  // Refused everywhere else, so "refused while walking" is total. Naming the
  // preset already in force is accepted and does nothing.
  bool request_preset(const std::string& id);
  // The same, naming a LEG SET rather than a preset — what the init buttons ask
  // for. HEXAPOD resolves to the last six-leg preset applied, QUADRUPED to the
  // first declared four-corner one. False if no preset stands on that set, or if
  // request_preset would refuse.
  bool request_leg_set(LegSet set);
  // Stands up on the applied preset; a quadruped one leaves the pair folded.
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
  // goes to update(); the return is what to shape. Optional.
  std::tuple<float, float, float> shape_reversal(
      float dt, std::pair<float, float> v_body_xy, float omega_z);


 private:
  void apply_strategy(const std::string& name);
  // Swap the applied preset alone, leaving every nominal where it is. The caller
  // owes the feet a matching stance — commit_new_nominal, or a ladder.
  void set_leg_set(std::size_t preset);
  // set_leg_set plus the stance that goes with it. A teleport of nominal_, so
  // only safe where no foot stands on the old one: the two ends of the stand
  // ladder, and the fault revert. A change from a stand reseats instead.
  void apply_preset(std::size_t preset);
  std::unique_ptr<InitializeController> build_initialize();
  std::unique_ptr<FoldController> build_fold();
  std::unique_ptr<EngagementController> build_engagement();
  std::unique_ptr<ReseatController> build_reseat(
      const std::map<std::string, Vec3>& target_stance);
  void commit_new_nominal(const std::map<std::string, Vec3>& new_nominal,
                          float applied_height);
  // The first preset standing on `set`, if any. Absent for QUADRUPED leaves the
  // four-corner mode unavailable.
  std::optional<std::size_t> preset_for_leg_set(LegSet set) const;
  // Copy the active preset's five knobs into config_, which is where every
  // consumer reads them.
  void apply_preset_knobs();
  // A legal gait for `set`, for the one case a caller can leave the preset and
  // the strategy naming different leg sets.
  std::string default_strategy_for(LegSet set) const;
  bool quadruped_available() const {
    return preset_for_leg_set(LegSet::QUADRUPED).has_value();
  }
  // The frozen standing-pose snapshot the reseat solve reads — per preset, since
  // one snapshot would drag a height change onto another footprint's radius.
  const ReseatGeometryByLeg& geometry_for(std::size_t preset) const {
    return presets_[preset].reseat_geometry;
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
  // What a ladder waits before lifting. Zero on six feet, where a mirrored pair
  // leaves the body inside what is left and there is nothing to wait for.
  float ladder_shift_time() const {
    return leg_set_ == LegSet::QUADRUPED ? config_.quadruped_shift_time : 0.0f;
  }
  // legs_ with the parked pair dropped: what every per-leg velocity, stride and
  // reversal computation is priced against.
  void refresh_active_legs();
  // A preset's stance re-solved to the applied height; the bases are stored at a
  // zero height offset.
  std::map<std::string, Vec3> stance_for(std::size_t preset) const;
  // Overwrite the parked pair's entries with their held position, so no walking
  // sub-controller has to know about parking. The two stand ladders pin the pair
  // themselves.
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
  // Whether letting the gait finish the settle beats the reseat ladder. Tripod
  // wins; the longer duty factors do not.
  bool settle_beats_reseat() const;
  void reset_swing_state();
  // Stand if the feet are home, hand over to the reseat ladder if the gait would
  // be slower at it, else keep settling.
  void finish_or_hand_off_settle();
  void finish_settling();
  // The only way to reach a stand from a stance the gait did not re-plant
  // itself: the ladder arcs each foot home, where assigning nominal_ would
  // teleport it.
  void hand_off_to_reseat();
  // The one way into RESEATING: sets the ladder, the target it commits on
  // arrival, and the state, so the five callers cannot drift apart.
  void begin_reseat(const std::map<std::string, Vec3>& target_stance,
                    float target_height);
  std::map<std::string, LegOutput> tick_reseat(float dt);
  // A preset change is two moves: the footprint first, at the plane the feet
  // already stand on, then the plane as one body translation with all six down.
  // A ladder handed a new plane would step the height down a pair at a time, or
  // read the stance as six airborne feet and land it in one move. Both halves
  // live inside RESEATING, so nothing reading /gait/state has to learn them.
  void begin_preset_reseat(std::size_t preset);
  // `preset`'s footprint at the plane the feet stand on now — the ladder's
  // target, and where the plane ramp starts from.
  std::map<std::string, Vec3> lateral_stance_for(std::size_t preset) const;
  std::map<std::string, LegOutput> tick_plane_ramp(float dt);
  // The RESEATING -> STAND handoff, shared by the ladder arriving with no plane
  // owed and by the plane ramp arriving after it.
  std::map<std::string, LegOutput> finish_reseat(
      std::map<std::string, LegOutput> out);
  // The middle pair's own move. Refuses unless leg_set_ is HEXAPOD, which makes
  // "the reseat already ran on six feet" structural.
  std::unique_ptr<PairFoldController> build_pair_fold(
      PairFoldDirection direction);
  std::map<std::string, LegOutput> tick_pair_fold(float dt);
  // The single commit point for a leg-set change: the set and the strategy that
  // walks it move together, or the engagement is rebuilt on the wrong swing
  // margin for as long as the pair is in the air.
  void commit_preset_change();
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

  // QUADRUPED iff the middle pair is at the folded pose. The whole leg-set
  // change runs HEXAPOD; the flip happens on the one tick where the pair is
  // actually at the pose the flag claims for it.
  LegSet leg_set_ = LegSet::HEXAPOD;
  // Every preset, in declaration order. Each holds its stance at a zero height
  // offset; nominal_ tracks whichever one is applied, at the applied height.
  std::vector<PresetSetup> presets_;
  std::size_t preset_ = 0;
  // The last hexapod preset and strategy applied. FAULT recovery reverts to
  // both, so the folded baseline is never paired with a four-leg stance.
  std::size_t fallback_preset_ = 0;
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
  // command is inside cmd_zero_tol (or a gait change is waiting), and back if it
  // returns. Zeroing it in one tick snapped every airborne foot. The settle is
  // entered only once this reaches zero.
  float cmd_gain_ = 1.0f;
  // Per-leg: lift-off came due while the settle was holding them, so the leg sits
  // this swing window out.
  std::map<std::string, bool> held_down_;
  // Per-leg: is the foot where its phase says it is? Written false at the
  // ENGAGING -> GAIT handoff, from what the engagement reports of its own
  // velocity envelope, and back to true at the leg's next touchdown under the
  // walk. The reversal ladder's reflection is the only reader.
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
  // The preset a change is moving TO, live for the whole transition. A pair fold
  // is owed exactly when its leg set differs from leg_set_ — that one bit tells
  // the two leg-set reseats apart without a second flag.
  std::optional<std::size_t> pending_preset_;

  std::map<std::string, Vec3> reseat_target_stance_;
  float reseat_target_height_ = 0.0f;
  // The stance the plane ramp eases onto, set only while a preset change still
  // owes the body its height. plane_ramping_ says the ramp is the thing
  // RESEATING is currently running.
  std::optional<std::map<std::string, Vec3>> plane_target_;
  std::map<std::string, Vec3> plane_origin_;
  float plane_elapsed_ = 0.0f;
  bool plane_ramping_ = false;
};

// Config builders reading the baked config_generated.hpp; no filesystem access.

EngineConfig engine_config_from_config();
std::array<JointAngles, kNumLegs> standing_pose_from_config();
// The baked kPresets table, as configured and as solved.
std::vector<PresetSpec> preset_specs_from_config();
std::vector<PresetSetup> preset_setups_from_config();
// One baked preset's solved standing pose, by id — the joint triples rather
// than the feet. Throws std::invalid_argument on an unknown id.
std::array<JointAngles, kNumLegs> preset_standing_pose_from_config(
    const std::string& id);
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

// Per-leg standing joint angles from one body height plus a per-group tip reach
// and splay. Throws hexa::UnreachableTarget on an impossible reach / height
// pair, and std::invalid_argument outside config::kJointLimits; `pose_key` names
// the config block in that message.
std::array<JointAngles, kNumLegs> standing_pose_from(
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius, const ::hexa::config::StandingPose& standing,
    const std::string& pose_key = "presets");

// The same for a preset that parks the middle pair. Those rows are copied from
// `middle_fallback`: the pair never stands here, but the six-leg helpers this
// feeds want a full array of rows that solve at every body height.
std::array<JointAngles, kNumLegs> parked_pair_standing_pose_from(
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius, const ::hexa::config::StandingPose& standing,
    const std::array<JointAngles, kNumLegs>& middle_fallback,
    const std::string& pose_key = "presets");

// Solve one configured preset into the form the engine consumes: its stance as
// feet, and the frozen geometry snapshot its own reseats read.
PresetSetup solve_preset(
    const PresetSpec& spec,
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius,
    const std::array<JointAngles, kNumLegs>& middle_fallback);

// Solve a whole table. The middle-row fallback is the DEFAULT preset's solved
// pose, which is the one preset guaranteed to stand on all six.
std::vector<PresetSetup> solve_presets(
    const std::vector<PresetSpec>& specs_in,
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius, std::size_t default_preset);

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
// The same from an already-solved stance, for a caller holding a PresetSetup
// rather than the joint triples it came from.
std::map<std::string, LegContext> leg_contexts_from_stance(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::map<std::string, Vec3>& nominal_stance);

std::unique_ptr<Engine> make_default_engine(
    const std::string& strategy_name,
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const EngineConfig& engine_cfg,
    const std::array<JointAngles, kNumLegs>& standing_pose,
    const std::array<JointAngles, kNumLegs>& folded_pose,
    const std::array<JointAngles, kNumLegs>& initialized_pose,
    float coxa_to_bottom, float foot_radius,
    // The preset table, already solved to feet (solve_presets). `standing_pose`
    // above must be the default preset's, since it is what the leg contexts and
    // the six-leg reseat geometry come from.
    std::vector<PresetSetup> presets, std::size_t default_preset);

// Wire string for /gait/leg_set — the same two words hexa_common's gait catalog
// uses, so no mapping table is needed between them.
std::string leg_set_value(LegSet set);

// Wire string for /gait/state.
std::string state_value(EngineState s);
// Uppercase name for log messages.
std::string state_name(EngineState s);

}  // namespace hexa::gait
