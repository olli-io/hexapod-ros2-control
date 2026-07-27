// Target-agnostic 200 Hz control brain (plan part 10, Tier 2). See pipeline.hpp.
//
// This is a straight lift of main.cpp's per-tick body (parts 07-09), with the
// I/O removed: map_joy -> supervisor -> control.shape -> engine.update ->
// posture.update -> compose/IK. The caller samples the input/clock/telemetry
// seam into a TickInput and applies the returned TickResult; nothing here
// touches the Pico SDK or ROS, so the whole tick compiles for host + sim and is
// golden-tested off-target.

#include "pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include "config_generated.hpp"
#include "kinematics/leg_ik.hpp"
#include "leg_index.hpp"

namespace hexa::pipeline {

namespace {

// Engine states in which a gait switch may be routed to the engine (mirrors
// teleop_joy._GAIT_SWITCH_STATES). STAND swaps immediately; the others latch a
// pending change the engine commits once it has settled back to a stand.
bool gait_switch_allowed(hexa::gait::EngineState s) {
  using E = hexa::gait::EngineState;
  return s == E::STAND || s == E::GAIT || s == E::SETTLING ||
         s == E::RESEATING;
}

// The relay-arming gate: every engine state may hold the rail closed except a
// latched FAULT, where the servos stay limp so the operator can reposition the
// feet before pressing Start. FOLDED is armable, so the robot comes up energized
// in the folded pose (the consumer staggers that energize leg by leg — see
// hexa::EnergizeSweep) and the INITIALIZE ladder runs with the rail already live.
bool engine_armable(hexa::gait::EngineState s) {
  return s != hexa::gait::EngineState::FAULT;
}

// /cmd_vel-non-zero test the posture chain uses to gate walking-only vs
// idle-only animations. Mirrors posture_node._twist_is_zero (tol 1e-4) on the
// map_joy command — the same /cmd_vel the ROS posture node subscribed to.
constexpr float kCmdVelZeroTol = 1e-4f;
bool cmd_is_walking(float vx, float vy, float wz) {
  return std::fabs(vx) >= kCmdVelZeroTol || std::fabs(vy) >= kCmdVelZeroTol ||
         std::fabs(wz) >= kCmdVelZeroTol;
}

}  // namespace

Pipeline::Pipeline() : Pipeline(PipelineConfig::baked()) {}

Pipeline::Pipeline(const PipelineConfig& cfg)
    : standing_pose_(hexa::gait::standing_pose_from(
          cfg.leg_specs, cfg.coxa_to_bottom, cfg.standing_pose)),
      nominal_stance_(
          hexa::gait::nominal_stance_from(cfg.leg_specs, standing_pose_)),
      engine_(hexa::gait::make_default_engine(
          cfg.default_gait, cfg.leg_specs, cfg.engine, standing_pose_,
          cfg.initial_pose, cfg.coxa_to_bottom)),
      caps_(cfg.caps),
      control_(cfg.control, cfg.caps, nominal_stance_, cfg.default_gait),
      posture_(cfg.posture),
      // Supervisor safety stays baked (kInputTimeoutS is from webteleop YAML and
      // kBattery from hardware.yaml's `battery:` ladder, both outside the
      // geometry+tuning scope).
      supervisor_(hexa::supervisor::Config{
          hexa::config::kInputTimeoutS,
          hexa::config::kBattery.warning_v,
          hexa::config::kBattery.fold_v,
          hexa::config::kBattery.cutoff_v,
          hexa::config::kBattery.hysteresis_v,
          hexa::config::kBattery.hold_s,
          kTickPeriodUs,
          kTickMarginUs,
      }),
      leg_specs_(cfg.leg_specs) {
  // Teleop mapping + velocity shaping. The JoyConfig's stick scaling tracks the
  // active gait's caps; map_joy owns the mode FSM / posture baseline in
  // JoyState. (Joy state is baked/unused on the ROS path.)
  joycfg_.gait_linear_max = cfg.caps.linear_max(cfg.default_gait);
  joycfg_.gait_angular_z_max = cfg.caps.angular_max(cfg.default_gait);
  joystate_.mode = hexa::teleop::mode_from_string(hexa::config::kInitialMode);
  for (std::size_t i = 0; i < hexa::config::kGaitCycle.size(); ++i) {
    if (hexa::config::kGaitCycle[i] == cfg.default_gait) {
      joystate_.current_gait_idx = static_cast<int>(i);
      break;
    }
  }

  // Seed the last-good angles with the standing pose so a leg held on the very
  // first tick (before any successful compose) is valid.
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    theta_[i * 3 + 0] = standing_pose_[i][0];
    theta_[i * 3 + 1] = standing_pose_[i][1];
    theta_[i * 3 + 2] = standing_pose_[i][2];
  }
}

int Pipeline::compose_gait(
    const std::map<std::string, hexa::gait::LegOutput>& out,
    const hexa::BodyPose& body_pose) {
  int unreachable = 0;
  for (int i = 0; i < hexa::kNumLegs; ++i) {
    const hexa::config::LegSpec& spec =
        leg_specs_[static_cast<std::size_t>(i)];
    const hexa::Vec3& target = out.at(hexa::gait::LEG_NAMES[i]).foot_target;
    const hexa::Vec3 in_offset = hexa::apply_body_pose(target, body_pose);
    const hexa::Vec3 in_leg = hexa::body_to_leg(in_offset, spec);
    try {
      const hexa::JointAngles a = hexa::inverse_kinematics(in_leg, spec);
      theta_[i * 3 + 0] = a[0];
      theta_[i * 3 + 1] = a[1];
      theta_[i * 3 + 2] = a[2];
    } catch (const hexa::UnreachableTarget&) {
      ++unreachable;  // hold last-good angles for this leg
    }
  }
  return unreachable;
}

TickResult Pipeline::tick(const TickInput& in) {
  // Joy path: map the gamepad snapshot (axes/buttons) to a CommandIntent using
  // the pipeline's own JoyConfig/JoyState, then run the core tick. The Pico and
  // the /joy Gazebo bridge come in here; hexa_locomotion calls the core directly.
  const CommandIntent cmd =
      hexa::teleop::map_joy(in.axes, in.buttons, joycfg_, joystate_, in.dt);
  return tick(cmd, in);
}

TickResult Pipeline::tick(const CommandIntent& jo, const TickInput& in) {
  // Jitter accounting at the tick edge (independent of the rate-limited battery
  // sample), then the control pipeline proper.
  supervisor_.record_tick(in.now_us);

  TickResult r;

  r.mode_changed = jo.mode_changed;
  r.mode = joystate_.mode;

  // Init edge: FOLDED -> INITIALIZE (start_initialize) or, from anywhere else, a
  // queued fold request. Mirrors hexa_gait's routing of the init button.
  r.init_request = jo.init_request;
  if (jo.init_request) {
    if (engine_->start_initialize()) {
      r.init_action = InitAction::kInitialized;
    } else if (engine_->request_fold()) {
      r.init_action = InitAction::kFoldRequested;
    }
  }

  // Hardware fault latch: a board over-current trip forces the engine into FAULT
  // (rail disarmed, servos limp for manual repositioning). Applied AFTER the init
  // edge so a still-asserted fault wins over a same-tick Start — you cannot
  // re-initialize onto a board that is still tripped. Recovery is the Start edge
  // above once the fault has cleared (start_initialize is valid from FAULT).
  if (in.hardware_fault) {
    engine_->enter_fault();
  }

  // Animation-mode selection routes into the posture stack (the /animation/mode
  // subscription). Empty restores the default stack; unknown names are
  // warn-and-ignored (posture_node._on_animation_mode).
  if (jo.has_animation_name) {
    r.has_animation_name = true;
    r.animation_name = std::string(jo.animation_name);
    r.animation_accepted = posture_.set_animation_mode(jo.animation_name);
  }

  // Gait switch, gated by engine state. STAND swaps immediately; the walking /
  // pausing states latch a pending change. On accept, the control accel cap and
  // the stick scale track the new gait.
  if (jo.has_gait_select) {
    r.has_gait_select = true;
    std::string name(jo.gait_select);
    r.gait_select = name;
    if (gait_switch_allowed(engine_->state())) {
      engine_->set_strategy(name);
      control_.set_gait(name);
      joycfg_.gait_linear_max = caps_.linear_max(name);
      joycfg_.gait_angular_z_max = caps_.angular_max(name);
      r.gait_accepted = true;
      r.gait_linear_max = joycfg_.gait_linear_max;
      r.gait_angular_z_max = joycfg_.gait_angular_z_max;
    }
  }

  const hexa::gait::EngineState st = engine_->state();

  // Supervisor (part 09): failsafe / telemetry / status-LED policy. Feed it the
  // raw teleop intent (walking off the unshaped command, as the ROS
  // posture/display nodes saw /cmd_vel), the engine posture, and any fresh
  // battery sample.
  hexa::supervisor::Observation obs;
  obs.now_us = in.now_us;
  obs.bt_connected = in.bt_connected;
  obs.last_input_us = in.last_input_us;
  obs.armable = engine_armable(st);
  obs.folded = (st == hexa::gait::EngineState::FOLDED);
  obs.walking = cmd_is_walking(jo.linear_x, jo.linear_y, jo.angular_z);
  obs.battery_valid = in.battery_valid;
  obs.battery_v = in.battery_v;
  obs.fault = in.hardware_fault;
  const hexa::supervisor::Decision dec = supervisor_.step(obs);
  r.decision = dec;
  r.relay_energized = dec.relay_energized;

  // Undervoltage rung 2: queue one fold, the same request the Start button
  // makes. After the init edge above, so a same-tick Start cannot stand the
  // robot back up onto a spent pack. If the engine refuses, rung 3 is the
  // backstop that cuts the rail regardless of posture.
  if (dec.request_fold) {
    r.undervolt_fold_requested = engine_->request_fold();
  }

  // Safe-stop: on a stale/lost link OR a low/critical battery, zero the command
  // before shaping so the engine settles (pauses -> reseats) instead of latching
  // the last velocity.
  float cmd_x = jo.linear_x, cmd_y = jo.linear_y, cmd_z = jo.angular_z;
  if (dec.force_zero) {
    cmd_x = 0.0f;
    cmd_y = 0.0f;
    cmd_z = 0.0f;
  }

  const auto [vx, vy, wz] = control_.shape(cmd_x, cmd_y, cmd_z, st, in.dt);
  const std::map<std::string, hexa::gait::LegOutput> out =
      engine_->update(in.dt, {vx, vy}, wz);

  // Posture (part 08): walking gates the animations off the post-watchdog
  // command (a stale link settles the body too); the user pose comes from
  // map_joy. update() derives its signals from the engine output, advances its
  // filters, and returns the clamped body_pose_target — IDENTITY while pre-stand.
  posture_.set_user_pose(hexa::posture::BodyPose{jo.pose_x, jo.pose_y, jo.pose_z,
                                                 jo.pose_roll, jo.pose_pitch,
                                                 jo.pose_yaw});
  const bool walking = cmd_is_walking(cmd_x, cmd_y, cmd_z);
  const hexa::posture::BodyPose body_pose = posture_.update(
      out, engine_->master_phase(), walking, st, engine_->strategy_name(), in.dt,
      static_cast<float>(in.now_us) * 1e-6f);

  r.unreachable = compose_gait(out, body_pose);
  std::copy(std::begin(theta_), std::end(theta_), std::begin(r.theta));

  r.engine_state = st;
  r.master_phase = engine_->master_phase();
  r.walking = walking;

  // Raw teleop intent for the on-board face policy: the unshaped command (as the
  // ROS display node saw /cmd_vel) and the user body pose (as it saw /body/pose).
  r.cmd_vx = jo.linear_x;
  r.cmd_vy = jo.linear_y;
  r.cmd_wz = jo.angular_z;
  r.pose_roll = jo.pose_roll;
  r.pose_pitch = jo.pose_pitch;
  r.pose_yaw = jo.pose_yaw;
  return r;
}

}  // namespace hexa::pipeline
