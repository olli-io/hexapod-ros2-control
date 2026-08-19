#include "pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <tuple>

#include "config_generated.hpp"
#include "kinematics/leg_ik.hpp"
#include "leg_index.hpp"

namespace hexa::pipeline {

namespace {

// FOLDED and FAULT swap the strategy the next stand will come up on, leg set
// included; STAND swaps immediately; the others latch a pending change the
// engine commits once it has settled back to a stand.
bool gait_switch_allowed(hexa::gait::EngineState s) {
  using E = hexa::gait::EngineState;
  return s == E::FOLDED || s == E::FAULT || s == E::STAND || s == E::GAIT ||
         s == E::SETTLING || s == E::RESEATING;
}

// Every state may hold the rail closed except a latched FAULT, where the servos
// stay limp so the operator can reposition the feet before pressing Start.
// FOLDED is armable, so the INITIALIZE ladder runs with the rail already live.
bool engine_armable(hexa::gait::EngineState s) {
  return s != hexa::gait::EngineState::FAULT;
}

// Gates the posture chain's walking-only vs idle-only animations.
constexpr float kCmdVelZeroTol = 1e-4f;
bool cmd_is_walking(float vx, float vy, float wz) {
  return std::fabs(vx) >= kCmdVelZeroTol || std::fabs(vy) >= kCmdVelZeroTol ||
         std::fabs(wz) >= kCmdVelZeroTol;
}

}  // namespace

Pipeline::Pipeline() : Pipeline(PipelineConfig::baked()) {}

Pipeline::Pipeline(const PipelineConfig& cfg)
    : standing_pose_(hexa::gait::standing_pose_from(
          cfg.leg_specs, cfg.coxa_to_bottom, cfg.foot_radius,
          cfg.standing_pose)),
      nominal_stance_(
          hexa::gait::nominal_stance_from(cfg.leg_specs, standing_pose_)),
      folded_pose_(cfg.folded_pose),
      engine_(hexa::gait::make_default_engine(
          cfg.default_gait, cfg.leg_specs, cfg.engine, standing_pose_,
          cfg.folded_pose, cfg.initialized_pose, cfg.coxa_to_bottom,
          cfg.foot_radius,
          hexa::gait::quad_standing_pose_from(
              cfg.leg_specs, cfg.coxa_to_bottom, cfg.foot_radius,
              cfg.quad_standing_pose, standing_pose_))),
      caps_(cfg.caps),
      control_(cfg.control, cfg.caps, nominal_stance_,
               hexa::gait::build_leg_contexts_from(cfg.leg_specs,
                                                   standing_pose_),
               cfg.engine.stride_length, cfg.engine.stride_length_radial,
               cfg.default_gait),
      posture_(cfg.posture),
      // Baked: both are outside the geometry+tuning scope.
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
  // The JoyConfig's stick scaling tracks the active gait's caps; map_joy owns the
  // mode FSM / posture baseline in JoyState. Both are unused on the ROS path.
  joycfg_.gait_linear_max = cfg.caps.linear_max(cfg.default_gait);
  joycfg_.gait_angular_z_max = cfg.caps.angular_max(cfg.default_gait);
  joystate_.mode = hexa::teleop::mode_from_string(hexa::config::kInitialMode);
  for (std::size_t i = 0; i < hexa::config::kGaitCycle.size(); ++i) {
    if (hexa::config::kGaitCycle[i] == cfg.default_gait) {
      joystate_.current_gait_idx = static_cast<int>(i);
      break;
    }
  }

  // So a leg held on the first tick, before any successful compose, is valid.
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
    const hexa::gait::LegOutput& leg = out.at(hexa::gait::LEG_NAMES[i]);
    if (leg.parked) {
      // Straight from the pose, bypassing both apply_body_pose and IK. Exact
      // (the foot target IS this pose's FK, so the round trip is identity), and
      // the only thing stopping a 50 mm support shift from dragging a leg that
      // is rigidly held in the air.
      const std::size_t idx = static_cast<std::size_t>(i);
      theta_[i * 3 + 0] = folded_pose_[idx][0];
      theta_[i * 3 + 1] = folded_pose_[idx][1];
      theta_[i * 3 + 2] = folded_pose_[idx][2];
      continue;
    }
    const hexa::Vec3& target = leg.foot_target;
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
  const CommandIntent cmd =
      hexa::teleop::map_joy(in.axes, in.buttons, joycfg_, joystate_, in.dt);
  return tick(cmd, in);
}

TickResult Pipeline::tick(const CommandIntent& jo, const TickInput& in) {
  // At the tick edge, independent of the rate-limited battery sample.
  supervisor_.record_tick(in.now_us);

  TickResult r;

  r.mode_changed = jo.mode_changed;
  r.mode = joystate_.mode;

  // Ahead of the init edge: an init request carries its leg set as the gait it
  // publishes, and start_initialize() reads the leg set off the strategy that is
  // applied by then.
  if (jo.has_gait_select) {
    r.has_gait_select = true;
    std::string name(jo.gait_select);
    r.gait_select = name;
    if (gait_switch_allowed(engine_->state()) && engine_->set_strategy(name)) {
      control_.set_gait(name);
      joycfg_.gait_linear_max = caps_.linear_max(name);
      joycfg_.gait_angular_z_max = caps_.angular_max(name);
      r.gait_accepted = true;
      r.gait_linear_max = joycfg_.gait_linear_max;
      r.gait_angular_z_max = joycfg_.gait_angular_z_max;
    }
  }

  // FOLDED -> INITIALIZE, on the leg set the applied strategy walks, or from
  // anywhere else a queued fold request.
  r.init_request = jo.init_request;
  if (jo.init_request) {
    if (engine_->start_initialize()) {
      r.init_action = InitAction::kInitialized;
    } else if (engine_->request_fold()) {
      r.init_action = InitAction::kFoldRequested;
    }
  }

  // After the init edge, so a still-asserted fault wins over a same-tick Start:
  // you cannot re-initialize onto a board that is still tripped. Recovery is that
  // same Start edge once the fault has cleared.
  if (in.hardware_fault) {
    engine_->enter_fault();
  }

  // Empty restores the default stack; unknown names are warn-and-ignored.
  if (jo.has_animation_name) {
    r.has_animation_name = true;
    r.animation_name = std::string(jo.animation_name);
    r.animation_accepted = posture_.set_animation_mode(jo.animation_name);
  }

  const hexa::gait::EngineState st = engine_->state();

  // Failsafe / telemetry / status-LED policy, fed the raw teleop intent (walking
  // off the unshaped command), the engine posture and any fresh battery sample.
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

  // Rung 2 queues one fold, the same request the Start button makes — after the
  // init edge, so a same-tick Start cannot stand the robot back up onto a spent
  // pack. Rung 3 is the backstop if the engine refuses.
  if (dec.request_fold) {
    r.undervolt_fold_requested = engine_->request_fold();
  }

  // Zero before shaping, so the engine settles rather than latching the last
  // velocity.
  float cmd_x = jo.linear_x, cmd_y = jo.linear_y, cmd_z = jo.angular_z;
  if (dec.force_zero) {
    cmd_x = 0.0f;
    cmd_y = 0.0f;
    cmd_z = 0.0f;
  }

  // Ahead of the shaping, like force_zero, so what comes out is a command the
  // limiter ramps rather than a step it fights.
  std::tie(cmd_x, cmd_y, cmd_z) =
      engine_->shape_reversal(in.dt, {cmd_x, cmd_y}, cmd_z);

  // The engine's applied leg set, so the envelope stops pricing the middles'
  // lever arms the moment the stand ladder commits one — and not before.
  control_.set_leg_set(engine_->leg_set());
  const auto [vx, vy, wz] = control_.shape(cmd_x, cmd_y, cmd_z, st, in.dt);
  const std::map<std::string, hexa::gait::LegOutput> out =
      engine_->update(in.dt, {vx, vy}, wz);

  // walking gates the animations off the post-watchdog command, so a stale link
  // settles the body too. update() returns the clamped body_pose_target, or
  // IDENTITY while pre-stand.
  posture_.set_user_pose(hexa::posture::BodyPose{jo.pose_x, jo.pose_y, jo.pose_z,
                                                 jo.pose_roll, jo.pose_pitch,
                                                 jo.pose_yaw});
  const bool walking = cmd_is_walking(cmd_x, cmd_y, cmd_z);
  const hexa::posture::BodyPose body_pose = posture_.update(
      out, engine_->master_phase(), walking, st, engine_->strategy_name(),
      engine_->leg_set(), in.dt, static_cast<float>(in.now_us) * 1e-6f);

  r.unreachable = compose_gait(out, body_pose);
  std::copy(std::begin(theta_), std::end(theta_), std::begin(r.theta));

  r.engine_state = st;
  r.master_phase = engine_->master_phase();
  r.walking = walking;

  // Raw teleop intent for the on-board face policy.
  r.cmd_vx = jo.linear_x;
  r.cmd_vy = jo.linear_y;
  r.cmd_wz = jo.angular_z;
  r.pose_x = jo.pose_x;
  r.pose_y = jo.pose_y;
  r.pose_roll = jo.pose_roll;
  r.pose_pitch = jo.pose_pitch;
  r.pose_yaw = jo.pose_yaw;
  return r;
}

}  // namespace hexa::pipeline
