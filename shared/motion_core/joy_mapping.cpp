#include "joy_mapping.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "config_generated.hpp"

namespace hexa::teleop {

namespace {

namespace cfgns = ::hexa::config;
using cfgns::JoyFn;
using cfgns::JoyInputKind;
using cfgns::JoyKeyRef;

// Raw int16 -> [-1, 1].
constexpr float kAxisScale = 1.0f / 32767.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

// Posture-mode scalar limits in radians; the baked kPostureLimits is in degrees.
//
// height_max/height_min are the posture stack's own envelope, not a teleop
// limit. Teleop keeps them only to saturate the height integrator, so holding
// the button past the stop cannot bank travel the pipeline will never honour.
struct PostureLimits {
  float x_max, y_max, roll_max, pitch_max, yaw_max;
  float yaw_tau, revert_tau, wiggle_pivot_forward_m;
  float height_max, height_min, height_rate;
};

constexpr PostureLimits posture_limits() {
  const auto& p = cfgns::kPostureLimits;
  const auto& pose = cfgns::kPosture;
  return PostureLimits{
      p.x_max,
      p.y_max,
      p.roll_max_deg * kDegToRad,
      p.pitch_max_deg * kDegToRad,
      p.yaw_max_deg * kDegToRad,
      p.yaw_tau_s,
      p.revert_tau_s,
      p.wiggle_pivot_forward_m,
      pose.body_height_max - pose.nominal_body_height,
      pose.body_height_min - pose.nominal_body_height,
      p.height_rate_m_per_s,
  };
}

// Sized off the generated enum so adding a JoyFn does not need an edit here.
using BindTable =
    std::array<JoyKeyRef, static_cast<std::size_t>(JoyFn::kCount)>;

const BindTable& table_for(Mode mode) {
  switch (mode) {
    case Mode::Posture:
      return cfgns::kBindPosture;
    case Mode::Animation:
      return cfgns::kBindAnimation;
    case Mode::Gait:
    default:
      return cfgns::kBindGait;
  }
}

const JoyKeyRef& binding(const BindTable& tbl, JoyFn fn) {
  return tbl[static_cast<std::size_t>(fn)];
}

bool pressed(const JoyKeyRef& k, const std::int16_t* axes,
             std::uint32_t buttons) {
  switch (k.kind) {
    case JoyInputKind::kButton:
      return (buttons >> k.index) & 1u;
    case JoyInputKind::kDpad: {
      const float value = k.sign * (static_cast<float>(axes[k.index]) * kAxisScale);
      return k.side > 0 ? value > 0.5f : value < -0.5f;
    }
    case JoyInputKind::kAxis:
      // Analog trigger as binary: released = +1.0, pressed = -1.0.
      return static_cast<float>(axes[k.index]) * kAxisScale < cfgns::kTriggerThreshold;
    case JoyInputKind::kUnbound:
    default:
      return false;
  }
}

float axis_value(const JoyKeyRef& k, const std::int16_t* axes) {
  if (k.kind != JoyInputKind::kAxis) {
    return 0.0f;
  }
  const float raw = static_cast<float>(axes[k.index]) * kAxisScale;
  return apply_deadband(k.sign * raw, cfgns::kDeadband);
}

float clipf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}  // namespace

float apply_deadband(float value, float deadband) {
  const float magnitude = std::fabs(value);
  if (magnitude < deadband) {
    return 0.0f;
  }
  const float span = 1.0f - deadband;
  if (span <= 0.0f) {
    return value;
  }
  return std::copysign((magnitude - deadband) / span, value);
}

std::array<float, 3> fit_drive_to_envelope(float drive_x, float drive_y,
                                           float drive_yaw) {
  const float deflection = std::max({std::fabs(drive_x), std::fabs(drive_y),
                                     std::fabs(drive_yaw)});
  if (deflection <= 0.0f) {
    return {0.0f, 0.0f, 0.0f};
  }

  float peak = 0.0f;
  for (const auto& r : ::hexa::config::kStanceUnit) {
    const float foot =
        std::hypot(drive_x - drive_yaw * r[1], drive_y + drive_yaw * r[0]);
    if (foot > peak) {
      peak = foot;
    }
  }
  if (peak <= 0.0f) {
    return {0.0f, 0.0f, 0.0f};
  }

  const float scale = std::min(1.0f, deflection / peak);
  return {drive_x * scale, drive_y * scale, drive_yaw * scale};
}

Mode mode_from_string(std::string_view name) {
  if (name == "posture") return Mode::Posture;
  if (name == "animation") return Mode::Animation;
  return Mode::Gait;
}

JoyOutput map_joy(const std::int16_t axes[bt_teleop::kNumAxes],
                  std::uint32_t buttons, const JoyConfig& cfg, JoyState& state,
                  float dt) {
  constexpr PostureLimits kP = posture_limits();
  const auto* tbl = &table_for(state.mode);

  // Rising edges only; posture wins ties.
  const bool gait_pressed = pressed(binding(*tbl, JoyFn::kGaitMode), axes, buttons);
  const bool posture_pressed =
      pressed(binding(*tbl, JoyFn::kPostureMode), axes, buttons);
  const bool animation_pressed =
      pressed(binding(*tbl, JoyFn::kAnimationMode), axes, buttons);
  const bool gait_edge = gait_pressed && !state.prev_gait_mode;
  const bool posture_edge = posture_pressed && !state.prev_posture_mode;
  const bool animation_edge = animation_pressed && !state.prev_animation_mode;
  state.prev_gait_mode = gait_pressed;
  state.prev_posture_mode = posture_pressed;
  state.prev_animation_mode = animation_pressed;
  bool mode_changed = false;
  const Mode prev_mode = state.mode;
  if (posture_edge && state.mode != Mode::Posture) {
    state.mode = Mode::Posture;
    mode_changed = true;
  } else if (gait_edge && state.mode != Mode::Gait) {
    state.mode = Mode::Gait;
    mode_changed = true;
  } else if (animation_edge && !state.quadruped) {
    // Every animation is written for six legs, and the four-corner stance is cut
    // for the support shift rather than for a body the operator swings around on
    // it. Gait and posture stay available; the way out of quadruped mode is the
    // fold that got into it.
    state.mode = state.mode == Mode::Animation ? Mode::Gait : Mode::Animation;
    mode_changed = true;
  }
  if (mode_changed) {
    tbl = &table_for(state.mode);
  }

  // Side effects of leaving / entering ANIMATION mode.
  bool has_animation_name = false;
  std::string_view animation_name_out = "";
  bool has_forced_gait = false;
  std::string_view forced_gait = "";
  if (prev_mode == Mode::Animation && state.mode != Mode::Animation) {
    state.animation_name = "";
    has_animation_name = true;
    animation_name_out = "";
  } else if (prev_mode != Mode::Animation && state.mode == Mode::Animation) {
    if (!cfgns::kAnimationModeAnimations.empty()) {
      state.current_animation_idx = 0;
      state.animation_name = cfgns::kAnimationModeAnimations[0];
      has_animation_name = true;
      animation_name_out = cfgns::kAnimationModeAnimations[0];
    }
    has_forced_gait = true;
    forced_gait = "tripod";
    for (std::size_t i = 0; i < cfgns::kGaitCycle.size(); ++i) {
      if (cfgns::kGaitCycle[i] == "tripod") {
        state.current_gait_idx = static_cast<int>(i);
        break;
      }
    }
  }

  // The two init buttons, with the two-press revert they share when the chassis
  // is in a non-default posture. Start asks for the six-leg stand, select for
  // the four-corner one; off the belly either is a fold, which the caller
  // resolves because map_joy cannot see the engine.
  const bool init_pressed = pressed(binding(*tbl, JoyFn::kInit), axes, buttons);
  const bool init_edge = init_pressed && !state.prev_init;
  state.prev_init = init_pressed;
  // Resolved against the GAIT table in EVERY mode, not the active one: the key
  // is bound only there, so reading the active mode's table would leave
  // prev_quad_init false while the button was held in posture mode and fire a
  // spurious edge the instant gait mode was entered.
  const bool quad_pressed = pressed(
      binding(table_for(Mode::Gait), JoyFn::kQuadrupedMode), axes, buttons);
  // Confined to GAIT mode, which is where the key is bound: everywhere else
  // `select` keeps its base binding (`record`), and firing both off one press
  // would record a posture on the way to standing up. The teleop boots into gait
  // mode, so the cold start is covered. The tracker still advances in the other
  // modes, so returning to gait with the button held fires nothing.
  const bool quad_edge =
      quad_pressed && !state.prev_quad_init && state.mode == Mode::Gait;
  state.prev_quad_init = quad_pressed;
  bool init_request = false;
  bool init_quadruped = false;
  if (init_edge || quad_edge) {
    const bool posture_modified =
        std::fabs(state.height_current) > 1e-4f ||
        std::fabs(state.yaw_current) > 1e-4f ||
        std::fabs(state.recorded_x) > 1e-4f ||
        std::fabs(state.recorded_y) > 1e-4f ||
        std::fabs(state.recorded_z) > 1e-4f ||
        std::fabs(state.recorded_roll) > 1e-4f ||
        std::fabs(state.recorded_pitch) > 1e-4f ||
        std::fabs(state.recorded_yaw) > 1e-4f;
    if (posture_modified) {
      state.reverting = true;
    } else {
      init_request = true;
      // Start wins a tie: six legs is the stand to land on when both edges
      // arrive in one tick.
      init_quadruped = quad_edge && !init_edge;
      state.quadruped = init_quadruped;
      // The leg set rides the gait, so every init edge republishes the gait that
      // walks the set it asked for. Idempotent when nothing changed, and it is
      // what keeps the engine's strategy and this flag from drifting apart when
      // the request lands as a fold instead of a stand.
      if (init_quadruped) {
        has_forced_gait = true;
        forced_gait = cfgns::kDefaultQuadrupedGait;
        for (std::size_t i = 0; i < cfgns::kQuadrupedGaitCycle.size(); ++i) {
          if (cfgns::kQuadrupedGaitCycle[i] == cfgns::kDefaultQuadrupedGait) {
            state.current_quadruped_gait_idx = static_cast<int>(i);
            break;
          }
        }
      } else if (!cfgns::kGaitCycle.empty()) {
        has_forced_gait = true;
        forced_gait =
            cfgns::kGaitCycle[static_cast<std::size_t>(state.current_gait_idx)];
      }
    }
  }

  if (state.reverting) {
    const float decay = std::exp(-dt / kP.revert_tau);
    state.height_current *= decay;
    state.recorded_x *= decay;
    state.recorded_y *= decay;
    state.recorded_z *= decay;
    state.recorded_roll *= decay;
    state.recorded_pitch *= decay;
    state.recorded_yaw *= decay;
    if (std::fabs(state.height_current) <= 1e-4f &&
        std::fabs(state.yaw_current) <= 1e-4f &&
        std::fabs(state.recorded_x) <= 1e-4f &&
        std::fabs(state.recorded_y) <= 1e-4f &&
        std::fabs(state.recorded_z) <= 1e-4f &&
        std::fabs(state.recorded_roll) <= 1e-4f &&
        std::fabs(state.recorded_pitch) <= 1e-4f &&
        std::fabs(state.recorded_yaw) <= 1e-4f) {
      state.height_current = 0.0f;
      state.recorded_x = 0.0f;
      state.recorded_y = 0.0f;
      state.recorded_z = 0.0f;
      state.recorded_roll = 0.0f;
      state.recorded_pitch = 0.0f;
      state.recorded_yaw = 0.0f;
      state.reverting = false;
    }
  }

  // Applied after the live posture is computed below.
  const bool record_pressed =
      pressed(binding(*tbl, JoyFn::kRecord), axes, buttons);
  const bool record_edge = record_pressed && !state.prev_record;
  state.prev_record = record_pressed;

  // Always resolved against the posture bindings.
  const auto& post_tbl = cfgns::kBindPosture;
  const float lx = axis_value(binding(post_tbl, JoyFn::kTiltRoll), axes);
  const float ly = axis_value(binding(post_tbl, JoyFn::kTiltPitch), axes);
  const float rx = axis_value(binding(post_tbl, JoyFn::kPoseY), axes);
  const float ry = axis_value(binding(post_tbl, JoyFn::kPoseX), axes);

  // Integrated in any mode, then clamped.
  const bool height_up = pressed(binding(*tbl, JoyFn::kHeightUp), axes, buttons);
  const bool height_down =
      pressed(binding(*tbl, JoyFn::kHeightDown), axes, buttons);
  const float net = (height_up ? 1.0f : 0.0f) - (height_down ? 1.0f : 0.0f);
  state.height_current += net * kP.height_rate * dt;
  if (state.height_current > kP.height_max) {
    state.height_current = kP.height_max;
  } else if (state.height_current < kP.height_min) {
    state.height_current = kP.height_min;
  }

  const auto& anim_tbl = cfgns::kBindAnimation;
  const bool anim_prev_pressed =
      pressed(binding(anim_tbl, JoyFn::kAnimationPrev), axes, buttons);
  const bool anim_next_pressed =
      pressed(binding(anim_tbl, JoyFn::kAnimationNext), axes, buttons);
  if (state.mode == Mode::Animation && !cfgns::kAnimationModeAnimations.empty()) {
    int delta = 0;
    if (anim_next_pressed && !state.prev_animation_next) delta += 1;
    if (anim_prev_pressed && !state.prev_animation_prev) delta -= 1;
    if (delta != 0) {
      const int n = static_cast<int>(cfgns::kAnimationModeAnimations.size());
      state.current_animation_idx =
          ((state.current_animation_idx + delta) % n + n) % n;
      const std::string_view new_name =
          cfgns::kAnimationModeAnimations[static_cast<std::size_t>(
              state.current_animation_idx)];
      if (state.animation_name != new_name) {
        state.animation_name = new_name;
        has_animation_name = true;
        animation_name_out = new_name;
      }
    }
  }
  state.prev_animation_prev = anim_prev_pressed;
  state.prev_animation_next = anim_next_pressed;

  // Suppressed in ANIMATION mode only. Which rotation it walks follows the leg
  // set the robot is standing on; the two are disjoint, so the cycler can never
  // ask for a gait the engine would refuse.
  bool has_gait_select = has_forced_gait;
  std::string_view gait_select = forced_gait;
  const bool gprev_pressed =
      pressed(binding(*tbl, JoyFn::kGaitPrev), axes, buttons);
  const bool gnext_pressed =
      pressed(binding(*tbl, JoyFn::kGaitNext), axes, buttons);
  const std::string_view* cycle = state.quadruped
                                       ? cfgns::kQuadrupedGaitCycle.data()
                                       : cfgns::kGaitCycle.data();
  const std::size_t cycle_size = state.quadruped
                                     ? cfgns::kQuadrupedGaitCycle.size()
                                     : cfgns::kGaitCycle.size();
  int& cycle_idx = state.quadruped ? state.current_quadruped_gait_idx
                                   : state.current_gait_idx;
  if (cycle_size > 0 && state.mode != Mode::Animation) {
    int delta = 0;
    if (gnext_pressed && !state.prev_gait_next) delta += 1;
    if (gprev_pressed && !state.prev_gait_prev) delta -= 1;
    if (delta != 0) {
      const int n = static_cast<int>(cycle_size);
      cycle_idx = ((cycle_idx + delta) % n + n) % n;
      gait_select = cycle[static_cast<std::size_t>(cycle_idx)];
      has_gait_select = true;
    }
  }
  state.prev_gait_prev = gprev_pressed;
  state.prev_gait_next = gnext_pressed;

  // One shared yaw target, so L1 + L2 does not double the yaw.
  const bool yaw_btn_left = pressed(binding(*tbl, JoyFn::kYawLeft), axes, buttons);
  const bool yaw_btn_right = pressed(binding(*tbl, JoyFn::kYawRight), axes, buttons);
  const bool wiggle_left = pressed(binding(*tbl, JoyFn::kWiggleLeft), axes, buttons);
  const bool wiggle_right = pressed(binding(*tbl, JoyFn::kWiggleRight), axes, buttons);
  const bool push_left = yaw_btn_left || wiggle_left;
  const bool push_right = yaw_btn_right || wiggle_right;
  float yaw_target = 0.0f;
  if (state.mode == Mode::Posture && push_left != push_right) {
    yaw_target = push_left ? kP.yaw_max : -kP.yaw_max;
  }
  const float wiggle_target =
      (state.mode == Mode::Posture && (wiggle_left || wiggle_right)) ? 1.0f : 0.0f;
  const float alpha = 1.0f - std::exp(-dt / kP.yaw_tau);
  state.yaw_current += (yaw_target - state.yaw_current) * alpha;
  state.wiggle_amount += (wiggle_target - state.wiggle_amount) * alpha;

  // Wiggle translation: rotation about a pivot at (+px, 0) == rotate about the
  // body centre + translate by (px*(1-cos yaw), -px*sin yaw).
  const float px = kP.wiggle_pivot_forward_m;
  const float wx = state.wiggle_amount * px * (1.0f - std::cos(state.yaw_current));
  const float wy = -state.wiggle_amount * px * std::sin(state.yaw_current);

  // Inert in quadruped mode. The live posture is welcome on four feet — posture
  // mode is not walking — but a RECORDED one bleeds through into gait mode,
  // where it would spend the same x-y envelope the support shift needs to carry
  // the body into the next support triangle, and that margin is millimetres.
  // Height is exempt by construction: it rides height_current, not the record.
  if (record_edge && state.mode == Mode::Posture && !state.quadruped) {
    state.reverting = false;
    state.recorded_x =
        clipf(state.recorded_x + ry * kP.x_max + wx, -kP.x_max, kP.x_max);
    state.recorded_y =
        clipf(state.recorded_y + rx * kP.y_max + wy, -kP.y_max, kP.y_max);
    state.recorded_z =
        clipf(state.recorded_z + state.height_current, kP.height_min, kP.height_max);
    state.recorded_roll =
        clipf(state.recorded_roll + (-lx) * kP.roll_max, -kP.roll_max, kP.roll_max);
    state.recorded_pitch = clipf(state.recorded_pitch + ly * kP.pitch_max,
                                 -kP.pitch_max, kP.pitch_max);
    state.recorded_yaw =
        clipf(state.recorded_yaw + state.yaw_current, -kP.yaw_max, kP.yaw_max);
    state.height_current = 0.0f;
    state.yaw_current = 0.0f;
  }

  JoyOutput out;
  out.mode_changed = mode_changed;
  out.init_request = init_request;
  out.init_quadruped = init_quadruped;
  out.has_gait_select = has_gait_select;
  out.gait_select = gait_select;
  out.has_animation_name = has_animation_name;
  out.animation_name = animation_name_out;

  if (state.mode == Mode::Posture) {
    out.pose_x = clipf(state.recorded_x + ry * kP.x_max + wx, -kP.x_max, kP.x_max);
    out.pose_y = clipf(state.recorded_y + rx * kP.y_max + wy, -kP.y_max, kP.y_max);
    out.pose_z = clipf(state.recorded_z + state.height_current, kP.height_min,
                       kP.height_max);
    out.pose_yaw =
        clipf(state.recorded_yaw + state.yaw_current, -kP.yaw_max, kP.yaw_max);
    out.pose_roll =
        clipf(state.recorded_roll + (-lx) * kP.roll_max, -kP.roll_max, kP.roll_max);
    out.pose_pitch =
        clipf(state.recorded_pitch + ly * kP.pitch_max, -kP.pitch_max, kP.pitch_max);
    return out;
  }

  // GAIT / ANIMATION: sticks drive velocity; recorded posture bleeds through.
  const float drive_x_raw =
      clipf(axis_value(binding(*tbl, JoyFn::kDriveX), axes) +
                axis_value(binding(*tbl, JoyFn::kDriveXAux), axes),
            -1.0f, 1.0f);
  const float drive_y_raw =
      clipf(axis_value(binding(*tbl, JoyFn::kDriveY), axes), -1.0f, 1.0f);
  const float drive_yaw_raw =
      clipf(axis_value(binding(*tbl, JoyFn::kDriveYaw), axes), -1.0f, 1.0f);
  const auto drive =
      fit_drive_to_envelope(drive_x_raw, drive_y_raw, drive_yaw_raw);
  out.linear_x = drive[0] * cfg.gait_linear_max;
  out.linear_y = drive[1] * cfg.gait_linear_max;
  out.angular_z = drive[2] * cfg.gait_angular_z_max;
  out.pose_x = state.recorded_x;
  out.pose_y = state.recorded_y;
  out.pose_z =
      clipf(state.recorded_z + state.height_current, kP.height_min, kP.height_max);
  out.pose_yaw = state.recorded_yaw;
  out.pose_roll = state.recorded_roll;
  out.pose_pitch = state.recorded_pitch;
  return out;
}

}  // namespace hexa::teleop
