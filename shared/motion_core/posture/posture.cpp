// Posture controller implementation — float fork of
// hexa_posture/posture_node.py.
#include "posture/posture.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "config_generated.hpp"  // hexa::config::kPosture / kEnabledAnimations

namespace hexa::posture {

namespace {
constexpr float kDegToRad = 0.017453292519943295f;

// Build one animation by name with the baked config amplitudes. Mirrors the
// posture node's overrides dict fused into _ANIMATION_FACTORIES: known name ->
// configured instance, unknown -> nullptr (the caller drops it, matching the
// Python builder's rejection of unknown names — the config is trusted, so an
// unknown name means a codegen/typo bug rather than user input).
std::shared_ptr<const Animation> make_animation(std::string_view name,
                                                const config::PostureConfig& p) {
  if (name == "still") {
    return std::make_shared<Still>();
  }
  if (name == "breathing") {
    return std::make_shared<Breathing>();
  }
  if (name == "gait_sway") {
    return std::make_shared<GaitSway>(p.gait_sway_gain, p.gait_sway_strength);
  }
  if (name == "gait_bounce") {
    return std::make_shared<GaitBounce>(p.gait_bounce_arc_height,
                                        p.gait_bounce_step_height_ref);
  }
  if (name == "vertical_body_roll") {
    return std::make_shared<VerticalBodyRoll>(
        p.vertical_body_roll_z_amplitude,
        p.vertical_body_roll_pitch_amplitude_deg * kDegToRad,
        p.vertical_body_roll_phase_offset);
  }
  if (name == "horizontal_body_roll") {
    return std::make_shared<HorizontalBodyRoll>(
        p.horizontal_body_roll_y_amplitude,
        p.horizontal_body_roll_yaw_amplitude_deg * kDegToRad,
        p.horizontal_body_roll_phase_offset);
  }
  if (name == "body_roll_3d") {
    return std::make_shared<BodyRoll3D>(
        p.body_roll_3d_z_amplitude,
        p.body_roll_3d_pitch_amplitude_deg * kDegToRad,
        p.body_roll_3d_y_amplitude,
        p.body_roll_3d_yaw_amplitude_deg * kDegToRad,
        p.body_roll_3d_horizontal_phase_offset,
        p.body_roll_3d_pitch_phase_offset, p.body_roll_3d_yaw_phase_offset);
  }
  return nullptr;
}

// Compose a stack from a list of animation names. Mirrors
// _build_animation_stack (unknown names are skipped rather than raised — see
// make_animation).
Stack build_stack(const std::vector<std::string_view>& names,
                  const config::PostureConfig& p) {
  std::vector<std::shared_ptr<const Animation>> layers;
  layers.reserve(names.size());
  for (const auto& name : names) {
    if (auto a = make_animation(name, p)) {
      layers.push_back(std::move(a));
    }
  }
  return Stack(std::move(layers));
}
}  // namespace

std::optional<std::pair<float, float>> stance_centroid_xy(
    const std::map<std::string, gait::LegOutput>& legs) {
  float sx = 0.0f;
  float sy = 0.0f;
  int n = 0;
  for (const auto& [name, leg] : legs) {
    if (leg.stance) {
      sx += leg.foot_target.x;
      sy += leg.foot_target.y;
      ++n;
    }
  }
  if (n < kMinStanceForCentroid) {
    return std::nullopt;
  }
  const float fn = static_cast<float>(n);
  return std::make_pair(sx / fn, sy / fn);
}

std::optional<float> max_swing_lift_z(
    const std::map<std::string, gait::LegOutput>& legs) {
  float stance_sum = 0.0f;
  int stance_n = 0;
  bool any_swing = false;
  float max_swing = 0.0f;
  for (const auto& [name, leg] : legs) {
    if (leg.stance) {
      stance_sum += leg.foot_target.z;
      ++stance_n;
    } else {
      if (!any_swing || leg.foot_target.z > max_swing) {
        max_swing = leg.foot_target.z;
      }
      any_swing = true;
    }
  }
  if (stance_n < kMinStanceForCentroid) {
    return std::nullopt;
  }
  if (!any_swing) {
    return 0.0f;
  }
  const float ground = stance_sum / static_cast<float>(stance_n);
  const float lift = max_swing - ground;
  return lift > 0.0f ? lift : 0.0f;
}

std::optional<std::pair<float, float>> lpf_step_xy(
    std::optional<std::pair<float, float>> prev,
    std::optional<std::pair<float, float>> raw, float tau, float dt) {
  if (!raw.has_value()) {
    return prev;
  }
  if (!prev.has_value()) {
    return raw;
  }
  const float denom = tau + dt;
  const float alpha = denom > 0.0f ? dt / denom : 1.0f;
  const auto [px, py] = *prev;
  const auto [rx, ry] = *raw;
  return std::make_pair(px + alpha * (rx - px), py + alpha * (ry - py));
}

std::optional<float> lpf_step_scalar(std::optional<float> prev,
                                     std::optional<float> raw, float tau,
                                     float dt) {
  if (!raw.has_value()) {
    return prev;
  }
  if (!prev.has_value()) {
    return raw;
  }
  const float denom = tau + dt;
  const float alpha = denom > 0.0f ? dt / denom : 1.0f;
  return *prev + alpha * (*raw - *prev);
}

bool posture_active(gait::EngineState state) {
  using E = gait::EngineState;
  switch (state) {
    case E::STAND:
    case E::ENGAGING:
    case E::GAIT:
    case E::SETTLING:
    case E::RESEATING:
      return true;
    case E::FOLDED:
    case E::INITIALIZE:
    case E::FOLDING:
    case E::FAULT:
      return false;
  }
  return false;
}

float slew_toward(float current, float target, float rate_per_s, float dt) {
  if (rate_per_s <= 0.0f || dt <= 0.0f) {
    return current;
  }
  const float step = rate_per_s * dt;
  if (target > current) {
    return std::min(target, current + step);
  }
  return std::max(target, current - step);
}

PostureController::PostureController()
    : PostureController(config::kPosture) {}

PostureController::PostureController(const config::PostureConfig& p)
    : limits_{p.pose_limit_x, p.pose_limit_y,
              // The ONE place absolute belly clearance becomes a pose offset:
              // BodyPose::z is a delta from the nominal stance, the config
              // states height off the ground. Everything downstream is offsets.
              p.body_height_max - p.nominal_body_height,
              p.body_height_min - p.nominal_body_height,
              p.pose_limit_roll, p.pose_limit_pitch, p.pose_limit_yaw},
      gait_body_animations_enabled_(p.gait_body_animations_enabled),
      activation_slew_rate_(p.gait_activation_slew_rate),
      // The animation reserve is symmetric on every axis including z, so it
      // spends the same budget up and down.
      anim_reserve_{p.animation_reserve_x,    p.animation_reserve_y,
                    p.animation_reserve_z,    -p.animation_reserve_z,
                    p.animation_reserve_roll, p.animation_reserve_pitch,
                    p.animation_reserve_yaw},
      centroid_tau_(p.support_centroid_tau),
      swing_lift_tau_(p.swing_lift_tau) {
  std::vector<std::string_view> enabled(config::kEnabledAnimations.begin(),
                                        config::kEnabledAnimations.end());
  default_stack_ = build_stack(enabled, p);

  // One dedicated stack per animation-mode entry: still + the named
  // animation(s), so gait_sway / gait_bounce don't bleed in while the user is
  // demoing a body animation. Comma-separated entries compose into one stack.
  for (std::string_view entry : config::kAnimationModeAnimations) {
    std::vector<std::string_view> names{"still"};
    std::size_t start = 0;
    while (start <= entry.size()) {
      std::size_t comma = entry.find(',', start);
      std::size_t end = comma == std::string_view::npos ? entry.size() : comma;
      std::string_view part = entry.substr(start, end - start);
      // Trim surrounding whitespace (mirrors the Python n.strip()).
      while (!part.empty() && part.front() == ' ') {
        part.remove_prefix(1);
      }
      while (!part.empty() && part.back() == ' ') {
        part.remove_suffix(1);
      }
      if (!part.empty()) {
        names.push_back(part);
      }
      if (comma == std::string_view::npos) {
        break;
      }
      start = comma + 1;
    }
    animation_stacks_.emplace(std::string(entry), build_stack(names, p));
  }
}

bool PostureController::set_animation_mode(std::string_view mode) {
  if (!mode.empty() &&
      animation_stacks_.find(std::string(mode)) == animation_stacks_.end()) {
    return false;
  }
  animation_mode_.assign(mode);
  return true;
}

BodyPose PostureController::update(
    const std::map<std::string, gait::LegOutput>& legs, float master_phase,
    bool walking, gait::EngineState state, std::string_view gait_name, float dt,
    float t) {
  // Derive the raw signals from the engine output; hold the previous raw
  // through a degenerate frame (mirrors _on_leg_targets guarding on None).
  if (auto raw = stance_centroid_xy(legs)) {
    latest_raw_centroid_ = raw;
  }
  if (auto raw = max_swing_lift_z(legs)) {
    latest_raw_swing_lift_ = raw;
  }

  // Advance the low-passes on the node's own cadence.
  support_centroid_xy_ =
      lpf_step_xy(support_centroid_xy_, latest_raw_centroid_, centroid_tau_, dt);
  swing_lift_z_ = lpf_step_scalar(swing_lift_z_, latest_raw_swing_lift_,
                                  swing_lift_tau_, dt);

  if (!posture_active(state)) {
    // Legs aren't at nominal stance — a body-pose offset would compose against
    // the wrong foot configuration. Hold IDENTITY and reset the crossfade so
    // the gait animations start faded-out next time posture re-activates.
    activation_ = 0.0f;
    return IDENTITY;
  }

  // Ramp the gait animations in/out instead of stepping at the walking edge.
  activation_ =
      slew_toward(activation_, walking ? 1.0f : 0.0f, activation_slew_rate_, dt);

  AnimationContext ctx;
  ctx.t = t;
  ctx.gait_name = gait_name;
  ctx.support_centroid_xy = support_centroid_xy_;
  ctx.swing_lift_z = swing_lift_z_;
  ctx.master_phase = gait::pymod(master_phase, 1.0f);

  const Stack& stack =
      animation_mode_.empty() ? default_stack_
                              : animation_stacks_.at(animation_mode_);
  // Evaluate the stack in both regimes and crossfade by the activation, so the
  // gait-active animations fade against the idle ones (the node owns the
  // transition, not the animations).
  ctx.walking = true;
  // Master switch: with gait body animations off the DEFAULT stack's gait
  // regime contributes nothing, so the crossfade fades the idle animations out
  // to a still body instead of into the walking ones. An explicitly selected
  // ANIMATION-mode stack is exempt — the user asked for that animation, so the
  // switch only governs the implicit gait_sway/gait_bounce layers. The
  // activation still tracks `walking`, so the fade itself (and its reversal on
  // stopping) is unchanged.
  const bool gait_regime_suppressed =
      !gait_body_animations_enabled_ && animation_mode_.empty();
  const BodyPose gait_out = gait_regime_suppressed ? IDENTITY : stack.eval(ctx);
  ctx.walking = false;
  const BodyPose idle_out = stack.eval(ctx);
  const BodyPose animated = lerp(idle_out, gait_out, activation_);

  // Layered clamp: the user pose and the animation each get their own budget.
  return compose_layered(user_pose_, animated, limits_, anim_reserve_);
}

}  // namespace hexa::posture
