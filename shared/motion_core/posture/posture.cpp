#include "posture/posture.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "config_generated.hpp"

namespace hexa::posture {

namespace {
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kPi = 3.141592653589793f;
// Below this a pair has no meaningful direction — the same floor pose.cpp's
// polar easing uses, and for the same reason.
constexpr float kPolarEps = 1e-6f;

// Unknown name -> nullptr, which the caller drops: the config is trusted, so an
// unknown name is a codegen/typo bug rather than user input.
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
  if (name == "support_shift") {
    return std::make_shared<SupportShift>(p.support_shift_gain);
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

// Unknown names are skipped rather than raised — see make_animation.
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
    if (leg.parked) {
      continue;  // carries no weight, so it is not a vertex of anything
    }
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

std::optional<std::pair<float, float>> anticipated_support_xy(
    const std::map<std::string, gait::LegOutput>& legs, float lead) {
  float sx = 0.0f;
  float sy = 0.0f;
  float total = 0.0f;
  int n = 0;
  for (const auto& [name, leg] : legs) {
    if (leg.parked || !leg.stance) {
      continue;
    }
    ++n;
    // phase runs 0 at lift-off up to 1, so 1 - phase is the runway left. A zero
    // lead makes every weight 1 and this the plain centroid.
    const float w =
        lead > 0.0f ? std::clamp((1.0f - leg.phase) / lead, 0.0f, 1.0f) : 1.0f;
    sx += w * leg.foot_target.x;
    sy += w * leg.foot_target.y;
    total += w;
  }
  if (n < kMinStanceForCentroid) {
    return std::nullopt;
  }
  // Only reachable if every stance leg is exactly at its lift-off; the plain
  // centroid is the honest answer for that tick.
  if (total <= 0.0f) {
    return stance_centroid_xy(legs);
  }
  return std::make_pair(sx / total, sy / total);
}

std::optional<float> max_swing_lift_z(
    const std::map<std::string, gait::LegOutput>& legs) {
  float stance_sum = 0.0f;
  int stance_n = 0;
  bool any_swing = false;
  float max_swing = 0.0f;
  for (const auto& [name, leg] : legs) {
    // A parked foot sits a quarter of a metre up. Counting it as a swing leg
    // would peg max_swing every tick and heave the body with it.
    if (leg.parked) {
      continue;
    }
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

std::optional<std::pair<float, float>> lpf_step_polar_xy(
    std::optional<std::pair<float, float>> prev,
    std::optional<std::pair<float, float>> raw, float tau, float dt) {
  if (!raw.has_value()) {
    return prev;
  }
  if (!prev.has_value() || tau <= 0.0f) {
    return raw;
  }
  const float denom = tau + dt;
  const float alpha = denom > 0.0f ? dt / denom : 1.0f;
  const auto [px, py] = *prev;
  const auto [rx, ry] = *raw;

  const float prev_r = std::hypot(px, py);
  const float raw_r = std::hypot(rx, ry);
  const float prev_a = prev_r > kPolarEps ? std::atan2(py, px) : 0.0f;
  // A raw target at the origin has no direction to sweep toward; hold the one
  // the value already has and let the radius run down to meet it.
  const float raw_a = raw_r > kPolarEps ? std::atan2(ry, rx) : prev_a;

  const float r = prev_r + alpha * (raw_r - prev_r);
  // Short way round, so a target crossing +/-pi does not sweep the long arc.
  const float a =
      prev_a + alpha * std::remainder(raw_a - prev_a, 2.0f * kPi);
  return std::make_pair(r * std::cos(a), r * std::sin(a));
}

bool posture_active(gait::EngineState state) {
  using E = gait::EngineState;
  switch (state) {
    case E::STAND:
    case E::ENGAGING:
    case E::GAIT:
    case E::SETTLING:
    case E::RESEATING:
    // The pair is moving but the body is standing on a full stance, and the
    // caller pins the user pose to identity here anyway. Going inactive would
    // reset the smoother and emit IDENTITY in one tick — a step from whatever
    // it was holding, which is exactly the jump the pin exists to avoid.
    case E::FOLDING_PAIR:
    case E::UNFOLDING_PAIR:
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
              // The one place absolute belly clearance becomes a pose offset:
              // BodyPose::z is a delta from the nominal stance.
              p.body_height_max - p.nominal_body_height,
              p.body_height_min - p.nominal_body_height,
              p.pose_limit_roll, p.pose_limit_pitch, p.pose_limit_yaw},
      gait_body_animations_enabled_(p.gait_body_animations_enabled),
      activation_slew_rate_(p.gait_activation_slew_rate),
      centroid_tau_(p.support_centroid_tau),
      swing_lift_tau_(p.swing_lift_tau),
      support_shift_lead_(p.support_shift_lead),
      support_shift_tau_(p.support_shift_tau) {
  pose_smoother_ = PoseSmoother(PoseSmootherConfig{
      p.pose_filter_tau, p.pose_filter_damping_ratio,
      p.pose_filter_snap_tol_linear, p.pose_filter_snap_tol_angular});

  std::vector<std::string_view> enabled(config::kEnabledAnimations.begin(),
                                        config::kEnabledAnimations.end());
  default_stack_ = build_stack(enabled, p);

  // Quadruped mode's stack is not configurable, and deliberately so: the support
  // shift is what holds the robot up on four feet, not a layer someone may want
  // to turn off. gait_bounce is absent for the same reason — a heave would spend
  // margin the creep has not got.
  quadruped_stack_ = build_stack({"support_shift"}, p);

  // still + the named animation(s), so gait_sway / gait_bounce cannot bleed into
  // a body animation the user is demoing. Comma-separated entries compose.
  for (std::string_view entry : config::kAnimationModeAnimations) {
    std::vector<std::string_view> names{"still"};
    std::size_t start = 0;
    while (start <= entry.size()) {
      std::size_t comma = entry.find(',', start);
      std::size_t end = comma == std::string_view::npos ? entry.size() : comma;
      std::string_view part = entry.substr(start, end - start);
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
    bool walking, gait::EngineState state, std::string_view gait_name,
    gait::LegSet leg_set, float dt, float t) {
  // Hold the previous raw through a degenerate frame.
  if (auto raw = stance_centroid_xy(legs)) {
    latest_raw_centroid_ = raw;
  }
  if (auto raw = anticipated_support_xy(legs, support_shift_lead_)) {
    latest_raw_anticipated_ = raw;
  }
  if (auto raw = max_swing_lift_z(legs)) {
    latest_raw_swing_lift_ = raw;
  }

  support_centroid_xy_ =
      lpf_step_xy(support_centroid_xy_, latest_raw_centroid_, centroid_tau_, dt);
  // Its own filter, and a polar one: this signal walks a rough circle around the
  // body — each handover passes the target on to the next support triangle — and
  // the body follows it directly. Lagged per-axis, every one of those turns comes
  // out as a corner; lagged in polar the body arcs through them.
  anticipated_support_xy_ = lpf_step_polar_xy(
      anticipated_support_xy_, latest_raw_anticipated_, support_shift_tau_, dt);
  swing_lift_z_ = lpf_step_scalar(swing_lift_z_, latest_raw_swing_lift_,
                                  swing_lift_tau_, dt);

  if (!posture_active(state)) {
    // The legs are not at nominal stance, so an offset would compose against the
    // wrong foot configuration. Reset the crossfade so the gait animations start
    // faded out when posture re-activates.
    activation_ = 0.0f;
    // The filter state has to agree with the emitted IDENTITY, or the body
    // springs from a stale offset on the way back.
    pose_smoother_.reset();
    return IDENTITY;
  }

  activation_ =
      slew_toward(activation_, walking ? 1.0f : 0.0f, activation_slew_rate_, dt);

  AnimationContext ctx;
  ctx.t = t;
  ctx.gait_name = gait_name;
  ctx.support_centroid_xy = support_centroid_xy_;
  ctx.anticipated_support_xy = anticipated_support_xy_;
  ctx.swing_lift_z = swing_lift_z_;
  ctx.master_phase = gait::pymod(master_phase, 1.0f);

  // Selected on the LEG SET, not the gait name: what makes the support shift
  // mandatory is having four feet, and a string compare would put a safety
  // decision behind a spelling. An explicitly selected animation mode still
  // wins — the operator asked for it — but nothing else can displace it.
  const bool quadruped = (leg_set == gait::LegSet::QUADRUPED);
  const Stack& stack = !animation_mode_.empty()
                           ? animation_stacks_.at(animation_mode_)
                           : (quadruped ? quadruped_stack_ : default_stack_);
  // Both regimes, crossfaded by the activation, so the gait-active animations
  // fade against the idle ones — the controller owns the transition, not the
  // animations.
  ctx.walking = true;
  // With gait body animations off the default stack's gait regime contributes
  // nothing, so the crossfade fades the idle animations out to a still body. An
  // explicitly selected animation mode is exempt: the switch only governs the
  // implicit gait_sway/gait_bounce layers. So is the quadruped stack — that one
  // is not an embellishment the operator may decline, it is what keeps the robot
  // upright on four feet.
  const bool gait_regime_suppressed =
      !gait_body_animations_enabled_ && animation_mode_.empty() && !quadruped;
  const BodyPose gait_out = gait_regime_suppressed ? IDENTITY : stack.eval(ctx);
  ctx.walking = false;
  const BodyPose idle_out = stack.eval(ctx);
  const BodyPose animated = lerp(idle_out, gait_out, activation_);

  // The user pose applies on four feet exactly as it does on six. It comes off
  // the same envelope as the support shift, so what keeps the creep's travel
  // intact is upstream: the teleop refuses a posture RECORD while quadruped, so
  // nothing but body height rides into gait mode and the live pose is only ever
  // held in posture mode, where the robot is not walking.
  // Clamped going in as well as out, so the filter never winds up against an
  // unreachable value.
  const BodyPose user_smoothed =
      pose_smoother_.step(clamp(user_pose_, limits_), limits_, dt);

  // One shared envelope: the user pose and the animation both spend it, and the
  // sum is clamped to it.
  return clamp(add(user_smoothed, animated), limits_);
}

}  // namespace hexa::posture
