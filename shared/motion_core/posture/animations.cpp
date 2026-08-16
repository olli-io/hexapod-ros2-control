#include "posture/animations.hpp"

#include <cmath>

namespace hexa::posture {

namespace {
constexpr float kTwoPi = 6.283185307179586f;
}  // namespace

BodyPose Still::eval(const AnimationContext&) const { return IDENTITY; }

BodyPose Breathing::eval(const AnimationContext& ctx) const {
  if (ctx.walking) {
    return IDENTITY;
  }
  const float omega = kTwoPi / period_;
  return BodyPose{0.0f, 0.0f, amplitude_ * std::sin(omega * ctx.t)};
}

BodyPose GaitSway::eval(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.support_centroid_xy.has_value()) {
    return IDENTITY;
  }
  const float k = gain_ * strength_;
  if (k == 0.0f) {
    return IDENTITY;
  }
  const auto [cx, cy] = *ctx.support_centroid_xy;
  return BodyPose{k * cx, k * cy};
}

BodyPose GaitBounce::eval(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.swing_lift_z.has_value()) {
    return IDENTITY;
  }
  if (ctx.gait_name != "tripod") {
    return IDENTITY;
  }
  if (arc_height_ == 0.0f || step_height_ref_ <= 0.0f) {
    return IDENTITY;
  }
  float ratio = *ctx.swing_lift_z / step_height_ref_;
  if (ratio < 0.0f) {
    ratio = 0.0f;
  } else if (ratio > 1.0f) {
    ratio = 1.0f;
  }
  return BodyPose{0.0f, 0.0f, arc_height_ * ratio};
}

BodyPose VerticalBodyRoll::eval(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.master_phase.has_value()) {
    return IDENTITY;
  }
  if (ctx.gait_name != "tripod") {
    return IDENTITY;
  }
  const float phi = *ctx.master_phase;
  BodyPose out;
  out.z = -z_amplitude_ * std::cos(kTwoPi * phi);
  out.pitch = -pitch_amplitude_ * std::sin(kTwoPi * (phi + pitch_phase_offset_));
  return out;
}

BodyPose HorizontalBodyRoll::eval(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.master_phase.has_value()) {
    return IDENTITY;
  }
  if (ctx.gait_name != "tripod") {
    return IDENTITY;
  }
  const float phi = *ctx.master_phase;
  BodyPose out;
  out.y = -y_amplitude_ * std::cos(kTwoPi * phi);
  out.yaw = yaw_amplitude_ * std::sin(kTwoPi * (phi + yaw_phase_offset_));
  return out;
}

BodyPose BodyRoll3D::eval(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.master_phase.has_value()) {
    return IDENTITY;
  }
  if (ctx.gait_name != "tripod") {
    return IDENTITY;
  }
  const float phi = *ctx.master_phase;
  const float phi_h = phi + horizontal_phase_offset_;
  BodyPose out;
  out.z = -z_amplitude_ * std::cos(kTwoPi * phi);
  out.pitch = -pitch_amplitude_ * std::sin(kTwoPi * (phi + pitch_phase_offset_));
  out.y = -y_amplitude_ * std::cos(kTwoPi * phi_h);
  out.yaw = yaw_amplitude_ * std::sin(kTwoPi * (phi_h + yaw_phase_offset_));
  return out;
}

BodyPose Stack::eval(const AnimationContext& ctx) const {
  BodyPose out = IDENTITY;
  for (const auto& layer : layers_) {
    out = add(out, layer->eval(ctx));
  }
  return out;
}

}  // namespace hexa::posture
