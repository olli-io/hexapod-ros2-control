// Port of hexa_posture/animations/*.py and the stack builder in posture_node.py.
//
// Sign conventions follow the Python code (not the docstrings): the pitch term
// of VerticalBodyRoll/BodyRoll3D is a NEGATIVE sine; the yaw terms are POSITIVE.
#include "hexa_posture_cpp/animations.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace hexa_posture {

namespace {
constexpr double kTwoPi = 2.0 * M_PI;
}  // namespace

BodyPose Stack::operator()(const AnimationContext& ctx) const {
  BodyPose out = IDENTITY;
  for (const auto& layer : layers_) {
    out = add(out, (*layer)(ctx));
  }
  return out;
}

BodyPose Still::operator()(const AnimationContext&) const { return IDENTITY; }

BodyPose Breathing::operator()(const AnimationContext& ctx) const {
  if (ctx.walking) {
    return IDENTITY;
  }
  const double omega = kTwoPi / period_;
  BodyPose p;
  p.z = amplitude_ * std::sin(omega * ctx.t);
  return p;
}

BodyPose GaitSway::operator()(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.support_centroid_xy.has_value()) {
    return IDENTITY;
  }
  const double k = gain_ * strength_;
  if (k == 0.0) {
    return IDENTITY;
  }
  const auto [cx, cy] = *ctx.support_centroid_xy;
  BodyPose p;
  p.x = k * cx;
  p.y = k * cy;
  return p;
}

BodyPose GaitBounce::operator()(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.swing_lift_z.has_value()) {
    return IDENTITY;
  }
  if (!ctx.gait_name.has_value() || *ctx.gait_name != "tripod") {
    return IDENTITY;
  }
  if (arc_height_ == 0.0 || step_height_ref_ <= 0.0) {
    return IDENTITY;
  }
  double ratio = *ctx.swing_lift_z / step_height_ref_;
  if (ratio < 0.0) {
    ratio = 0.0;
  } else if (ratio > 1.0) {
    ratio = 1.0;
  }
  BodyPose p;
  p.z = arc_height_ * ratio;
  return p;
}

BodyPose VerticalBodyRoll::operator()(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.master_phase.has_value()) {
    return IDENTITY;
  }
  if (!ctx.gait_name.has_value() || *ctx.gait_name != "tripod") {
    return IDENTITY;
  }
  const double phi = *ctx.master_phase;
  BodyPose p;
  p.z = -z_amplitude_ * std::cos(kTwoPi * phi);
  p.pitch = -pitch_amplitude_ * std::sin(kTwoPi * (phi + pitch_phase_offset_));
  return p;
}

BodyPose HorizontalBodyRoll::operator()(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.master_phase.has_value()) {
    return IDENTITY;
  }
  if (!ctx.gait_name.has_value() || *ctx.gait_name != "tripod") {
    return IDENTITY;
  }
  const double phi = *ctx.master_phase;
  BodyPose p;
  p.y = -y_amplitude_ * std::cos(kTwoPi * phi);
  p.yaw = yaw_amplitude_ * std::sin(kTwoPi * (phi + yaw_phase_offset_));
  return p;
}

BodyPose BodyRoll3D::operator()(const AnimationContext& ctx) const {
  if (!ctx.walking || !ctx.master_phase.has_value()) {
    return IDENTITY;
  }
  if (!ctx.gait_name.has_value() || *ctx.gait_name != "tripod") {
    return IDENTITY;
  }
  const double phi = *ctx.master_phase;
  const double phi_h = phi + horizontal_phase_offset_;
  BodyPose p;
  p.z = -z_amplitude_ * std::cos(kTwoPi * phi);
  p.pitch = -pitch_amplitude_ * std::sin(kTwoPi * (phi + pitch_phase_offset_));
  p.y = -y_amplitude_ * std::cos(kTwoPi * phi_h);
  p.yaw = yaw_amplitude_ * std::sin(kTwoPi * (phi_h + yaw_phase_offset_));
  return p;
}

namespace {

using FactoryFn = std::function<AnimationPtr()>;

// Name -> default-constructed animation. Mirrors _ANIMATION_FACTORIES. Keyed by
// std::map so the "available" list in error messages comes out sorted.
const std::map<std::string, FactoryFn>& animation_factories() {
  static const std::map<std::string, FactoryFn> factories = {
      {"still", [] { return std::make_shared<Still>(); }},
      {"breathing", [] { return std::make_shared<Breathing>(); }},
      {"gait_sway", [] { return std::make_shared<GaitSway>(); }},
      {"gait_bounce", [] { return std::make_shared<GaitBounce>(); }},
      {"vertical_body_roll",
       [] { return std::make_shared<VerticalBodyRoll>(); }},
      {"horizontal_body_roll",
       [] { return std::make_shared<HorizontalBodyRoll>(); }},
      {"body_roll_3d", [] { return std::make_shared<BodyRoll3D>(); }},
  };
  return factories;
}

}  // namespace

Stack build_animation_stack(
    const std::vector<std::string>& names,
    const std::map<std::string, AnimationPtr>& overrides) {
  const auto& factories = animation_factories();
  std::vector<std::string> unknown;
  for (const auto& n : names) {
    if (factories.find(n) == factories.end()) {
      unknown.push_back(n);
    }
  }
  if (!unknown.empty()) {
    std::ostringstream oss;
    oss << "unknown animation(s) [";
    for (std::size_t i = 0; i < unknown.size(); ++i) {
      oss << (i ? ", " : "") << "'" << unknown[i] << "'";
    }
    oss << "]; available: [";
    bool first = true;
    for (const auto& [name, _] : factories) {
      oss << (first ? "" : ", ") << "'" << name << "'";
      first = false;
    }
    oss << "]";
    throw std::invalid_argument(oss.str());
  }
  std::vector<AnimationPtr> layers;
  layers.reserve(names.size());
  for (const auto& n : names) {
    const auto it = overrides.find(n);
    if (it != overrides.end()) {
      layers.push_back(it->second);
    } else {
      layers.push_back(factories.at(n)());
    }
  }
  return Stack(std::move(layers));
}

}  // namespace hexa_posture
