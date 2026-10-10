#include "gait/reversal.hpp"

#include <algorithm>
#include <cmath>

namespace hexa::gait {

namespace {

// How far apart two travel directions must be to count as a reversal rather than
// a turn. The reflection is exactly right for a leg that reverses and
// progressively wrong as the turn shortens; at 120 degrees the reversing
// component is still the larger half of the change.
constexpr float kReversalCos = -0.5f;

// Slack on the hold speed before the reflection may fire. The stride is flat
// across the band above the knee, so arriving a little fast costs nothing.
constexpr float kHoldTolerance = 1.05f;

// Share of the knee (or of a slower request) the shaped command must carry the
// other way before the crossing is over. The limiter lands asymptotically only
// on the request, never on the knee, so a hair short is arrival.
constexpr float kCrossingArrival = 0.95f;

float dot(std::pair<float, float> a, std::pair<float, float> b) {
  return a.first * b.first + a.second * b.second;
}

float norm(std::pair<float, float> v) { return std::hypot(v.first, v.second); }

}  // namespace

float max_leg_speed(const std::map<std::string, LegContext>& legs,
                    std::pair<float, float> v_xy, float omega) {
  float fastest = 0.0f;
  for (const auto& [name, v] : per_leg_planar_velocity(legs, v_xy, omega)) {
    (void)name;
    fastest = std::max(fastest, norm(v));
  }
  return fastest;
}

bool travel_reverses(const std::map<std::string, LegContext>& legs,
                     std::pair<float, float> request_xy, float request_omega,
                     std::pair<float, float> reference_xy,
                     float reference_omega, float zero_tol) {
  if (norm(reference_xy) + std::fabs(reference_omega) <= zero_tol ||
      norm(request_xy) + std::fabs(request_omega) <= zero_tol) {
    return false;
  }
  // A body still heading the same way with the yaw still turning the same way
  // cannot have reversed all six. Cheap enough for a steady walk to run every
  // tick instead of the two maps the per-leg test below builds.
  if (dot(request_xy, reference_xy) > 0.0f &&
      request_omega * reference_omega >= 0.0f) {
    return false;
  }
  const auto want = per_leg_planar_velocity(legs, request_xy, request_omega);
  const auto have = per_leg_planar_velocity(legs, reference_xy, reference_omega);
  for (const auto& [name, v] : have) {
    const float carried = norm(v);
    const float asked = norm(want.at(name));
    if (carried <= zero_tol || asked <= zero_tol) {
      return false;
    }
    if (dot(v, want.at(name)) > kReversalCos * carried * asked) {
      return false;
    }
  }
  return true;
}

ReversalGate::Output ReversalGate::step(
    const std::map<std::string, LegContext>& legs, const Input& in) {
  const Output pass{in.request_xy, in.request_omega, false};
  timer_ += in.dt;

  if (stage_ == Stage::IDLE) {
    if (!in.walking ||
        !travel_reverses(legs, in.request_xy, in.request_omega, in.applied_xy,
                         in.applied_omega, in.zero_tol)) {
      return pass;
    }
    hold_xy_ = in.applied_xy;
    hold_omega_ = in.applied_omega;
    const float carrying = max_leg_speed(legs, hold_xy_, hold_omega_);
    // Below the knee the stride is already shorter than the one asked for
    // next, so the mirror would over-credit every leg.
    if (!in.can_hold || carrying < in.knee_speed) {
      enter(Stage::RECOGNISED);
      return pass;
    }
    enter(Stage::HOLDING);
    hold_scale_ = in.knee_speed / carrying;
  } else if (!in.walking ||
             !travel_reverses(legs, in.request_xy, in.request_omega, hold_xy_,
                              hold_omega_, in.zero_tol)) {
    enter(Stage::IDLE);
    return pass;
  }

  switch (stage_) {
    case Stage::HOLDING: {
      if (in.engaging) {
        timer_ = 0.0f;
      }
      // The timeout is the backstop: an unmirrored reversal is the old
      // behaviour, not a broken one.
      if (!in.can_hold || timer_ >= in.timeout) {
        enter(Stage::RECOGNISED);
        return pass;
      }
      const float carrying =
          max_leg_speed(legs, in.applied_xy, in.applied_omega);
      if (in.ready && carrying <= in.knee_speed * kHoldTolerance) {
        enter(Stage::CROSSING);
        return {in.request_xy, in.request_omega, true};
      }
      // Slowed to the knee, not stopped: the mirror is only exact against a
      // stride the legs are walking.
      return {{hold_xy_.first * hold_scale_, hold_xy_.second * hold_scale_},
              hold_omega_ * hold_scale_,
              false};
    }
    case Stage::CROSSING: {
      // Over once the shaped command opposes the hold and carries the knee
      // again. Capped at the request, so a reversal into a creep slower than
      // the knee ends when the shaper has converged.
      const bool crossed =
          dot(in.applied_xy, hold_xy_) + in.applied_omega * hold_omega_ <= 0.0f;
      const float carrying =
          max_leg_speed(legs, in.applied_xy, in.applied_omega);
      const float target =
          kCrossingArrival *
          std::min(in.knee_speed,
                   max_leg_speed(legs, in.request_xy, in.request_omega));
      if ((crossed && carrying >= target) || timer_ >= in.timeout) {
        enter(Stage::RECOGNISED);
      }
      return pass;
    }
    default:
      return pass;
  }
}

}  // namespace hexa::gait
