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

  if (armed_) {
    // Not while engaging: the timeout is the backstop for a gait that never
    // offers an all-down window, and an engagement offers none by construction.
    // Counting it here would spend the whole budget before the walk it hands
    // over to gets its first one.
    if (!in.engaging) {
      held_for_ += in.dt;
    }
    // Out of the walk, no longer opposed, or no longer able to reflect: let go.
    // The timeout is the backstop — an unreflected reversal is the old
    // behaviour, not a broken one.
    const bool still_reversing =
        in.walking && in.can_mirror && held_for_ < in.timeout &&
        travel_reverses(legs, in.request_xy, in.request_omega, hold_xy_,
                        hold_omega_, in.zero_tol);
    if (!still_reversing) {
      armed_ = false;
      handled_ = false;
      return pass;
    }
  } else if (handled_) {
    handled_ = in.walking &&
               travel_reverses(legs, in.request_xy, in.request_omega, hold_xy_,
                               hold_omega_, in.zero_tol);
    return pass;
  } else {
    if (!in.walking ||
        !travel_reverses(legs, in.request_xy, in.request_omega, in.applied_xy,
                         in.applied_omega, in.zero_tol)) {
      return pass;
    }
    // Latched before the hold is decided, because a reversal this ladder cannot
    // help is still a reversal: it is what reversing() reports and what the
    // clear test below measures the request against.
    hold_xy_ = in.applied_xy;
    hold_omega_ = in.applied_omega;
    const float carrying =
        max_leg_speed(legs, in.applied_xy, in.applied_omega);
    // Below the knee the walk is already on a shorter stride than the one asked
    // for next, so the reflection would over-credit every leg. Recognised, not
    // held: the command goes through as it always did.
    if (!in.can_mirror || carrying < in.knee_speed) {
      handled_ = true;
      return pass;
    }
    armed_ = true;
    held_for_ = 0.0f;
    hold_scale_ = in.knee_speed / carrying;
  }

  // At the hold speed with every foot planted and the schedule true of the feet,
  // reflect one onto the other and hand the command back. Both of the first two
  // are stated: `engaging` because the clock this reflects is not the one the
  // engagement runs, `feet_on_schedule` because the engagement hands the walk legs
  // it has not yet squared up. One implies the other today; they are two different
  // reasons and neither is the other's shorthand.
  const float carrying = max_leg_speed(legs, in.applied_xy, in.applied_omega);
  if (!in.engaging && in.feet_on_schedule && in.all_planted &&
      carrying <= in.knee_speed * kHoldTolerance) {
    armed_ = false;
    handled_ = true;
    return {in.request_xy, in.request_omega, true};
  }

  // Otherwise hold the carried travel, slowed to the knee. Not stopped: the
  // reflection is only exact against a stride the legs are actually walking.
  return {{hold_xy_.first * hold_scale_, hold_xy_.second * hold_scale_},
          hold_omega_ * hold_scale_,
          false};
}

}  // namespace hexa::gait
