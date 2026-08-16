// Command shaping across a travel reversal.
//
// When travel reverses, the legs that landed most recently have no excursion
// left in the new direction: their anchors pin against the stance ceiling and
// half the legs drag. The fix is to re-register the schedule instead of
// re-planting them — reflected about the swing end (GaitClock::mirror), stance
// progress s becomes 1 - s, so a leg's remaining runway is the runway it just
// consumed, which is where its foot is standing.
//
// That identity needs the feet where the schedule says they are, so nothing may
// be in the air (a swing's origin is latched at lift-off) and the stride must be
// the one the legs both have been and will be walking. Stride is pinned from the
// knee up to the velocity cap; below the knee the clock outruns the travel and
// the feet bunch toward nominal (24-29 mm off at 40% of knee speed), so
// reflecting there hands every leg runway it does not have.
//
// Hence the ladder: hold the walk at the knee, wait for the gait's next all-down
// window, reflect, release. A reversal already below the knee is not held (its
// feet are canonical for a shorter stride than the one being asked for), nor are
// gaits that never have all six feet down (crawl, surf).
#pragma once

#include <map>
#include <string>
#include <utility>

#include "gait/gaits/base.hpp"

namespace hexa::gait {

float max_leg_speed(const std::map<std::string, LegContext>& legs,
                    std::pair<float, float> v_xy, float omega);

// Does `request` reverse the travel of every leg, relative to `reference`? Asked
// per leg, not per axis: a yaw flip under a straight walk reverses no leg, while
// a yaw flip on the spot reverses all six. Legs slower than `zero_tol` either
// way are not travelling.
bool travel_reverses(const std::map<std::string, LegContext>& legs,
                     std::pair<float, float> request_xy, float request_omega,
                     std::pair<float, float> reference_xy,
                     float reference_omega, float zero_tol);

// The ladder as a pure state machine: given what the robot carries, what is
// asked of it and whether the gait has all six feet down, it answers with the
// command to run and whether to reflect the phase circle now.
class ReversalGate {
 public:
  struct Input {
    // The velocity shaper's own state, not the request: the reference is the
    // travel being reversed.
    std::pair<float, float> applied_xy{0.0f, 0.0f};
    float applied_omega = 0.0f;
    std::pair<float, float> request_xy{0.0f, 0.0f};
    float request_omega = 0.0f;
    bool walking = false;      // the engine is in GAIT
    bool all_planted = false;  // no foot in the air right now
    bool can_mirror = false;   // this gait has an all-down window at all
    // Leg speed below which the feet stop tracking the schedule; the ladder
    // holds the walk here and reflects here.
    float knee_speed = 0.0f;
    // Wait for a window this long before letting the command through unreflected.
    float timeout = 0.0f;
    float zero_tol = 0.0f;
    float dt = 0.0f;
  };

  struct Output {
    std::pair<float, float> v_xy{0.0f, 0.0f};
    float omega = 0.0f;
    bool mirror = false;
  };

  Output step(const std::map<std::string, LegContext>& legs, const Input& in);
  bool armed() const { return armed_; }
  void reset() { armed_ = false; }

 private:
  bool armed_ = false;
  // One reversal is handled once: the tick after it fires the arm test is still
  // true (old travel, new request) and re-arming would reflect straight back.
  // Clears when the command stops opposing the travel the ladder began from.
  bool handled_ = false;
  // The travel latched at arm time, and the scale that walks it at the knee.
  // Later tests read this, not the live command, which the hold itself moves.
  std::pair<float, float> hold_xy_{0.0f, 0.0f};
  float hold_omega_ = 0.0f;
  float hold_scale_ = 1.0f;
  float held_for_ = 0.0f;
};

}  // namespace hexa::gait
