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
//
// The ladder spans the engagement as well as the walk. It holds there — the
// engagement re-plans off the live command every tick, so a hold at the knee
// simply walks that ladder out at the knee — but it cannot reflect there: the
// engagement runs its own master clock, and the handoff into GAIT reseeds the
// gait clock this reflects. So the hold carries across the handoff and reflects
// on the other side.
//
// Not at the first all-down window there, though. The engagement's body-velocity
// envelope outlasts the first touchdowns on every gait but tripod, and a foot that
// landed under a half-open envelope has covered less ground than its phase has
// spent — 31% of a stride on a ripple's rear leg. Those legs need one swing under
// the walk before the reflection may register anything against them, which is
// what EngagementController::foot_on_schedule reports and Input::feet_on_schedule
// carries here.
//
// Recognising a reversal is worth something even where the ladder declines to
// hold one, which is what reversing() is for. The velocity limiter slews the
// planar command vector straight through the origin, so every sign flip spends a
// tenth of a second or more inside cmd_zero_tol; without the latch the engine
// reads that crossing as a released stick and re-plants a robot that is only
// turning around.
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
    bool walking = false;      // the engine is running a walk: GAIT or ENGAGING
    // ... and that walk is still the engagement ladder. Two things are suspended
    // there: the reflection, which acts on the gait clock the engagement does not
    // use and the handoff reseeds anyway, and the timeout, since an engagement
    // has no all-down window to miss.
    bool engaging = false;
    // The reflection's other premise: every planted foot stands where its stance
    // progress says. Separate from `engaging` because it outlives it — the
    // engagement hands the walk legs whose first stance it walked under a
    // half-open velocity envelope, so their excursion lags their phase, and one
    // swing under the walk is what squares them up. Unlike `engaging` it does not
    // stop the timeout: waiting for it is waiting inside the walk, which is what
    // the two-cycle budget is for.
    bool feet_on_schedule = false;
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
  // A reversal is in flight, held or not. What the engine reads to keep a command
  // sweeping through zero from arming a settle: the reversal is what the operator
  // asked for, and the crossing is only the shaper on its way there.
  bool reversing() const { return armed_ || handled_; }
  // Both flags: handled_ is the in-flight latch the engine reads, and a ladder
  // torn down with the walk is not holding a reversal for it.
  void reset() {
    armed_ = false;
    handled_ = false;
  }

 private:
  bool armed_ = false;
  // One reversal is handled once: the tick after it fires the arm test is still
  // true (old travel, new request) and re-arming would reflect straight back.
  // Also set where the ladder declines to hold — below the knee, or a gait with
  // no all-down window — since the reversal happened either way and only the
  // reflection did not. Clears when the command stops opposing the travel the
  // ladder began from.
  bool handled_ = false;
  // The travel this reversal turns away from, latched whether or not it is held,
  // and the scale that walks it at the knee. Later tests read this, not the live
  // command, which the hold itself moves.
  std::pair<float, float> hold_xy_{0.0f, 0.0f};
  float hold_omega_ = 0.0f;
  float hold_scale_ = 1.0f;
  float held_for_ = 0.0f;
};

}  // namespace hexa::gait
