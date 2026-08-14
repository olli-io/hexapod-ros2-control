// Command shaping across a travel reversal.
//
// Turning the command around under a walking robot is not a velocity change like
// any other. Every planted foot is somewhere between its AEP and its PEP, and
// the moment travel reverses, the ones that landed most recently have no
// excursion left in the new direction at all: their anchors pin against the
// stance ceiling and stop tracking the ground for the rest of their stance
// window. Half the legs drag.
//
// The fix is not to re-plant them but to re-register the *schedule* against
// them: reflected about the swing end, a leg's stance progress s becomes 1 - s,
// so the runway it has left in the new direction is the runway it just consumed
// in the old one — which is exactly where its foot is standing. A leg at
// touchdown maps to lift-off, because the old AEP it stands on is the new PEP.
// One reflection fixes all six at once (GaitClock::mirror).
//
// That identity holds only where the foot is where the schedule says it is,
// e == (0.5 - s) * stride. Two things have to be true for that:
//
//  - **Nothing in the air.** The reflection reverses swing progress, and a
//    swing's origin end is latched at lift-off.
//  - **The stride is the one the leg has been walking, and the one it will keep
//    walking.** Stride is pinned at stride_length across the whole band from the
//    knee — where derive_cycle_time stops stretching the cycle — up to the
//    velocity cap, and the clock is distance-locked throughout it, so the
//    identity is exact there to within a millimetre. Below the knee the cycle
//    time saturates, the clock outruns the travel, and the feet bunch toward
//    nominal: measured 24-29 mm off at 40% of the knee speed. Reflecting there
//    hands every leg tens of millimetres of runway it does not have, and it
//    ploughs into its excursion ceiling — inward, under the body, for a middle
//    leg walking sideways.
//
// So the ladder does not bleed the walk out before mirroring: it *holds* it at
// the knee — the slowest speed that still walks a full stride — waits for the
// gait's next all-down window, reflects there, and releases. Holding rather than
// stopping is the whole point; the stride the reflection is computed against is
// then the same stride the legs go on to walk.
//
// A reversal already below the knee is not held: its feet are canonical for its
// own shorter stride, but the walk it is being asked for is a longer one, so the
// reflection would over-credit them just the same. Its excursions are smaller in
// proportion, and it reverses as it always did. So do gaits whose swings run end
// to end (crawl, surf) and never have all six feet down.
#pragma once

#include <map>
#include <string>
#include <utility>

#include "gait/gaits/base.hpp"

namespace hexa::gait {

// The fastest any leg travels under this command.
float max_leg_speed(const std::map<std::string, LegContext>& legs,
                    std::pair<float, float> v_xy, float omega);

// Does `request` reverse the travel of every leg, relative to `reference`?
//
// Asked per leg rather than per axis because that is where the answer differs: a
// yaw command flipping under a straight walk turns each leg a little without
// reversing any of them, while a yaw flip on the spot reverses all six. Legs
// carrying less than `zero_tol` either way are not travelling, so nothing about
// them is being reversed.
bool travel_reverses(const std::map<std::string, LegContext>& legs,
                     std::pair<float, float> request_xy, float request_omega,
                     std::pair<float, float> reference_xy,
                     float reference_omega, float zero_tol);

// The ladder, as a pure state machine: it is told what the robot is carrying,
// what is being asked of it, and whether the gait has all six feet down; it
// answers with the command to run and whether to reflect the phase circle now.
class ReversalGate {
 public:
  struct Input {
    // What the robot is actually carrying — the velocity shaper's own state, not
    // the request, so the ladder's reference is the travel being reversed.
    std::pair<float, float> applied_xy{0.0f, 0.0f};
    float applied_omega = 0.0f;
    std::pair<float, float> request_xy{0.0f, 0.0f};
    float request_omega = 0.0f;
    bool walking = false;      // the engine is in GAIT
    bool all_planted = false;  // no foot in the air right now
    bool can_mirror = false;   // this gait has an all-down window at all
    // Leg speed below which the cycle time saturates and the feet stop tracking
    // the schedule. The ladder holds the walk here and reflects here.
    float knee_speed = 0.0f;
    // How long to wait for a window before giving up and letting the command
    // through unreflected.
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
  // One reversal is handled once. The tick after it fires, the robot is still
  // carrying the old travel while the stick still asks for the new one, so the
  // arm test is still true — and re-arming there would hold the walk back at the
  // knee and reflect the schedule straight back to where it started. This stays
  // set until the command stops opposing the travel the ladder began from, which
  // is to say until the reversal it was raised for has actually happened.
  bool handled_ = false;
  // The travel latched at arm time, and the scale that walks it at the knee.
  // Every later test is against this, not against the live command: the hold
  // changes the live one, and a direction read off it would drift with it.
  std::pair<float, float> hold_xy_{0.0f, 0.0f};
  float hold_omega_ = 0.0f;
  float hold_scale_ = 1.0f;
  float held_for_ = 0.0f;
};

}  // namespace hexa::gait
