// The reversal ladder: command shaping across a travel reversal. Rationale in
// docs/leg-phases.md §3.
//
// Invariant: GaitClock::mirror maps stance progress s to 1 - s, so a leg's new
// runway is the runway it just consumed. That is exact only with every foot
// planted, on schedule, and on the stride pinned from the knee up. Hence the
// ladder: hold at the knee, mirror at the next all-down window, hold the gait
// clock through the crossing.
#pragma once

#include <map>
#include <string>
#include <utility>

#include "gait/gaits/base.hpp"

namespace hexa::gait {

// How far apart two travel directions must be to count as a reversal rather than
// a turn. The reflection is exactly right for a leg that reverses and
// progressively wrong as the turn shortens; at 120 degrees the reversing
// component is still the larger half of the change. The velocity limiter turns
// below it and passes through zero above it, so the two agree.
inline constexpr float kReversalCos = -0.5f;

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
// asked of it and whether the walk is ready, it answers with the command to run
// and whether to mirror the gait clock now.
class ReversalGate {
 public:
  enum class Stage {
    IDLE,
    // A reversal the ladder does not hold: below the knee, a gait with no
    // all-down window, or a hold that timed out. Also the stage after the
    // crossing. Latched until the request stops opposing the travel.
    RECOGNISED,
    // The carried travel walks on at the knee until the walk is ready. Spans
    // the engagement: its hold keeps a quadruped's support shift in step.
    HOLDING,
    // Mirrored; the shaped command passes through zero and the engine holds the
    // gait clock. Ends at the knee the other way, or on the timeout.
    CROSSING,
  };

  struct Input {
    // The velocity shaper's own state, not the request: the reference is the
    // travel being reversed.
    std::pair<float, float> applied_xy{0.0f, 0.0f};
    float applied_omega = 0.0f;
    std::pair<float, float> request_xy{0.0f, 0.0f};
    float request_omega = 0.0f;
    bool walking = false;   // GAIT or ENGAGING
    // The engagement has no all-down window, so the hold's timeout counts from
    // the handoff.
    bool engaging = false;
    bool can_hold = false;  // walking, on a gait with an all-down window
    // Every foot planted and standing where its stance progress says: the
    // mirror is exact now.
    bool ready = false;
    float knee_speed = 0.0f;
    float timeout = 0.0f;  // per stage
    float zero_tol = 0.0f;
    float dt = 0.0f;
  };

  struct Output {
    std::pair<float, float> v_xy{0.0f, 0.0f};
    float omega = 0.0f;
    bool mirror = false;
  };

  Output step(const std::map<std::string, LegContext>& legs, const Input& in);
  Stage stage() const { return stage_; }
  // A reversal is in flight, held or not. The engine reads this to keep a
  // command sweeping through zero from arming a settle.
  bool reversing() const { return stage_ != Stage::IDLE; }
  bool crossing() const { return stage_ == Stage::CROSSING; }
  void reset() { enter(Stage::IDLE); }

 private:
  void enter(Stage stage) {
    stage_ = stage;
    timer_ = 0.0f;
  }

  Stage stage_ = Stage::IDLE;
  float timer_ = 0.0f;
  // The travel this reversal turns away from, latched at recognition. Later
  // tests read this, not the live command, which the hold itself moves.
  std::pair<float, float> hold_xy_{0.0f, 0.0f};
  float hold_omega_ = 0.0f;
  float hold_scale_ = 1.0f;
};

}  // namespace hexa::gait
