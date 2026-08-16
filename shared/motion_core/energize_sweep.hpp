// Staggered servo-rail energize sweep. The Servo 2040 never drives a servo the
// host has not commanded — a servo starts being driven on its first SET after
// the relay closes — so the host owns the energize order, and doing all 18 in one
// tick lands an inrush big enough to nuisance-trip the board.
//
// At the relay OFF -> ON edge the legs come up `interval_s` apart instead. Pure:
// the caller supplies dt and maps the returned count onto its wiring table.
//
// "Legs live" counts in *pin* order, not LEG_NAMES order — the sweep is about
// physical power draw, so it follows the harness.
#pragma once

#include "leg_index.hpp"

namespace hexa {

class EnergizeSweep {
 public:
  // interval_s <= 0 disables the sweep; arm() then brings every leg up at once.
  explicit EnergizeSweep(float interval_s) : interval_s_(interval_s) {}

  // Relay OFF -> ON edge. The first leg is live in the same tick.
  void arm() {
    elapsed_ = 0.0f;
    legs_ = interval_s_ > 0.0f ? 1 : kNumLegs;
  }

  // Relay ON -> OFF edge: nothing is driven.
  void disarm() {
    elapsed_ = 0.0f;
    legs_ = 0;
  }

  // Legs now live; a no-op while disarmed or once complete.
  int step(float dt) {
    if (legs_ <= 0 || done()) return legs_;
    elapsed_ += dt;
    while (elapsed_ >= interval_s_ && legs_ < kNumLegs) {
      elapsed_ -= interval_s_;
      ++legs_;
    }
    return legs_;
  }

  int legs() const { return legs_; }

  bool done() const { return legs_ >= kNumLegs; }

  float interval_s() const { return interval_s_; }

 private:
  float interval_s_;
  float elapsed_ = 0.0f;
  int legs_ = 0;
};

}  // namespace hexa
