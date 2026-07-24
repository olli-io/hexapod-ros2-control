// Staggered servo-rail energize sweep.
//
// The Servo 2040 firmware never drives a servo the host has not commanded: a
// `SET RELAY 1` closes the relay with every servo limp, and each servo starts
// being driven only on its first `SET` after that (servo `SET`s sent while the
// relay is open are discarded, not buffered — see the driver's protocol.md,
// "Energizing servos: relay-first, host-ordered"). The host therefore owns the
// energize order, and driving all 18 servos in the same tick lands their
// combined inrush as one step large enough to nuisance-trip the board's
// over-current protection.
//
// This is the policy that spreads that step out: at the relay OFF -> ON edge the
// legs come up one at a time, `interval_s` apart, so the inrush arrives as six
// small steps instead of one spike. It is pure — no clocks, no I/O — so the
// caller (hexa_hardware on the ROS host, main.cpp on the Pico) supplies dt and
// maps the returned leg count onto its own wiring table.
//
// "Legs live" always counts in *pin* order (rear -> front with the current
// wiring), not the canonical LEG_NAMES order: the sweep is about the physical
// power draw, so it follows the harness.
#pragma once

#include "leg_index.hpp"

namespace hexa {

class EnergizeSweep {
 public:
  // `interval_s` is the stagger between consecutive legs. <= 0 disables the
  // sweep: arm() then brings every leg up at once (the pre-sweep behaviour).
  explicit EnergizeSweep(float interval_s) : interval_s_(interval_s) {}

  // Relay OFF -> ON edge: restart the sweep. The first leg is live immediately,
  // in the same tick the relay closes.
  void arm() {
    elapsed_ = 0.0f;
    legs_ = interval_s_ > 0.0f ? 1 : kNumLegs;
  }

  // Relay ON -> OFF edge (fold, fault, deactivate): nothing is driven.
  void disarm() {
    elapsed_ = 0.0f;
    legs_ = 0;
  }

  // Advance the sweep by `dt` seconds and return the number of legs now live.
  // A no-op while disarmed or once the sweep has completed.
  int step(float dt) {
    if (legs_ <= 0 || done()) return legs_;
    elapsed_ += dt;
    while (elapsed_ >= interval_s_ && legs_ < kNumLegs) {
      elapsed_ -= interval_s_;
      ++legs_;
    }
    return legs_;
  }

  // Legs currently energized, counted in pin order. 0 while disarmed.
  int legs() const { return legs_; }

  // True once every leg is live (also true immediately after arm() when the
  // sweep is disabled).
  bool done() const { return legs_ >= kNumLegs; }

  float interval_s() const { return interval_s_; }

 private:
  float interval_s_;
  float elapsed_ = 0.0f;
  int legs_ = 0;
};

}  // namespace hexa
