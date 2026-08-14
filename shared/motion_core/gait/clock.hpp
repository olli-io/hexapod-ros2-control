// Phase clock for the gait engine. Float fork of clock.hpp (plan part 06).
//
// A GaitClock owns the engine's master phase in [0, 1) and projects it through
// per-leg phase offsets. Strategies stay pure functions of phase: the clock is
// the only place where time enters the gait chain.
#pragma once

#include <map>
#include <string>

#include "gait/types.hpp"

namespace hexa::gait {

// Per-leg cycle start, relative to the master phase, in [0, 1).
class PhaseOffsets {
 public:
  // Validates that every leg is present and each offset is in [0, 1).
  explicit PhaseOffsets(std::map<std::string, float> offsets);

  const std::map<std::string, float>& offsets() const { return offsets_; }
  float at(const std::string& leg) const { return offsets_.at(leg); }

  // Every leg's cycle start negated. Reverses the direction the wave travels
  // down the body, which is what a gait walking the other way wants; a tripod's
  // table is its own negation, so for tripod this is the identity.
  PhaseOffsets reversed() const;

 private:
  std::map<std::string, float> offsets_;
};

// Master phase clock with per-leg projections. advance() integrates the master
// phase modulo one cycle; phases() returns each leg's (master + offset) mod 1.
class GaitClock {
 public:
  explicit GaitClock(PhaseOffsets offsets);

  float master() const { return master_; }
  const PhaseOffsets& offsets() const { return offsets_; }
  void reset(float master = 0.0f);
  void advance(float dt, float cycle_time);
  std::map<std::string, float> phases() const;

  // Reflect every leg's phase about `about`: phase -> pymod(about - phase, 1).
  // Reflecting the master and negating the offsets is identically that map, and
  // reflecting the master alone is not — it misses every leg by twice its own
  // offset. Reflecting twice returns the clock exactly where it started.
  //
  // Reflected about the swing end, this is the map a reversal wants: a leg's
  // stance progress s becomes 1 - s, so the runway it has left in the new travel
  // direction is the runway it just consumed in the old one. A leg at touchdown
  // lifts off immediately, because the old AEP it stands on is the new PEP.
  //
  // Exact only where the feet are where the schedule says they are — foot
  // excursion e == (0.5 - s) * stride — and only with every foot planted, since
  // on a leg in the air it reverses swing progress against a latched origin.
  void mirror(float about);

  // Replace the offset table, e.g. to restore a strategy's canonical one after a
  // mirror. Leaves the master phase alone.
  void set_offsets(PhaseOffsets offsets) { offsets_ = std::move(offsets); }

 private:
  PhaseOffsets offsets_;
  float master_ = 0.0f;
};

}  // namespace hexa::gait
