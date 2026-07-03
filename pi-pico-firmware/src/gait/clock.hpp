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

 private:
  std::map<std::string, float> offsets_;
};

// Master phase clock with per-leg projections. advance() integrates the master
// phase modulo one cycle; phases() returns each leg's (master + offset) mod 1.
class GaitClock {
 public:
  explicit GaitClock(PhaseOffsets offsets);

  float master() const { return master_; }
  void reset(float master = 0.0f);
  void advance(float dt, float cycle_time);
  std::map<std::string, float> phases() const;

 private:
  PhaseOffsets offsets_;
  float master_ = 0.0f;
};

}  // namespace hexa::gait
