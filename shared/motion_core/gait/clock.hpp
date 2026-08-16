// A GaitClock owns the engine's master phase in [0, 1) and projects it through
// per-leg phase offsets. It is the only place where time enters the gait chain.
#pragma once

#include <map>
#include <string>

#include "gait/types.hpp"

namespace hexa::gait {

// Per-leg cycle start, relative to the master phase, in [0, 1).
class PhaseOffsets {
 public:
  explicit PhaseOffsets(std::map<std::string, float> offsets);

  const std::map<std::string, float>& offsets() const { return offsets_; }
  float at(const std::string& leg) const { return offsets_.at(leg); }

  // Every offset negated: reverses the direction the wave travels down the body.
  PhaseOffsets reversed() const;

 private:
  std::map<std::string, float> offsets_;
};

class GaitClock {
 public:
  explicit GaitClock(PhaseOffsets offsets);

  float master() const { return master_; }
  const PhaseOffsets& offsets() const { return offsets_; }
  void reset(float master = 0.0f);
  void advance(float dt, float cycle_time);
  std::map<std::string, float> phases() const;

  // Reflect every leg's phase about `about`: phase -> pymod(about - phase, 1).
  // Takes reflecting the master *and* negating the offsets; the master alone
  // misses every leg by twice its own offset.
  //
  // About the swing end this is the reversal map: stance progress s becomes
  // 1 - s. Exact only with every foot planted and standing where its phase says
  // (excursion e == (0.5 - s) * stride).
  void mirror(float about);

  // Leaves the master phase alone.
  void set_offsets(PhaseOffsets offsets) { offsets_ = std::move(offsets); }

 private:
  PhaseOffsets offsets_;
  float master_ = 0.0f;
};

}  // namespace hexa::gait
