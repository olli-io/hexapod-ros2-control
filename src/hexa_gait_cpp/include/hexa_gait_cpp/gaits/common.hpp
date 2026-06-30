// Shared helpers for metachronal gaits (crawl, ripple) and tripod. Port of
// gaits/_common.py.
//
// metachronal_offsets() is the Wilson posterior->anterior sequence with a
// contralateral half-cycle offset. phased_foot_target is the pure
// (phase, stride, leg) -> body-frame target shared across all standard
// strategies.
#pragma once

#include "hexa_gait_cpp/clock.hpp"
#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

// Wilson's posterior->anterior protraction wave with the contralateral side
// half a cycle out of phase. Offsets are the mirror of lift-off times. Shared
// by crawl and ripple (they differ only in duty factor).
const PhaseOffsets& metachronal_offsets();

// Shared (phase, stride, leg) -> foot-target body-frame helper. Swing window is
// [0, 1 - beta); stance window is [1 - beta, 1). Stance is a quartic Bezier
// from AEP toward PEP at constant tip velocity; swing is the two-curve
// swing_arc.
Vec3 phased_foot_target(double phase, const StrideParams& stride,
                        const LegContext& leg);

}  // namespace hexa_gait
