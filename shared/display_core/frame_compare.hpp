#pragma once

// Frame-equality check shared by every renderer of the eye core.
//
// Repo-owned, deliberately NOT part of core/ — that directory is vendored
// verbatim from hexapod-esp32-display (see README.md) and already carries one
// local divergence. This lives here so both the Pi node (display_node.cpp) and
// the Pico firmware (face.cpp) skip unchanged frames by the same rule.

#include "EyeAnim.h"

namespace eyes {

// Exact equality is intended: drawEye is a pure function of these fields, so
// bit-identical frames rasterize to identical pixels. EyeAnim emits a constant
// frame while idle (lid == 1.0f, settled integer gaze, phase 0), so this
// reliably fires. SCANNING steps its phase in discrete jumps, so the spinner
// redraws once per step rather than once per render tick — comparing `phase` is
// what keeps the spinner turning instead of freezing on its first frame.
inline bool sameFrame(const AnimFrame& a, const AnimFrame& b) {
    return a.expr == b.expr && a.lid == b.lid && a.gx == b.gx && a.gy == b.gy &&
           a.phase == b.phase;
}

}  // namespace eyes
