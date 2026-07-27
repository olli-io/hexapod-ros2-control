#pragma once

#include <stdint.h>

#include "Expression.h"

class Display;

// Targets, not displayed state: a stateful renderer animates toward these
// (blink-through expression swaps, eased gaze) at its own pace.
struct RenderState {
    Expression    expr;
    GazeDirection gaze;
    // 0 = the renderer's default 220 ms dart (easeOutCubic). Non-zero switches
    // gaze changes to a slow easeInOutSine drift of this duration, for
    // continuous motion (breathing) instead of a glance.
    uint32_t      gazeEaseMs = 0;
    // Scale on the gaze offset target (1 = full kGazeMax travel). Face
    // animations use this to shrink their drift amplitude.
    float         gazeScale = 1.0f;
};

class IRenderer {
public:
    virtual ~IRenderer() = default;
    virtual void render(Display& display, const RenderState& state, uint32_t nowMs) = 0;

    // One-shot blink request (TRIGGER_BLINK). No-op for renderers
    // without a blink mechanic.
    virtual void requestBlink() {}
};
