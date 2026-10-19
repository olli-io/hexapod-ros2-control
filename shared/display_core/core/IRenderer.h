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
    // Per-axis scale on the gaze offset target (1 = full kGazeMax travel).
    // Face animations shrink their drift amplitude with it; pose mode sets it
    // in proportion to the body tilt.
    float         gazeScaleX = 1.0f;
    float         gazeScaleY = 1.0f;
};

class IRenderer {
public:
    virtual ~IRenderer() = default;
    virtual void render(Display& display, const RenderState& state, uint32_t nowMs) = 0;

    // One-shot blink request (TRIGGER_BLINK). No-op for renderers
    // without a blink mechanic.
    virtual void requestBlink() {}
};
