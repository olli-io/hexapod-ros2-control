#pragma once

#include <stdint.h>

#include "Expression.h"

class Display;

// Targets, not displayed state: a stateful renderer animates toward these
// (blink-through expression swaps, eased gaze) at its own pace.
struct RenderState {
    Expression    expr;
    GazeDirection gaze;
};

class IRenderer {
public:
    virtual ~IRenderer() = default;
    virtual void render(Display& display, const RenderState& state, uint32_t nowMs) = 0;

    // One-shot blink request (TRIGGER_BLINK). No-op for renderers
    // without a blink mechanic.
    virtual void requestBlink() {}
};
