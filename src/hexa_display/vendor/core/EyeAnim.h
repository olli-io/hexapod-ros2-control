#pragma once

#include <stdint.h>

#include "IRenderer.h"

namespace eyes {

// One evaluated animation frame: what to draw right now.
struct AnimFrame {
    Expression expr;
    float      lid;
    int        gx, gy;
};

// Platform-free animation state machine on top of the target RenderState:
//   - expression changes pass through a 260 ms blink, swapping at midpoint
//   - gaze eases to its target over 220 ms (easeOutCubic)
//   - idle blinks fire autonomously every 2.2–5.2 s
// Randomness is injected so the same code runs on-device (esp_random) and
// in the host simulator/tests (rand).
class EyeAnim {
public:
    using RandFn = uint32_t (*)();
    explicit EyeAnim(RandFn rand) : _rand(rand) {}

    void requestBlink() { _blinkRequested = true; }

    // Advance to nowMs and return the frame to draw.
    AnimFrame update(const RenderState& state, uint32_t nowMs);

private:
    void  startBlink(uint32_t nowMs);
    float updateBlink(const RenderState& state, uint32_t nowMs);  // returns lid
    void  updateGaze(const RenderState& state, uint32_t nowMs);
    uint32_t nextIdleDelay();

    RandFn _rand;

    Expression    _shownExpr = Expression::NEUTRAL;
    GazeDirection _shownGaze = GazeDirection::CENTER;

    bool     _blinkActive     = false;
    bool     _swapDone        = false;  // midpoint expression swap fired
    bool     _blinkRequested  = false;  // one-shot from TRIGGER_BLINK
    uint32_t _blinkStartMs    = 0;
    uint32_t _nextIdleBlinkMs = 0;

    float    _gx = 0, _gy = 0;          // current gaze offset, px
    float    _gazeFromX = 0, _gazeFromY = 0;
    float    _gazeToX = 0, _gazeToY = 0;
    uint32_t _gazeStartMs = 0;

    bool _started = false;              // arms the idle timer on first frame
};

}  // namespace eyes
