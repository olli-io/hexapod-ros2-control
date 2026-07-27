#include "EyeAnim.h"

#include <math.h>

#include "EyeRaster.h"

namespace {

// Timing, matching the JS prototype. Wall-clock driven, fps-independent.
constexpr uint32_t kBlinkMs    = 260;   // full blink: close + open
constexpr uint32_t kLookMs     = 220;   // gaze ease duration
constexpr uint32_t kIdleMinMs  = 2200;  // idle blink: min interval...
constexpr uint32_t kIdleRandMs = 3001;  // ...plus rand() % this
constexpr float    kLidShut    = 0.92f; // lid travels 1 → 1-kLidShut → 1

// SCANNING spinner. Quantized into kSpinSteps discrete angles so a settled
// panel still skips redundant rasters/SPI flushes: at 30 steps per ~1 s turn
// the arc moves ~12° a step, which reads as continuous rotation while costing
// 30 frames a second rather than the render loop's 60.
constexpr uint32_t kSpinSteps    = 30;
constexpr uint32_t kSpinStepMs   = 33;  // ~0.99 s per full turn

float easeOutCubic(float t) { return 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t); }

// Starts and ends at rest: back-to-back drift steps chain into one continuous
// oscillation instead of darting and holding.
float easeInOutSine(float t) { return 0.5f - 0.5f * cosf(t * 3.14159265f); }

// GazeDirection → unit offsets, indexed by enum value.
struct Dir {
    int8_t dx, dy;
};
constexpr Dir kGazeDir[static_cast<size_t>(GazeDirection::_COUNT)] = {
    {0, 0},   // CENTER
    {0, -1},  // UP
    {0, 1},   // DOWN
    {-1, 0},  // LEFT
    {1, 0},   // RIGHT
    {-1, -1}, // UP_LEFT
    {1, -1},  // UP_RIGHT
    {-1, 1},  // DOWN_LEFT
    {1, 1},   // DOWN_RIGHT
};

}  // namespace

namespace eyes {

uint32_t EyeAnim::nextIdleDelay() { return kIdleMinMs + _rand() % kIdleRandMs; }

void EyeAnim::startBlink(uint32_t nowMs) {
    _blinkActive  = true;
    _swapDone     = false;
    _blinkStartMs = nowMs;
}

float EyeAnim::updateBlink(const RenderState& state, uint32_t nowMs) {
    if (!_started) {
        _started         = true;
        _nextIdleBlinkMs = nowMs + nextIdleDelay();
    }

    if (!_blinkActive) {
        // A target change after the previous blink's midpoint chains a new
        // blink here; idle and commanded blinks swap nothing.
        if (state.expr != _shownExpr || _blinkRequested ||
            nowMs >= _nextIdleBlinkMs) {
            _blinkRequested = false;
            startBlink(nowMs);
        }
    }
    if (!_blinkActive) return 1.0f;
    _blinkRequested = false;  // already blinking; absorb the request

    const float p = static_cast<float>(nowMs - _blinkStartMs) / kBlinkMs;
    if (p >= 1.0f) {
        if (!_swapDone) _shownExpr = state.expr;  // midpoint frame was skipped
        _blinkActive     = false;
        _nextIdleBlinkMs = nowMs + nextIdleDelay();
        return 1.0f;
    }
    if (p >= 0.5f && !_swapDone) {
        _shownExpr = state.expr;
        _swapDone  = true;
    }
    const float half = p < 0.5f ? p / 0.5f : (1.0f - p) / 0.5f;  // 0 → 1 → 0
    return 1.0f - kLidShut * half;
}

void EyeAnim::updateGaze(const RenderState& state, uint32_t nowMs) {
    if (state.gaze != _shownGaze) {
        _shownGaze   = state.gaze;
        const Dir d  = kGazeDir[static_cast<size_t>(state.gaze)];
        _gazeFromX   = _gx;
        _gazeFromY   = _gy;
        _gazeToX     = static_cast<float>(d.dx * kGazeMaxX) * state.gazeScale;
        _gazeToY     = static_cast<float>(d.dy * kGazeMaxY) * state.gazeScale;
        _gazeStartMs = nowMs;
    }
    const uint32_t lookMs = state.gazeEaseMs ? state.gazeEaseMs : kLookMs;
    const float t = fminf(1.0f, static_cast<float>(nowMs - _gazeStartMs) / lookMs);
    const float e = state.gazeEaseMs ? easeInOutSine(t) : easeOutCubic(t);
    _gx = _gazeFromX + (_gazeToX - _gazeFromX) * e;
    _gy = _gazeFromY + (_gazeToY - _gazeFromY) * e;
}

float EyeAnim::spinPhase(uint32_t nowMs) const {
    return static_cast<float>((nowMs / kSpinStepMs) % kSpinSteps) / kSpinSteps;
}

AnimFrame EyeAnim::update(const RenderState& state, uint32_t nowMs) {
    const float lid = updateBlink(state, nowMs);
    updateGaze(state, nowMs);
    // The spinner belongs to the expression actually on screen, not the target:
    // the phase must stay 0 until the blink-through has swapped SCANNING in.
    const float phase =
        _shownExpr == Expression::SCANNING ? spinPhase(nowMs) : 0.0f;
    return {_shownExpr, lid,
            static_cast<int>(lroundf(_gx)), static_cast<int>(lroundf(_gy)), phase};
}

}  // namespace eyes
