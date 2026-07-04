#include "ExpressionController.h"

#include "Config.h"
#include "IRenderer.h"

namespace {
constexpr const char* kExprNames[static_cast<size_t>(Expression::_COUNT)] = {
    "NEUTRAL", "HAPPY", "SLEEPY", "DEAD", "GREEDY", "WOOZY", "ANGRY", "LOVE",
};
constexpr const char* kGazeNames[static_cast<size_t>(GazeDirection::_COUNT)] = {
    "CENTER", "UP", "DOWN", "LEFT", "RIGHT",
    "UP_LEFT", "UP_RIGHT", "DOWN_LEFT", "DOWN_RIGHT",
};
}  // namespace

const char* expressionName(Expression e) {
    return isValid(e) ? kExprNames[static_cast<size_t>(e)] : "?";
}
const char* gazeName(GazeDirection g) {
    return isValid(g) ? kGazeNames[static_cast<size_t>(g)] : "?";
}

ExpressionController::ExpressionController(Display& display, IRenderer& renderer)
    : _display(display), _renderer(renderer) {}

bool ExpressionController::setExpression(Expression e) {
    if (!isValid(e)) return false;
    _expr = e;
    return true;
}

bool ExpressionController::setGaze(GazeDirection g) {
    if (!isValid(g)) return false;
    _gaze = g;
    return true;
}

void ExpressionController::triggerBlink() {
    _renderer.requestBlink();
}

void ExpressionController::tick(uint32_t nowMs) {
    if (nowMs - _lastRenderMs < cfg::MIN_RENDER_INTERVAL_MS) return;

    const RenderState s = _linkUp
        ? RenderState{_expr, _gaze}
        : RenderState{Expression::DEAD, GazeDirection::CENTER};
    _renderer.render(_display, s, nowMs);

    _lastRenderMs = nowMs;
}
