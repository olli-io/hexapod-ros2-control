#pragma once

#include <stdint.h>

#include "Expression.h"

class Display;
class IRenderer;

class ExpressionController {
public:
    ExpressionController(Display& display, IRenderer& renderer);

    bool setExpression(Expression e);   // false if e out of range
    bool setGaze(GazeDirection g);

    Expression    expression() const { return _expr; }
    GazeDirection gaze()       const { return _gaze; }

    // Forward a one-shot blink request to the renderer.
    void triggerBlink();

    // Link supervision: while the link is down the eyes render DEAD with
    // centered gaze. The commanded expression/gaze are kept and resume
    // when the link comes back.
    void setLinkUp(bool up) { _linkUp = up; }
    bool linkUp() const { return _linkUp; }

    // Called from the loop. Renders every frame (animations run inside the
    // renderer); rate-limited to ~60 Hz.
    void tick(uint32_t nowMs);

private:
    Display&      _display;
    IRenderer&    _renderer;
    Expression    _expr = Expression::NEUTRAL;
    GazeDirection _gaze = GazeDirection::CENTER;
    bool          _linkUp = true;
    uint32_t      _lastRenderMs = 0;
};
