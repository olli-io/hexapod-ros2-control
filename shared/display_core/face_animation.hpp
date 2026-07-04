// Face animation sequences for the face renderer.
//
// Pure header/impl: no rclcpp, no I/O, no clocks. A face animation is a fixed,
// looping sequence of timed steps — a gaze target and/or a blink trigger. The
// node owns the clock (see FaceAnimationRunner): it tracks elapsed time since
// the animation started plus a fired-step counter, and asks dueSteps() each
// tick which steps to relay. The renderer (EyeAnim) still eases gaze and
// auto-blinks on top, so a sparse step sequence reads as smooth motion.
//
// Note: "face animation" is deliberately distinct from the posture animation
// stack in hexa_posture (/animation/mode) — these only drive the display.
//
//  - breathing — slow vertical gaze drift (up -> center -> down -> center)
//    while the display waits for the robot stack to initialize.
//  - idling — look-around-and-blink burst while the hexapod stands idle, spaced
//    a random 5-10 s apart (repeat_range_s) so the eyes rest at center between
//    glances rather than scanning continuously.

#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "Expression.h"  // GazeDirection

namespace hexa::display {

struct FaceAnimationStep {
    double at_s = 0.0;
    // nullopt = this step triggers no gaze change (blink-only step).
    std::optional<GazeDirection> gaze;
    bool blink = false;
};

// A fixed looping sequence of timed steps. The validating constructor throws
// std::invalid_argument on malformed data (mirrors the Python __post_init__).
class FaceAnimation {
public:
    // repeat_range_s: optional (min_s, max_s) rest between cycles. When set, the
    // runner plays one cycle (the burst within period_s) then waits a fresh
    // random interval from this range before the next cycle. When unset the
    // animation loops every period_s.
    FaceAnimation(std::string name, double period_s,
                  std::vector<FaceAnimationStep> steps,
                  std::optional<std::pair<double, double>> repeat_range_s = std::nullopt);

    const std::string& name() const { return name_; }
    double period_s() const { return period_s_; }
    const std::vector<FaceAnimationStep>& steps() const { return steps_; }
    const std::optional<std::pair<double, double>>& repeat_range_s() const {
        return repeat_range_s_;
    }

private:
    std::string name_;
    double period_s_;
    std::vector<FaceAnimationStep> steps_;
    std::optional<std::pair<double, double>> repeat_range_s_;
};

// Total steps due since the animation started, across cycles.
std::int64_t stepCountAt(const FaceAnimation& animation, double elapsed_s);

// Steps newly due at elapsed_s given fired_count already fired. Returns
// (steps, new_fired_count). After a stall longer than one period only the most
// recent cycle's worth of steps is replayed, so a hiccup cannot queue an
// unbounded blink burst.
std::pair<std::vector<FaceAnimationStep>, std::int64_t>
dueSteps(const FaceAnimation& animation, double elapsed_s, std::int64_t fired_count);

// The registered animations (Meyers singletons; stable references).
const FaceAnimation& breathing();
const FaceAnimation& idling();

// name -> animation registry. Returns nullptr for an unknown name.
const FaceAnimation* findFaceAnimation(const std::string& name);
const std::map<std::string, const FaceAnimation*>& faceAnimations();

}  // namespace hexa::display
