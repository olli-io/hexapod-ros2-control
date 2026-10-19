#include "expression_policy.hpp"

#include <algorithm>
#include <cmath>

namespace hexa::display {

const std::set<std::string>& idleGaitStates() {
    static const std::set<std::string> kStates = {"folded", "stand"};
    return kStates;
}

const std::set<std::string>& idlingGaitStates() {
    static const std::set<std::string> kStates = {"stand"};
    return kStates;
}

const std::unordered_map<std::string, Expression>& defaultExpressionMap() {
    static const std::unordered_map<std::string, Expression> kMap = {
        {"folded", Expression::SLEEPY},
        {"initialize", Expression::NEUTRAL},
        {"stand", Expression::NEUTRAL},
        {"engaging", Expression::NEUTRAL},
        {"gait", Expression::NEUTRAL},
        {"settling", Expression::NEUTRAL},
        {"reseating", Expression::NEUTRAL},
        // A leg-set change: standing and working, not going to sleep. Left out
        // of idleGaitStates for the same reason reseating is — the robot is
        // moving, so the idling look-around must stay off.
        {"folding_pair", Expression::NEUTRAL},
        {"unfolding_pair", Expression::NEUTRAL},
        {"folding", Expression::SLEEPY},
    };
    return kMap;
}

namespace {

// (vertical sign, horizontal sign) -> Gaze. Vertical +1 = UP, horizontal
// +1 = LEFT (REP-103 +y / positive wz are both leftward).
GazeDirection gazeFromSigns(int v, int h) {
    if (v > 0) {
        if (h > 0) return GazeDirection::UP_LEFT;
        if (h < 0) return GazeDirection::UP_RIGHT;
        return GazeDirection::UP;
    }
    if (v < 0) {
        if (h > 0) return GazeDirection::DOWN_LEFT;
        if (h < 0) return GazeDirection::DOWN_RIGHT;
        return GazeDirection::DOWN;
    }
    if (h > 0) return GazeDirection::LEFT;
    if (h < 0) return GazeDirection::RIGHT;
    return GazeDirection::CENTER;
}

// Inverse of gazeFromSigns: the (vertical, horizontal) signs for a gaze.
std::pair<int, int> signsFromGaze(GazeDirection gaze) {
    switch (gaze) {
        case GazeDirection::CENTER:     return {0, 0};
        case GazeDirection::UP:         return {1, 0};
        case GazeDirection::DOWN:       return {-1, 0};
        case GazeDirection::LEFT:       return {0, 1};
        case GazeDirection::RIGHT:      return {0, -1};
        case GazeDirection::UP_LEFT:    return {1, 1};
        case GazeDirection::UP_RIGHT:   return {1, -1};
        case GazeDirection::DOWN_LEFT:  return {-1, 1};
        case GazeDirection::DOWN_RIGHT: return {-1, -1};
        default:                        return {0, 0};
    }
}

bool cmdVelIsZero(const PolicyInputs& in) {
    return std::abs(in.vx) < kCmdVelZeroTol && std::abs(in.vy) < kCmdVelZeroTol &&
           std::abs(in.wz) < kCmdVelZeroTol;
}

bool gaitStateIn(const std::optional<std::string>& state,
                 const std::set<std::string>& states) {
    return state.has_value() && states.count(*state) > 0;
}

bool isIdle(const PolicyInputs& in) {
    return cmdVelIsZero(in) && gaitStateIn(in.gait_state, idleGaitStates()) &&
           in.animation_mode.empty();
}

// Pose mode: feet planted, the sticks moving the body rather than the robot.
bool poseModeActive(const PolicyInputs& in) {
    return cmdVelIsZero(in) && gaitStateIn(in.gait_state, idlingGaitStates());
}

bool stickHeld(double magnitude, double threshold, bool prev_held, double exit_ratio) {
    if (threshold <= 0.0) {  // 0 disables the stick, as with the battery rungs
        return false;
    }
    return magnitude >= (prev_held ? threshold * exit_ratio : threshold);
}

// Recover last tick's PostureSticks from the expression it produced, so decide()
// carries the enter/exit hysteresis across ticks without holding state.
PostureSticks sticksFromExpression(Expression expression, const PolicyConfig& config) {
    if (expression == config.posture_both_expression) return {true, true};
    if (expression == config.posture_tilt_expression) return {true, false};
    if (expression == config.posture_shift_expression) return {false, true};
    return {};
}

std::optional<Expression> postureStickExpression(const PostureSticks& sticks,
                                                 const PolicyConfig& config) {
    if (sticks.tilt && sticks.shift) return config.posture_both_expression;
    if (sticks.tilt) return config.posture_tilt_expression;
    if (sticks.shift) return config.posture_shift_expression;
    return std::nullopt;
}

double norm(double value, double scale) {
    return scale > 0.0 ? value / scale : 0.0;
}

struct Gaze {
    GazeDirection direction = GazeDirection::CENTER;
    float scale_x = 1.0f;
    float scale_y = 1.0f;
};

DisplayTarget target(Expression expression, const Gaze& gaze) {
    return {expression, gaze.direction, gaze.scale_x, gaze.scale_y};
}

int sign(double value) {
    return (value > 0.0) - (value < 0.0);
}

Gaze decideGaze(const PolicyInputs& in, const PolicyConfig& config) {
    double v = 0.0;
    double h = 0.0;
    if (!cmdVelIsZero(in)) {
        // gait-active: vertical gaze follows body pitch — walking forward or
        // backward must not move the gaze up or down. +pitch tips the nose
        // down -> gaze DOWN, hence the negation. Horizontal gaze leads the
        // strafe/turn direction (+vy and +wz are both leftward).
        v = norm(-in.pitch, config.pose_pitch_full_rad);
        h = norm(in.vy, config.gaze_vy_max) +
            config.gaze_wz_weight * norm(in.wz, config.gaze_wz_max);
    } else {
        // pose mode: the eyes counter the body rotation, so they hold on a
        // point in the world. Nose down (+pitch) -> UP; nose left (+yaw) ->
        // RIGHT; lean right (+roll) -> LEFT. Body x/y shift leaves the gaze
        // centered.
        v = norm(in.pitch, config.pose_pitch_full_rad);
        h = norm(in.roll - in.yaw, config.pose_tilt_full_rad);
    }
    // Travel is proportional to the input, full at |1|.
    v = std::clamp(v, -1.0, 1.0);
    h = std::clamp(h, -1.0, 1.0);
    return {gazeFromSigns(sign(v), sign(h)), static_cast<float>(std::abs(h)),
            static_cast<float>(std::abs(v))};
}

bool poseIsLevel(const PolicyInputs& in, const PolicyConfig& config) {
    // Same axes as the pose-mode gaze; idling yields once the tilt is clear of
    // these thresholds.
    return std::abs(in.pitch) < config.pose_pitch_threshold_rad &&
           std::abs(in.yaw - in.roll) < config.pose_tilt_threshold_rad;
}

}  // namespace

PostureSticks postureSticks(const PolicyInputs& in, const PolicyConfig& config,
                            PostureSticks prev) {
    return {stickHeld(std::hypot(in.roll, in.pitch), config.posture_tilt_threshold_rad,
                      prev.tilt, config.posture_exit_ratio),
            stickHeld(std::hypot(in.x, in.y), config.posture_shift_threshold_m,
                      prev.shift, config.posture_exit_ratio)};
}

GazeDirection toScreenGaze(GazeDirection gaze) {
    const auto [v, h] = signsFromGaze(gaze);
    return gazeFromSigns(v, -h);
}

DisplayTarget decide(const PolicyInputs& in, const PolicyConfig& config,
                     const DisplayTarget& prev) {
    // Gaze CENTER: the spinners turn about the eye centers, and a gaze offset
    // would slide the whole ring off-axis mid-rotation.
    if (in.busy) {
        return {config.scanning_expression, GazeDirection::CENTER};
    }
    if (in.battery_critical) {
        return {config.battery_critical_expression, GazeDirection::CENTER};
    }
    const Gaze gaze = decideGaze(in, config);
    if (const auto it = config.gesture_expression_map.find(in.gesture);
        it != config.gesture_expression_map.end()) {
        return target(it->second, gaze);
    }
    if (in.battery_low && isIdle(in)) {
        return target(config.battery_warning_expression, gaze);
    }
    if (!in.animation_mode.empty()) {
        return target(config.animation_expression, gaze);
    }
    // Pose mode: the gait state sits at "stand" for as long as the operator
    // poses the body, so the face answers the sticks instead of the map — one
    // expression per stick, a third for both at once.
    if (poseModeActive(in)) {
        const PostureSticks sticks = postureSticks(
            in, config, sticksFromExpression(prev.expression, config));
        if (const auto expression = postureStickExpression(sticks, config)) {
            return target(*expression, gaze);
        }
    }
    // No /gait/state heard yet: the robot boots folded, so stay asleep until
    // the controller reports otherwise. An unknown state string stays NEUTRAL.
    Expression expression =
        in.gait_state ? Expression::NEUTRAL : Expression::SLEEPY;
    if (in.gait_state) {
        const auto it = config.expression_map.find(*in.gait_state);
        if (it != config.expression_map.end()) {
            expression = it->second;
        }
    }
    return target(expression, gaze);
}

std::optional<std::string> selectFaceAnimation(const PolicyInputs& in,
                                               const PolicyConfig& config) {
    // The spinners own the gaze while scanning; a drift animation would fight
    // them for it.
    if (in.busy) {
        return std::nullopt;
    }
    if (in.battery_critical || in.battery_low) {
        return std::nullopt;
    }
    if (!in.animation_mode.empty()) {
        return std::nullopt;
    }
    // The sleeping face breathes: while the stack is not up yet (no /gait/state
    // heard) or the robot is folded, run the slow breathing drift.
    if (!in.gait_state || *in.gait_state == "folded") {
        return std::string("breathing");
    }
    // A posed body is not an idle one: a stick is held, and the idling drift
    // would fight the pose-following gaze for the eyes. Enter thresholds only —
    // unlike decide(), this has no previous tick to hold a stick across.
    const PostureSticks sticks = postureSticks(in, config, PostureSticks{});
    if (gaitStateIn(in.gait_state, idlingGaitStates()) && cmdVelIsZero(in) &&
        poseIsLevel(in, config) && !sticks.tilt && !sticks.shift) {
        return std::string("idling");
    }
    return std::nullopt;
}

BatteryMonitor::BatteryMonitor(double warning_v, double critical_v,
                               double hysteresis_v, double hold_s)
    : warning_v_(warning_v),
      critical_v_(critical_v),
      hysteresis_v_(hysteresis_v),
      hold_s_(hold_s) {}

BatteryMonitor::StepResult BatteryMonitor::step(double voltage, double t,
                                                double threshold, bool active,
                                                std::optional<double> below_since) const {
    if (threshold <= 0.0) {
        return {false, std::nullopt};
    }
    if (active) {
        if (voltage > threshold + hysteresis_v_) {
            return {false, std::nullopt};
        }
        return {true, below_since};
    }
    if (voltage < threshold) {
        if (!below_since) {
            below_since = t;
        }
        if (t - *below_since >= hold_s_) {
            return {true, below_since};
        }
        return {false, below_since};
    }
    return {false, std::nullopt};
}

BatteryMonitor::Flags BatteryMonitor::update(double voltage, double t) {
    StepResult warn = step(voltage, t, warning_v_, low_, below_warning_since_);
    low_ = warn.active;
    below_warning_since_ = warn.below_since;
    StepResult crit = step(voltage, t, critical_v_, critical_, below_critical_since_);
    critical_ = crit.active;
    below_critical_since_ = crit.below_since;
    return {low_, critical_};
}

}  // namespace hexa::display
