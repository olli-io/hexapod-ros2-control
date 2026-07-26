#include "expression_policy.hpp"

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
        {"gait", Expression::HAPPY},
        {"settling", Expression::NEUTRAL},
        {"reseating", Expression::NEUTRAL},
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

double norm(double value, double scale) {
    return scale > 0.0 ? value / scale : 0.0;
}

GazeDirection decideGaze(const PolicyInputs& in, const PolicyConfig& config,
                         const DisplayTarget& prev) {
    const auto [prev_v, prev_h] = signsFromGaze(prev.gaze);
    // Vertical gaze follows body pitch in both modes — walking forward or
    // backward must not move the gaze up or down. +pitch tips the nose down ->
    // gaze DOWN, hence the negation.
    const int sv = quantizeAxis(-in.pitch, prev_v, config.pose_pitch_threshold_rad,
                                config.gaze_exit_ratio);
    if (!cmdVelIsZero(in)) {
        // gait-active: horizontal gaze leads the strafe/turn direction (+vy and
        // +wz are both leftward).
        const double horizontal = norm(in.vy, config.gaze_vy_max) +
                                  config.gaze_wz_weight * norm(in.wz, config.gaze_wz_max);
        const int sh = quantizeAxis(horizontal, prev_h, config.gaze_deadband,
                                    config.gaze_exit_ratio);
        return gazeFromSigns(sv, sh);
    }
    // pose mode: horizontal gaze follows body tilt. +yaw turns the nose left ->
    // LEFT; +roll leans the body right -> RIGHT, hence the minus sign.
    const int sh = quantizeAxis(in.yaw - in.roll, prev_h, config.pose_tilt_threshold_rad,
                                config.gaze_exit_ratio);
    return gazeFromSigns(sv, sh);
}

bool poseIsLevel(const PolicyInputs& in, const PolicyConfig& config) {
    // Same axes and thresholds as the pose-mode gaze, so idling yields exactly
    // where tilt-following gaze would take over.
    return std::abs(in.pitch) < config.pose_pitch_threshold_rad &&
           std::abs(in.yaw - in.roll) < config.pose_tilt_threshold_rad;
}

}  // namespace

int quantizeAxis(double value, int prev_sign, double deadband, double exit_ratio) {
    if (std::abs(value) >= deadband) {
        return value > 0.0 ? 1 : -1;
    }
    if (prev_sign != 0 && std::abs(value) >= deadband * exit_ratio) {
        if ((value > 0.0) == (prev_sign > 0)) {
            return prev_sign;
        }
    }
    return 0;
}

DisplayTarget decide(const PolicyInputs& in, const PolicyConfig& config,
                     const DisplayTarget& prev) {
    if (in.battery_critical) {
        return {config.battery_critical_expression, GazeDirection::CENTER};
    }
    const GazeDirection gaze = decideGaze(in, config, prev);
    if (in.battery_low && isIdle(in)) {
        return {config.battery_warning_expression, gaze};
    }
    if (!in.animation_mode.empty()) {
        return {config.animation_expression, gaze};
    }
    Expression expression = Expression::NEUTRAL;
    if (in.gait_state) {
        const auto it = config.expression_map.find(*in.gait_state);
        if (it != config.expression_map.end()) {
            expression = it->second;
        }
    }
    return {expression, gaze};
}

std::optional<std::string> selectFaceAnimation(const PolicyInputs& in,
                                               const PolicyConfig& config) {
    if (in.battery_critical || in.battery_low) {
        return std::nullopt;
    }
    if (!in.animation_mode.empty()) {
        return std::nullopt;
    }
    if (!in.gait_state) {
        return std::string("breathing");
    }
    if (gaitStateIn(in.gait_state, idlingGaitStates()) && cmdVelIsZero(in) &&
        poseIsLevel(in, config)) {
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
