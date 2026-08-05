// Expression + gaze decision policy for the face.
//
// Pure header/impl: no rclcpp, no I/O, no clocks. The node feeds a PolicyInputs
// snapshot into decide() each tick and applies the returned DisplayTarget to
// the renderer.
//
// Precedence (highest first):
//   1. busy -> SCANNING, gaze CENTER. Top of the ladder because it is the one
//      state the operator asked for by hand — a hold on a front-panel button,
//      either the Bluetooth pairing scan or the network-mode switch. While they
//      wait on the spinners, nothing else may take the panel — not even a flat
//      pack, which they can read off the info screen anyway.
//   2. battery critical -> DEAD, gaze CENTER, unconditional.
//   3. battery warning -> SLEEPY, but only when idle (cmd_vel ~ 0, gait state in
//      the idle set, no animation mode) — a warning must not mask the face
//      mid-walk.
//   4. animation mode non-empty -> the configured animation expression (WOOZY).
//   5. pose mode with a posture stick held -> that stick's expression: tilt
//      (left stick) HAPPY, shift (right stick) LOVE, both ANGRY. Sticks at
//      rest fall through to the map below, which stands the face at NEUTRAL.
//   6. gait-state map from YAML; unknown state -> NEUTRAL; no /gait/state
//      heard yet -> SLEEPY (the robot boots folded and asleep).
//
// Gaze: the vertical axis always follows body pitch (nose up -> UP) — driving
// forward or backward never moves the gaze. The horizontal axis tracks cmd_vel
// during gait-active motion (left -> LEFT per REP-103) and body yaw/roll in pose
// mode; DEAD always forces CENTER. Axis quantization uses enter/exit hysteresis
// so the gaze doesn't chatter at the deadband edge.
//
// The only stateful piece is BatteryMonitor (hold time + voltage hysteresis),
// kept separate from decide() and deterministic given a (voltage, t) sequence.

#pragma once

#include <optional>
#include <set>
#include <string>
#include <unordered_map>

#include "Expression.h"  // Expression, GazeDirection

namespace hexa::display {

constexpr double kCmdVelZeroTol = 1e-4;

// Gait states in which the battery warning may take the face over — anything
// else counts as "busy" and keeps the normal expression.
const std::set<std::string>& idleGaitStates();

// Gait states in which the idling face animation may run. Narrower than the
// idle set on purpose: folded shows SLEEPY with the breathing drift instead.
const std::set<std::string>& idlingGaitStates();

// Default gait-state -> expression map; overridable per state from YAML.
const std::unordered_map<std::string, Expression>& defaultExpressionMap();

struct PolicyInputs {
    std::optional<std::string> gait_state;  // nullopt = no /gait/state heard yet
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    std::string animation_mode;
    // Body-pose offset (/body/pose). x/y are the body shift, roll/pitch the
    // body tilt — in pose mode the right stick drives the first pair and the
    // left stick the second, which is all the policy needs to tell them apart.
    double x = 0.0;
    double y = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    bool battery_low = false;
    bool battery_critical = false;
    // hexa_buttons is working on something the operator asked for by holding a
    // front-panel button and should wait out: a Bluetooth pairing scan
    // (/bluetooth/scanning) or a network-mode switch (/display/busy). One flag
    // rather than two because the face's answer to both is the same spinners —
    // the node ORs the topics before it gets here.
    bool busy = false;
};

struct PolicyConfig {
    std::unordered_map<std::string, Expression> expression_map;
    Expression animation_expression = Expression::WOOZY;
    Expression battery_warning_expression = Expression::SLEEPY;
    Expression battery_critical_expression = Expression::DEAD;
    Expression scanning_expression = Expression::SCANNING;
    // Pose mode, per posture stick (see PostureSticks).
    Expression posture_tilt_expression = Expression::HAPPY;
    Expression posture_shift_expression = Expression::LOVE;
    Expression posture_both_expression = Expression::ANGRY;
    double posture_tilt_threshold_rad = 0.03;
    double posture_shift_threshold_m = 0.005;
    double posture_exit_ratio = 0.6;
    double gaze_deadband = 0.15;
    double gaze_exit_ratio = 0.6;
    double gaze_wz_weight = 1.0;
    double gaze_vy_max = 0.1;
    double gaze_wz_max = 0.5;
    double pose_pitch_threshold_rad = 0.08;
    double pose_tilt_threshold_rad = 0.08;
    double idling_start_delay_s = 4.0;
};

struct DisplayTarget {
    Expression expression = Expression::NEUTRAL;
    GazeDirection gaze = GazeDirection::CENTER;

    bool operator==(const DisplayTarget& o) const {
        return expression == o.expression && gaze == o.gaze;
    }
    bool operator!=(const DisplayTarget& o) const { return !(*this == o); }
};

inline constexpr DisplayTarget kIdleTarget{Expression::NEUTRAL, GazeDirection::CENTER};

// Flip a gaze from the robot's frame (LEFT = the robot's left, REP-103 +y) into
// the eye core's panel coordinates (LEFT = toward x = 0). The two disagree by
// the horizontal sign; the panel itself stays unmirrored. Self-inverse.
GazeDirection toScreenGaze(GazeDirection gaze);

// Which posture stick the operator is holding, read off the body-pose offset it
// produces: the left stick tilts the body (roll/pitch), the right stick shifts
// it (x/y). Yaw and height are button-bound, not stick-bound, so they are not
// part of either. A stick counts as held once its offset magnitude reaches the
// threshold, and stays held until it falls below threshold * posture_exit_ratio
// — the same enter/exit shape as the gaze axes, so a hand resting on the
// threshold does not blink the face back and forth.
struct PostureSticks {
    bool tilt = false;   // left stick
    bool shift = false;  // right stick
};

PostureSticks postureSticks(const PolicyInputs& inputs, const PolicyConfig& config,
                            PostureSticks prev);

// Sign-quantize value to {-1, 0, +1} with hysteresis. Enters a direction at
// |value| >= deadband; once entered, holds it until |value| drops below
// deadband * exit_ratio. A sign flip past the full deadband switches directly
// without passing through 0.
int quantizeAxis(double value, int prev_sign, double deadband, double exit_ratio);

DisplayTarget decide(const PolicyInputs& inputs, const PolicyConfig& config,
                     const DisplayTarget& prev);

// Pick the face animation for this tick, or nullopt. Breathing runs while the
// face sleeps — no gait state heard yet, or folded; idling runs while the
// hexapod stands idle, level, and command-free. Battery flags, an active
// posture animation mode, and Bluetooth scanning suppress both.
std::optional<std::string> selectFaceAnimation(const PolicyInputs& inputs,
                                               const PolicyConfig& config);

// Debounce raw battery voltage into (low, critical) flags. A threshold of 0.0
// disables that flag entirely. A flag raises only after the voltage has stayed
// below the threshold for hold_s seconds, and clears only once it rises above
// threshold + hysteresis_v (no hold on the way up: good news is immediate).
class BatteryMonitor {
public:
    struct Flags {
        bool low = false;
        bool critical = false;
        bool operator==(const Flags& o) const {
            return low == o.low && critical == o.critical;
        }
    };

    BatteryMonitor(double warning_v = 0.0, double critical_v = 0.0,
                   double hysteresis_v = 0.3, double hold_s = 3.0);

    Flags update(double voltage, double t);

private:
    struct StepResult {
        bool active;
        std::optional<double> below_since;
    };
    StepResult step(double voltage, double t, double threshold, bool active,
                    std::optional<double> below_since) const;

    double warning_v_;
    double critical_v_;
    double hysteresis_v_;
    double hold_s_;
    bool low_ = false;
    bool critical_ = false;
    std::optional<double> below_warning_since_;
    std::optional<double> below_critical_since_;
};

}  // namespace hexa::display
