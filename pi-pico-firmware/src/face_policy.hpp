// Pure face-config builder (part 11) — no Pico SDK, so face.cpp and the host
// test (shared/motion_core/test/test_face_config.cpp) share one implementation.
//
// Turns the baked hexa::config face knobs (config_generated.hpp, from
// display.yaml) into the expression/gaze PolicyConfig the shared policy library
// consumes. Expression names resolve to the Expression enum here, at runtime, so
// the generated header never depends on the vendored eye core.
#pragma once

#include <string>
#include <string_view>

#include "config_generated.hpp"
#include "supervisor.hpp"

#include "Expression.h"
#include "FaceNames.h"
#include "expression_policy.hpp"

namespace face {

inline Expression exprOrNeutral(std::string_view name) {
    if (auto e = parseExpression(std::string(name))) return *e;
    return Expression::NEUTRAL;  // bake-time validated; defensive fallback
}

inline hexa::display::PolicyConfig buildPolicyConfig() {
    hexa::display::PolicyConfig c;
    for (const auto& e : hexa::config::kFaceExpressionMap)
        c.expression_map[std::string(e.state)] = exprOrNeutral(e.expression);
    c.animation_expression = exprOrNeutral(hexa::config::kFaceAnimationExpression);
    c.battery_warning_expression =
        exprOrNeutral(hexa::config::kFaceBatteryWarningExpression);
    c.battery_critical_expression =
        exprOrNeutral(hexa::config::kFaceBatteryCriticalExpression);
    c.scanning_expression = exprOrNeutral(hexa::config::kFaceScanningExpression);

    c.posture_tilt_expression =
        exprOrNeutral(hexa::config::kFacePostureTiltExpression);
    c.posture_shift_expression =
        exprOrNeutral(hexa::config::kFacePostureShiftExpression);
    c.posture_both_expression =
        exprOrNeutral(hexa::config::kFacePostureBothExpression);

    const auto& po = hexa::config::kFacePosture;
    c.posture_tilt_threshold_rad = po.tilt_threshold_rad;
    c.posture_shift_threshold_m = po.shift_threshold_m;
    c.posture_exit_ratio = po.exit_ratio;

    const auto& gz = hexa::config::kFaceGaze;
    c.gaze_deadband = gz.gaze_deadband;
    c.gaze_exit_ratio = gz.gaze_exit_ratio;
    c.gaze_wz_weight = gz.gaze_wz_weight;
    c.gaze_vy_max = gz.gaze_vy_max;
    c.gaze_wz_max = gz.gaze_wz_max;
    c.pose_pitch_threshold_rad = gz.pose_pitch_threshold_rad;
    c.pose_tilt_threshold_rad = gz.pose_tilt_threshold_rad;
    c.idling_start_delay_s = gz.idling_start_delay_s;
    return c;
}

// The face's two battery flags, off the supervisor's undervoltage ladder.
//
// The Pi node debounces a raw voltage through its own BatteryMonitor; on the
// Pico the supervisor has already done that work and exposes the result as a
// latched rung, so the face reads the rung rather than re-deriving it. kWarn is
// the "idle-only" warning face; kFold and kCutoff both mean the robot is going
// down, which is the critical face.
struct BatteryFlags {
    bool low = false;
    bool critical = false;
};

inline BatteryFlags undervoltFlags(hexa::supervisor::UndervoltStage stage) {
    using S = hexa::supervisor::UndervoltStage;
    return {stage >= S::kWarn, stage >= S::kFold};
}

}  // namespace face
