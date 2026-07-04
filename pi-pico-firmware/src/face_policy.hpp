// Pure face-config builder (part 11) — no Pico SDK, so face.cpp and the host
// test (test/host/test_face_policy.cpp) share one implementation.
//
// Turns the baked hexa::config face knobs (config_generated.hpp, from
// display.yaml) into the expression/gaze PolicyConfig the shared policy library
// consumes. Expression names resolve to the Expression enum here, at runtime, so
// the generated header never depends on the vendored eye core.
#pragma once

#include <string>
#include <string_view>

#include "config_generated.hpp"

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

}  // namespace face
