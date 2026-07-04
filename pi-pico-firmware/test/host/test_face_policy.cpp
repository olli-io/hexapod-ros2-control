// Host test for the firmware face config wiring (part 11).
//
// The eye policy itself (decide / selectFaceAnimation / the animation runner) is
// exhaustively covered by hexa_display's gtest suite — that same source compiles
// here. What this checks is the FIRMWARE path the ROS node never exercises:
// display.yaml -> gen_config.py -> config_generated.hpp -> face::buildPolicyConfig()
// -> the shared policy. i.e. that the baked constants flow into the policy with
// the right names/types and produce the expected expression per gait state.
//
// Self-contained (no gtest) so it builds with a plain host compiler as well as
// through ctest; a non-zero exit means a failed check.

#include <cstdio>
#include <optional>
#include <string>

#include "expression_policy.hpp"
#include "face_policy.hpp"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);   \
            ++g_failures;                                                 \
        }                                                                 \
    } while (0)

// A PolicyInputs shaped exactly as main.cpp's core0 assembler builds it: gait
// state present (link up), everything else neutral.
hexa::display::PolicyInputs walking(const std::string& state) {
    hexa::display::PolicyInputs in;
    in.gait_state = state;
    return in;
}

void test_config_bake() {
    const hexa::display::PolicyConfig c = face::buildPolicyConfig();

    // Gait-state -> expression map, straight from display.yaml's defaults.
    CHECK(c.expression_map.at("gait") == Expression::HAPPY);
    CHECK(c.expression_map.at("folded") == Expression::SLEEPY);
    CHECK(c.expression_map.at("paused") == Expression::SLEEPY);
    CHECK(c.expression_map.at("stand") == Expression::NEUTRAL);
    CHECK(c.expression_map.at("initialize") == Expression::NEUTRAL);

    CHECK(c.animation_expression == Expression::WOOZY);
    CHECK(c.battery_warning_expression == Expression::SLEEPY);
    CHECK(c.battery_critical_expression == Expression::DEAD);

    // A couple of gaze/idling scalars survive the bake as floats.
    CHECK(c.gaze_deadband > 0.14 && c.gaze_deadband < 0.16);
    CHECK(c.idling_start_delay_s > 3.9 && c.idling_start_delay_s < 4.1);
}

void test_decide_precedence() {
    const hexa::display::PolicyConfig c = face::buildPolicyConfig();
    const hexa::display::DisplayTarget prev;

    // Gait-state map drives the expression through the baked config.
    CHECK(decide(walking("gait"), c, prev).expression == Expression::HAPPY);
    CHECK(decide(walking("folded"), c, prev).expression == Expression::SLEEPY);
    CHECK(decide(walking("stand"), c, prev).expression == Expression::NEUTRAL);

    // Animation mode outranks the gait map.
    hexa::display::PolicyInputs anim = walking("gait");
    anim.animation_mode = "wave";
    CHECK(decide(anim, c, prev).expression == Expression::WOOZY);

    // Battery-critical outranks everything and forces the gaze center.
    hexa::display::PolicyInputs crit = walking("gait");
    crit.battery_critical = true;
    crit.wz = 5.0;  // would otherwise pull the gaze hard left
    const hexa::display::DisplayTarget t = decide(crit, c, prev);
    CHECK(t.expression == Expression::DEAD);
    CHECK(t.gaze == GazeDirection::CENTER);
}

void test_boot_breathing() {
    const hexa::display::PolicyConfig c = face::buildPolicyConfig();
    // Before the gamepad first pairs the assembler leaves gait_state unset — the
    // policy asks for the boot "breathing" animation.
    hexa::display::PolicyInputs booting;  // gait_state == nullopt
    const std::optional<std::string> anim = selectFaceAnimation(booting, c);
    CHECK(anim.has_value() && *anim == "breathing");
}

}  // namespace

int main() {
    test_config_bake();
    test_decide_precedence();
    test_boot_breathing();
    if (g_failures == 0) {
        std::printf("test_face_policy: all checks passed\n");
        return 0;
    }
    std::printf("test_face_policy: %d check(s) failed\n", g_failures);
    return 1;
}
