// Hexapod face on the Pico 2 W (part 11).
//
// Closes the LED-face gap the firmware left open (part 09's status LED "stands
// in for the dropped hexa_display face"): the real 256x64 SH1122 OLED eyes,
// driven by the SAME shared source as the sim/ROS face — the vendored eye core
// (EyeAnim/EyeRaster) plus the pure policy library (expression_policy /
// face_animation). A link-time swap of only the panel transport (Sh1122PanelPico).
//
// Split across the two RP2350 cores so the ~8 ms SPI flush never touches the
// 5 ms / 200 Hz control tick:
//   - core0 runs the expression/gaze POLICY at kFaceUpdateRateHz from the
//     pipeline state it already has, and publishes a tiny render target;
//   - core1 runs the 60 Hz EyeAnim + raster + SPI flush against that target.
// See face.cpp. core0 only ever holds the target mutex for a struct copy, so the
// control loop's timing guarantee is preserved.

#pragma once

#include <cstdint>
#include <string_view>

#include "gait/engine.hpp"  // hexa::gait::EngineState

namespace face {

// Robot-state snapshot the core0 policy consumes. main.cpp builds it each
// control tick from the pipeline TickResult (mirrors the ROS face's topic
// inputs: /gait/state, /cmd_vel, /body/pose, /animation/mode, battery).
struct FaceState {
    hexa::gait::EngineState engine_state = hexa::gait::EngineState::FOLDED;
    bool  link_up = false;  // gamepad paired — gates the boot "breathing" anim
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;        // unshaped command (walking gaze)
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // user body pose (pitch/tilt gaze)
    bool  battery_low = false, battery_critical = false;
    bool  animation_event = false;         // an accepted animation select this tick
    std::string_view animation_name = "";  // current mode ("" = none/default)
};

// Build the policy + panel config from the baked hexa::config and, if
// kFaceEnabled, launch the core1 render loop. Call once at boot (after stdio).
void init();

// Core0 policy step. Rate-limited internally to kFaceUpdateRateHz (cheap no-op
// on the off-ticks). now_s is monotonic seconds (time_us_64() * 1e-6).
void tick(const FaceState& state, double now_s);

}  // namespace face
