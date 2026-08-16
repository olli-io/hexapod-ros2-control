// Hexapod face on the Pico 2 W (part 11).
//
// The 256x64 SH1122 OLED eyes, driven by the same shared source as the sim/ROS
// face — the vendored eye core (EyeAnim/EyeRaster) plus the pure policy library
// — with only the panel transport swapped (Sh1122PanelPico).
//
// Split across the two cores so the SPI flush never touches the 5 ms control
// tick: core0 runs the policy at kFaceUpdateRateHz and publishes a small render
// target; core1 runs EyeAnim + raster + flush at kFaceRenderHz against it. core0
// holds the target mutex only for a struct copy. main.cpp owns core1's
// scheduler; this module never launches a core.

#pragma once

#include <cstdint>
#include <string_view>

#include "gait/engine.hpp"  // hexa::gait::EngineState

namespace face {

// Mirrors the ROS face's topic inputs (/gait/state, /cmd_vel, /body/pose,
// /animation/mode, battery), built each control tick from the pipeline result.
struct FaceState {
    hexa::gait::EngineState engine_state = hexa::gait::EngineState::FOLDED;
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;        // unshaped command (walking gaze)
    float x = 0.0f, y = 0.0f;                     // body shift (posture expression)
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // body tilt (pitch/tilt gaze)
    bool  battery_low = false, battery_critical = false;
    // No pad bound — BTstack is scanning. The Pico's analog of the Pi's
    // /bluetooth/scanning: outranks every other rule and wears the spinner.
    bool  busy = false;
    bool  animation_event = false;         // an accepted animation select this tick
    std::string_view animation_name = "";  // current mode ("" = none/default)
};

// Core0, once at boot, before core1 launches.
void init();

// Core0. Rate-limited internally to kFaceUpdateRateHz. now_s is monotonic.
void tick(const FaceState& state, double now_s);

// Core0. Replace the eyes with a text screen until now_s + seconds, then revert.
// The Pi latches text mode on /display/text and clears it with an empty message;
// here the only producer is a button press, which always wants a timed screen.
// Text longer than the payload buffer is truncated, not wrapped — keep screens
// inside the budget test_button.cpp enforces.
void show_text(const char* text, double now_s, double seconds);

// Core1 only. Panel SPI/GPIO belong to the core that renders.
void core1_init();
void render_tick();  // call at kFaceRenderHz

// Heartbeat diagnostics.
std::uint64_t flush_count();
std::uint64_t last_flush_us();
std::uint32_t actual_spi_hz();

}  // namespace face
