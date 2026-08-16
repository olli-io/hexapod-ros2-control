// Bluepad32 pairs a gamepad over the CYW43439; this adapts its state into the
// raw axes[]/buttons[] layout teleop_joy.yaml's `base` block expects.
#pragma once

#include <cstdint>

namespace bt_teleop {

// Must match teleop_joy.yaml `base.axes`.
enum Axis : int {
    kLeftStickX = 0,
    kLeftStickY = 1,
    kL2 = 2,  // analog trigger (rest = +max, pressed = -max)
    kRightStickX = 3,
    kRightStickY = 4,
    kR2 = 5,  // analog trigger
    kDpadX = 6,
    kDpadY = 7,
    kNumAxes = 8,
};

// Must match teleop_joy.yaml `base.buttons`.
enum Button : int {
    kA = 0,
    kB = 1,
    kX = 2,
    kY = 3,
    kL1 = 4,
    kR1 = 5,
    kSelect = 6,
    kStart = 7,
    kNumButtons = 8,
};

// Linux-joystick int16 convention; map_joy's 1/32767 scale lands these at ±1.0.
constexpr int16_t kAxisMax = 32767;
constexpr int16_t kAxisMin = -32767;

// This module OWNS cyw43_arch — do not call cyw43_arch_init() elsewhere. It comes
// up on core1 so Bluetooth servicing never jitters core0's control loop:
//   core0: init() -> multicore_launch_core1(...) -> wait_ready()
//   core1: core1_init() -> other core1 subsystems -> core1_signal() -> loop
// Call the core1_* functions from core1 only.
bool init();
bool core1_init();
void core1_signal(bool ok);
void core1_poll_led();  // call at ~5 ms
bool wait_ready();

// Open / close a pairing window. Only needed to meet a NEW pad — link keys
// persist in flash. Nothing scans at boot; the front-panel button opens it.
void start_pairing();
void stop_pairing();
bool scanning();

// Published by core0, written to the cyw43 GPIO by core1. Lock-free.
void set_led(bool on);

// No-op: BTstack is serviced in the background on core1. Kept so a future
// poll-mode build has a call site.
void pump();

// Latest snapshot. False, with a neutral state (sticks centered, triggers at
// +max), when idle.
bool read(int16_t axes[kNumAxes], uint32_t& buttons);

bool connected();

// time_us_64() of the most recent controller frame, 0 before the first. Lets the
// watchdog catch a pad that stopped sending without disconnecting.
std::uint64_t last_data_us();

}  // namespace bt_teleop
