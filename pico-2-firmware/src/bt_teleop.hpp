// Bluetooth gamepad teleop (plan part 02).
//
// Bluepad32 (bare Pico SDK C "platform" API, uni.h) pairs a gamepad over the
// CYW43439 and this module adapts its state into the exact raw axes[]/buttons[]
// layout that hexa_teleop/config/teleop_joy.yaml's `base` block expects — the
// drop-in replacement for the ROS `joy_publisher.py`. Part 07 feeds these
// arrays straight into the ported `map_joy` unchanged.
//
// BTstack runs in the cyw43 background context, so controller state arrives via
// platform callbacks; the cooperative loop just calls read() each tick.
#pragma once

#include <cstdint>

namespace bt_teleop {

// Axis indices — must match teleop_joy.yaml `base.axes`.
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

// Button indices — must match teleop_joy.yaml `base.buttons`.
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

// Axis extremes in the Linux-joystick int16 convention. map_joy applies the
// 1/32767 scale, so these land at ±1.0 after scaling.
constexpr int16_t kAxisMax = 32767;
constexpr int16_t kAxisMin = -32767;

// Bring up cyw43_arch, BTstack and Bluepad32, and register the custom platform.
// This OWNS cyw43_arch — do not call cyw43_arch_init() elsewhere. Returns false
// if cyw43_arch init fails (Bluepad32/BTstack bring-up is otherwise fire and
// forget: it runs in the background and pairs on the next controller).
bool init();

// Service hook for the cooperative loop. No-op in the background-serviced build
// (BTstack is driven by the cyw43 async context); kept for interface symmetry
// and a possible future poll-mode build. Safe to call every tick.
void pump();

// Copy the latest adapted snapshot into `axes` (kNumAxes entries) and `buttons`
// (bitmask: bit i set == button index i pressed). Returns true if a controller
// is connected; when idle it writes the safe neutral state (sticks/dpad
// centered, triggers released at +max, no buttons) and returns false.
bool read(int16_t axes[kNumAxes], uint32_t& buttons);

// True while a controller is connected. Convenience for status LEDs / the
// part 09 link-timeout failsafe.
bool connected();

// time_us_64() timestamp of the most recent controller-data frame (updated on
// every store_snapshot). The part 09 input watchdog compares this against the
// current time to catch stale-but-still-"connected" input (a pad that stopped
// sending without a clean disconnect). Returns 0 before the first frame.
std::uint64_t last_data_us();

}  // namespace bt_teleop
