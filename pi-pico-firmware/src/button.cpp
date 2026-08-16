// Front-panel button — GPIO seam + dispatch. See button.hpp.
//
// The discrimination and the screen strings are pure and live in button_fsm.hpp
// / button_screens.hpp (host-tested in shared/motion_core/test/test_button.cpp);
// this file is only the hardware and the wiring to face + bt_teleop.

#include "button.hpp"

#include <optional>

#include "hardware/gpio.h"

#include "bt_teleop.hpp"
#include "button_fsm.hpp"
#include "button_screens.hpp"
#include "config_generated.hpp"
#include "dbg.hpp"
#include "face.hpp"

namespace button {
namespace {

const auto& kCfg = hexa::config::kPicoButton;

std::optional<PressDetector> g_detector;

// Deadline for an open pairing window; 0 = closed.
double g_pair_until_s = 0.0;

// Active-low with the internal pull-up: a button to ground needs no external
// parts, and an unconnected pin reads released rather than stuck-pressed.
bool line_down() { return !gpio_get(kCfg.pin); }

void show_battery(double now_s, bool valid, float voltage) {
    char text[64];
    batteryScreenText(text, sizeof(text), valid, voltage,
                      hexa::config::kBatteryEmptyV, hexa::config::kBatteryFullV);
    face::show_text(text, now_s, kCfg.screen_s);
    HEXA_DBG("[button] battery screen: %s\n", valid ? "shown" : "no reading");
}

void open_pairing(double now_s) {
    char text[64];
    pairingScreenText(text, sizeof(text));
    face::show_text(text, now_s, kCfg.screen_s);
    bt_teleop::start_pairing();
    g_pair_until_s = now_s + static_cast<double>(kCfg.pair_window_s);
    HEXA_DBG("[button] pairing window open for %.0f s\n",
             static_cast<double>(kCfg.pair_window_s));
}

void close_pairing(const char* why) {
    bt_teleop::stop_pairing();
    g_pair_until_s = 0.0;
    HEXA_DBG("[button] pairing window closed (%s)\n", why);
}

}  // namespace

void init() {
    gpio_init(kCfg.pin);
    gpio_set_dir(kCfg.pin, GPIO_IN);
    gpio_pull_up(kCfg.pin);
    g_detector.emplace(kCfg.hold_s, kCfg.debounce_s);
    HEXA_DBG("[button] GP%u ready — press for battery, hold %.0f s to pair\n",
             static_cast<unsigned>(kCfg.pin), static_cast<double>(kCfg.hold_s));
}

void tick(double now_s, bool battery_valid, float battery_v, bool pad_connected) {
    if (!g_detector) return;

    // Close an open window early once a pad binds: the whole point of the
    // window was to meet one, and leaving it open only invites a second.
    if (g_pair_until_s > 0.0) {
        if (pad_connected) {
            close_pairing("pad connected");
        } else if (now_s >= g_pair_until_s) {
            close_pairing("timed out");
        }
    }

    switch (g_detector->update(line_down(), now_s)) {
        case Event::kPress:
            show_battery(now_s, battery_valid, battery_v);
            break;
        case Event::kHold:
            open_pairing(now_s);
            break;
        case Event::kNone:
            break;
    }
}

}  // namespace button
