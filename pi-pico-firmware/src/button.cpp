// Front-panel buttons — GPIO seam + dispatch. See button.hpp.
//
// The discrimination and the screen logic are pure and live in button_fsm.hpp
// / button_screens.hpp (host-tested in shared/motion_core/test/test_button.cpp);
// this file is only the hardware and the wiring to face + bt_teleop.

#include "button.hpp"

#include <cstring>
#include <optional>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "bt_teleop.hpp"
#include "button_fsm.hpp"
#include "button_screens.hpp"
#include "config_generated.hpp"
#include "dbg.hpp"
#include "face.hpp"

namespace button {
namespace {

const auto& kCfg = hexa::config::kPicoButton;

// One physical button: its line, debounce/hold detector and last stable level.
struct Line {
    unsigned pin;
    Button id;
    std::optional<PressDetector> detector;
    bool was_down = false;
};

Line g_info{kCfg.info_pin, Button::kInfo};
Line g_bt{kCfg.bluetooth_pin, Button::kBluetooth};
std::optional<ScreenSequencer> g_sequencer;
bool g_was_connected = false;
char g_text[face::kTextMax] = {};  // last text handed to the face

// Active-low with the internal pull-up: a button to ground needs no external
// parts, and an unconnected pin reads released rather than stuck-pressed.
bool line_down(const Line& l) { return !gpio_get(l.pin); }

void init_line(Line& l) {
    gpio_init(l.pin);
    gpio_set_dir(l.pin, GPIO_IN);
    gpio_pull_up(l.pin);
}

// Sample the boot level after the pull-ups settle, so a button held at boot
// needs a release before it counts.
void arm_line(Line& l) {
    l.was_down = line_down(l);
    l.detector.emplace(kCfg.hold_s, kCfg.debounce_s, l.was_down);
    HEXA_DBG("[button] GP%u ready%s\n", l.pin,
             l.was_down ? " (held at boot: ignored until released)" : "");
}

void poll_line(Line& l, ScreenSequencer& seq, double now_s) {
    const Event e = l.detector->update(line_down(l), now_s);
    const bool down = l.detector->down();
    if (down && !l.was_down) seq.press(l.id, now_s);
    l.was_down = down;
    if (e == Event::kPress) seq.release(l.id, now_s);
    if (e == Event::kHold) seq.hold(l.id, now_s);
}

const char* screen_name(Screen s) {
    switch (s) {
        case Screen::kNone: return "none";
        case Screen::kBattery: return "battery";
        case Screen::kBluetooth: return "bluetooth";
        case Screen::kScanning: return "scanning";
    }
    return "?";
}

void on_screen_change(Screen from, Screen to) {
    HEXA_DBG("[button] screen -> %s\n", screen_name(to));
    if (to == Screen::kScanning) bt_teleop::start_pairing();
    if (from == Screen::kScanning) bt_teleop::stop_pairing();
}

}  // namespace

void init() {
    init_line(g_info);
    init_line(g_bt);
    sleep_us(10);
    arm_line(g_info);
    arm_line(g_bt);
    g_sequencer.emplace(kCfg.screen_s, kCfg.pair_window_s);
}

void tick(double now_s, bool pad_connected, bool battery_valid, float battery_v) {
    if (!g_sequencer) return;
    ScreenSequencer& seq = *g_sequencer;
    const Screen before = seq.screen();

    poll_line(g_info, seq, now_s);
    poll_line(g_bt, seq, now_s);

    if (pad_connected && !g_was_connected) seq.controllerSeen(now_s);
    g_was_connected = pad_connected;

    seq.tick(now_s);

    const Screen after = seq.screen();
    if (after != before) on_screen_change(before, after);

    // Rebuilt every tick so an open screen follows the pack and the pad.
    char name[face::kTextMax];
    bt_teleop::controller_name(name, sizeof(name));
    ScreenInputs in;
    in.controller = name;
    in.battery_valid = battery_valid;
    in.battery_v = battery_v;
    in.battery_empty_v = hexa::config::kBatteryEmptyV;
    in.battery_full_v = hexa::config::kBatteryFullV;
    in.hold_s = kCfg.hold_s;
    char text[face::kTextMax];
    screenText(text, sizeof(text), after, in);
    if (std::strcmp(text, g_text) != 0) {
        std::memcpy(g_text, text, sizeof(g_text));
        face::set_text(g_text);
    }
}

}  // namespace button
