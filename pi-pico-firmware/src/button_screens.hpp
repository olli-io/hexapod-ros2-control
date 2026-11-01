// The front-panel buttons' screen sequencer and screen text.
//
// A port of hexa_buttons (screen_logic.py, info_text.py), so the buttons behave
// the same on both boards:
//   - info press toggles the battery screen,
//   - bluetooth press toggles the controller-status screen,
//   - a bluetooth hold starts a pairing scan,
//   - while a scan runs, either press cancels it, as do scan_timeout_s and a
//     controller connecting.
// The Pico has no network, so the info hold does nothing and the battery
// screen carries no address.
//
// Pure and allocation-free (snprintf into a caller-owned buffer), so it is
// host-tested in shared/motion_core/test/test_button.cpp.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdio>

namespace button {

enum class Button { kInfo, kBluetooth };

enum class Screen {
    kNone,       // the face owns the panel
    kBattery,
    kBluetooth,  // controller status
    kScanning,   // pairing scan; no text, the face wears its spinners
};

class ScreenSequencer {
public:
    ScreenSequencer(float screen_timeout_s, float scan_timeout_s)
        : _screen_timeout_s(screen_timeout_s), _scan_timeout_s(scan_timeout_s) {}

    Screen screen() const { return _screen; }

    // Debounced down edge.
    void press(Button b, double t) {
        bool& suppress = suppressFor(b);
        suppress = false;
        if (_screen == Screen::kScanning) {
            // The press is the cancel, so its release must not open a screen.
            suppress = true;
            set(Screen::kNone, t);
        }
    }

    // Release short of hold_s. A release after a hold never arrives here
    // (PressDetector swallows it).
    void release(Button b, double t) {
        bool& suppress = suppressFor(b);
        if (suppress) {
            suppress = false;
            return;
        }
        const Screen own = b == Button::kInfo ? Screen::kBattery : Screen::kBluetooth;
        set(_screen == own ? Screen::kNone : own, t);
    }

    void hold(Button b, double t) {
        if (b != Button::kBluetooth || _screen == Screen::kScanning) return;
        set(Screen::kScanning, t);
    }

    // A controller connected: a running scan has succeeded.
    void controllerSeen(double t) {
        if (_screen == Screen::kScanning) set(Screen::kNone, t);
    }

    void tick(double t) {
        if (_screen == Screen::kNone) return;
        // A scan ignores screen_timeout_s: it is a job, not something to read.
        const float limit =
            _screen == Screen::kScanning ? _scan_timeout_s : _screen_timeout_s;
        if (t - _entered_at >= static_cast<double>(limit)) set(Screen::kNone, t);
    }

private:
    bool& suppressFor(Button b) {
        return b == Button::kInfo ? _suppress_info_release : _suppress_bt_release;
    }

    void set(Screen s, double t) {
        if (s == _screen) return;
        _screen = s;
        _entered_at = t;  // every transition restarts the timeout clock
    }

    float  _screen_timeout_s;
    float  _scan_timeout_s;
    Screen _screen = Screen::kNone;
    double _entered_at = 0.0;
    bool   _suppress_info_release = false;
    bool   _suppress_bt_release = false;
};

// Characters per panel line (info_text.py LINE_BUDGET). A longer controller
// name is clipped rather than wrapped, so the hold hint stays on the panel.
constexpr int kLineBudget = 30;

// Pack percentage 0-100, clamped. A non-positive span (mis-set config) reads 0
// rather than dividing by zero. Rounds half away from zero, as info_text.py
// does, so the same pack reads the same on both boards.
inline int batteryPercent(float voltage, float empty_v, float full_v) {
    const float span = full_v - empty_v;
    if (span <= 0.0f) return 0;
    float ratio = (voltage - empty_v) / span;
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    return static_cast<int>(std::lround(ratio * 100.0f));
}

// info_text.py battery_screen(), battery line only. `valid` false (no Chica
// reply) shows dashes, not a fabricated 0 % that reads as a dead pack.
inline void batteryScreenText(char* out, std::size_t n, bool valid, float voltage,
                              float empty_v, float full_v) {
    if (out == nullptr || n == 0) return;
    if (!valid) {
        std::snprintf(out, n, "Battery -> -- %%  ( --.- V )");
        return;
    }
    std::snprintf(out, n, "Battery -> %d %%  ( %.1f V )",
                  batteryPercent(voltage, empty_v, full_v),
                  static_cast<double>(voltage));
}

// info_text.py bluetooth_screen(). `controller` empty = none connected.
// `hold_s` is the value the button waits for, so the hint can never promise a
// hold the button does not honour.
inline void bluetoothScreenText(char* out, std::size_t n, const char* controller,
                                float hold_s) {
    if (out == nullptr || n == 0) return;
    const long hold = std::lround(hold_s);
    if (controller == nullptr || controller[0] == '\0') {
        std::snprintf(out, n, "No connected controllers\nHold %ld seconds to pair",
                      hold);
        return;
    }
    std::snprintf(out, n, "Connected to:\n%.*s\nHold %ld seconds to pair",
                  kLineBudget, controller, hold);
}

// What a screen renders from.
struct ScreenInputs {
    const char* controller = "";
    bool  battery_valid = false;
    float battery_v = 0.0f;
    float battery_empty_v = 0.0f;
    float battery_full_v = 0.0f;
    float hold_s = 0.0f;
};

// Text for a screen; empty hands the panel back to the face.
inline void screenText(char* out, std::size_t n, Screen screen,
                       const ScreenInputs& in) {
    if (out == nullptr || n == 0) return;
    switch (screen) {
        case Screen::kBattery:
            batteryScreenText(out, n, in.battery_valid, in.battery_v,
                              in.battery_empty_v, in.battery_full_v);
            return;
        case Screen::kBluetooth:
            bluetoothScreenText(out, n, in.controller, in.hold_s);
            return;
        case Screen::kNone:
        case Screen::kScanning:
            out[0] = '\0';
            return;
    }
}

}  // namespace button
