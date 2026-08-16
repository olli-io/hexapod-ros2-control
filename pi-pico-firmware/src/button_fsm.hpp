// Front-panel button: debounce + short-press / hold discrimination.
//
// Pure — no Pico SDK, no clock of its own — so the caller polls it with a raw
// line level and a monotonic timestamp and it is host-testable. The Pico glue
// (GPIO setup, dispatch) is button.cpp; the ROS side has no counterpart, since
// hexa_buttons is the Pi's own device with its own features.
//
// Hold wins: once a press reaches hold_s it fires kHold immediately, and the
// eventual release is silent. Otherwise the release fires kPress. That is the
// gpiozero when_held / when_released split the Pi's buttons use, so the two
// boards feel the same under the thumb.
#pragma once

namespace button {

enum class Event {
    kNone,
    kPress,  // released before hold_s
    kHold,   // held to hold_s; fires once, mid-press
};

class PressDetector {
public:
    PressDetector(float hold_s, float debounce_s)
        : _hold_s(hold_s), _debounce_s(debounce_s) {}

    // `down` is the debounced-by-us raw level (true = pressed). `now_s` is
    // monotonic seconds. Call at any rate at least a few times per debounce_s.
    Event update(bool down, double now_s) {
        // Debounce: ignore level changes inside the settling window. Bounces on
        // a mechanical contact land within a few ms of the edge; anything later
        // is a real transition.
        if (down != _stable && (now_s - _last_change_s) >= _debounce_s) {
            _stable = down;
            _last_change_s = now_s;
            if (down) {
                _press_start_s = now_s;
                _hold_fired = false;
            } else if (!_hold_fired) {
                return Event::kPress;
            }
        }

        if (_stable && !_hold_fired &&
            (now_s - _press_start_s) >= static_cast<double>(_hold_s)) {
            _hold_fired = true;
            return Event::kHold;
        }
        return Event::kNone;
    }

    bool down() const { return _stable; }

private:
    float  _hold_s;
    float  _debounce_s;
    bool   _stable = false;
    bool   _hold_fired = false;
    double _last_change_s = -1e9;  // so the first edge is never debounced away
    double _press_start_s = 0.0;
};

}  // namespace button
