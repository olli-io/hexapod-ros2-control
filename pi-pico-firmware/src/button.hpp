// Front-panel buttons on the Pico.
//
// Info button: press toggles the battery screen. Bluetooth button: press
// toggles the controller-status screen, hold starts a pairing scan. The Pi's
// hexa_buttons, ported (button_screens.hpp). A producer into the face and into
// bt_teleop, never the other way round — the face stays a pure sink, exactly
// the relationship hexa_buttons has with hexa_display.
//
// Core0 only: polled from the control tick, which at 200 Hz is far finer than
// the debounce window.
#pragma once

namespace button {

// Configure the GPIOs (input, pull-up, active-low). Call once at boot.
void init();

// Poll the lines and dispatch. `pad_connected` rising ends a running scan;
// `battery_valid`/`battery_v` are the latest Chica reading. now_s is monotonic
// seconds.
void tick(double now_s, bool pad_connected, bool battery_valid, float battery_v);

}  // namespace button
