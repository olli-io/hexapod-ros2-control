// Front-panel button on the Pico.
//
// One button, two actions: press shows the pack percentage on the panel, hold
// opens a Bluetooth pairing window. A producer into the face and into
// bt_teleop, never the other way round — the face stays a pure sink, exactly
// the relationship hexa_buttons has with hexa_display on the Pi.
//
// Core0 only: polled from the control tick, which at 200 Hz is far finer than
// the debounce window.
#pragma once

namespace button {

// Configure the GPIO (input, pull-up, active-low). Call once at boot.
void init();

// Poll the line and dispatch. `battery_valid`/`battery_v` are the latest Chica
// reading, `pad_connected` closes an open pairing window early. now_s is
// monotonic seconds.
void tick(double now_s, bool battery_valid, float battery_v, bool pad_connected);

}  // namespace button
