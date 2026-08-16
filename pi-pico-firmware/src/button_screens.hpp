// What the front-panel button puts on the panel, and the pack voltage ->
// percentage map behind it.
//
// Pure and allocation-free (snprintf into a caller-owned buffer): the render
// path this feeds runs on core1, which must never touch the heap. The Pi's
// equivalent is hexa_buttons/info_text.py — the percentage formula is kept
// identical to it deliberately, so the same pack reads the same on both faces.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdio>

namespace button {

// Pack percentage 0-100, clamped. A non-positive span (mis-set config) reads 0
// rather than dividing by zero. lroundf rounds half away from zero, so a 50.5 %
// pack shows 51 — these are read off a panel by a human.
inline int batteryPercent(float voltage, float empty_v, float full_v) {
    const float span = full_v - empty_v;
    if (span <= 0.0f) return 0;
    float ratio = (voltage - empty_v) / span;
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    return static_cast<int>(lroundf(ratio * 100.0f));
}

// Two centered lines: percentage, then the raw volts it came from. The volts
// matter because the percentage is a linear approximation of a LiPo curve — a
// reader who knows the pack can sanity-check it.
//
// `valid` false means the Chica GET did not answer; say so rather than showing
// a percentage derived from a stale or zero reading.
inline void batteryScreenText(char* out, std::size_t n, bool valid, float voltage,
                              float empty_v, float full_v) {
    if (out == nullptr || n == 0) return;
    if (!valid) {
        std::snprintf(out, n, "Battery\nno reading");
        return;
    }
    std::snprintf(out, n, "Battery %d %%\n%.1f V",
                  batteryPercent(voltage, empty_v, full_v),
                  static_cast<double>(voltage));
}

// Shown while a pairing window is open, so the hold has visible confirmation
// even though the eyes also switch to the scanning spinner afterwards.
inline void pairingScreenText(char* out, std::size_t n) {
    if (out == nullptr || n == 0) return;
    std::snprintf(out, n, "Pairing\nput the pad in\npairing mode");
}

}  // namespace button
