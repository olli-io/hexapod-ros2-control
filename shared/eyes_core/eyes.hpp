// Drawing of the eyes and every mood. Port of reference/eyes.py.
#pragma once

#include <string>
#include <vector>

#include "image.hpp"

constexpr int kW = 256, kH = 64;   // display size
constexpr int kSS = 4;             // supersampling, smooth edges in 16 greys

// Top-level moods. "scanning" is a family; see kScanning.
extern const std::vector<std::string> kMoods;
// Scanning sub moods, in the order `--mood scanning` steps through them.
extern const std::vector<std::string> kScanning;
constexpr double kSubSeconds = 15.0;   // time on each sub mood when cycling

// dx, dy: look offset in px. blink: 1.0 open .. ~0.08 closed.
// `mood` is a top-level mood other than "scanning", or one of kScanning
// (including "scanning" itself, meaning both eyes clockwise).
// phase: 0..1 lap of the animation phase, t: seconds since start.
Image draw_eyes(double dx, double dy, double blink, const std::string& mood,
                double phase, double t);
