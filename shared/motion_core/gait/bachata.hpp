// Bachata routine: a hardcoded strafing dance in tripod beats, played from a
// stand on a preset that names it. One pure function of time gives all six
// feet and the body term; the engine only keeps the clock and the stop.
//
// A bar is eight beats. Each half is led by the tripod whose middle leg is on
// the travel side. The left half: tripod B (left middle) steps left, A steps
// left, B steps left, A taps in place. The right half mirrors it, led by A. A step beat moves the body [1/2, 1, 1/2] of `stride` along the travel,
// so every half bar starts and ends with all six feet on nominal.
//
// The body shifts away from the two-leg side of the swinging tripod and tilts
// down the same way, both at their extreme at each touchdown. It rises with
// the square of the shift, so it draws a U that peaks at each touchdown. A tap is a
// half sway: side, centre, same side, and lifts the feet half as high.
#pragma once

#include <map>
#include <optional>
#include <string>

#include "gait/gaits/base.hpp"
#include "gait/types.hpp"
#include "kinematics/body_transform.hpp"

namespace hexa::gait {

inline constexpr int kBachataBeatsPerHalf = 4;

struct BachataParams {
  float beat_time = 0.0f;  // s
  float stride = 0.0f;     // m, body travel of the middle step beat
  float sway_y = 0.0f;     // m, body shift at a touchdown, toward the tilt
  float sway_z = 0.0f;     // m, body rise at a touchdown
  float sway_roll = 0.0f;  // rad, body tilt at a touchdown
  // Share of a beat at each end with all six feet down.
  float planted_share = 0.0f;
  SwingProfile swing;
};

struct BachataFrame {
  std::map<std::string, LegOutput> legs;
  BodyPose body;
};

// The routine at t s from its start. `halves` set: the number of half bars
// danced before the one still outro beat that eases the body back to
// identity; past it the robot stands on nominal.
BachataFrame bachata_sample(float t, std::optional<int> halves,
                            const BachataParams& params,
                            const std::map<std::string, Vec3>& nominal);

// The end of the outro for `halves` half bars.
float bachata_duration(int halves, const BachataParams& params);

}  // namespace hexa::gait
