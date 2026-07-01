// Pure telemetry helpers for the posture node. Port of the free functions and
// module constants in hexa_posture/posture_node.py.
//
// These turn raw inputs (gait state, /cmd_vel, /legs/targets) into the gate /
// bool / scalar / vector signals the tick loop needs. They are ROS-free: the
// node unpacks messages into plain data before calling them, which keeps this
// code unit-testable without spinning a node.
#pragma once

#include <array>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "hexa_posture_cpp/types.hpp"

namespace hexa_posture {

// Gait engine states in which body posture is meaningful. In the other states
// (folded, initialize, folding) the legs are not at the nominal footprint, so a
// body-pose offset would compose against the wrong foot configuration. The pause
// trio and reseating are included so a persistent body pose keeps applying while
// the engine soft-releases or walks the feet to a new stance.
inline const std::set<std::string> POSTURE_ACTIVE_STATES = {
    "stand", "engaging", "gait", "pausing", "paused", "resuming", "reseating"};

// True iff `state` is one in which the animation stack should run.
bool is_posture_active(const std::string& state);

// Below this magnitude, every /cmd_vel component counts as zero.
constexpr double CMD_VEL_ZERO_TOL = 1e-4;

// True iff all six twist components are within CMD_VEL_ZERO_TOL of zero.
bool twist_is_zero(double linear_x, double linear_y, double linear_z,
                   double angular_x, double angular_y, double angular_z);

// Minimum stance legs needed to define a meaningful support polygon. During
// swing transitions the count can dip; callers hold the previous value rather
// than emit a noisy signal.
constexpr int MIN_STANCE_FOR_CENTROID = 3;

// Mean of foot (x, y) over legs flagged stance=true. Returns nullopt when fewer
// than MIN_STANCE_FOR_CENTROID legs are in stance.
std::optional<std::pair<double, double>> stance_centroid_xy(
    const std::array<Vec3, LEG_COUNT>& feet,
    const std::array<bool, LEG_COUNT>& stance);

// Max foot lift (m) above the stance polygon's mean z: max(swing foot.z) −
// mean(stance foot.z), clamped >= 0. Returns nullopt when the stance polygon is
// degenerate; returns 0.0 (observed and quiet) when no leg is in swing.
std::optional<double> max_swing_lift_z(
    const std::array<Vec3, LEG_COUNT>& feet,
    const std::array<bool, LEG_COUNT>& stance);

// One first-order low-pass step on a scalar signal. `raw` empty holds `prev`;
// `prev` empty seeds from `raw` (no startup transient); else
// alpha = dt / (tau + dt) (or 1.0 if tau + dt <= 0).
std::optional<double> lpf_step_scalar(std::optional<double> prev,
                                      std::optional<double> raw, double tau,
                                      double dt);

// One first-order low-pass step on an XY signal. Same seed/hold semantics as
// lpf_step_scalar, applied per component.
std::optional<std::pair<double, double>> lpf_step_xy(
    std::optional<std::pair<double, double>> prev,
    std::optional<std::pair<double, double>> raw, double tau, double dt);

}  // namespace hexa_posture
