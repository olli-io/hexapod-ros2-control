#include "gesture/validate.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

#include "kinematics/leg_ik.hpp"

namespace hexa::gesture {

namespace {

void check_body_keyframe(const std::string& id, const BodyKeyframe& k,
                         const posture::PoseLimits& limits) {
  const auto fail = [&](const char* axis) {
    throw std::invalid_argument("gesture '" + id + "' body keyframe at t=" +
                                std::to_string(k.t) + " s: " + axis +
                                " is outside the posture pose limits");
  };
  if (k.hold || k.home) {
    return;
  }
  if (std::fabs(k.x) > limits.x) fail("x");
  if (std::fabs(k.y) > limits.y) fail("y");
  if (k.z > limits.z_max || k.z < limits.z_min) fail("z");
  if (std::fabs(k.roll) > limits.roll) fail("roll");
  if (std::fabs(k.pitch) > limits.pitch) fail("pitch");
  if (std::fabs(k.yaw) > limits.yaw) fail("yaw");
}

// An oscillation swings each axis at most amp off the pose before it; both
// extremes must sit inside the limits.
void check_body_oscillation(const std::string& id, const BodyKeyframe& k,
                            const BodyKeyframe& base,
                            const posture::PoseLimits& limits) {
  for (float sign : {-1.0f, 1.0f}) {
    BodyKeyframe edge = base;
    edge.t = k.t;
    edge.x += sign * std::fabs(k.amp[0]);
    edge.y += sign * std::fabs(k.amp[1]);
    edge.z += sign * std::fabs(k.amp[2]);
    edge.roll += sign * std::fabs(k.amp[3]);
    edge.pitch += sign * std::fabs(k.amp[4]);
    edge.yaw += sign * std::fabs(k.amp[5]);
    check_body_keyframe(id, edge, limits);
  }
}

void check_oscillation_timing(const std::string& where, float t, float period,
                              float ramp, float span) {
  if (!(period > 0.0f) || !(ramp > 0.0f)) {
    throw std::invalid_argument(where + ": oscillate at t=" +
                                std::to_string(t) +
                                ": period and ramp must be > 0");
  }
  if (2.0f * ramp > span + 1e-6f) {
    throw std::invalid_argument(
        where + ": oscillate at t=" + std::to_string(t) + ": two ramps of " +
        std::to_string(ramp) + " s do not fit in its " + std::to_string(span) +
        " s segment");
  }
}

void check_leg_angles(const std::string& id, const std::string& leg, float t,
                      const JointAngles& a, const gait::kin::LegSpec& spec,
                      float ground_z) {
  static constexpr std::array<const char*, 3> kJointNames = {"coxa", "femur",
                                                              "tibia"};
  for (std::size_t j = 0; j < 3; ++j) {
    const auto& lim = ::hexa::config::kJointLimits[j];
    if (a[j] < lim.lower || a[j] > lim.upper) {
      throw std::invalid_argument(
          "gesture '" + id + "': " + leg + " " + kJointNames[j] + " at t=" +
          std::to_string(t) + " s is " + std::to_string(a[j]) +
          " rad, outside the joint limit window [" + std::to_string(lim.lower) +
          ", " + std::to_string(lim.upper) + "] rad");
    }
  }
  const float height = gait::kin::forward_kinematics(a, spec).z - ground_z;
  if (height < -kPlantedHeight) {
    throw std::invalid_argument("gesture '" + id + "': " + leg + " at t=" +
                                std::to_string(t) + " s is " +
                                std::to_string(-height) +
                                " m below the ground plane");
  }
}

// Samples per oscillation period. The joints swing together, so the box their
// amplitudes span is far wider than the path; the path itself is checked.
constexpr int kOscillationSamples = 64;

// A pose knot is checked as it is; an oscillate knot along its swing about
// `base`, the pose before it, over its `span` s segment.
void check_leg_keyframe(const std::string& id, const std::string& leg,
                        const LegKeyframe& k, const JointAngles& base,
                        float span, const gait::kin::LegSpec& spec,
                        float ground_z) {
  if (k.hold || k.home || k.start) {
    return;
  }
  if (!k.oscillate) {
    check_leg_angles(id, leg, k.t, {k.coxa, k.femur, k.tibia}, spec, ground_z);
    return;
  }
  const float dt = k.period / kOscillationSamples;
  for (float tau = 0.0f; tau < span; tau += dt) {
    JointAngles a = base;
    for (std::size_t j = 0; j < 3; ++j) {
      a[j] += oscillation(k, j, tau, span);
    }
    check_leg_angles(id, leg, k.t - span + tau, a, spec, ground_z);
  }
}

// Every ease between a live knot (the implicit start at t = 0, a start, a
// home, or a hold of one) and a fixed one must outlast the stand step.
void check_step_fits(const std::string& id, const std::string& leg,
                     const std::vector<LegKeyframe>& keys,
                     const StandStep& step) {
  bool prev_live = true;
  float prev_t = 0.0f;
  for (const LegKeyframe& k : keys) {
    const bool live = k.start || k.home || (k.hold && prev_live);
    if (live != prev_live && k.t - prev_t <= step.time) {
      throw std::invalid_argument(
          "gesture '" + id + "': " + leg + " eases between its stand and a "
          "pose in " + std::to_string(k.t - prev_t) + " s at t=" +
          std::to_string(k.t) + " s, not longer than the stand step's " +
          std::to_string(step.time) + " s");
    }
    prev_live = live;
    prev_t = k.t;
  }
}

}  // namespace

void check_track_shape(const std::vector<LegKeyframe>& keys,
                       const std::string& where) {
  bool live = true;
  bool seen_joints = false;
  float prev_t = 0.0f;
  for (const LegKeyframe& k : keys) {
    if (k.oscillate) {
      if (live) {
        throw std::invalid_argument(
            where + ": an oscillate at t=" + std::to_string(k.t) +
            " must follow a pose, not start, home or a hold of them");
      }
      check_oscillation_timing(where, k.t, k.period, k.ramp, k.t - prev_t);
    } else if (k.start) {
      if (seen_joints) {
        throw std::invalid_argument(
            where + ": start must come before the first pose");
      }
      live = true;
    } else if (k.home) {
      live = true;
    } else if (!k.hold) {
      if (live && k.transition == GestureTransition::CONTINUOUS) {
        throw std::invalid_argument(
            where + ": a continuous keyframe at t=" + std::to_string(k.t) +
            " must follow a pose, not start, home or a hold of them");
      }
      live = false;
      seen_joints = true;
    }
    prev_t = k.t;
  }
  if (keys.empty() || !keys.back().home) {
    throw std::invalid_argument(where +
                                ": the track must end with a home keyframe");
  }
}

void validate_gestures(
    const std::vector<GestureSpec>& specs,
    const std::map<std::string, gait::kin::LegSpec>& leg_specs,
    const std::map<std::string, Vec3>& nominal_stance,
    const posture::PoseLimits& limits) {
  for (const GestureSpec& spec : specs) {
    const StandStep& step = spec.stand_step;
    if (step.height < 0.0f || step.time < 0.0f || step.overlap < 0.0f ||
        step.overlap > step.time) {
      throw std::invalid_argument("gesture '" + spec.id +
                                  "': stand_step height, time and overlap "
                                  "must not be negative, nor overlap exceed "
                                  "time");
    }
    BodyKeyframe base{};
    for (const BodyKeyframe& k : spec.body) {
      check_body_keyframe(spec.id, k, limits);
      if (k.oscillate) {
        check_oscillation_timing("gesture '" + spec.id + "': body", k.t,
                                 k.period, k.ramp, k.t - base.t);
        check_body_oscillation(spec.id, k, base, limits);
      }
      const float t = k.t;
      if (k.home) {
        base = BodyKeyframe{};
      } else if (!k.hold && !k.oscillate) {
        base = k;
      }
      base.t = t;
    }
    if (!spec.body.empty() && !spec.body.back().home) {
      throw std::invalid_argument("gesture '" + spec.id +
                                  "': the body track must end with a home "
                                  "keyframe");
    }
    for (const LegTrack& track : spec.legs) {
      const std::string leg(leg_name(track.leg));
      check_track_shape(track.keys, "gesture '" + spec.id + "': " + leg);
      const gait::kin::LegSpec& ls = leg_specs.at(leg);
      const float ground_z = body_to_leg(nominal_stance.at(leg), ls).z;
      JointAngles base{};
      float prev_t = 0.0f;
      for (const LegKeyframe& k : track.keys) {
        check_leg_keyframe(spec.id, leg, k, base, k.t - prev_t, ls, ground_z);
        if (!k.hold && !k.home && !k.start && !k.oscillate) {
          base = {k.coxa, k.femur, k.tibia};
        }
        prev_t = k.t;
      }
      check_step_fits(spec.id, leg, track.keys, spec.stand_step);
    }
  }
}

}  // namespace hexa::gesture
