// Gesture player: one gesture's per-leg and body keyframe tables, completed
// with the implicit start and return knots and played off one clock. Built by
// the engine when a gesture starts and dropped when it is done. No clocks of
// its own: it accumulates the dt it is given.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "config_generated.hpp"
#include "gait/kinematics.hpp"
#include "gait/types.hpp"
#include "gesture/keyframe.hpp"
#include "kinematics/body_transform.hpp"
#include "leg_index.hpp"
#include "vec3.hpp"

namespace hexa::gesture {

// One leg's keyframes, in time order, as configured (preserve knots
// unresolved). The baked kGestureLegTracks rows and the YAML loader both land
// here.
struct LegTrack {
  Leg leg = Leg::L_FRONT;
  std::vector<LegKeyframe> keys;
};

struct GestureSpec {
  std::string id;
  // Each track eases back to nominal / identity over this after its last knot.
  float return_time = 0.0f;
  std::vector<LegTrack> legs;
  std::vector<BodyKeyframe> body;
};

// What the engine reports of a running gesture.
struct GestureProgress {
  std::string id;
  float t = 0.0f;  // s since the gesture started
  BodyPose body;   // the body track's term, nominal-relative
};

// Below this lift above the standing ground plane a tracked foot counts as
// planted.
inline constexpr float kPlantedHeight = 1e-4f;

class GesturePlayer {
 public:
  // start_feet is where the feet stand as the gesture begins (the implicit
  // start knot); nominal is where every track returns to. Both in the body
  // frame; each is solved to joint angles once, here. Legs without a track are
  // held at nominal.
  GesturePlayer(const GestureSpec& spec,
                const std::map<std::string, Vec3>& start_feet,
                const std::map<std::string, Vec3>& nominal,
                const std::map<std::string, gait::kin::LegSpec>& leg_specs);

  // Advance by dt and return all six legs. A tracked leg is `direct`: its
  // joint angles are the sampled track, its foot_target their FK, its phase
  // the gesture's progress and `stance` = planted. An untracked one is
  // nominal, phase 0, stance.
  std::map<std::string, gait::LegOutput> update(float dt);

  BodyPose body() const;
  bool done() const { return elapsed_ >= duration_; }
  float t() const { return elapsed_; }
  float duration() const { return duration_; }
  const std::string& id() const { return id_; }

 private:
  struct Track {
    std::string name;
    gait::kin::LegSpec spec;
    float ground_z = 0.0f;  // standing tip z in the leg frame
    std::vector<LegKeyframe> keys;  // complete: start, knots, return
  };

  // A track's sampled joint angles at the current time.
  static JointAngles sample(const Track& track, float t);

  std::string id_;
  std::vector<Track> tracks_;
  std::vector<BodyKeyframe> body_;  // complete, like a track's keys
  std::map<std::string, Vec3> nominal_;
  float duration_ = 0.0f;
  float elapsed_ = 0.0f;
};

// The baked kGestures table as GestureSpecs; PipelineConfig::baked() and the
// default engine read it. No filesystem access.
std::vector<GestureSpec> gesture_specs_from_config();

}  // namespace hexa::gesture
