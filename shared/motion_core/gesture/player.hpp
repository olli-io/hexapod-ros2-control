// Gesture player: one gesture's per-leg and body keyframe tables, completed
// with the implicit start knot and played off one clock. Built by the engine
// when a gesture starts and dropped when it is done. No clocks of its own: it
// accumulates the dt it is given.
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

// One leg's keyframes, in time order, as configured (hold, home and start
// knots unresolved). The baked kGestureLegTracks rows and the YAML loader both
// land here.
struct LegTrack {
  Leg leg = Leg::L_FRONT;
  std::vector<LegKeyframe> keys;
};

// Every track ends on a home knot (validate_gestures enforces it), so the
// gesture hands a clean stand back.
struct GestureSpec {
  std::string id;
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

// A leg track's knots are fixed (joint angles) or live. A live knot — the
// implicit start, a `start`, a `home`, and any hold after one — is the
// standing leg under whatever body pose is on at that moment. The player does
// not know that pose, so on a segment with one live end it reports the fixed
// end's angles and the ease weight, and the pipeline blends them with the IK
// of the stance under the live pose. Between two live knots the leg is not
// direct at all: it stands on nominal and follows the body like an untracked
// one.
class GesturePlayer {
 public:
  // nominal is the stance a live knot stands on, in the body frame; each leg
  // is solved to joint angles once, here. Legs without a track are held at
  // nominal.
  GesturePlayer(const GestureSpec& spec,
                const std::map<std::string, Vec3>& nominal,
                const std::map<std::string, gait::kin::LegSpec>& leg_specs);

  // Advance by dt and return all six legs. A tracked leg between fixed knots
  // is `direct` at weight 1: its joint angles are the sampled track, its
  // foot_target their FK and `stance` = planted. Easing out of or into a live
  // knot it is `direct` at the ease weight with the fixed knot's angles and
  // foot_target = nominal, the stance the live part stands on. On a live
  // stretch it is nominal, stance. Every tracked leg's phase is the gesture's
  // progress; an untracked one is nominal, phase 0, stance.
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
    JointAngles home{};     // the stance solved with no body pose on
    std::vector<LegKeyframe> keys;  // complete: start knot, then the table
    std::vector<bool> live;         // per knot
  };

  // A track at time t: the direct weight (0 = live, 1 = fully direct) and the
  // angles that weight applies to.
  struct Sample {
    float weight = 0.0f;
    JointAngles joints{};
  };
  static Sample sample(const Track& track, float t);

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
