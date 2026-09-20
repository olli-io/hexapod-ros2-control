#include "gesture/player.hpp"

#include <algorithm>

#include "kinematics/leg_ik.hpp"

namespace hexa::gesture {

namespace {

LegKeyframe live_knot(float t) {
  LegKeyframe k{};
  k.t = t;
  k.transition = GestureTransition::EASE;
  return k;
}

BodyKeyframe body_knot(float t, const BodyPose& p,
                       GestureTransition transition) {
  BodyKeyframe k{};
  k.t = t;
  k.x = p.x;
  k.y = p.y;
  k.z = p.z;
  k.roll = p.roll;
  k.pitch = p.pitch;
  k.yaw = p.yaw;
  k.transition = transition;
  return k;
}

JointAngles angles_of(const LegKeyframe& k) { return {k.coxa, k.femur, k.tibia}; }

// Knot 0 is the implicit start, live. A start or home knot is live; a hold
// knot takes the previous knot's value and liveness. The flags are cleared
// once read, so the table is plain values plus the `live` mask.
std::vector<bool> resolve_stand_ins(std::vector<LegKeyframe>& keys) {
  std::vector<bool> live(keys.size(), false);
  live[0] = true;
  for (std::size_t i = 1; i < keys.size(); ++i) {
    if (keys[i].start || keys[i].home) {
      live[i] = true;
    } else if (keys[i].hold) {
      keys[i].coxa = keys[i - 1].coxa;
      keys[i].femur = keys[i - 1].femur;
      keys[i].tibia = keys[i - 1].tibia;
      live[i] = live[i - 1];
    }
    keys[i].hold = false;
    keys[i].home = false;
    keys[i].start = false;
  }
  return live;
}

// The body's home is the identity pose, which a zeroed knot already is.
void resolve_stand_ins(std::vector<BodyKeyframe>& keys) {
  for (std::size_t i = 1; i < keys.size(); ++i) {
    if (keys[i].hold) {
      keys[i].x = keys[i - 1].x;
      keys[i].y = keys[i - 1].y;
      keys[i].z = keys[i - 1].z;
      keys[i].roll = keys[i - 1].roll;
      keys[i].pitch = keys[i - 1].pitch;
      keys[i].yaw = keys[i - 1].yaw;
    }
    keys[i].hold = false;
    keys[i].home = false;
  }
}

}  // namespace

GesturePlayer::GesturePlayer(
    const GestureSpec& spec, const std::map<std::string, Vec3>& nominal,
    const std::map<std::string, gait::kin::LegSpec>& leg_specs)
    : id_(spec.id), nominal_(nominal) {
  gait::require_all_legs(nominal, "gesture nominal");
  gait::require_all_legs(leg_specs, "gesture leg_specs");

  for (const LegTrack& track : spec.legs) {
    if (track.keys.empty()) {
      continue;
    }
    Track t;
    t.name = std::string(leg_name(track.leg));
    t.spec = leg_specs.at(t.name);
    const Vec3 nominal_leg = body_to_leg(nominal.at(t.name), t.spec);
    t.ground_z = nominal_leg.z;
    // On the default preset, inside the reach annulus by construction.
    t.home = gait::kin::inverse_kinematics(nominal_leg, t.spec);
    t.keys.reserve(track.keys.size() + 1);
    t.keys.push_back(live_knot(0.0f));
    t.keys.insert(t.keys.end(), track.keys.begin(), track.keys.end());
    t.live = resolve_stand_ins(t.keys);
    duration_ = std::max(duration_, t.keys.back().t);
    tracks_.push_back(std::move(t));
  }

  if (!spec.body.empty()) {
    body_.reserve(spec.body.size() + 1);
    body_.push_back(body_knot(0.0f, BodyPose{}, GestureTransition::EASE));
    body_.insert(body_.end(), spec.body.begin(), spec.body.end());
    resolve_stand_ins(body_);
    duration_ = std::max(duration_, body_.back().t);
  }
}

std::map<std::string, gait::LegOutput> GesturePlayer::update(float dt) {
  elapsed_ += dt;
  const float progress =
      duration_ > 0.0f ? std::min(elapsed_ / duration_, 1.0f) : 1.0f;

  std::map<std::string, gait::LegOutput> out;
  for (const auto& n : gait::LEG_NAMES) {
    out[n] = gait::LegOutput{nominal_.at(n), 0.0f, true};
  }
  for (const Track& track : tracks_) {
    gait::LegOutput& leg = out[track.name];
    leg.phase = progress;
    const Sample s = sample(track, elapsed_);
    if (s.weight <= 0.0f) {
      continue;  // live: the stance under the body, like an untracked leg
    }
    leg.direct = true;
    leg.direct_weight = s.weight;
    leg.joints = s.joints;
    // Fully direct, the foot is the joints' FK. Blending, the live end is
    // unknown here: foot_target is the stance the pipeline solves it from,
    // and the planted flag reads the blend against the unposed stance.
    JointAngles probe = s.joints;
    if (s.weight < 1.0f) {
      for (std::size_t j = 0; j < 3; ++j) {
        probe[j] = track.home[j] + (s.joints[j] - track.home[j]) * s.weight;
      }
    }
    const Vec3 in_leg = gait::kin::forward_kinematics(probe, track.spec);
    if (s.weight >= 1.0f) {
      leg.foot_target = leg_to_body(in_leg, track.spec);
    }
    leg.stance = in_leg.z - track.ground_z <= kPlantedHeight;
  }
  return out;
}

GesturePlayer::Sample GesturePlayer::sample(const Track& track, float t) {
  const LegKeyframe* keys = track.keys.data();
  const std::size_t n = track.keys.size();
  Sample s;
  if (t <= keys[0].t) {
    return s;
  }
  if (t >= keys[n - 1].t) {
    if (!track.live[n - 1]) {
      s.weight = 1.0f;
      s.joints = angles_of(keys[n - 1]);
    }
    return s;
  }
  std::size_t i = 1;
  while (i + 1 < n && t > keys[i].t) {
    ++i;
  }
  const bool a_live = track.live[i - 1];
  const bool b_live = track.live[i];
  if (a_live && b_live) {
    return s;
  }
  if (!a_live && !b_live) {
    s.weight = 1.0f;
    s.joints = {
        track_value(keys, n, t, [](const LegKeyframe& k) { return k.coxa; }),
        track_value(keys, n, t, [](const LegKeyframe& k) { return k.femur; }),
        track_value(keys, n, t, [](const LegKeyframe& k) { return k.tibia; })};
    return s;
  }
  // One live end: the ease between the two, weighted onto the fixed end.
  const float h = keys[i].t - keys[i - 1].t;
  const float u = h > 0.0f ? (t - keys[i - 1].t) / h : 1.0f;
  const float e = ease5(u);
  if (a_live) {
    s.weight = e;
    s.joints = angles_of(keys[i]);
  } else {
    s.weight = 1.0f - e;
    s.joints = angles_of(keys[i - 1]);
  }
  return s;
}

BodyPose GesturePlayer::body() const {
  if (body_.empty()) {
    return BodyPose{};
  }
  const BodyKeyframe* keys = body_.data();
  const std::size_t n = body_.size();
  BodyPose p;
  p.x = track_value(keys, n, elapsed_, [](const BodyKeyframe& k) { return k.x; });
  p.y = track_value(keys, n, elapsed_, [](const BodyKeyframe& k) { return k.y; });
  p.z = track_value(keys, n, elapsed_, [](const BodyKeyframe& k) { return k.z; });
  p.roll = track_value(keys, n, elapsed_,
                       [](const BodyKeyframe& k) { return k.roll; });
  p.pitch = track_value(keys, n, elapsed_,
                        [](const BodyKeyframe& k) { return k.pitch; });
  p.yaw = track_value(keys, n, elapsed_,
                      [](const BodyKeyframe& k) { return k.yaw; });
  return p;
}

std::vector<GestureSpec> gesture_specs_from_config() {
  namespace cfg = ::hexa::config;
  std::vector<GestureSpec> out;
  out.reserve(cfg::kGestures.size());
  for (const auto& g : cfg::kGestures) {
    GestureSpec spec;
    spec.id = std::string(g.id);
    for (std::size_t ti = 0; ti < g.leg_track_count; ++ti) {
      const auto& row = cfg::kGestureLegTracks[g.first_leg_track + ti];
      LegTrack track;
      track.leg = row.leg;
      track.keys.assign(cfg::kGestureLegKeyframes.begin() + row.first,
                        cfg::kGestureLegKeyframes.begin() + row.first +
                            row.count);
      spec.legs.push_back(std::move(track));
    }
    spec.body.assign(
        cfg::kGestureBodyKeyframes.begin() + g.first_body_key,
        cfg::kGestureBodyKeyframes.begin() + g.first_body_key + g.body_key_count);
    out.push_back(std::move(spec));
  }
  return out;
}

}  // namespace hexa::gesture
