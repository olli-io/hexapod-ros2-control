#include "gesture/player.hpp"

#include <algorithm>

#include "kinematics/leg_ik.hpp"

namespace hexa::gesture {

namespace {

LegKeyframe leg_knot(float t, const JointAngles& a,
                     GestureTransition transition) {
  LegKeyframe k{};
  k.t = t;
  k.coxa = a[0];
  k.femur = a[1];
  k.tibia = a[2];
  k.transition = transition;
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

// A hold knot takes the previous knot's value (the implicit start is always
// knot 0, so there is one); a home knot takes the stance.
void resolve_stand_ins(std::vector<LegKeyframe>& keys, const JointAngles& home) {
  for (std::size_t i = 1; i < keys.size(); ++i) {
    if (keys[i].hold) {
      keys[i].coxa = keys[i - 1].coxa;
      keys[i].femur = keys[i - 1].femur;
      keys[i].tibia = keys[i - 1].tibia;
    } else if (keys[i].home) {
      keys[i].coxa = home[0];
      keys[i].femur = home[1];
      keys[i].tibia = home[2];
    }
    keys[i].hold = false;
    keys[i].home = false;
  }
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
    const GestureSpec& spec, const std::map<std::string, Vec3>& start_feet,
    const std::map<std::string, Vec3>& nominal,
    const std::map<std::string, gait::kin::LegSpec>& leg_specs)
    : id_(spec.id), nominal_(nominal) {
  gait::require_all_legs(start_feet, "gesture start_feet");
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
    // Both stand on the default preset, inside the reach annulus by
    // construction; an exact FK round trip.
    const JointAngles start = gait::kin::inverse_kinematics(
        body_to_leg(start_feet.at(t.name), t.spec), t.spec);
    const JointAngles home = gait::kin::inverse_kinematics(nominal_leg, t.spec);
    t.keys.reserve(track.keys.size() + 1);
    t.keys.push_back(leg_knot(0.0f, start, GestureTransition::EASE));
    t.keys.insert(t.keys.end(), track.keys.begin(), track.keys.end());
    resolve_stand_ins(t.keys, home);
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
    const JointAngles a = sample(track, elapsed_);
    const Vec3 in_leg = gait::kin::forward_kinematics(a, track.spec);
    gait::LegOutput& leg = out[track.name];
    leg.foot_target = leg_to_body(in_leg, track.spec);
    leg.phase = progress;
    leg.stance = in_leg.z - track.ground_z <= kPlantedHeight;
    leg.direct = true;
    leg.joints = a;
  }
  return out;
}

JointAngles GesturePlayer::sample(const Track& track, float t) {
  const LegKeyframe* keys = track.keys.data();
  const std::size_t n = track.keys.size();
  return {track_value(keys, n, t, [](const LegKeyframe& k) { return k.coxa; }),
          track_value(keys, n, t, [](const LegKeyframe& k) { return k.femur; }),
          track_value(keys, n, t, [](const LegKeyframe& k) { return k.tibia; })};
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
