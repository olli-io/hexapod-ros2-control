#include "gesture/player.hpp"

#include <algorithm>

namespace hexa::gesture {

namespace {

LegKeyframe leg_knot(float t, const LegPolar& p, GestureTransition transition) {
  LegKeyframe k{};
  k.t = t;
  k.angle = p.angle;
  k.reach = p.reach;
  k.height = p.height;
  k.transition = transition;
  k.preserve = false;
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
  k.preserve = false;
  return k;
}

// A preserve knot takes the previous knot's value; the implicit start is
// always knot 0, so there is always one.
void resolve_preserve(std::vector<LegKeyframe>& keys) {
  for (std::size_t i = 1; i < keys.size(); ++i) {
    if (!keys[i].preserve) {
      continue;
    }
    keys[i].angle = keys[i - 1].angle;
    keys[i].reach = keys[i - 1].reach;
    keys[i].height = keys[i - 1].height;
    keys[i].preserve = false;
  }
}

void resolve_preserve(std::vector<BodyKeyframe>& keys) {
  for (std::size_t i = 1; i < keys.size(); ++i) {
    if (!keys[i].preserve) {
      continue;
    }
    keys[i].x = keys[i - 1].x;
    keys[i].y = keys[i - 1].y;
    keys[i].z = keys[i - 1].z;
    keys[i].roll = keys[i - 1].roll;
    keys[i].pitch = keys[i - 1].pitch;
    keys[i].yaw = keys[i - 1].yaw;
    keys[i].preserve = false;
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
    const LegPolar start =
        polar_from_leg_frame(body_to_leg(start_feet.at(t.name), t.spec),
                             t.ground_z);
    const LegPolar home = polar_from_leg_frame(nominal_leg, t.ground_z);
    t.keys.reserve(track.keys.size() + 2);
    t.keys.push_back(leg_knot(0.0f, start, GestureTransition::EASE));
    t.keys.insert(t.keys.end(), track.keys.begin(), track.keys.end());
    t.keys.push_back(leg_knot(track.keys.back().t + spec.return_time, home,
                              GestureTransition::EASE));
    resolve_preserve(t.keys);
    duration_ = std::max(duration_, t.keys.back().t);
    tracks_.push_back(std::move(t));
  }

  if (!spec.body.empty()) {
    body_.reserve(spec.body.size() + 2);
    body_.push_back(body_knot(0.0f, BodyPose{}, GestureTransition::EASE));
    body_.insert(body_.end(), spec.body.begin(), spec.body.end());
    body_.push_back(body_knot(spec.body.back().t + spec.return_time,
                              BodyPose{}, GestureTransition::EASE));
    resolve_preserve(body_);
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
    const LegKeyframe* keys = track.keys.data();
    const std::size_t n = track.keys.size();
    LegPolar p;
    p.angle = track_value(keys, n, elapsed_,
                          [](const LegKeyframe& k) { return k.angle; });
    p.reach = track_value(keys, n, elapsed_,
                          [](const LegKeyframe& k) { return k.reach; });
    p.height = track_value(keys, n, elapsed_,
                           [](const LegKeyframe& k) { return k.height; });
    const Vec3 target =
        leg_to_body(leg_frame_from_polar(p, track.ground_z), track.spec);
    out[track.name] =
        gait::LegOutput{target, progress, p.height <= kPlantedHeight};
  }
  return out;
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
    spec.return_time = g.return_time;
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
