#include "pipeline_config_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "gait/engine.hpp"
#include "gait/gaits/registry.hpp"
#include "gait/limits.hpp"
#include "gait/types.hpp"
#include "gesture/player.hpp"
#include "gesture/validate.hpp"
#include "leg_index.hpp"
#include "vec3.hpp"

namespace hexa::locomotion {

namespace {

float f(const YAML::Node& n) { return n.as<float>(); }

YAML::Node params(const YAML::Node& root, const char* node_name) {
  return root[node_name]["ros__parameters"];
}

// Mirrors gen_config.py to_urdf_rad (coxa +, femur -, tibia pi-).
double to_urdf_rad(const std::string& joint_type, double deg) {
  const double rad = deg * M_PI / 180.0;
  if (joint_type == "coxa") return rad;
  if (joint_type == "femur") return -rad;
  if (joint_type == "tibia") return M_PI - rad;
  throw std::runtime_error("unknown joint type: " + joint_type);
}

// Mirrors gen_config.py joint_limits(): geometry.yaml joints.<type> windows in
// URDF rad, ordered coxa, femur, tibia like a JointAngles triple.
std::array<hexa::config::JointLimits, 3> load_joint_limits(const YAML::Node& geo) {
  static constexpr std::array<const char*, 3> kJointTypes = {"coxa", "femur",
                                                              "tibia"};
  std::array<hexa::config::JointLimits, 3> out{};
  for (std::size_t j = 0; j < 3; ++j) {
    const YAML::Node cfg = geo["joints"][kJointTypes[j]];
    const double a = to_urdf_rad(kJointTypes[j], cfg["lower_limit_deg"].as<double>());
    const double b = to_urdf_rad(kJointTypes[j], cfg["upper_limit_deg"].as<double>());
    out[j].lower = static_cast<float>(std::min(a, b));
    out[j].upper = static_cast<float>(std::max(a, b));
    out[j].effort = f(cfg["effort"]);
    out[j].velocity = f(cfg["velocity"]);
  }
  return out;
}

// ── gestures.yaml ──
// Mirrors gen_config.py gestures(): the same checks, the same flattening of the
// multi-leg authoring format into per-leg tables, so the parity test can compare
// the two position by position.

// t and transition, shared by both tracks; t strictly increasing and > 0.
float keyframe_t(const YAML::Node& kf, const std::string& where,
                 float prev_t, bool have_prev) {
  if (!kf.IsMap()) {
    throw std::runtime_error(where + ": a keyframe must be a mapping");
  }
  if (!kf["t"]) {
    throw std::runtime_error(where + ": missing t");
  }
  const float t = f(kf["t"]);
  if (!(t > 0.0f)) {
    throw std::runtime_error(where + ": t must be > 0 (the start is implicit)");
  }
  if (have_prev && !(t > prev_t)) {
    throw std::runtime_error(where + ": t must increase down the list");
  }
  return t;
}

// The stand-in words a leg keyframe may carry, and the two a body one may.
const std::vector<std::string> kLegStandIns = {"hold", "home", "start"};
const std::vector<std::string> kBodyStandIns = {"hold", "home"};

// The stand-in word set on the keyframe for every track, else "".
std::string stand_in_word(const YAML::Node& kf, const std::string& where,
                          const std::vector<std::string>& words) {
  std::string found;
  for (const auto& w : words) {
    if (kf[w] && kf[w].as<bool>()) {
      if (!found.empty()) {
        throw std::runtime_error(where + ": " + found + " and " + w +
                                 " exclude each other");
      }
      found = w;
    }
  }
  return found;
}

// A stand-in keyframe implies its transition: a hold is static, a start is
// where the track begins and a home always eases in. Any other keyframe must
// say.
hexa::config::GestureTransition transition_from(
    const YAML::Node& kf, const std::string& where,
    const std::vector<std::string>& words) {
  const std::string stand_in = stand_in_word(kf, where, words);
  if (!stand_in.empty()) {
    if (kf["transition"]) {
      throw std::runtime_error(where + ": " + stand_in + ": true takes no transition");
    }
    return hexa::config::GestureTransition::EASE;
  }
  const std::string name = kf["transition"] ? kf["transition"].as<std::string>() : "";
  if (name == "ease") return hexa::config::GestureTransition::EASE;
  if (name == "continuous") return hexa::config::GestureTransition::CONTINUOUS;
  throw std::runtime_error(where + ": transition must be ease or continuous");
}

bool is_leg_stand_in_word(const YAML::Node& v) {
  if (!v.IsScalar()) return false;
  const std::string s = v.as<std::string>();
  return std::find(kLegStandIns.begin(), kLegStandIns.end(), s) !=
         kLegStandIns.end();
}

void reject_unknown_keys(const YAML::Node& map, const std::set<std::string>& known,
                         const std::string& where) {
  for (const auto& kv : map) {
    const auto key = kv.first.as<std::string>();
    if (!known.count(key)) {
      throw std::runtime_error(where + ": unknown key " + key);
    }
  }
}

// A leg keyframe's (coxa, femur, tibia) in URDF rad, inside the limits. Each
// joint interpolates within the range its neighbouring keyframes span, so a
// per-keyframe check covers the whole path.
hexa::JointAngles leg_entry_angles(
    const YAML::Node& v, const std::array<hexa::config::JointLimits, 3>& limits,
    const std::string& where) {
  static constexpr std::array<const char*, 3> kJointTypes = {"coxa", "femur",
                                                              "tibia"};
  hexa::JointAngles out{};
  for (std::size_t j = 0; j < 3; ++j) {
    const std::string key = std::string(kJointTypes[j]) + "_deg";
    const double deg = v[key].as<double>();
    const float rad = static_cast<float>(to_urdf_rad(kJointTypes[j], deg));
    if (rad < limits[j].lower || rad > limits[j].upper) {
      throw std::runtime_error(where + ": " + key + " = " + std::to_string(deg) +
                               " is outside the joint limits in geometry.yaml");
    }
    out[j] = rad;
  }
  return out;
}

std::vector<hexa::gesture::GestureSpec> load_gestures(
    const std::string& path,
    const std::array<hexa::config::JointLimits, 3>& limits) {
  static const std::set<std::string> kLegKeyframeKeys = {
      "t", "transition", "hold", "home", "start", "l_front", "l_middle",
      "l_rear", "r_front", "r_middle", "r_rear"};
  static const std::set<std::string> kLegEntryKeys = {"coxa_deg", "femur_deg",
                                                      "tibia_deg"};
  static const std::set<std::string> kBodyKeyframeKeys = {
      "t", "transition", "hold", "home", "x", "y", "z",
      "roll_deg", "pitch_deg", "yaw_deg"};
  static const std::set<std::string> kGestureKeys = {"id", "legs", "body"};
  const YAML::Node doc = YAML::LoadFile(path);
  const YAML::Node list = doc["gestures"];
  std::vector<hexa::gesture::GestureSpec> out;
  if (!list || list.IsNull()) {
    return out;
  }
  std::set<std::string> seen;
  for (const auto& entry : list) {
    hexa::gesture::GestureSpec spec;
    spec.id = entry["id"].as<std::string>();
    const std::string gwhere = "gestures.yaml " + spec.id;
    if (!seen.insert(spec.id).second) {
      throw std::runtime_error("gestures.yaml: duplicate id " + spec.id);
    }
    if (entry["return_time"]) {
      throw std::runtime_error(
          gwhere + ": return_time is gone; end every track with a home keyframe "
                   "instead");
    }
    reject_unknown_keys(entry, kGestureKeys, gwhere);

    const YAML::Node keyframes = entry["legs"];
    // Pass 1: validate, and find every leg the gesture moves.
    std::vector<std::string> moved;
    float prev_t = 0.0f;
    bool have_prev = false;
    for (std::size_t i = 0; keyframes && i < keyframes.size(); ++i) {
      const YAML::Node kf = keyframes[i];
      const std::string where = gwhere + ".legs[" + std::to_string(i) + "]";
      prev_t = keyframe_t(kf, where, prev_t, have_prev);
      have_prev = true;
      transition_from(kf, where, kLegStandIns);
      reject_unknown_keys(kf, kLegKeyframeKeys, where);
      std::vector<std::string> legs_here;
      for (const auto& leg : hexa::gait::LEG_NAMES) {
        if (kf[leg]) legs_here.push_back(leg);
      }
      const std::string stand_in = stand_in_word(kf, where, kLegStandIns);
      if (!stand_in.empty()) {
        if (!legs_here.empty()) {
          throw std::runtime_error(where + ": " + stand_in +
                                   ": true takes no leg entries");
        }
        continue;
      }
      if (legs_here.empty()) {
        throw std::runtime_error(
            where +
            ": names no leg (use hold: true, home: true or start: true for all)");
      }
      for (const auto& leg : legs_here) {
        const YAML::Node v = kf[leg];
        if (is_leg_stand_in_word(v)) {
          // a stand-in
        } else if (v.IsMap()) {
          reject_unknown_keys(v, kLegEntryKeys, where + "." + leg);
          if (!v["coxa_deg"] || !v["femur_deg"] || !v["tibia_deg"]) {
            throw std::runtime_error(
                where + "." + leg + ": needs coxa_deg, femur_deg and tibia_deg");
          }
          leg_entry_angles(v, limits, where + "." + leg);
        } else {
          throw std::runtime_error(where + "." + leg +
                                   ": a mapping, hold, home or start");
        }
        if (std::find(moved.begin(), moved.end(), leg) == moved.end()) {
          moved.push_back(leg);
        }
      }
    }
    // Pass 2: per-leg tables, legs in Leg order.
    for (std::size_t li = 0; li < hexa::kNumLegs; ++li) {
      const std::string& leg = hexa::gait::LEG_NAMES[li];
      if (std::find(moved.begin(), moved.end(), leg) == moved.end()) {
        continue;
      }
      hexa::gesture::LegTrack track;
      track.leg = static_cast<hexa::Leg>(li);
      for (std::size_t i = 0; i < keyframes.size(); ++i) {
        const YAML::Node kf = keyframes[i];
        std::string stand_in = stand_in_word(kf, gwhere, kLegStandIns);
        if (stand_in.empty() && !kf[leg]) {
          continue;
        }
        const YAML::Node v = kf[leg];
        if (stand_in.empty() && is_leg_stand_in_word(v)) {
          stand_in = v.as<std::string>();
        }
        hexa::config::LegKeyframe row{};
        row.t = f(kf["t"]);
        row.transition = transition_from(kf, gwhere, kLegStandIns);
        row.hold = stand_in == "hold";
        row.home = stand_in == "home";
        row.start = stand_in == "start";
        if (stand_in.empty()) {
          const hexa::JointAngles a =
              leg_entry_angles(v, limits, gwhere + "." + leg);
          row.coxa = a[0];
          row.femur = a[1];
          row.tibia = a[2];
        }
        track.keys.push_back(row);
      }
      hexa::gesture::check_track_shape(track.keys, gwhere + "." + leg);
      spec.legs.push_back(std::move(track));
    }

    const YAML::Node body = entry["body"];
    prev_t = 0.0f;
    have_prev = false;
    for (std::size_t i = 0; body && i < body.size(); ++i) {
      const YAML::Node kf = body[i];
      const std::string where = gwhere + ".body[" + std::to_string(i) + "]";
      hexa::config::BodyKeyframe row{};
      row.t = keyframe_t(kf, where, prev_t, have_prev);
      prev_t = row.t;
      have_prev = true;
      reject_unknown_keys(kf, kBodyKeyframeKeys, where);
      row.transition = transition_from(kf, where, kBodyStandIns);
      const std::string stand_in = stand_in_word(kf, where, kBodyStandIns);
      row.hold = stand_in == "hold";
      row.home = stand_in == "home";
      const bool any_axis = kf["x"] || kf["y"] || kf["z"] || kf["roll_deg"] ||
                            kf["pitch_deg"] || kf["yaw_deg"];
      if (!stand_in.empty() && any_axis) {
        throw std::runtime_error(where + ": " + stand_in +
                                 ": true takes no axis values");
      }
      if (stand_in.empty()) {
        const auto deg = [&](const char* key) {
          return kf[key] ? static_cast<float>(kf[key].as<double>() * M_PI / 180.0)
                         : 0.0f;
        };
        row.x = kf["x"] ? f(kf["x"]) : 0.0f;
        row.y = kf["y"] ? f(kf["y"]) : 0.0f;
        row.z = kf["z"] ? f(kf["z"]) : 0.0f;
        row.roll = deg("roll_deg");
        row.pitch = deg("pitch_deg");
        row.yaw = deg("yaw_deg");
      }
      spec.body.push_back(row);
    }

    if (!spec.body.empty() && !spec.body.back().home) {
      throw std::runtime_error(gwhere +
                               ".body: the track must end with a home keyframe");
    }
    if (spec.legs.empty() && spec.body.empty()) {
      throw std::runtime_error(gwhere + ": has no keyframes");
    }
    out.push_back(std::move(spec));
  }
  return out;
}

}  // namespace

hexa::pipeline::PipelineConfig load_pipeline_config_from_yaml(
    const std::string& geometry_path, const std::string& tuning_path,
    const std::string& gestures_path) {
  const YAML::Node geo = YAML::LoadFile(geometry_path);
  const YAML::Node tun = YAML::LoadFile(tuning_path);
  const YAML::Node g = params(tun, "gait_node");
  const YAML::Node c = params(tun, "control_node");
  const YAML::Node p = params(tun, "posture_node");

  hexa::pipeline::PipelineConfig cfg;

  // Six LegSpecs by symmetry: rear x -> -x, yaw -> pi - yaw; right y -> -y, yaw -> -yaw.
  const YAML::Node leg = geo["leg"];
  const float coxa_len = f(leg["coxa_length"]);
  const float femur_len = f(leg["femur_length"]);
  const float tibia_len = f(leg["tibia_length"]);
  const YAML::Node mounts = geo["mounts"];
  const YAML::Node front = mounts["l_front"];
  const YAML::Node middle = mounts["l_middle"];

  std::map<std::string, hexa::config::LegSpec> specs;
  for (const std::string side : {"l", "r"}) {
    for (const std::string name : {"front", "middle", "rear"}) {
      const YAML::Node& ref = (name == "middle") ? middle : front;
      const double ref_yaw = ref["yaw_deg"].as<double>() * M_PI / 180.0;
      const double ref_x = ref["x"].as<double>();
      const double ref_y = ref["y"].as<double>();

      const double x_fr = (name == "rear") ? -ref_x : ref_x;
      const double yaw_fr = (name == "rear") ? (M_PI - ref_yaw) : ref_yaw;
      const double mx = x_fr;
      const double my = (side == "r") ? -ref_y : ref_y;
      const double myaw = (side == "r") ? -yaw_fr : yaw_fr;

      hexa::config::LegSpec spec;
      spec.mount_xyz = hexa::Vec3(static_cast<float>(mx),
                                  static_cast<float>(my), 0.0f);
      spec.mount_yaw = static_cast<float>(myaw);
      spec.coxa_len = coxa_len;
      spec.femur_len = femur_len;
      spec.tibia_len = tibia_len;
      specs[side + "_" + name] = spec;
    }
  }

  // Splay is the left leg's, positive outward; standing_pose_from owns the
  // rear/right negation.
  const auto leg_group_stance = [&](const YAML::Node& grp) {
    return hexa::config::LegGroupStance{
        f(grp["tip_reach"]),
        static_cast<float>(to_urdf_rad("coxa", grp["coxa_deg"].as<double>()))};
  };
  {
    // Declaration order is load-bearing: the baked kPresets table is indexed.
    const YAML::Node list = g["presets"];
    if (!list || !list.IsSequence() || list.size() == 0) {
      throw std::runtime_error(
          "tuning.yaml gait_node.presets: must be a non-empty list");
    }
    const auto default_id = g["default_preset"].as<std::string>();
    for (const auto& entry : list) {
      hexa::gait::PresetSpec spec;
      spec.id = entry["id"].as<std::string>();
      const auto leg_set = entry["leg_set"].as<std::string>();
      if (leg_set == "quadruped") {
        spec.leg_set = hexa::gait::LegSet::QUADRUPED;
      } else if (leg_set == "hexapod") {
        spec.leg_set = hexa::gait::LegSet::HEXAPOD;
      } else {
        throw std::runtime_error("tuning.yaml presets." + spec.id +
                                 ".leg_set must be 'hexapod' or 'quadruped'");
      }
      const YAML::Node sp = entry["standing_pose"];
      spec.standing.body_height = f(sp["body_height"]);
      for (std::size_t gi = 0; gi < hexa::kNumLegGroups; ++gi) {
        const std::string name(hexa::LEG_GROUP_NAMES[gi]);
        // A parked middle pair has no entry; fill from front as gen_config.py
        // does. solve_preset replaces the row outright.
        spec.standing.groups[gi] =
            leg_group_stance(sp[name] ? sp[name] : sp["front"]);
      }
      // No fallback: an omitted key is a load error, not a silent inheritance.
      spec.stride_length = f(entry["stride_length"]);
      spec.stride_length_radial = f(entry["stride_length_radial"]);
      spec.min_swing_time = f(entry["min_swing_time"]);
      spec.max_swing_time = f(entry["max_swing_time"]);
      spec.step_height = f(entry["step_height"]);
      if (spec.id == default_id) {
        cfg.default_preset = cfg.presets.size();
      }
      cfg.presets.push_back(std::move(spec));
    }
    if (cfg.presets[cfg.default_preset].id != default_id) {
      throw std::runtime_error("tuning.yaml default_preset: no preset '" +
                               default_id + "'");
    }
  }

  // femur/tibia uniform; coxa by symmetry in degrees (rear negates, then right
  // negates) before the deg->rad conversion.
  const auto rest_pose = [&](const char* key) {
    const YAML::Node p = geo[key];
    const float femur = static_cast<float>(
        to_urdf_rad("femur", p["femur"]["above_horizontal_deg"].as<double>()));
    const float tibia = static_cast<float>(
        to_urdf_rad("tibia", p["tibia"]["interior_deg"].as<double>()));
    const YAML::Node coxa_cfg = p["coxa"];
    std::map<std::string, hexa::JointAngles> out;
    for (const std::string side : {"l", "r"}) {
      for (const std::string name : {"front", "middle", "rear"}) {
        const double ref_deg =
            coxa_cfg[(name == "middle") ? "l_middle_deg" : "l_front_deg"]
                .as<double>();
        const double after_fr = (name == "rear") ? -ref_deg : ref_deg;
        const double after_lr = (side == "r") ? -after_fr : after_fr;
        const float coxa = static_cast<float>(to_urdf_rad("coxa", after_lr));
        out[side + "_" + name] = {coxa, femur, tibia};
      }
    }
    return out;
  };
  const auto folded = rest_pose("folded_pose");
  const auto initialized = rest_pose("initialized_pose");

  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const std::string& nm = hexa::gait::LEG_NAMES[i];
    cfg.leg_specs[i] = specs.at(nm);
    cfg.folded_pose[i] = folded.at(nm);
    cfg.initialized_pose[i] = initialized.at(nm);
  }
  cfg.coxa_to_bottom = f(geo["body"]["coxa_to_bottom"]);
  cfg.foot_radius = f(geo["foot"]["radius"]);

  auto& e = cfg.engine;
  // Preset-owned knobs, seeded from the default; the engine rewrites them on
  // every preset change.
  {
    const auto& d = cfg.presets[cfg.default_preset];
    e.stride_length = d.stride_length;
    e.stride_length_radial = d.stride_length_radial;
    e.min_swing_time = d.min_swing_time;
    e.max_swing_time = d.max_swing_time;
    e.step_height = d.step_height;
  }
  e.swing_width = f(g["swing_width"]);
  e.touchdown_velocity = f(g["touchdown_velocity"]);
  e.touchdown_probe_fraction = f(g["touchdown_probe_fraction"]);
  e.swing_phase_margin = f(g["swing_phase_margin"]);
  e.quadruped_swing_phase_margin = f(g["quadruped_swing_phase_margin"]);
  e.controller_dt = f(g["controller_dt"]);
  e.cmd_zero_tol = f(g["cmd_zero_tol"]);
  e.settle_debounce_delay = f(g["settle"]["debounce_delay"]);
  e.settle_swing_time = f(g["settle"]["swing_time"]);
  e.init_unfold_time = f(g["initialize"]["unfold_time"]);
  e.init_pair_swing_time = f(g["initialize"]["pair_swing_time"]);
  e.init_lift_body_time = f(g["initialize"]["lift_body_time"]);
  e.init_place_clearance = f(g["initialize"]["place_clearance"]);
  e.init_swing_clearance = f(g["initialize"]["swing_clearance"]);
  e.reseat_pose_settle_delay = f(g["reseat"]["pose_settle_delay"]);
  e.reseat_height_change_threshold = f(g["reseat"]["height_change_threshold"]);
  e.reseat_pair_swing_time = f(g["reseat"]["pair_swing_time"]);
  e.reseat_pair_dwell_time = f(g["reseat"]["pair_dwell_time"]);
  e.reseat_swing_clearance = f(g["reseat"]["swing_clearance"]);
  e.reseat_plane_ramp_time = f(g["reseat"]["plane_ramp_time"]);
  e.quadruped_shift_time = f(g["quadruped"]["shift_time"]);
  e.pair_fold_swing_time = f(g["pair_fold"]["swing_time"]);
  e.pair_fold_dwell_time = f(g["pair_fold"]["dwell_time"]);
  e.support_shift_lead = f(p["support_shift_lead"]);

  // Velocity caps, per preset and per registered gait. There is no angular knob
  // in YAML: the cap is the linear one over the outermost standing foot's radius.
  const float yaw_bias = f(g["yaw_bias"]);
  const auto setups =
      hexa::gait::solve_presets(cfg.presets, cfg.leg_specs, cfg.coxa_to_bottom,
                                cfg.foot_radius, cfg.default_preset);
  cfg.caps_by_preset.clear();
  for (const auto& setup : setups) {
    const float r_outer = hexa::gait::outer_stance_radius(setup.nominal_stance);
    hexa::gait::VelocityCaps caps;
    for (const auto& [gait_name, factory] : hexa::gait::strategies()) {
      const auto strategy = factory();
      const float duty = strategy->duty_factor();
      // Keys off the realized swing/stance split of the gait's own leg set, not
      // the nominal duty factor. Must match gen_config.py velocity_caps().
      const float swing_end = hexa::gait::swing_end_phase(
          duty, hexa::gait::swing_phase_margin_for(
                    strategy->leg_set(), e.swing_phase_margin,
                    e.quadruped_swing_phase_margin));
      const float linear_max = setup.stride_length * swing_end /
                               (setup.min_swing_time * (1.0f - swing_end));
      caps.linear_max_by_gait[gait_name] = linear_max;
      caps.angular_max_by_gait[gait_name] = linear_max / r_outer;
      // yaw_bias is a feel knob keyed to nominal duty; no preset moves it.
      caps.yaw_bias_by_gait[gait_name] =
          0.5f + (yaw_bias - 0.5f) * (1.5f - duty);
    }
    cfg.caps_by_preset[setup.id] = std::move(caps);
  }
  cfg.default_gait = g["default_gait"].as<std::string>();

  cfg.control.vmax_ramp_time_linear = f(c["vmax_ramp_time_linear"]);
  cfg.control.vmax_ramp_time_angular = f(c["vmax_ramp_time_angular"]);
  cfg.control.snap_tol_linear = f(c["snap_tol_linear"]);
  cfg.control.snap_tol_angular = f(c["snap_tol_angular"]);

  auto& ps = cfg.posture;
  ps.gait_sway_gain = f(p["gait_sway_gain"]);
  ps.gait_sway_strength = f(p["gait_sway_strength"]);
  ps.vertical_body_roll_z_amplitude = f(p["vertical_body_roll_z_amplitude"]);
  ps.vertical_body_roll_pitch_amplitude_deg =
      f(p["vertical_body_roll_pitch_amplitude_deg"]);
  ps.vertical_body_roll_phase_offset = f(p["vertical_body_roll_phase_offset"]);
  ps.horizontal_body_roll_y_amplitude =
      f(p["horizontal_body_roll_y_amplitude"]);
  ps.horizontal_body_roll_yaw_amplitude_deg =
      f(p["horizontal_body_roll_yaw_amplitude_deg"]);
  ps.horizontal_body_roll_phase_offset =
      f(p["horizontal_body_roll_phase_offset"]);
  ps.body_roll_3d_z_amplitude = f(p["body_roll_3d_z_amplitude"]);
  ps.body_roll_3d_pitch_amplitude_deg = f(p["body_roll_3d_pitch_amplitude_deg"]);
  ps.body_roll_3d_y_amplitude = f(p["body_roll_3d_y_amplitude"]);
  ps.body_roll_3d_yaw_amplitude_deg = f(p["body_roll_3d_yaw_amplitude_deg"]);
  ps.body_roll_3d_horizontal_phase_offset =
      f(p["body_roll_3d_horizontal_phase_offset"]);
  ps.body_roll_3d_pitch_phase_offset = f(p["body_roll_3d_pitch_phase_offset"]);
  ps.body_roll_3d_yaw_phase_offset = f(p["body_roll_3d_yaw_phase_offset"]);
  ps.gait_bounce_arc_height = f(p["gait_bounce_arc_height"]);
  ps.gait_bounce_step_height_ref = f(p["gait_bounce_step_height_ref"]);
  ps.support_centroid_tau = f(p["support_centroid_tau"]);
  ps.swing_lift_tau = f(p["swing_lift_tau"]);
  ps.support_shift_gain = f(p["support_shift_gain"]);
  ps.support_shift_lead = f(p["support_shift_lead"]);
  ps.support_shift_tau = f(p["support_shift_tau"]);
  ps.gait_activation_slew_rate = f(p["gait_activation_slew_rate"]);
  ps.pose_filter_tau = f(p["pose_filter_tau"]);
  ps.pose_filter_damping_ratio = f(p["pose_filter_damping_ratio"]);
  ps.pose_filter_snap_tol_linear = f(p["pose_filter_snap_tol_linear"]);
  ps.pose_filter_snap_tol_angular = f(p["pose_filter_snap_tol_angular"]);
  ps.gait_body_animations_enabled =
      p["gait_body_animations_enabled"].as<bool>();
  ps.pose_limit_x = f(p["pose_limit_x"]);
  ps.pose_limit_y = f(p["pose_limit_y"]);
  ps.pose_limit_roll = f(p["pose_limit_roll"]);
  ps.pose_limit_pitch = f(p["pose_limit_pitch"]);
  ps.pose_limit_yaw = f(p["pose_limit_yaw"]);
  ps.body_height_max = f(p["body_height_max_m"]);
  ps.body_height_min = f(p["body_height_min_m"]);
  ps.nominal_body_height = cfg.presets[cfg.default_preset].standing.body_height;
  // Every preset's height must sit inside the envelope: a preset change
  // re-plants onto it, and a clamped nominal lands the body where nobody asked.
  for (const auto& preset : cfg.presets) {
    const float h = preset.standing.body_height;
    if (!(ps.body_height_min < h && h < ps.body_height_max)) {
      throw std::runtime_error(
          "posture_node body_height_min_m/body_height_max_m must bracket "
          "gait_node presets." + preset.id + ".standing_pose.body_height");
    }
  }

  cfg.gestures = load_gestures(gestures_path, load_joint_limits(geo));

  return cfg;
}

hexa::pipeline::PipelineConfig load_pipeline_config(rclcpp::Node& node) {
  const std::string share =
      ament_index_cpp::get_package_share_directory("hexa_description");
  const std::string geometry_path = share + "/config/geometry.yaml";
  const std::string tuning_path = share + "/config/tuning.yaml";
  const std::string gestures_path = share + "/config/gestures.yaml";

  try {
    hexa::pipeline::PipelineConfig cfg = load_pipeline_config_from_yaml(
        geometry_path, tuning_path, gestures_path);
    RCLCPP_INFO(node.get_logger(),
                "hexa_locomotion: loaded runtime config (gait=%s, %zu gestures) "
                "from %s + %s + %s",
                cfg.default_gait.c_str(), cfg.gestures.size(),
                geometry_path.c_str(), tuning_path.c_str(),
                gestures_path.c_str());
    return cfg;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node.get_logger(),
                 "hexa_locomotion: runtime config load failed (%s); using baked "
                 "defaults",
                 ex.what());
    return hexa::pipeline::PipelineConfig::baked();
  }
}

}  // namespace hexa::locomotion
