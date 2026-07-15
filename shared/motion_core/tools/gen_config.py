#!/usr/bin/env python3
"""Bake the repo's runtime YAML config into a constexpr C++ header (plan part 04).

The Pico firmware has no filesystem, so it cannot load hexa_description /
hexa_gait / hexa_teleop / hexa_posture / hexa_control / hexa_hardware YAML at
runtime the way the ROS2 nodes do. This tool reads those same YAMLs and emits
``src/config_generated.hpp`` — one ``constexpr`` mirror per config surface — so
hexa_description stays the single source of truth for the numbers while the
firmware bakes them in at build time. CMake runs it pre-build (host + Pico).

The transforms here are ports of the ROS2 loaders and MUST stay in lockstep:

  - leg-mount six-leg symmetry expansion + deg->rad joint conventions mirror
    ``hexa_kinematics_cpp/src/description_loader.cpp``,
  - per-gait linear_max / yaw_bias mirror ``hexa_gait_cpp/src/limits.cpp`` (with
    the duty_factor / unstable table from ``gaits/registry.cpp``, which is code,
    not YAML),
  - servo pulse calibration (deg_at_center -> urdf_rad_at_center) mirrors
    ``hexa_hardware/src/joint_calibration.cpp``.

If those sources change their math, update this generator too.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

try:
    import yaml
except ImportError:  # pragma: no cover - dependency is present in the dev image
    sys.stderr.write("gen_config.py requires PyYAML (pip install pyyaml)\n")
    raise

# Canonical six-leg order — must match leg_index.hpp / the libs' LEG_NAMES.
LEG_NAMES = ["l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear"]

# Face expression names — lowercase of the Expression enum in
# src/hexa_display/vendor/core/Expression.h (the SSoT). Used only to fail the
# build fast on an expression-name typo in display.yaml; the name→enum mapping
# itself happens at runtime in the firmware via face::parseExpression, so this
# header never depends on Expression.h. Keep in sync if the enum grows.
EXPRESSION_NAMES = {
    "neutral", "happy", "sleepy", "dead", "greedy", "woozy", "angry", "love",
}

# Virtual D-pad directions — port of joy_mapping.DPAD_DIRECTIONS. Maps the
# bindable key name to (physical axis name, sign after normalisation that counts
# as "pressed").
DPAD_DIRECTIONS = {
    "dpad_up": ("dpad_y", 1),
    "dpad_down": ("dpad_y", -1),
    "dpad_left": ("dpad_x", -1),
    "dpad_right": ("dpad_x", 1),
}

# The functions map_joy queries, in a fixed order that also defines the C++
# ``JoyFn`` enum. Each is (enum name, joy_mapping function name). The generator
# pre-resolves every (mode, function) pair to a JoyKeyRef so the firmware needs
# no runtime string handling; the resolution mirrors joy_mapping's
# ``_resolve_function_key`` (mode bindings first, then base) + the key
# classification in ``button_pressed_for`` / ``axis_value_for``.
JOY_FUNCTIONS = [
    ("kGaitMode", "gait_mode"),
    ("kPostureMode", "posture_mode"),
    ("kAnimationMode", "animation_mode"),
    ("kInit", "init"),
    ("kRecord", "record"),
    ("kYawLeft", "yaw_left"),
    ("kYawRight", "yaw_right"),
    ("kWiggleLeft", "wiggle_left"),
    ("kWiggleRight", "wiggle_right"),
    ("kHeightUp", "height_up"),
    ("kHeightDown", "height_down"),
    ("kGaitPrev", "gait_prev"),
    ("kGaitNext", "gait_next"),
    ("kAnimationPrev", "animation_prev"),
    ("kAnimationNext", "animation_next"),
    ("kDriveX", "drive_x"),
    ("kDriveY", "drive_y"),
    ("kDriveYaw", "drive_yaw"),
    ("kPoseX", "pose_x"),
    ("kPoseY", "pose_y"),
    ("kTiltRoll", "tilt_roll"),
    ("kTiltPitch", "tilt_pitch"),
]


def resolve_joy_keyref(function, base_bindings, mode_bindings, btn, ax, signs):
    """Resolve one function to a (kind, index, sign, side) JoyKeyRef tuple.

    Ports joy_mapping._resolve_function_key (mode bindings searched first, then
    base) and the key classification the runtime helpers apply: a physical
    button, a virtual D-pad direction, an analog trigger / stick axis, or
    unbound. ``sign``/``side`` are only meaningful for the axis / D-pad kinds.
    """
    key = None
    for k, fn in mode_bindings.items():
        if fn == function:
            key = k
            break
    if key is None:
        for k, fn in base_bindings.items():
            if fn == function:
                key = k
                break
    if key is None:
        return ("kUnbound", 0, 1.0, 0)
    if key in btn:
        return ("kButton", btn[key], 1.0, 0)
    if key in DPAD_DIRECTIONS:
        axis_name, side = DPAD_DIRECTIONS[key]
        if axis_name not in ax:
            return ("kUnbound", 0, 1.0, 0)
        return ("kDpad", ax[axis_name], signs.get(axis_name, 1.0), side)
    if key in ax:
        return ("kAxis", ax[key], signs.get(key, 1.0), 0)
    return ("kUnbound", 0, 1.0, 0)

# Gait duty factors + stability flags. Not in YAML: they live on the strategy
# classes in hexa_gait_cpp/src/gaits/registry.cpp (the single source of truth).
# Keep in sync with that file. Order is the teleop gait_cycle order.
GAITS = [
    # name,       duty_factor, unstable
    ("tripod",    0.5,         False),
    ("surf",      5.0 / 8.0,   True),
    ("tetrapod",  2.0 / 3.0,   False),
    ("crawl",     2.0 / 3.0,   True),
    ("ripple",    5.0 / 6.0,   False),
]


# ── formatting helpers ──────────────────────────────────────────────────────

def fl(x) -> str:
    """Float literal that always carries a '.'/'e' so it is a valid C++ float."""
    s = repr(float(x))  # shortest round-tripping decimal; never a bare integer
    return s + "f"


def cstr(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


# ── joint-angle conventions (mirror description_loader.cpp) ──────────────────

def center_field(joint_type: str) -> str:
    return {"coxa": "deg", "femur": "above_horizontal_deg",
            "tibia": "interior_deg"}[joint_type]


def to_urdf_rad(joint_type: str, deg: float) -> float:
    if joint_type == "coxa":
        return math.radians(deg)
    if joint_type == "femur":
        return -math.radians(deg)
    if joint_type == "tibia":
        return math.pi - math.radians(deg)
    raise ValueError(f"unknown joint type: {joint_type}")


JOINT_TYPES = ["coxa", "femur", "tibia"]


# ── loaders ─────────────────────────────────────────────────────────────────

def load_yaml(path: str):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def leg_specs(geometry: dict):
    """Six LegSpecs by symmetry — port of load_leg_specs()."""
    leg = geometry["leg"]
    coxa_len = leg["coxa_length"]
    femur_len = leg["femur_length"]
    tibia_len = leg["tibia_length"]
    front = geometry["mounts"]["l_front"]
    middle = geometry["mounts"]["l_middle"]

    out = {}
    for side in ("l", "r"):
        for name in ("front", "middle", "rear"):
            ref = middle if name == "middle" else front
            ref_yaw = math.radians(ref["yaw_deg"])
            ref_x, ref_y = ref["x"], ref["y"]
            x_fr = -ref_x if name == "rear" else ref_x
            yaw_fr = (math.pi - ref_yaw) if name == "rear" else ref_yaw
            mx = x_fr
            my = -ref_y if side == "r" else ref_y
            myaw = -yaw_fr if side == "r" else yaw_fr
            out[f"{side}_{name}"] = dict(
                mount_xyz=(mx, my, 0.0), mount_yaw=myaw,
                coxa_len=coxa_len, femur_len=femur_len, tibia_len=tibia_len)
    return out


def joint_limits(geometry: dict):
    """Per-joint-type servo window in URDF rad — port of load_joint_limits()."""
    joints = geometry["joints"]
    out = {}
    for jt in JOINT_TYPES:
        cfg = joints[jt]
        center = to_urdf_rad(jt, cfg[center_field(jt)])
        a = to_urdf_rad(jt, cfg["lower_limit_deg"])
        b = to_urdf_rad(jt, cfg["upper_limit_deg"])
        lower, upper = min(a, b), max(a, b)
        if not (lower <= center <= upper):
            raise ValueError(f"{jt} center outside limit window")
        out[jt] = dict(center=center, lower=lower, upper=upper,
                       effort=cfg["effort"], velocity=cfg["velocity"])
    return out


# tuning.yaml gait_node standing_pose leaf key per joint type. Distinct from
# center_field() (the geometry.yaml joints: field names) — the ros params carry
# the joint name in the key, e.g. standing_pose.coxa_deg.
_STANDING_FIELD = {
    "coxa": "coxa_deg",
    "femur": "femur_above_horizontal_deg",
    "tibia": "tibia_interior_deg",
}


def standing_pose(gait: dict) -> tuple:
    """Uniform (coxa, femur, tibia) at-rest angles — port of load_standing_pose.

    Reads the standing pose from tuning.yaml's gait_node standing_pose block
    (passed in as the unwrapped ``gait`` ros params), matching how the gait
    node now sources it as ros parameters.
    """
    sp = gait["standing_pose"]
    return tuple(to_urdf_rad(jt, sp[_STANDING_FIELD[jt]]) for jt in JOINT_TYPES)


def initial_pose(geometry: dict):
    """Per-leg (coxa, femur, tibia) startup angles — port of load_initial_pose."""
    init = geometry["initial_pose"]
    femur = to_urdf_rad("femur", init["femur"]["above_horizontal_deg"])
    tibia = to_urdf_rad("tibia", init["tibia"]["interior_deg"])
    coxa_cfg = init["coxa"]
    out = {}
    for side in ("l", "r"):
        for name in ("front", "middle", "rear"):
            ref_deg = coxa_cfg["l_middle_deg" if name == "middle" else "l_front_deg"]
            after_fr = -ref_deg if name == "rear" else ref_deg
            after_lr = -after_fr if side == "r" else after_fr
            coxa = to_urdf_rad("coxa", after_lr)
            out[f"{side}_{name}"] = (coxa, femur, tibia)
    return out


def velocity_caps(gait: dict):
    """Per-gait linear_max + yaw_bias — port of load_velocity_caps()."""
    stride = gait["stride_length"]
    min_swing = gait["min_swing_time"]
    yaw_bias = gait["yaw_bias"]
    caps = []
    for name, duty, unstable in GAITS:
        linear_max = stride * (1.0 - duty) / (min_swing * duty)
        yaw_bias_eff = 0.5 + (yaw_bias - 0.5) * (1.5 - duty)
        caps.append(dict(name=name, duty=duty, unstable=unstable,
                         linear_max=linear_max, yaw_bias=yaw_bias_eff))
    return caps


def calibration_by_pin(calibration: dict):
    """Endpoint pulse widths keyed by pin — port of load_calibration().

    servo_calibration.yaml's `calibration_values` list is pin-ordered (index i
    is pin i+1); each entry's own `pin` field must match its position.
    """
    values = calibration.get("calibration_values")
    if not isinstance(values, list):
        raise ValueError("servo_calibration.yaml missing 'calibration_values' list")
    by_pin = {}
    for i, entry in enumerate(values):
        pin = entry["pin"]
        if pin != i + 1:
            raise ValueError(
                f"calibration_values[{i}] pin {pin} must equal index+1 ({i + 1})")
        by_pin[pin] = (entry["us_at_plus_45"], entry["us_at_minus_45"])
    return by_pin


def hardware_joints(hw: dict, calibration: dict):
    """18 servo calibrations, sorted by pin — port of joint_calibration.cpp."""
    dac = hw.get("deg_at_center", {})
    deg_at_center = {"coxa": dac.get("coxa", 0.0), "femur": dac.get("femur", 0.0),
                     "tibia": dac.get("tibia", 0.0)}
    endpoints = calibration_by_pin(calibration)

    def urdf_center(pos: str) -> float:
        rad = math.radians(deg_at_center[pos])
        return {"coxa": rad, "femur": -rad, "tibia": math.pi - rad}[pos]

    # Authoritative name→segment map for the fixed 6-leg set (mirrors
    # joint_calibration.cpp's kPositions). An unknown joint name is rejected.
    joint_positions = {
        f"{side}_{leg}_{seg}_joint": seg
        for side in ("l", "r")
        for leg in ("front", "middle", "rear")
        for seg in ("coxa", "femur", "tibia")
    }

    rows = []
    for name, j in hw["joints"].items():
        if name not in joint_positions:
            raise ValueError(f"unknown joint name '{name}'")
        pos = joint_positions[name]
        pin = j["pin"]
        if pin not in endpoints:
            raise ValueError(
                f"joint '{name}' pin {pin} has no servo_calibration.yaml entry")
        us_plus, us_minus = endpoints[pin]
        rows.append(dict(
            name=name, pin=pin, joint_position=pos,
            us_at_plus_45=us_plus, us_at_minus_45=us_minus,
            urdf_rad_at_center=urdf_center(pos),
            direction=j.get("direction", 1),
            min_us=j["min_us"], max_us=j["max_us"]))
    rows.sort(key=lambda r: r["pin"])
    return rows


# ── header emission ─────────────────────────────────────────────────────────

def emit(geometry, gait, teleop, posture, control, hardware, calibration,
         webteleop, display, sources) -> str:
    specs = leg_specs(geometry)
    limits = joint_limits(geometry)
    stand = standing_pose(gait)
    initial = initial_pose(geometry)
    caps = velocity_caps(gait)
    joints = hardware_joints(hardware, calibration)

    L = []
    w = L.append

    w("// AUTO-GENERATED by tools/gen_config.py — DO NOT EDIT.")
    w("//")
    w("// Regenerate with `python3 tools/gen_config.py` (CMake runs it pre-build).")
    w("// hexa_description and the other config packages remain the single source")
    w("// of truth; this header is a baked mirror for the filesystem-less firmware.")
    w("// Sourced from:")
    for s in sources:
        w(f"//   - {s}")
    w("#pragma once")
    w("")
    w("#include <array>")
    w("#include <cstdint>")
    w("#include <string_view>")
    w("")
    w('#include "leg_index.hpp"')
    w('#include "vec3.hpp"')
    w("")
    w("namespace hexa::config {")
    w("")

    # ── kinematics / geometry ──
    w("// ── Leg geometry (hexa_description/config/geometry.yaml) ──")
    w("struct LegSpec {")
    w("  Vec3 mount_xyz;    // coxa pivot in body frame (m)")
    w("  float mount_yaw;   // rotation about body +z (rad)")
    w("  float coxa_len;")
    w("  float femur_len;")
    w("  float tibia_len;")
    w("};")
    w("")
    w("// One LegSpec per leg, indexed by Leg (leg_index.hpp order).")
    w(f"inline constexpr std::array<LegSpec, kNumLegs> kLegSpecs = {{{{")
    for leg in LEG_NAMES:
        s = specs[leg]
        mx, my, mz = s["mount_xyz"]
        w(f"    {{Vec3({fl(mx)}, {fl(my)}, {fl(mz)}), {fl(s['mount_yaw'])}, "
          f"{fl(s['coxa_len'])}, {fl(s['femur_len'])}, {fl(s['tibia_len'])}}},"
          f"  // {leg}")
    w("}};")
    w("")
    w(f"inline constexpr float kCoxaToBottom = {fl(geometry['body']['coxa_to_bottom'])};"
      "  // m")
    w("")

    # ── joint limits ──
    w("// ── Per-joint-type servo limits, IK-convention radians ──")
    w("struct JointLimits {")
    w("  float center;")
    w("  float lower;")
    w("  float upper;")
    w("  float effort;    // Nm")
    w("  float velocity;  // rad/s")
    w("};")
    w("")
    w("// Indexed 0=coxa, 1=femur, 2=tibia (same order as a JointAngles triple).")
    w("inline constexpr std::array<JointLimits, 3> kJointLimits = {{")
    for jt in JOINT_TYPES:
        m = limits[jt]
        w(f"    {{{fl(m['center'])}, {fl(m['lower'])}, {fl(m['upper'])}, "
          f"{fl(m['effort'])}, {fl(m['velocity'])}}},  // {jt}")
    w("}};")
    w("")

    # ── standing / initial pose ──
    w("// ── Rest / startup joint angles (rad, IK-convention) ──")
    w("// Standing pose is uniform across legs; foot targets come from FK per leg.")
    w(f"inline constexpr JointAngles kStandingPose = "
      f"{{{fl(stand[0])}, {fl(stand[1])}, {fl(stand[2])}}};")
    w("")
    w("// Per-leg startup pose (coxa varies by symmetry), indexed by Leg.")
    w("inline constexpr std::array<JointAngles, kNumLegs> kInitialPose = {{")
    for leg in LEG_NAMES:
        a = initial[leg]
        w(f"    {{{fl(a[0])}, {fl(a[1])}, {fl(a[2])}}},  // {leg}")
    w("}};")
    w("")

    # ── gait engine ──
    w("// ── Gait engine knobs (hexa_description/config/tuning.yaml) ──")
    w("struct EngineConfig {")
    fields = [
        ("stride_length", gait["stride_length"]),
        ("min_swing_time", gait["min_swing_time"]),
        ("max_swing_time", gait["max_swing_time"]),
        ("step_height", gait["step_height"]),
        ("swing_width", gait["swing_width"]),
        ("controller_dt", gait["controller_dt"]),
        ("cmd_zero_tol", gait["cmd_zero_tol"]),
        ("pause_debounce_delay", gait["pause_debounce_delay"]),
        ("pause_to_reseat_delay", gait["pause_to_reseat_delay"]),
        ("gait_change_pause_to_reseat_delay",
         gait["gait_change_pause_to_reseat_delay"]),
        ("max_reset_time", gait["max_reset_time"]),
        ("init_pair_swing_time", gait["initialize"]["pair_swing_time"]),
        ("init_lift_body_time", gait["initialize"]["lift_body_time"]),
        ("init_swing_clearance", gait["initialize"]["swing_clearance"]),
        ("init_place_feet_clearance", gait["initialize"]["place_feet_clearance"]),
        ("reseat_pose_settle_delay", gait["reseat"]["pose_settle_delay"]),
        ("reseat_height_change_threshold", gait["reseat"]["height_change_threshold"]),
        ("reseat_pair_swing_time", gait["reseat"]["pair_swing_time"]),
        ("reseat_pair_dwell_time", gait["reseat"]["pair_dwell_time"]),
        ("reseat_swing_clearance", gait["reseat"]["swing_clearance"]),
    ]
    for fname, _ in fields:
        w(f"  float {fname};")
    w("};")
    w("")
    w("inline constexpr EngineConfig kEngine = {")
    for fname, val in fields:
        w(f"    {fl(val)},  // {fname}")
    w("};")
    w("")

    # ── velocity caps / gaits ──
    w("// ── Per-gait velocity caps (derived; mirror hexa_gait_cpp limits.cpp) ──")
    w("struct GaitSpec {")
    w("  std::array<char, 12> name;  // NUL-padded; use gait_name() to compare")
    w("  float duty_factor;")
    w("  bool unstable;")
    w("  float linear_max;  // m/s")
    w("  float yaw_bias;    // effective, per-gait")
    w("};")
    w("")

    def name_arr(name: str) -> str:
        chars = list(name.encode())
        chars += [0] * (12 - len(chars))
        return "{" + ", ".join(str(c) for c in chars) + "}"

    w("inline constexpr std::array<GaitSpec, "
      f"{len(caps)}> kGaits = {{{{")
    for c in caps:
        w(f"    {{{name_arr(c['name'])}, {fl(c['duty'])}, "
          f"{'true' if c['unstable'] else 'false'}, {fl(c['linear_max'])}, "
          f"{fl(c['yaw_bias'])}}},  // {c['name']}")
    w("}};")
    w("")
    w(f"inline constexpr float kAngularMax = {fl(gait['angular_z_max'])};"
      "  // rad/s, shared")
    w("")

    # ── teleop ──
    base = teleop["base"]
    btn = base["buttons"]
    ax = base["axes"]
    signs = base.get("axis_signs", {})

    def sign(a):
        return signs.get(a, 1.0)

    w("// ── Joystick teleop (hexa_teleop/config/teleop_joy.yaml) ──")
    w("struct JoyButtons {  // sensor_msgs/Joy.buttons[] indices")
    for k in ("a", "b", "x", "y", "l1", "r1", "select", "start"):
        w(f"  int {k};")
    w("};")
    w("inline constexpr JoyButtons kButtons = {"
      + ", ".join(str(btn[k]) for k in
                  ("a", "b", "x", "y", "l1", "r1", "select", "start")) + "};")
    w("")
    axis_keys = ("left_stick_x", "left_stick_y", "l2", "right_stick_x",
                 "right_stick_y", "r2", "dpad_x", "dpad_y")
    w("struct JoyAxes {  // sensor_msgs/Joy.axes[] indices")
    for k in axis_keys:
        w(f"  int {k};")
    w("};")
    w("inline constexpr JoyAxes kAxes = {"
      + ", ".join(str(ax[k]) for k in axis_keys) + "};")
    w("")
    w("struct JoyAxisSigns {  // per-axis normalisation (default +1)")
    for k in axis_keys:
        w(f"  float {k};")
    w("};")
    w("inline constexpr JoyAxisSigns kAxisSigns = {"
      + ", ".join(fl(sign(k)) for k in axis_keys) + "};")
    w("")
    w(f"inline constexpr float kDeadband = {fl(base['deadband'])};")
    w(f"inline constexpr float kTriggerThreshold = {fl(base['trigger_threshold'])};"
      "  // pressed = axis < this")
    w("")
    # Posture-mode scalar limits.
    p = teleop["posture"]
    ph = p["height"]
    w("// Posture-mode scalar limits (teleop_joy.yaml posture:).")
    w("struct PostureLimits {")
    for f in ("x_max", "y_max", "roll_max_deg", "pitch_max_deg", "yaw_max_deg",
              "yaw_tau_s", "revert_tau_s", "wiggle_pivot_forward_m",
              "height_max_m", "height_min_m", "height_rate_m_per_s"):
        w(f"  float {f};")
    w("};")
    w("inline constexpr PostureLimits kPostureLimits = {"
      + ", ".join(fl(v) for v in (
          p["x_max"], p["y_max"], p["roll_max_deg"], p["pitch_max_deg"],
          p["yaw_max_deg"], p["yaw_tau_s"], p["revert_tau_s"],
          p["wiggle_pivot_forward_m"], ph["max_m"], ph["min_m"],
          ph["rate_m_per_s"])) + "};")
    w("")
    # Modes / gait cycle.
    w(f"inline constexpr std::string_view kInitialMode = {cstr(teleop['initial_mode'])};")
    w(f"inline constexpr std::string_view kDefaultGait = {cstr(teleop['default_gait'])};")
    w(f"inline constexpr bool kAllowUnstableGaits = "
      f"{'true' if teleop['allow_unstable_gaits'] else 'false'};")
    # kGaitCycle is the runtime rotation the teleop cycler walks — already
    # filtered by allow_unstable_gaits (port of joy_mapping.resolve_gait_cycle),
    # so the firmware cycler matches the ROS node's accepted set.
    unstable_names = {name for name, _, u in GAITS if u}
    cycle_raw = [str(g) for g in teleop["gait_cycle"]]
    if teleop["allow_unstable_gaits"]:
        cycle = cycle_raw
    else:
        cycle = [g for g in cycle_raw if g not in unstable_names]
    w(f"inline constexpr std::array<std::string_view, {len(cycle)}> kGaitCycle = {{"
      + ", ".join(cstr(g) for g in cycle) + "};")
    w("")

    # ── teleop binding resolution (pre-resolved per mode) ──
    base_bindings = base["bindings"]
    w("// ── Teleop binding resolution (baked; mirrors joy_mapping key search) ──")
    w("// One JoyKeyRef per (mode, function): the generator ran joy_mapping's")
    w("// mode-then-base function search + key classification, so map_joy resolves")
    w("// a function to its physical input by table lookup — no string handling.")
    w("enum class JoyFn : std::uint8_t {")
    for fn_name, _ in JOY_FUNCTIONS:
        w(f"  {fn_name},")
    w("  kCount,")
    w("};")
    w("")
    w("enum class JoyInputKind : std::uint8_t { kUnbound, kButton, kDpad, kAxis };")
    w("")
    w("struct JoyKeyRef {")
    w("  JoyInputKind kind;")
    w("  std::int16_t index;  // buttons[]/axes[] index (kButton/kDpad/kAxis)")
    w("  float sign;          // axis sign (kDpad/kAxis)")
    w("  std::int8_t side;    // dpad direction +1/-1 (kDpad)")
    w("};")
    w("")
    for mode_name, mode_key in (("Posture", "posture"), ("Gait", "gait"),
                                ("Animation", "animation")):
        mode_bindings = teleop[mode_key]["bindings"]
        w(f"inline constexpr std::array<JoyKeyRef, {len(JOY_FUNCTIONS)}> "
          f"kBind{mode_name} = {{{{")
        for fn_name, fn_str in JOY_FUNCTIONS:
            kind, idx, s, side = resolve_joy_keyref(
                fn_str, base_bindings, mode_bindings, btn, ax, signs)
            w(f"    {{JoyInputKind::{kind}, {idx}, {fl(s)}, {side}}},"
              f"  // {fn_str}")
        w("}};")
        w("")

    # ── posture animation stack ──
    pn = posture["posture_node"]["ros__parameters"]
    w("// ── Posture animation stack (hexa_description/config/tuning.yaml) ──")
    enabled = pn["enabled_animations"]
    w(f"inline constexpr std::array<std::string_view, {len(enabled)}> "
      "kEnabledAnimations = {" + ", ".join(cstr(a) for a in enabled) + "};")
    anim_mode = pn["animation_mode_animations"]
    w(f"inline constexpr std::array<std::string_view, {len(anim_mode)}> "
      "kAnimationModeAnimations = {" + ", ".join(cstr(a) for a in anim_mode) + "};")
    w("")
    w("struct PostureConfig {")
    posture_fields = [
        ("gait_sway_gain", pn["gait_sway_gain"]),
        ("gait_sway_strength", pn["gait_sway_strength"]),
        ("vertical_body_roll_z_amplitude", pn["vertical_body_roll_z_amplitude"]),
        ("vertical_body_roll_pitch_amplitude_deg",
         pn["vertical_body_roll_pitch_amplitude_deg"]),
        ("vertical_body_roll_phase_offset", pn["vertical_body_roll_phase_offset"]),
        ("horizontal_body_roll_y_amplitude", pn["horizontal_body_roll_y_amplitude"]),
        ("horizontal_body_roll_yaw_amplitude_deg",
         pn["horizontal_body_roll_yaw_amplitude_deg"]),
        ("horizontal_body_roll_phase_offset", pn["horizontal_body_roll_phase_offset"]),
        ("body_roll_3d_z_amplitude", pn["body_roll_3d_z_amplitude"]),
        ("body_roll_3d_pitch_amplitude_deg", pn["body_roll_3d_pitch_amplitude_deg"]),
        ("body_roll_3d_y_amplitude", pn["body_roll_3d_y_amplitude"]),
        ("body_roll_3d_yaw_amplitude_deg", pn["body_roll_3d_yaw_amplitude_deg"]),
        ("body_roll_3d_horizontal_phase_offset",
         pn["body_roll_3d_horizontal_phase_offset"]),
        ("body_roll_3d_pitch_phase_offset", pn["body_roll_3d_pitch_phase_offset"]),
        ("body_roll_3d_yaw_phase_offset", pn["body_roll_3d_yaw_phase_offset"]),
        ("gait_bounce_arc_height", pn["gait_bounce_arc_height"]),
        ("gait_bounce_step_height_ref", pn["gait_bounce_step_height_ref"]),
        ("support_centroid_tau", pn["support_centroid_tau"]),
        ("swing_lift_tau", pn["swing_lift_tau"]),
        # Gait-animation crossfade + layered clamp (posture layering fix).
        ("gait_activation_slew_rate", pn["gait_activation_slew_rate"]),
        ("animation_reserve_x", pn["animation_reserve_x"]),
        ("animation_reserve_y", pn["animation_reserve_y"]),
        ("animation_reserve_z", pn["animation_reserve_z"]),
        ("animation_reserve_roll", pn["animation_reserve_roll"]),
        ("animation_reserve_pitch", pn["animation_reserve_pitch"]),
        ("animation_reserve_yaw", pn["animation_reserve_yaw"]),
    ]
    for fname, _ in posture_fields:
        w(f"  float {fname};")
    w("};")
    w("inline constexpr PostureConfig kPosture = {")
    for fname, val in posture_fields:
        w(f"    {fl(val)},  // {fname}")
    w("};")
    w("")

    # ── control velocity shaping ──
    w("// ── Control velocity shaping (hexa_description/config/tuning.yaml) ──")
    w("struct ControlConfig {")
    control_fields = [
        ("vmax_ramp_time_linear", control["vmax_ramp_time_linear"]),
        ("vmax_ramp_time_angular", control["vmax_ramp_time_angular"]),
        ("snap_tol_linear", control["snap_tol_linear"]),
        ("snap_tol_angular", control["snap_tol_angular"]),
    ]
    for fname, _ in control_fields:
        w(f"  float {fname};")
    w("};")
    w("inline constexpr ControlConfig kControl = {"
      + ", ".join(fl(v) for _, v in control_fields) + "};")
    w("")

    # ── hardware / servo calibration ──
    w("// ── Servo 2040 wiring + calibration (hexa_description/config/"
      "hardware.yaml + servo_calibration.yaml) ──")
    w("struct JointCal {")
    w("  std::uint8_t pin;")
    w("  float us_at_plus_45;")
    w("  float us_at_minus_45;")
    w("  float urdf_rad_at_center;")
    w("  std::int8_t direction;  // +1 normal, -1 mirror-mounted servo")
    w("  std::uint16_t min_us;")
    w("  std::uint16_t max_us;")
    w("};")
    w("")
    w("// Sorted by pin (1..18). Table order is the joint order: l_front,")
    w("// l_middle, l_rear, r_front, r_middle, r_rear, each {coxa, femur, tibia}.")
    w("inline constexpr std::array<JointCal, 18> kJointCals = {{")
    for j in joints:
        w(f"    {{{j['pin']}, {fl(j['us_at_plus_45'])}, {fl(j['us_at_minus_45'])}, "
          f"{fl(j['urdf_rad_at_center'])}, {j['direction']}, {j['min_us']}, {j['max_us']}}},"
          f"  // {j['name']}")
    w("}};")
    w("")
    w(f"inline constexpr std::uint8_t kRelayPin = "
      f"{hardware['relay']['pin']};  // high = servo rail energised")
    aux = hardware["aux"]
    bv = aux["battery_voltage"]
    bc = aux["battery_current"]
    w(f"inline constexpr std::uint8_t kBatteryVoltagePin = {bv['pin']};")
    w(f"inline constexpr float kBatteryVoltageScale = {fl(bv['scale'])};"
      "  // 14-bit count -> V")
    w(f"inline constexpr std::uint8_t kBatteryCurrentPin = {bc['pin']};")
    w(f"inline constexpr float kBatteryCurrentScale = {fl(bc['scale'])};"
      "  // 14-bit count -> A")
    w("")

    # ── Integration / failsafe (part 09) ──
    # The RP2350 firmware fuses the ROS system's separate safety knobs into one
    # onboard supervisor, so their sources stay authoritative:
    #   - input_timeout_s     — hexa_webteleop safety watchdog (the gamepad has no
    #     ROS equivalent; the webteleop timeout is the canonical "stale input"
    #     value the firmware mirrors for its BT link),
    #   - get_period_ticks    — hexa_hardware aux GET decimation (poll every Nth
    #     control tick),
    #   - battery_* thresholds — hexa_display BatteryMonitor debounce (0.0
    #     disables a flag; shipped disabled — the divider scale is uncalibrated).
    w("// ── Integration / failsafe (part 09) ──")
    safety = (webteleop.get("safety", {}) or {})
    input_timeout_s = float(safety.get("input_timeout_s", 0.5))
    w(f"inline constexpr float kInputTimeoutS = {fl(input_timeout_s)};"
      "  // webteleop safety.input_timeout_s; stale BT input -> zero cmd_vel")
    get_period_ticks = int(hardware["parser"]["get_period_ticks"])
    w(f"inline constexpr int kGetPeriodTicks = {get_period_ticks};"
      "  // hardware.yaml parser.get_period_ticks; battery GET every Nth tick")
    w("")
    dp = display["display_node"]["ros__parameters"]
    w("struct BatteryThresholds {  // hexa_display BatteryMonitor (0 disables a flag)")
    w("  float warning_v;")
    w("  float critical_v;")
    w("  float hysteresis_v;")
    w("  float hold_s;")
    w("};")
    w("inline constexpr BatteryThresholds kBattery = {"
      + ", ".join(fl(dp[k]) for k in (
          "battery_warning_v", "battery_critical_v",
          "battery_hysteresis_v", "battery_hold_s")) + "};")
    w("")

    # ── Face expression/gaze policy + SH1122 panel (part 11) ──
    # The eye policy runs on core0 and the SH1122 render loop on core1 (see
    # pi-pico-firmware/src/face.cpp). Expression names resolve to the Expression
    # enum at RUNTIME via face::parseExpression, so this header stays free of the
    # vendored eye core; names are validated here only to fail the build on a typo.
    pico_panel = display.get("pico_panel", {}) or {}

    def face_expr(param: str) -> str:
        name = str(dp[param]).lower()
        if name not in EXPRESSION_NAMES:
            raise ValueError(
                f"display.yaml {param}: unknown expression {name!r} "
                f"(valid: {', '.join(sorted(EXPRESSION_NAMES))})")
        return name

    w("// ── Face policy + panel (hexa_display/config/display.yaml) ──")
    w(f"inline constexpr bool kFaceEnabled = "
      f"{'true' if pico_panel.get('enabled', True) else 'false'};")
    w(f"inline constexpr float kFaceUpdateRateHz = {fl(dp['update_rate_hz'])};"
      "  // policy tick (core0)")
    w(f"inline constexpr float kFaceRenderHz = {fl(dp['render_hz'])};"
      "  // EyeAnim/raster (core1)")
    w("")
    w("// Gait-state -> expression. Resolve `expression` with face::parseExpression.")
    w("struct FaceExpressionEntry { std::string_view state; std::string_view expression; };")
    states = [k[len("expression_map."):] for k in dp
              if k.startswith("expression_map.")]
    w(f"inline constexpr std::array<FaceExpressionEntry, {len(states)}> "
      "kFaceExpressionMap = {{")
    for st in states:
        w(f"    {{{cstr(st)}, {cstr(face_expr('expression_map.' + st))}}},")
    w("}};")
    w("")
    w(f"inline constexpr std::string_view kFaceAnimationExpression = "
      f"{cstr(face_expr('animation_expression'))};")
    w(f"inline constexpr std::string_view kFaceBatteryWarningExpression = "
      f"{cstr(face_expr('battery_warning_expression'))};")
    w(f"inline constexpr std::string_view kFaceBatteryCriticalExpression = "
      f"{cstr(face_expr('battery_critical_expression'))};")
    w("")
    w("struct FaceGazeConfig {")
    gaze_fields = [
        ("gaze_deadband", dp["gaze_deadband"]),
        ("gaze_exit_ratio", dp["gaze_exit_ratio"]),
        ("gaze_wz_weight", dp["gaze_wz_weight"]),
        ("gaze_vy_max", dp["gaze_vy_max"]),
        ("gaze_wz_max", dp["gaze_wz_max"]),
        ("pose_pitch_threshold_rad", dp["pose_pitch_threshold_rad"]),
        ("pose_tilt_threshold_rad", dp["pose_tilt_threshold_rad"]),
        ("idling_start_delay_s", dp["idling_start_delay_s"]),
    ]
    for fname, _ in gaze_fields:
        w(f"  float {fname};")
    w("};")
    w("inline constexpr FaceGazeConfig kFaceGaze = {"
      + ", ".join(fl(v) for _, v in gaze_fields) + "};")
    w("")
    w("// SH1122 panel wiring on the Pico (display.yaml pico_panel:, firmware-only).")
    w("struct FacePanel {")
    w("  std::uint8_t spi_index;  // 0 = spi0, 1 = spi1")
    w("  std::uint8_t sck;")
    w("  std::uint8_t mosi;")
    w("  std::uint8_t cs;")
    w("  std::uint8_t dc;")
    w("  std::uint8_t rst;")
    w("  std::uint32_t spi_hz;")
    w("};")
    w("inline constexpr FacePanel kFacePanel = {"
      f"{int(pico_panel.get('spi_index', 0))}, {int(pico_panel.get('sck', 18))}, "
      f"{int(pico_panel.get('mosi', 19))}, {int(pico_panel.get('cs', 17))}, "
      f"{int(pico_panel.get('dc', 20))}, {int(pico_panel.get('rst', 21))}, "
      f"{int(pico_panel.get('spi_hz', 8_000_000))}u}};")
    w("")

    w("}  // namespace hexa::config")
    w("")
    return "\n".join(L)


# ── entry point ─────────────────────────────────────────────────────────────

def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))  # shared/motion_core/tools
    shared_pkg = os.path.dirname(here)                  # shared/motion_core
    default_repo = os.path.dirname(os.path.dirname(shared_pkg))  # workspace root
    default_out = os.path.join(shared_pkg, "config_generated.hpp")

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo-root", default=default_repo,
                    help="workspace root containing src/hexa_* (default: auto)")
    ap.add_argument("--out", default=default_out,
                    help="output header path "
                         "(default: shared/motion_core/config_generated.hpp)")
    args = ap.parse_args()

    cfg_dir = os.path.join(args.repo_root, "src")
    # gait / control / posture all come from hexa_description's tuning.yaml —
    # the single source of truth the ROS nodes read. Baking from the same file
    # keeps the firmware constants in lockstep with sim/robot with no overlay
    # merge to mirror.
    tuning_yaml = f"{cfg_dir}/hexa_description/config/tuning.yaml"
    paths = {
        "geometry": f"{cfg_dir}/hexa_description/config/geometry.yaml",
        "gait": tuning_yaml,
        "teleop": f"{cfg_dir}/hexa_teleop/config/teleop_joy.yaml",
        "posture": tuning_yaml,
        "control": tuning_yaml,
        "hardware": f"{cfg_dir}/hexa_description/config/hardware.yaml",
        "calibration": f"{cfg_dir}/hexa_description/config/servo_calibration.yaml",
        "webteleop": f"{cfg_dir}/hexa_webteleop/config/webteleop.yaml",
        "display": f"{cfg_dir}/hexa_display/config/display.yaml",
    }
    missing = [p for p in paths.values() if not os.path.isfile(p)]
    if missing:
        sys.stderr.write("gen_config.py: missing source YAML(s):\n")
        for m in missing:
            sys.stderr.write(f"  {m}\n")
        return 1

    loaded = {k: load_yaml(v) for k, v in paths.items()}
    # tuning.yaml is a ros2 params file; unwrap control and gait to their flat
    # knob blocks so the rest of this tool reads the same keys the nodes expose
    # as ros params (nested initialize:/reseat: maps preserved). posture stays
    # wrapped and is unwrapped at its own use sites. Each key loaded tuning.yaml
    # in its own parse, so unwrapping one does not touch the others.
    loaded["control"] = loaded["control"]["control_node"]["ros__parameters"]
    loaded["gait"] = loaded["gait"]["gait_node"]["ros__parameters"]
    # Dedupe the provenance list: tuning.yaml backs three logical sources.
    sources = list(dict.fromkeys(
        os.path.relpath(p, args.repo_root) for p in paths.values()
    ))
    header = emit(loaded["geometry"], loaded["gait"],
                  loaded["teleop"], loaded["posture"], loaded["control"],
                  loaded["hardware"], loaded["calibration"],
                  loaded["webteleop"], loaded["display"], sources)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    # Only rewrite when content changes so CMake dependency timestamps stay clean.
    old = None
    if os.path.isfile(args.out):
        with open(args.out, "r") as f:
            old = f.read()
    if old != header:
        with open(args.out, "w") as f:
            f.write(header)
        sys.stderr.write(f"gen_config.py: wrote {args.out}\n")
    else:
        sys.stderr.write(f"gen_config.py: {args.out} up to date\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
