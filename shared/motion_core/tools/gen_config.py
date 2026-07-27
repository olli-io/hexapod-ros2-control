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
    ``hexa_locomotion/src/pipeline_config_loader.cpp``,
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


def bo(x) -> str:
    """C++ bool literal from a YAML boolean."""
    return "true" if x else "false"


# ── joint-angle conventions (mirror pipeline_config_loader.cpp) ──────────────

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
    """Per-joint-type travel window in URDF rad — port of load_joint_limits()."""
    joints = geometry["joints"]
    out = {}
    for jt in JOINT_TYPES:
        cfg = joints[jt]
        a = to_urdf_rad(jt, cfg["lower_limit_deg"])
        b = to_urdf_rad(jt, cfg["upper_limit_deg"])
        lower, upper = min(a, b), max(a, b)
        out[jt] = dict(lower=lower, upper=upper,
                       effort=cfg["effort"], velocity=cfg["velocity"])
    return out


def standing_pose(gait: dict, geometry: dict) -> dict:
    """At-rest stance scalars from tuning.yaml's gait_node standing_pose block.

    The stance is described by where the feet sit, not by joint angles: the
    tip radius out from each leg's own coxa axis, the belly clearance, and the
    corner-leg splay. The femur/tibia angles follow from a 2-link IK solve that
    motion_core owns (gait::standing_pose_from) — deliberately not duplicated
    here, so there is exactly one copy of that math.
    """
    sp = gait["standing_pose"]
    tip_radius = sp["tip_radius"]
    body_height = sp["body_height"]

    # Reachability guard so a bad edit fails at build time rather than throwing
    # UnreachableTarget on the robot. The angles themselves are checked against
    # the joint limits by standing_pose_from when the config loads.
    leg = geometry["leg"]
    coxa_len, femur_len, tibia_len = (
        leg["coxa_length"], leg["femur_length"], leg["tibia_length"])
    depth = geometry["body"]["coxa_to_bottom"] + body_height
    if tip_radius <= coxa_len:
        raise ValueError(
            f"tuning.yaml standing_pose.tip_radius = {tip_radius} m must exceed "
            f"the coxa length ({coxa_len} m)")
    reach = math.hypot(tip_radius - coxa_len, depth)
    if not (abs(femur_len - tibia_len) <= reach <= femur_len + tibia_len):
        raise ValueError(
            f"tuning.yaml standing_pose tip_radius = {tip_radius} m / "
            f"body_height = {body_height} m puts the foot {reach:.4f} m from the "
            f"femur joint; reach annulus is "
            f"[{abs(femur_len - tibia_len):.4f}, {femur_len + tibia_len:.4f}] m")

    return dict(tip_radius=tip_radius, body_height=body_height,
                corner_leg_coxa=to_urdf_rad("coxa", sp["corner_leg_coxa_deg"]))


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
    margin = gait["swing_phase_margin"]
    caps = []
    for name, duty, unstable in GAITS:
        # The cap is stride_length covered in one stance, so it keys off the
        # *realized* split (swing_end_phase in gaits/base.cpp), not the nominal
        # duty factor: the phase margin lengthens stance and lowers top speed.
        # Keep this identical to the other three copies of the formula —
        # pipeline_config_loader.cpp, hexa_common/limits.py, gen_joy_golden.py —
        # or the loader-vs-baked parity test fails.
        swing_end = (1.0 - duty) * (1.0 - min(max(margin, 0.0), 0.4))
        linear_max = stride * swing_end / (min_swing * (1.0 - swing_end))
        # yaw_bias stays keyed to the gait's nominal duty: it is a feel knob for
        # how a gait gives way under a saturating command, not a timing budget.
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


def hardware_joints(hw: dict, calibration: dict, limits: dict):
    """18 servo calibrations in canonical joint order — port of joint_calibration.cpp.

    The row order is LEG_NAMES x JOINT_TYPES, i.e. the same order as the
    pipeline's ``theta[18]``, so ``kJointCals[i]`` is the calibration for
    ``theta[i]``. Each row carries its own ``pin``, so the wiring is data rather
    than position — the firmware sorts by pin itself when it needs the harness
    order (SET run-grouping, the per-leg energize sweep).
    """
    dac = hw.get("deg_at_center", {})
    deg_at_center = {"coxa": dac.get("coxa", 0.0), "femur": dac.get("femur", 0.0),
                     "tibia": dac.get("tibia", 0.0)}
    endpoints = calibration_by_pin(calibration)

    def urdf_center(pos: str) -> float:
        rad = math.radians(deg_at_center[pos])
        return {"coxa": rad, "femur": -rad, "tibia": math.pi - rad}[pos]

    # A servo whose center sits outside the joint's travel window cannot reach
    # half its range. deg_at_center is per-segment, so check the three once
    # rather than per joint row. Both sides are URDF radians here.
    for pos in JOINT_TYPES:
        center = urdf_center(pos)
        lower, upper = limits[pos]["lower"], limits[pos]["upper"]
        if not (lower <= center <= upper):
            raise ValueError(
                f"hardware.yaml deg_at_center.{pos} = {deg_at_center[pos]} deg "
                f"({center:.4f} rad) is outside the geometry.yaml limit window "
                f"[{lower:.4f}, {upper:.4f}] rad")

    # Authoritative name→segment map for the fixed 6-leg set (mirrors
    # joint_calibration.cpp's kPositions). An unknown joint name is rejected.
    joint_positions = {
        f"{side}_{leg}_{seg}_joint": seg
        for side in ("l", "r")
        for leg in ("front", "middle", "rear")
        for seg in ("coxa", "femur", "tibia")
    }

    # Shared electrical clamp (servo_defaults.pulse_us); per-servo `pulse_us`
    # overrides it. Mirrors joint_calibration.cpp's PulseClamp default.
    dflt_pulse = hw.get("servo_defaults", {}).get("pulse_us", {})
    dflt_min, dflt_max = dflt_pulse.get("min", 500), dflt_pulse.get("max", 2500)

    rows = []
    for name, j in hw["servos"].items():
        if name not in joint_positions:
            raise ValueError(f"unknown joint name '{name}'")
        pos = joint_positions[name]
        pin = j["pin"]
        if pin not in endpoints:
            raise ValueError(
                f"joint '{name}' pin {pin} has no servo_calibration.yaml entry")
        us_plus, us_minus = endpoints[pin]
        pulse = j.get("pulse_us", {})
        rows.append(dict(
            name=name, pin=pin, joint_position=pos,
            us_at_plus_45=us_plus, us_at_minus_45=us_minus,
            urdf_rad_at_center=urdf_center(pos),
            direction=-1 if j.get("reversed", False) else 1,
            min_us=pulse.get("min", dflt_min), max_us=pulse.get("max", dflt_max)))
    canonical = [f"{leg}_{seg}_joint" for leg in LEG_NAMES for seg in JOINT_TYPES]
    missing = [n for n in canonical if n not in {r["name"] for r in rows}]
    if missing:
        raise ValueError(f"hardware.yaml servos is missing {', '.join(missing)}")
    order = {name: i for i, name in enumerate(canonical)}
    rows.sort(key=lambda r: order[r["name"]])
    return rows


# ── header emission ─────────────────────────────────────────────────────────

def emit(geometry, gait, teleop, posture, control, hardware, calibration,
         webteleop, display, sources) -> str:
    specs = leg_specs(geometry)
    limits = joint_limits(geometry)
    stand = standing_pose(gait, geometry)
    initial = initial_pose(geometry)
    caps = velocity_caps(gait)
    joints = hardware_joints(hardware, calibration, limits)

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
    w("// ── Per-joint-type travel limits, IK-convention radians ──")
    w("struct JointLimits {")
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
        w(f"    {{{fl(m['lower'])}, {fl(m['upper'])}, "
          f"{fl(m['effort'])}, {fl(m['velocity'])}}},  // {jt}")
    w("}};")
    w("")

    # ── standing / initial pose ──
    w("// ── Rest / startup pose ──")
    w("// The standing pose is described by where the feet sit, not by joint")
    w("// angles: gait::standing_pose_from solves the per-leg triple from these")
    w("// (femur/tibia uniform, coxa mirrored by the corner splay).")
    w("struct StandingPose {")
    w("  float tip_radius;       // m, coxa axis -> foot tip, in the leg frame")
    w("  float body_height;      // m, body bottom -> ground")
    w("  float corner_leg_coxa;  // rad, front-left; mirrored fr/lr, middles 0")
    w("};")
    w("")
    w(f"inline constexpr StandingPose kStandingPose = "
      f"{{{fl(stand['tip_radius'])}, {fl(stand['body_height'])}, "
      f"{fl(stand['corner_leg_coxa'])}}};")
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
        ("swing_apex_fraction", gait["swing_apex_fraction"]),
        ("touchdown_velocity", gait["touchdown_velocity"]),
        ("touchdown_probe_height", gait["touchdown_probe_height"]),
        ("liftoff_velocity", gait["liftoff_velocity"]),
        ("swing_phase_margin", gait["swing_phase_margin"]),
        ("controller_dt", gait["controller_dt"]),
        ("cmd_zero_tol", gait["cmd_zero_tol"]),
        ("settle_debounce_delay", gait["settle"]["debounce_delay"]),
        ("settle_swing_time", gait["settle"]["swing_time"]),
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
    # No height_{max,min}_m here: the body-height envelope is declared once, in
    # tuning.yaml's posture_node block (body_height_{max,min}_m), and joy_mapping
    # derives its integrator saturation from that. Only the rate — a teleop feel
    # knob, not a limit — belongs to teleop_joy.yaml.
    w("// Posture-mode scalar limits (teleop_joy.yaml posture:).")
    w("struct PostureLimits {")
    for f in ("x_max", "y_max", "roll_max_deg", "pitch_max_deg", "yaw_max_deg",
              "yaw_tau_s", "revert_tau_s", "wiggle_pivot_forward_m",
              "height_rate_m_per_s"):
        w(f"  float {f};")
    w("};")
    w("inline constexpr PostureLimits kPostureLimits = {"
      + ", ".join(fl(v) for v in (
          p["x_max"], p["y_max"], p["roll_max_deg"], p["pitch_max_deg"],
          p["yaw_max_deg"], p["yaw_tau_s"], p["revert_tau_s"],
          p["wiggle_pivot_forward_m"], ph["rate_m_per_s"])) + "};")
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
    # The nominal stance must sit strictly inside its own envelope, or the body
    # is clamped away from rest on the very first tick.
    if not (pn["body_height_min_m"] < stand["body_height"]
            < pn["body_height_max_m"]):
        raise ValueError(
            f"tuning.yaml posture_node body_height_min_m = "
            f"{pn['body_height_min_m']} m / body_height_max_m = "
            f"{pn['body_height_max_m']} m must bracket gait_node "
            f"standing_pose.body_height = {stand['body_height']} m")
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
        # Composed-pose clamp envelope. body_height_{max,min} are ABSOLUTE belly
        # clearance; nominal_body_height is the same standing-pose height the
        # gait engine solves the stance from, carried here so PostureController
        # can turn the pair into the pose offsets BodyPose::z actually is.
        ("pose_limit_x", pn["pose_limit_x"]),
        ("pose_limit_y", pn["pose_limit_y"]),
        ("body_height_max", pn["body_height_max_m"]),
        ("body_height_min", pn["body_height_min_m"]),
        ("nominal_body_height", stand["body_height"]),
        ("pose_limit_roll", pn["pose_limit_roll"]),
        ("pose_limit_pitch", pn["pose_limit_pitch"]),
        ("pose_limit_yaw", pn["pose_limit_yaw"]),
    ]
    for fname, _ in posture_fields:
        w(f"  float {fname};")
    # Gait-active body-animation master switch — the one non-float field, so it
    # is emitted outside the float loop (kept last in the aggregate init).
    w("  bool gait_body_animations_enabled;")
    w("};")
    w("inline constexpr PostureConfig kPosture = {")
    for fname, val in posture_fields:
        w(f"    {fl(val)},  // {fname}")
    w(f"    {bo(pn['gait_body_animations_enabled'])},"
      "  // gait_body_animations_enabled")
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
    w("// Table order IS the joint order of the pipeline's theta[18]: l_front,")
    w("// l_middle, l_rear, r_front, r_middle, r_rear, each {coxa, femur, tibia},")
    w("// so kJointCals[i] calibrates theta[i]. The wiring lives in each row's")
    w("// `pin` (hardware.yaml), NOT in the row order — sort by pin for the")
    w("// harness order (SET run-grouping, per-leg energize sweep).")
    w("inline constexpr std::array<JointCal, 18> kJointCals = {{")
    for j in joints:
        w(f"    {{{j['pin']}, {fl(j['us_at_plus_45'])}, {fl(j['us_at_minus_45'])}, "
          f"{fl(j['urdf_rad_at_center'])}, {j['direction']}, {j['min_us']}, {j['max_us']}}},"
          f"  // {j['name']}")
    w("}};")
    w("")
    # Chica command-index map + telemetry scales are fixed board/protocol
    # constants (protocol.md, hexapod-servo2040-driver), not configured in
    # hardware.yaml (the Servo2040 firmware owns the relay GPIO; voltage/current
    # arrive in fixed protocol units). They match the firmware's own constants in
    # pi-pico-firmware/src/servo_out.cpp, the ROS reference
    # hexa_hardware/servo2040_protocol.hpp, and the test_config.cpp expectations.
    # CURR (24) and VOLT (25) are consecutive; STATUS (27) is a read-only latched
    # over-current fault register; RELAY is SET on index 26.
    w("inline constexpr std::uint8_t kRelayPin = 26;"
      "  // SET index; high = servo rail energised")
    w("inline constexpr std::uint8_t kBatteryCurrentPin = 24;  // CURR")
    w("inline constexpr std::uint8_t kBatteryVoltagePin = 25;  // VOLT")
    w("inline constexpr std::uint8_t kStatusPin = 27;"
      "  // read-only latched over-current fault register")
    w("inline constexpr float kBatteryCurrentScale = 0.01f;"
      "  // centi-units: count -> A")
    w("inline constexpr float kBatteryVoltageScale = 0.01f;"
      "  // centi-units: count -> V")
    w("inline constexpr float kTripAmpsPerCount = 0.1f;"
      "  // STATUS trip current: count -> A")
    w("")

    # ── Integration / failsafe (part 09) ──
    # The RP2350 firmware fuses the ROS system's separate safety knobs into one
    # onboard supervisor, so their sources stay authoritative:
    #   - input_timeout_s     — hexa_webteleop safety watchdog (the gamepad has no
    #     ROS equivalent; the webteleop timeout is the canonical "stale input"
    #     value the firmware mirrors for its BT link),
    #   - get_period_ticks    — the firmware's own aux GET decimation (poll every
    #     Nth control tick). Firmware-only: hexa_hardware runs the same poll off
    #     its control cycle entirely, paced by parser.aux_period_ms in wall
    #     clock, because there a GET round trip inside the 200 Hz cycle overran
    #     the controller-manager budget,
    #   - battery thresholds  — hardware.yaml `battery:`, the undervoltage
    #     ladder's SSoT (0.0 disables a rung; shipped disabled — the divider
    #     scale is uncalibrated). hexa_display's battery_*_v params are the
    #     face's expression mapping, not the safety policy.
    w("// ── Integration / failsafe (part 09) ──")
    safety = (webteleop.get("safety", {}) or {})
    input_timeout_s = float(safety.get("input_timeout_s", 0.5))
    w(f"inline constexpr float kInputTimeoutS = {fl(input_timeout_s)};"
      "  // webteleop safety.input_timeout_s; stale BT input -> zero cmd_vel")
    get_period_ticks = int(hardware["parser"]["get_period_ticks"])
    w(f"inline constexpr int kGetPeriodTicks = {get_period_ticks};"
      "  // hardware.yaml parser.get_period_ticks; battery GET every Nth tick")
    # Inrush stagger at the relay OFF->ON edge. The board drives a servo only
    # once the host has SET it, so the host owns the energize order; bringing
    # the legs up one at a time keeps the combined inrush below the board's
    # over-current tiers. 0 opts out (every leg at once).
    sweep_ms = int((hardware.get("init", {}) or {}).get("sweep_leg_interval_ms", 150))
    if sweep_ms < 0:
        raise SystemExit("hardware.yaml init.sweep_leg_interval_ms must be >= 0")
    w(f"inline constexpr int kSweepLegIntervalMs = {sweep_ms};"
      "  // hardware.yaml init.sweep_leg_interval_ms; per-leg energize stagger")
    w("")
    # Ordered warning_v > fold_v > cutoff_v so a deeper rung implies the
    # shallower ones. A rung at 0.0 is disabled and skipped by the check.
    dp = display["display_node"]["ros__parameters"]
    batt = (hardware.get("battery", {}) or {})
    rungs = [float(batt.get(k, 0.0)) for k in ("warning_v", "fold_v", "cutoff_v")]
    for name, value in zip(("warning_v", "fold_v", "cutoff_v"), rungs):
        if value < 0.0:
            raise SystemExit(f"hardware.yaml battery.{name} must be >= 0")
    enabled = [(n, v) for n, v in zip(("warning_v", "fold_v", "cutoff_v"), rungs) if v > 0.0]
    for (hi_name, hi), (lo_name, lo) in zip(enabled, enabled[1:]):
        if lo >= hi:
            raise SystemExit(
                f"hardware.yaml battery.{lo_name} ({lo}) must be below "
                f"battery.{hi_name} ({hi}) — the ladder escalates downward")
    hysteresis_v = float(batt.get("hysteresis_v", 0.3))
    hold_s = float(batt.get("hold_s", 3.0))
    w("struct BatteryThresholds {  // undervoltage ladder (0 disables a rung)")
    w("  float warning_v;  // rung 1: beep, robot stays drivable")
    w("  float fold_v;     // rung 2: fold, then cut the rail at FOLDED")
    w("  float cutoff_v;   // rung 3: cut the rail now, latched off")
    w("  float hysteresis_v;")
    w("  float hold_s;")
    w("};")
    w("inline constexpr BatteryThresholds kBattery = {"
      + ", ".join(fl(v) for v in (*rungs, hysteresis_v, hold_s))
      + "};  // hardware.yaml battery:")
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
