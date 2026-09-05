#!/usr/bin/env python3
"""Bake the repo's runtime YAML config into a constexpr C++ header.

The Pico firmware has no filesystem, so it cannot load the YAMLs the ROS2 nodes
read at runtime. This emits ``config_generated.hpp`` from those same files, so
hexa_description stays the single source of truth. CMake runs it pre-build.

The transforms here are ports of the ROS2 loaders and MUST stay in lockstep:

  - symmetry expansion + deg->rad conventions mirror
    ``hexa_locomotion/src/pipeline_config_loader.cpp``,
  - per-gait linear_max / yaw_bias mirror ``gait/limits.cpp``, with the
    duty_factor / unstable table from ``gaits/registry.cpp``,
  - servo pulse calibration mirrors ``hexa_hardware/src/joint_calibration.cpp``.
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

# Lowercased mirror of kExprNames in
# shared/display_core/core/ExpressionController.cpp. Validated here only so a
# typo in display.yaml fails the bake instead of resolving to NEUTRAL at runtime;
# the name -> enum mapping itself happens in the firmware.
EXPRESSION_NAMES = {
    "neutral", "happy", "sleepy", "dead", "greedy", "woozy", "angry", "love",
    "scanning",
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

# The functions map_joy queries, in the fixed order that also defines the C++
# ``JoyFn`` enum. Every (mode, function) pair is pre-resolved to a JoyKeyRef so
# the firmware needs no runtime string handling; mode bindings win over base.
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
    ("kQuadrupedMode", "quadruped_mode"),
    ("kDriveX", "drive_x"),
    ("kDriveXAux", "drive_x_aux"),
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
    # name,             duty_factor, unstable, leg_set
    ("tripod",          0.5,         False,    "hexapod"),
    ("surf",            5.0 / 8.0,   True,     "hexapod"),
    ("tetrapod",        2.0 / 3.0,   False,    "hexapod"),
    ("crawl",           2.0 / 3.0,   True,     "hexapod"),
    ("ripple",          5.0 / 6.0,   False,    "hexapod"),
    # Quadruped leg set — reachable only through the teleop select toggle, and
    # rotated among themselves by quadruped_gait_cycle, never by gait_cycle. The
    # leg set is carried here (as in the Python catalog) because the swing phase
    # margin, and so the derived velocity cap, is per leg set; the strategy class
    # stays its source of truth for the engine.
    ("quad_walk",       3.0 / 4.0,   False,    "quadruped"),
    ("quad_canter",     3.0 / 4.0,   False,    "quadruped"),
]

# NUL-padded width of GaitSpec::name. Must exceed the longest name above.
GAIT_NAME_LEN = 16


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


LEG_GROUPS = ("front", "middle", "rear")
# Quadruped mode stands on the corners alone, so its stance block has no middle.
QUAD_GROUPS = ("front", "rear")


def group_splay(group: str, side: str, coxa_deg: float) -> float:
    """Left-leg coxa_deg -> this leg's own splay, in radians.

    The YAML value is the *left* leg's, and positive means outward — away from
    the body's fore/aft centreline. Turning that into a joint angle is two
    negations: rear legs face the other way down the body, and right legs
    mirror left ones. Keep in step with standing_pose_from in
    shared/motion_core/gait/engine.cpp, which applies exactly this rule.
    """
    sign = -1.0 if group == "rear" else 1.0
    if side == "r":
        sign = -sign
    return sign * math.radians(coxa_deg)


def standing_pose(gait: dict, geometry: dict,
                  key: str = "default_standing_pose",
                  groups: tuple = LEG_GROUPS) -> dict:
    """At-rest stance from one of tuning.yaml's gait_node standing-pose blocks.

    `key` is "default_standing_pose" (the six-leg stance) or
    "quad_standing_pose" (quadruped mode's four corners, whose middle pair does
    not stand at all — pass groups=("front", "rear") for it).

    The stance is described by where the feet sit, not by joint angles: one
    belly clearance for the body, plus a tip reach and a splay for each of the
    three leg pairs. The femur/tibia angles follow from a 2-link IK solve that
    motion_core owns (gait::standing_pose_from) — deliberately not duplicated
    here, so there is exactly one copy of that math.
    """
    sp = gait[key]
    body_height = sp["body_height"]

    # Reachability guard so a bad edit fails at build time rather than throwing
    # UnreachableTarget on the robot. The angles themselves are checked against
    # the joint limits by standing_pose_from when the config loads.
    leg = geometry["leg"]
    coxa_len, femur_len, tibia_len = (
        leg["coxa_length"], leg["femur_length"], leg["tibia_length"])
    # IK targets the foot sphere's centre, which sits one radius above the
    # ground contact — same subtraction as gait::standing_pose_from.
    depth = (geometry["body"]["coxa_to_bottom"] + body_height
             - geometry["foot"]["radius"])

    out_groups = []
    for group in groups:
        cfg = sp[group]
        tip_reach = cfg["tip_reach"]
        if tip_reach <= coxa_len:
            raise ValueError(
                f"tuning.yaml {key}.{group}.tip_reach = "
                f"{tip_reach} m must exceed the coxa length ({coxa_len} m)")
        reach = math.hypot(tip_reach - coxa_len, depth)
        if not (abs(femur_len - tibia_len) <= reach <= femur_len + tibia_len):
            raise ValueError(
                f"tuning.yaml {key}.{group}.tip_reach = "
                f"{tip_reach} m / body_height = {body_height} m puts the foot "
                f"{reach:.4f} m from the femur joint; reach annulus is "
                f"[{abs(femur_len - tibia_len):.4f}, "
                f"{femur_len + tibia_len:.4f}] m")
        out_groups.append(dict(tip_reach=tip_reach,
                               coxa=to_urdf_rad("coxa", cfg["coxa_deg"])))

    return dict(body_height=body_height, groups=out_groups)


def rest_pose(geometry: dict, key: str):
    """Per-leg (coxa, femur, tibia) angles for one of the two belly-rest poses.

    `key` is "folded_pose" (power-up, and where quadruped mode parks the middle
    pair) or "initialized_pose" (unfold endpoint); the two share a schema so
    this reads either.
    """
    init = geometry[key]
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


def unit_stance_xy(gait: dict, geometry: dict):
    """Standing feet in the body plane, over the outermost foot's radius.

    Port of hexa_common/limits.py's standing_stance_xy + unit_stance_xy: the
    mount symmetry expansion, then the tip laid its group's tip_reach out from
    the coxa axis at the leg's splay. Teleop's fit_drive_to_envelope bounds the
    implied per-leg foot speed in stick units rather than m/s, and normalising by
    r_outer is what cancels the caps out of that bound — every gait's angular
    cap is its linear cap over exactly this radius, so one table covers them
    all. Keep in step with the Python original or the golden trace diverges.
    """
    sp = gait["default_standing_pose"]
    mounts = geometry["mounts"]

    stance = []
    for side in ("l", "r"):
        for name in LEG_GROUPS:
            ref = mounts["l_middle" if name == "middle" else "l_front"]
            ref_yaw = math.radians(ref["yaw_deg"])
            x = -ref["x"] if name == "rear" else ref["x"]
            yaw = (math.pi - ref_yaw) if name == "rear" else ref_yaw
            y = -ref["y"] if side == "r" else ref["y"]
            if side == "r":
                yaw = -yaw

            tip_reach = sp[name]["tip_reach"]
            splay = group_splay(name, side, sp[name]["coxa_deg"])

            stance.append((f"{side}_{name}",
                           x + tip_reach * math.cos(yaw + splay),
                           y + tip_reach * math.sin(yaw + splay)))

    r_outer = max(math.hypot(x, y) for _, x, y in stance)
    if r_outer <= 0.0:
        raise ValueError(
            "outer stance radius is zero — every foot sits on the body axis, "
            "so the standing pose has no yaw authority; check tuning.yaml "
            "default_standing_pose")
    return [(name, x / r_outer, y / r_outer) for name, x, y in stance]


def velocity_caps(gait: dict):
    """Per-gait linear_max + yaw_bias — port of load_velocity_caps()."""
    stride = gait["stride_length"]
    min_swing = gait["min_swing_time"]
    yaw_bias = gait["yaw_bias"]
    margins = {"hexapod": gait["swing_phase_margin"],
               "quadruped": gait["quadruped_swing_phase_margin"]}
    caps = []
    for name, duty, unstable, leg_set in GAITS:
        # stride_length covered in one stance, so it keys off the *realized*
        # split (swing_end_phase), not the nominal duty factor — and off the
        # margin the gait's own LEG SET walks on (swing_phase_margin_for). Keep
        # identical to the other three copies — pipeline_config_loader.cpp,
        # hexa_common/limits.py, gen_joy_golden.py.
        margin = margins[leg_set]
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
    quad_stand = standing_pose(gait, geometry, "quad_standing_pose",
                               QUAD_GROUPS)
    folded = rest_pose(geometry, "folded_pose")
    initialized = rest_pose(geometry, "initialized_pose")
    caps = velocity_caps(gait)
    stance_unit = unit_stance_xy(gait, geometry)
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
    w("// Radius of the spherical foot tip. IK targets the sphere's centre, so a")
    w("// ground-contact height is this much below the target it is solved from.")
    w(f"inline constexpr float kFootRadius = {fl(geometry['foot']['radius'])};"
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

    # ── standing / rest poses ──
    w("// ── Rest / startup pose ──")
    w("// The standing pose is described by where the feet sit, not by joint")
    w("// angles: gait::standing_pose_from solves the per-leg triple from these")
    w("// (femur/tibia uniform within a group, coxa mirrored onto each leg).")
    w("struct LegGroupStance {")
    w("  float tip_reach;  // m, coxa axis -> foot tip, planar, in the leg frame")
    w("  float coxa;       // rad, left leg; rear and right mirror it")
    w("};")
    w("")
    w("struct StandingPose {")
    w("  float body_height;  // m, body bottom -> ground")
    w("  // Indexed by LegGroup: front, middle, rear.")
    w("  std::array<LegGroupStance, kNumLegGroups> groups;")
    w("};")
    w("")
    w("inline constexpr StandingPose kStandingPose = {")
    w(f"    {fl(stand['body_height'])},")
    w("    {{")
    for name, grp in zip(LEG_GROUPS, stand["groups"]):
        w(f"        {{{fl(grp['tip_reach'])}, {fl(grp['coxa'])}}},  // {name}")
    w("    }},")
    w("};")
    w("")
    w("// Quadruped mode's stance: the four corners on a symmetric rectangle.")
    w("// The middle pair is not part of it — it is parked at kFoldedPose — so")
    w("// the schema has no middle entry.")
    w("struct CornerStandingPose {")
    w("  float body_height;  // m, body bottom -> ground")
    w("  LegGroupStance front;")
    w("  LegGroupStance rear;")
    w("};")
    w("")
    w("inline constexpr CornerStandingPose kQuadStandingPose = {")
    w(f"    {fl(quad_stand['body_height'])},")
    for name, grp in zip(QUAD_GROUPS, quad_stand["groups"]):
        w(f"    {{{fl(grp['tip_reach'])}, {fl(grp['coxa'])}}},  // {name}")
    w("};")
    w("")
    w("// Per-leg power-up pose (coxa varies by symmetry), indexed by Leg. Also")
    w("// the pose a fold ends on, the baseline FOLDED/FAULT hold, and where")
    w("// quadruped mode parks the middle pair.")
    w("inline constexpr std::array<JointAngles, kNumLegs> kFoldedPose = {{")
    for leg in LEG_NAMES:
        a = folded[leg]
        w(f"    {{{fl(a[0])}, {fl(a[1])}, {fl(a[2])}}},  // {leg}")
    w("}};")
    w("")
    w("// Per-leg pose the unfold ladder reaches before the reseat ladder stands")
    w("// the robot up. Same schema, still belly-resting.")
    w("inline constexpr std::array<JointAngles, kNumLegs> kInitializedPose = {{")
    for leg in LEG_NAMES:
        a = initialized[leg]
        w(f"    {{{fl(a[0])}, {fl(a[1])}, {fl(a[2])}}},  // {leg}")
    w("}};")
    w("")

    # ── gait engine ──
    w("// ── Gait engine knobs (hexa_description/config/tuning.yaml) ──")
    w("struct EngineConfig {")
    fields = [
        ("stride_length", gait["stride_length"]),
        ("stride_length_radial", gait["stride_length_radial"]),
        ("min_swing_time", gait["min_swing_time"]),
        ("max_swing_time", gait["max_swing_time"]),
        ("step_height", gait["step_height"]),
        ("swing_width", gait["swing_width"]),
        ("touchdown_velocity", gait["touchdown_velocity"]),
        ("touchdown_probe_fraction", gait["touchdown_probe_fraction"]),
        ("swing_phase_margin", gait["swing_phase_margin"]),
        ("quadruped_swing_phase_margin",
         gait["quadruped_swing_phase_margin"]),
        ("controller_dt", gait["controller_dt"]),
        ("cmd_zero_tol", gait["cmd_zero_tol"]),
        ("settle_debounce_delay", gait["settle"]["debounce_delay"]),
        ("settle_swing_time", gait["settle"]["swing_time"]),
        ("init_unfold_time", gait["initialize"]["unfold_time"]),
        ("init_pair_swing_time", gait["initialize"]["pair_swing_time"]),
        ("init_lift_body_time", gait["initialize"]["lift_body_time"]),
        ("init_place_clearance", gait["initialize"]["place_clearance"]),
        ("init_swing_clearance", gait["initialize"]["swing_clearance"]),
        ("reseat_pose_settle_delay", gait["reseat"]["pose_settle_delay"]),
        ("reseat_height_change_threshold", gait["reseat"]["height_change_threshold"]),
        ("reseat_pair_swing_time", gait["reseat"]["pair_swing_time"]),
        ("reseat_pair_dwell_time", gait["reseat"]["pair_dwell_time"]),
        ("reseat_swing_clearance", gait["reseat"]["swing_clearance"]),
        ("quadruped_shift_time", gait["quadruped"]["shift_time"]),
        ("pair_fold_swing_time", gait["pair_fold"]["swing_time"]),
        ("pair_fold_dwell_time", gait["pair_fold"]["dwell_time"]),
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
    w(f"  std::array<char, {GAIT_NAME_LEN}> name;"
      "  // NUL-padded; use gait_name() to compare")
    w("  float duty_factor;")
    w("  bool unstable;")
    w("  float linear_max;  // m/s")
    w("  float yaw_bias;    // effective, per-gait")
    w("};")
    w("")

    def name_arr(name: str) -> str:
        chars = list(name.encode())
        if len(chars) >= GAIT_NAME_LEN:
            raise ValueError(
                f"gait name {name!r} needs {len(chars) + 1} bytes; GAIT_NAME_LEN "
                f"is {GAIT_NAME_LEN} — widen it and regenerate")
        chars += [0] * (GAIT_NAME_LEN - len(chars))
        return "{" + ", ".join(str(c) for c in chars) + "}"

    w("inline constexpr std::array<GaitSpec, "
      f"{len(caps)}> kGaits = {{{{")
    for c in caps:
        w(f"    {{{name_arr(c['name'])}, {fl(c['duty'])}, "
          f"{'true' if c['unstable'] else 'false'}, {fl(c['linear_max'])}, "
          f"{fl(c['yaw_bias'])}}},  // {c['name']}")
    w("}};")
    w("")

    w("// Standing feet over the outermost foot's radius — the unitless lever")
    w("// arms teleop's fit_drive_to_envelope bounds stick input against. Not")
    w("// per-gait: normalising by r_outer cancels the caps out of the per-leg")
    w("// foot-speed constraint, since every gait's angular cap is its linear")
    w("// cap over exactly that radius.")
    w("inline constexpr std::array<std::array<float, 2>, "
      f"{len(stance_unit)}> kStanceUnit = {{{{")
    for name, x, y in stance_unit:
        w(f"    {{{fl(x)}, {fl(y)}}},  // {name}")
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
    # Posture-mode scalar limits. Shared by both teleop front ends, so they are
    # declared once in tuning.yaml's teleop_node block, not per front end.
    p = posture["teleop_node"]["ros__parameters"]["posture"]
    # No height_{max,min}_m here: the envelope is declared once, in tuning.yaml's
    # posture_node block. Only the rate — a feel knob — belongs to teleop_joy.yaml.
    ph = teleop["posture"]["height"]
    # Same envelope check hexa_common.load_posture_scalar_limits makes, so the
    # firmware cannot bake a stick throw the posture stack would clamp away.
    envelope = posture["posture_node"]["ros__parameters"]
    for key, cap_key in (("x_max", "pose_limit_x"), ("y_max", "pose_limit_y"),
                         ("roll_max_deg", "pose_limit_roll"),
                         ("pitch_max_deg", "pose_limit_pitch"),
                         ("yaw_max_deg", "pose_limit_yaw")):
        value = math.radians(p[key]) if key.endswith("_deg") else p[key]
        if value > envelope[cap_key]:
            raise ValueError(
                f"tuning.yaml teleop_node posture.{key} = {p[key]} "
                f"({value:.4g}) reaches past posture_node {cap_key} = "
                f"{envelope[cap_key]}")
    w("// Posture-mode scalar limits (tuning.yaml teleop_node posture:).")
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
    # The firmware has one input path and no way to ask for a leg-set change,
    # so it needs only the two presets the init buttons stand up on — the
    # six-leg one the operator boots into, and the four-corner one `select`
    # takes. Presets beyond those two are a web-teleop affair; the pad cannot
    # reach them, so nothing is baked for them here.
    leg_sets_of = {name: ls for name, _d, _u, ls in GAITS}

    def preset_by_leg_set(leg_set):
        for entry in teleop["presets"]["list"]:
            names = [str(g) for g in entry["gait_cycle"]]
            if names and leg_sets_of[names[0]] == leg_set:
                return entry
        raise ValueError(
            f"presets: no preset walks the {leg_set} leg set; the init buttons "
            f"stand up on one of each")

    default_id = str(teleop["presets"]["default"])
    by_id = {str(e["id"]): e for e in teleop["presets"]["list"]}
    if default_id not in by_id:
        raise ValueError(f"presets.default: no preset {default_id!r}")
    hexapod_preset = preset_by_leg_set("hexapod")
    quadruped_preset = preset_by_leg_set("quadruped")
    w(f"inline constexpr std::string_view kDefaultGait = "
      f"{cstr(str(hexapod_preset['default_gait']))};")
    w(f"inline constexpr bool kAllowUnstableGaits = "
      f"{'true' if teleop['allow_unstable_gaits'] else 'false'};")
    # kGaitCycle is the runtime rotation the teleop cycler walks — already
    # filtered by allow_unstable_gaits (port of joy_mapping.resolve_gait_cycle),
    # so the firmware cycler matches the ROS node's accepted set. The quadruped
    # rotation is the second one, walked while the robot stands on four legs;
    # keeping them apart is what stops the cycler landing on a leg set the
    # operator did not ask for.
    unstable_names = {name for name, _, u, _ls in GAITS if u}
    leg_sets = leg_sets_of

    def resolve_cycle(raw, leg_set, key):
        """Port of joy_mapping.resolve_gait_cycle: validate, then filter."""
        names = [str(g) for g in raw]
        for g in names:
            if g not in leg_sets:
                raise ValueError(f"{key}: unknown gait {g!r}")
            if leg_sets[g] != leg_set:
                raise ValueError(
                    f"{key}: {g!r} walks the {leg_sets[g]} leg set, not "
                    f"{leg_set}")
        if teleop["allow_unstable_gaits"]:
            return names
        return [g for g in names if g not in unstable_names]

    cycle = resolve_cycle(hexapod_preset["gait_cycle"], "hexapod",
                          f"presets.{hexapod_preset['id']}.gait_cycle")
    quad_cycle = resolve_cycle(quadruped_preset["gait_cycle"], "quadruped",
                               f"presets.{quadruped_preset['id']}.gait_cycle")
    quad_default = str(quadruped_preset["default_gait"])
    if quad_default not in quad_cycle:
        raise ValueError(
            f"default_quadruped_gait: {quad_default!r} must be in "
            f"quadruped_gait_cycle={quad_cycle}")
    w(f"inline constexpr std::array<std::string_view, {len(cycle)}> kGaitCycle = {{"
      + ", ".join(cstr(g) for g in cycle) + "};")
    w(f"inline constexpr std::string_view kDefaultQuadrupedGait = "
      f"{cstr(quad_default)};")
    w(f"inline constexpr std::array<std::string_view, {len(quad_cycle)}> "
      f"kQuadrupedGaitCycle = {{" + ", ".join(cstr(g) for g in quad_cycle) + "};")
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
            f"default_standing_pose.body_height = {stand['body_height']} m")
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
        # SupportShift: gain on the lift-off-weighted stance centroid, the
        # lookahead in cycles, and that signal's own filter — taken in polar,
        # so a handover arcs instead of cornering.
        ("support_shift_gain", pn["support_shift_gain"]),
        ("support_shift_lead", pn["support_shift_lead"]),
        ("support_shift_tau", pn["support_shift_tau"]),
        # Gait-animation crossfade (posture layering fix).
        ("gait_activation_slew_rate", pn["gait_activation_slew_rate"]),
        # Spring/inertia smoother on the commanded body pose: tau = 1/omega_n,
        # damping_ratio = zeta, both frame-rate independent.
        ("pose_filter_tau", pn["pose_filter_tau"]),
        ("pose_filter_damping_ratio", pn["pose_filter_damping_ratio"]),
        ("pose_filter_snap_tol_linear", pn["pose_filter_snap_tol_linear"]),
        ("pose_filter_snap_tol_angular", pn["pose_filter_snap_tol_angular"]),
        # Composed-pose clamp envelope. body_height_{max,min} are ABSOLUTE belly
        # clearance; nominal_body_height is carried alongside so
        # PostureController can turn the pair into the offsets BodyPose::z is.
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
    # Fixed board/protocol constants (protocol.md), not hardware.yaml: CURR (24)
    # and VOLT (25) are consecutive, STATUS (27) is the read-only latched
    # over-current register, RELAY is SET on 26.
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

    # The firmware fuses the ROS system's separate safety knobs into one onboard
    # supervisor, so their sources stay authoritative:
    #   - input_timeout_s  — hexa_webteleop's watchdog, the canonical stale-input
    #     value the firmware mirrors for its BT link,
    #   - get_period_ticks — firmware-only aux GET decimation. hexa_hardware runs
    #     the same poll off its control cycle in wall clock, because there a GET
    #     round trip inside the 200 Hz cycle overran the manager's budget,
    #   - battery         — hardware.yaml `battery:`, the ladder's SSoT (0.0
    #     disables a rung; shipped disabled, the divider scale is uncalibrated).
    #     hexa_display's battery_*_v params are the face's mapping, not policy.
    w("// ── Integration / failsafe (part 09) ──")
    safety = (webteleop.get("safety", {}) or {})
    input_timeout_s = float(safety.get("input_timeout_s", 0.5))
    w(f"inline constexpr float kInputTimeoutS = {fl(input_timeout_s)};"
      "  // webteleop safety.input_timeout_s; stale BT input -> zero cmd_vel")
    get_period_ticks = int(hardware["parser"]["get_period_ticks"])
    w(f"inline constexpr int kGetPeriodTicks = {get_period_ticks};"
      "  // hardware.yaml parser.get_period_ticks; battery GET every Nth tick")
    # Inrush stagger at the relay OFF->ON edge: the board drives a servo only once
    # the host has SET it, so bringing the legs up one at a time keeps the
    # combined inrush below the over-current tiers. 0 opts out.
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

    # Panel readout only — the safety ladder above never consults these.
    empty_v = float(batt.get("empty_v", 6.6))
    full_v = float(batt.get("full_v", 8.4))
    if full_v <= empty_v:
        raise SystemExit(
            f"hardware.yaml battery.full_v ({full_v}) must be above "
            f"battery.empty_v ({empty_v}) — the percentage span would be empty")
    w("// Linear voltage -> percentage endpoints for the panel readout.")
    w(f"inline constexpr float kBatteryEmptyV = {fl(empty_v)};")
    w(f"inline constexpr float kBatteryFullV = {fl(full_v)};")
    w("")

    # ── Pico front-panel button (hardware.yaml pico_button:, firmware-only) ──
    btn = (hardware.get("pico_button", {}) or {})
    btn_pin = int(btn["pin"])
    btn_hold_s = float(btn["hold_s"])
    btn_debounce_s = float(btn["debounce_s"])
    btn_screen_s = float(btn["screen_s"])
    btn_pair_window_s = float(btn["pair_window_s"])
    for name, value in (("hold_s", btn_hold_s), ("debounce_s", btn_debounce_s),
                        ("screen_s", btn_screen_s),
                        ("pair_window_s", btn_pair_window_s)):
        if value <= 0.0:
            raise SystemExit(f"hardware.yaml pico_button.{name} must be > 0")
    if btn_debounce_s >= btn_hold_s:
        raise SystemExit(
            "hardware.yaml pico_button.debounce_s must be below hold_s — "
            "a press could never be held long enough to register")
    w("struct PicoButton {  // hardware.yaml pico_button:")
    w("  std::uint8_t pin;")
    w("  float hold_s;         // press below this = short press, at/above = hold")
    w("  float debounce_s;")
    w("  float screen_s;       // battery screen dwell before the face returns")
    w("  float pair_window_s;  // pairing stays open this long, or until a pad binds")
    w("};")
    w(f"inline constexpr PicoButton kPicoButton = {{{btn_pin}, "
      f"{fl(btn_hold_s)}, {fl(btn_debounce_s)}, {fl(btn_screen_s)}, "
      f"{fl(btn_pair_window_s)}}};")
    w("")

    # Face expression/gaze policy + SH1122 panel. Expression names resolve to the
    # enum at runtime, so this header stays free of the eye core.
    #
    # `pico_panel` keys are Pico-only. The ones with a display_node counterpart
    # (render_hz) are read from HERE with a hard subscript, so a missing key fails
    # the bake rather than falling back to the Pi's deliberately different value.
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
    w(f"inline constexpr float kFaceRenderHz = {fl(pico_panel['render_hz'])};"
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
    w(f"inline constexpr std::string_view kFaceScanningExpression = "
      f"{cstr(face_expr('scanning_expression'))};")
    w("")
    w("// Pose mode, per posture stick: tilt = left (roll/pitch), shift = right (x/y).")
    w(f"inline constexpr std::string_view kFacePostureTiltExpression = "
      f"{cstr(face_expr('posture_tilt_expression'))};")
    w(f"inline constexpr std::string_view kFacePostureShiftExpression = "
      f"{cstr(face_expr('posture_shift_expression'))};")
    w(f"inline constexpr std::string_view kFacePostureBothExpression = "
      f"{cstr(face_expr('posture_both_expression'))};")
    w("")
    w("struct FacePostureConfig {")
    posture_fields = [
        ("tilt_threshold_rad", dp["posture_tilt_threshold_rad"]),
        ("shift_threshold_m", dp["posture_shift_threshold_m"]),
        ("exit_ratio", dp["posture_exit_ratio"]),
    ]
    for fname, _ in posture_fields:
        w(f"  float {fname};")
    w("};")
    w("inline constexpr FacePostureConfig kFacePosture = {"
      + ", ".join(fl(v) for _, v in posture_fields) + "};")
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
      f"{int(pico_panel['spi_hz'])}u}};")
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
    # gait / control / posture all come from hexa_description's tuning.yaml, the
    # same file the ROS nodes read, so there is no overlay merge to mirror.
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
    # tuning.yaml is a ros2 params file: unwrap control and gait to their flat
    # knob blocks so the rest of this tool reads the keys the nodes expose as ros
    # params. posture stays wrapped and is unwrapped at its own use sites.
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
