#!/usr/bin/env python3
"""Generate a map_joy golden trace from the real Python reference (plan part 07).

Imports the untouched pure ``hexa_teleop.joy_mapping`` (no rclpy) straight from
the ROS2 source tree, builds a ``JoyConfig`` from the same YAMLs the firmware
bakes, and replays a scripted axes/buttons trace through ``map_joy``. Each frame
and its resulting ``JoyOutput`` is emitted into ``joy_golden_generated.hpp`` so
``test_joy_mapping.cpp`` can drive the float C++ port over identical inputs and
assert the same outputs (the part-07 host verification).

The trace is defined in raw Linux-joystick int16 axes (rest = 0, extremes =
±32767; the l2/r2 triggers rest = +32767) + a button bitmask, so both sides
scale the inputs identically (Python divides by 32767, the C++ port multiplies
by 1/32767). Divergence is then purely the double->float port of the mapping
math, checked under a loose tolerance in the C++ test.
"""

from __future__ import annotations

import argparse
import dataclasses
import os
import sys

import yaml

# Gait duty factors (mirror gen_config.GAITS / hexa_gait registry) — needed only
# to derive the default gait's linear cap for the stick scaling.
GAITS = [
    ("tripod", 0.5, False),
    ("surf", 5.0 / 8.0, True),
    ("tetrapod", 2.0 / 3.0, False),
    ("crawl", 2.0 / 3.0, True),
    ("ripple", 5.0 / 6.0, False),
]

# The four-corner rotation, kept separate because resolve_gait_cycle is handed
# the other leg set's names as foreign. The default gait is always a six-leg
# one, so no duty factor here feeds the stick caps.
QUAD_GAITS = [
    ("quad_walk", 3.0 / 4.0, False),
    ("quad_canter", 3.0 / 4.0, False),
]

DT = 0.02  # matches the firmware tick / teleop_joy PUBLISH_RATE_HZ

NEUTRAL = [0, 0, 32767, 0, 0, 32767, 0, 0]  # l2 (2) / r2 (5) rest = +max


def frame(**changes):
    """Build one (axes, buttons) frame from neutral with named overrides.

    Axis keys: lx ly l2 rx ry r2 dx dy (Joy.axes[0..7]); buttons is a bitmask.
    """
    axes = list(NEUTRAL)
    key_idx = {"lx": 0, "ly": 1, "l2": 2, "rx": 3, "ry": 4, "r2": 5,
               "dx": 6, "dy": 7}
    for k, i in key_idx.items():
        if k in changes:
            axes[i] = int(changes[k])
    return axes, int(changes.get("buttons", 0))


def build_trace():
    """A scripted trace exercising drive, cyclers, posture, init/revert, anim."""
    f = []
    # ── quadruped: the stand, its own gait rotation, and posture on four feet ──
    # First, while nothing is recorded and no height integrated: an init off a
    # modified posture arms the revert instead of standing, so anywhere later in
    # this trace `select` would never reach the leg set at all.
    f.append(frame(buttons=1 << 6))                    # select: the quad stand
    f.append(frame())
    f.append(frame(dx=-32767))                         # gait_next: the QUAD cycle
    f.append(frame())
    f.append(frame(dx=32767))                          # gait_prev: back again
    f.append(frame())
    f.append(frame(dx=32767))                          # and past the wrap
    f.append(frame())
    f.append(frame(buttons=1 << 3))                    # to POSTURE, still quad
    f.append(frame(lx=22000, ry=-12000))               # live tilt + pose
    f.append(frame(lx=22000, ry=-12000, buttons=1 << 6))  # record: inert here
    f.append(frame(lx=22000, ry=-12000))               # release the button
    f.append(frame())                                  # release the sticks
    f.append(frame(buttons=1 << 0))                    # to GAIT: nothing rode in
    f.append(frame())
    f.append(frame(buttons=1 << 7))                    # start: back to six legs
    f.append(frame())
    # ── GAIT mode (initial) ──
    f.append(frame())                                  # idle
    f.append(frame(ry=-32767))                         # drive forward
    f.append(frame(ry=-32767, rx=16000))               # + strafe
    f.append(frame(lx=-20000))                         # drive yaw
    f.append(frame())                                  # release
    # ── the two forward sources and the envelope fit ──
    f.append(frame(ly=-32767))                         # forward on the yaw stick
    f.append(frame(ly=-20000, ry=-20000))              # both sources, summing
    f.append(frame(ly=-32767, ry=-32767))              # both full, sum clipped
    f.append(frame(ly=-32767, ry=16000))               # opposing, subtracting
    f.append(frame(ry=-32767, rx=-32767))              # full diagonal
    f.append(frame(ly=-32767, lx=-32767))              # full forward + full yaw
    f.append(frame(ry=-32767, rx=-32767, lx=32767))    # all three saturated
    f.append(frame(ry=-3600))                          # just past the deadband
    f.append(frame())                                  # release
    f.append(frame(dx=-32767))                         # gait_next (rising)
    f.append(frame())                                  # release
    f.append(frame(dx=32767))                          # gait_prev (rising)
    f.append(frame())
    f.append(frame(dy=-32767))                         # height_up
    f.append(frame(dy=-32767))                         # hold, integrate
    f.append(frame(dy=-32767))
    # ── to POSTURE (button y = idx 3) ──
    f.append(frame(buttons=1 << 3))
    f.append(frame(lx=22000, ly=-15000, rx=10000, ry=-12000))  # tilt + pose
    f.append(frame(buttons=1 << 4))                    # yaw_left (l1), ease
    f.append(frame(buttons=1 << 4))
    f.append(frame(l2=-32767))                         # wiggle_left (l2 trigger)
    f.append(frame(l2=-32767))
    f.append(frame(buttons=1 << 6))                    # record (select)
    f.append(frame())                                  # settle
    # ── init with modified posture -> revert decay ──
    f.append(frame(buttons=1 << 7))                    # init (start) -> reverting
    f.append(frame())
    f.append(frame())
    f.append(frame())
    # ── back to GAIT (button a = idx 0) ──
    f.append(frame(buttons=1 << 0))
    f.append(frame(ry=-30000))                         # drive forward again
    # ── ANIMATION (button b = idx 1) ──
    f.append(frame(buttons=1 << 1))                    # enter ANIMATION
    f.append(frame(dx=-32767))                         # animation_next (rising)
    f.append(frame())
    f.append(frame(dx=-32767))                         # animation_next again
    f.append(frame())
    f.append(frame(buttons=1 << 1))                    # toggle back to GAIT
    f.append(frame())
    return f


def fl(x) -> str:
    return repr(float(x)) + "f"


def cstr(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def emit(frames, expected, caps) -> str:
    L = []
    w = L.append
    w("// AUTO-GENERATED by test/host/gen_joy_golden.py — DO NOT EDIT.")
    w("// map_joy golden trace from the Python hexa_teleop reference (part 07).")
    w("#pragma once")
    w("")
    w("#include <cstdint>")
    w("#include <string_view>")
    w("")
    w("namespace joy_golden {")
    w("")
    w("// Stick full-scale caps the reference ran with, so the C++ test seeds")
    w("// JoyConfig from the same numbers instead of re-deriving them. Both are")
    w("// the default gait's, and the angular one is the linear one over the")
    w("// outermost standing foot's planar radius.")
    w(f"inline constexpr float kGaitLinearMax = {fl(caps[0])};")
    w(f"inline constexpr float kGaitAngularZMax = {fl(caps[1])};")
    w("")
    w("struct Frame {")
    w("  std::int16_t axes[8];")
    w("  std::uint32_t buttons;")
    w("};")
    w("")
    w("struct Expected {")
    w("  float linear_x, linear_y, angular_z;")
    w("  float pose_x, pose_y, pose_z, pose_yaw, pose_roll, pose_pitch;")
    w("  bool mode_changed, init_request;")
    w("  bool has_gait_select;")
    w("  std::string_view gait_select;")
    w("  bool has_animation_name;")
    w("  std::string_view animation_name;")
    w("  bool init_quadruped;")
    w("};")
    w("")
    w(f"inline constexpr Frame kFrames[] = {{")
    for axes, buttons in frames:
        w("    {{" + ", ".join(str(a) for a in axes) + f"}}, {buttons}u}},")
    w("};")
    w("")
    w(f"inline constexpr Expected kExpected[] = {{")
    for o in expected:
        gs = cstr(o.gait_select) if o.gait_select is not None else '""'
        an = cstr(o.animation_name) if o.animation_name is not None else '""'
        w("    {"
          + ", ".join(fl(v) for v in (o.linear_x, o.linear_y, o.angular_z,
                                      o.pose_x, o.pose_y, o.pose_z, o.pose_yaw,
                                      o.pose_roll, o.pose_pitch))
          + f", {'true' if o.mode_changed else 'false'}"
          + f", {'true' if o.init_request else 'false'}"
          + f", {'true' if o.gait_select is not None else 'false'}, {gs}"
          + f", {'true' if o.animation_name is not None else 'false'}, {an}"
          + f", {'true' if o.init_quadruped else 'false'}}},")
    w("};")
    w("")
    w("inline constexpr int kNumFrames = "
      f"static_cast<int>(sizeof(kFrames) / sizeof(kFrames[0]));")
    w("")
    w("}  // namespace joy_golden")
    w("")
    return "\n".join(L)


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))  # shared/motion_core/test
    shared_dir = os.path.dirname(os.path.dirname(here))  # shared/
    default_repo = os.path.dirname(shared_dir)           # workspace root

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo-root", default=default_repo)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    src = os.path.join(args.repo_root, "src")
    # Import the pure reference module directly (no package __init__, no rclpy).
    sys.path.insert(0, os.path.join(src, "hexa_teleop", "hexa_teleop"))
    import joy_mapping as jm  # noqa: E402

    # Reuse the shared caps helper rather than re-deriving the stance geometry
    # here; it is the same module hexa_teleop loads at runtime.
    sys.path.insert(0, os.path.join(src, "hexa_common"))
    from hexa_common.limits import (  # noqa: E402
        outer_stance_radius,
        unit_stance_xy,
    )
    from hexa_common.teleop_scalar_limits import (  # noqa: E402
        load_posture_scalar_limits,
    )

    teleop = yaml.safe_load(open(f"{src}/hexa_teleop/config/teleop_joy.yaml"))
    # gait + posture knobs come from hexa_description's tuning.yaml (single
    # source of truth). It is a ros2 params file; unwrap gait to its flat block.
    tuning_path = f"{src}/hexa_description/config/tuning.yaml"
    geometry_path = f"{src}/hexa_description/config/geometry.yaml"
    tuning = yaml.safe_load(open(tuning_path))
    gait = tuning["gait_node"]["ros__parameters"]
    posture = tuning

    base_raw = teleop["base"]
    base = jm.BaseConfig(
        deadband=float(base_raw["deadband"]),
        trigger_threshold=float(base_raw["trigger_threshold"]),
        button_index={str(k): int(v) for k, v in base_raw["buttons"].items()},
        axis_index={str(k): int(v) for k, v in base_raw["axes"].items()},
        axis_sign={str(k): float(v)
                   for k, v in base_raw.get("axis_signs", {}).items()},
        bindings={str(k): str(v) for k, v in base_raw["bindings"].items()},
    )
    posture_raw = teleop["posture"]
    height = posture_raw["height"]
    # Absolute belly clearance in tuning.yaml, converted to the pose offsets
    # pose_z carries. Mirrors joy_mapping.cpp's posture_limits(); sourcing it
    # anywhere else would drift the golden from the C++ mapper.
    nominal_height = float(gait["default_standing_pose"]["body_height"])
    pn = posture["posture_node"]["ros__parameters"]
    posture_cfg = jm.PostureConfig(
        bindings={str(k): str(v) for k, v in posture_raw["bindings"].items()},
        # Scalar limits: tuning.yaml's teleop_node block, through the same
        # loader hexa_teleop runs.
        **dataclasses.asdict(load_posture_scalar_limits(tuning_path)),
        height_max=float(pn["body_height_max_m"]) - nominal_height,
        height_min=float(pn["body_height_min_m"]) - nominal_height,
        height_rate=float(height["rate_m_per_s"]),
    )

    hexapod_names = {n for n, _, _ in GAITS}
    quad_names = {n for n, _, _ in QUAD_GAITS}
    known = hexapod_names | quad_names
    unstable = {n for n, _, u in GAITS + QUAD_GAITS if u}
    allow_unstable = bool(teleop.get("allow_unstable_gaits", False))
    # map_joy still holds exactly two rotations, so the trace is generated from
    # the two presets the init buttons stand up on — the same two gen_config.py
    # bakes for the firmware. A third preset is a web-teleop affair and does not
    # reach this mapping.
    def preset_for(leg_set):
        for entry in teleop["presets"]["list"]:
            names = [str(g) for g in entry["gait_cycle"]]
            if names and (names[0] in quad_names) == (leg_set == "quadruped"):
                return entry
        raise ValueError(f"presets: none walks the {leg_set} leg set")

    hexapod_preset = preset_for("hexapod")
    quadruped_preset = preset_for("quadruped")
    gait_cycle = jm.resolve_gait_cycle(
        tuple(str(n) for n in hexapod_preset["gait_cycle"]), known, unstable,
        allow_unstable, foreign_gaits=quad_names)
    quadruped_gait_cycle = jm.resolve_gait_cycle(
        tuple(str(n) for n in quadruped_preset["gait_cycle"]), known, unstable,
        allow_unstable, foreign_gaits=hexapod_names,
        key="quadruped_gait_cycle")
    default_quadruped_gait = str(quadruped_preset["default_gait"])
    default_gait = str(hexapod_preset["default_gait"])
    duty = {n: d for n, d, _ in GAITS}[default_gait]
    stride = float(gait["stride_length"])
    min_swing = float(gait["min_swing_time"])
    # The hexapod margin unconditionally: the quadruped gaits are deliberately
    # kept out of GAITS above, so the default gait is always a six-leg one and
    # the quadruped margin can never be the one in play here.
    margin = float(gait["swing_phase_margin"])
    # Realized swing/stance split, not the nominal duty factor — same formula as
    # gen_config.py velocity_caps(), pipeline_config_loader.cpp and limits.py.
    swing_end = (1.0 - duty) * (1.0 - min(max(margin, 0.0), 0.4))
    gait_linear_max = stride * swing_end / (min_swing * (1.0 - swing_end))
    # The angular stick cap is the linear one over the yaw lever arm — the
    # outermost standing foot's planar radius. No YAML knob.
    gait_angular_z_max = gait_linear_max / outer_stance_radius(
        geometry_path, tuning_path)
    animation_list = tuple(
        str(a) for a in
        posture["posture_node"]["ros__parameters"]["animation_mode_animations"])

    cfg = jm.JoyConfig(
        base=base,
        gait=jm.ModeConfig(
            bindings={str(k): str(v)
                      for k, v in teleop["gait"]["bindings"].items()}),
        posture=posture_cfg,
        animation=jm.ModeConfig(
            bindings={str(k): str(v)
                      for k, v in teleop["animation"]["bindings"].items()}),
        gait_cycle=gait_cycle,
        quadruped_gait_cycle=quadruped_gait_cycle,
        default_quadruped_gait=default_quadruped_gait,
        gait_linear_max=gait_linear_max,
        gait_angular_z_max=gait_angular_z_max,
        stance_unit=unit_stance_xy(geometry_path, tuning_path),
        animation_list=animation_list,
    )

    initial_mode = str(teleop.get("initial_mode", jm.GAIT))
    state = jm.JoyState(mode=initial_mode)
    state.current_gait_idx = gait_cycle.index(default_gait)
    state.current_quadruped_gait_idx = quadruped_gait_cycle.index(
        default_quadruped_gait)

    frames = build_trace()
    expected = []
    for axes_i16, buttons in frames:
        axes_f = [a / 32767.0 for a in axes_i16]
        btn = [(buttons >> i) & 1 for i in range(8)]
        expected.append(jm.map_joy(axes_f, btn, cfg, state, DT))

    header = emit(frames, expected,
                  (gait_linear_max, gait_angular_z_max))
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    old = None
    if os.path.isfile(args.out):
        old = open(args.out).read()
    if old != header:
        open(args.out, "w").write(header)
        sys.stderr.write(f"gen_joy_golden.py: wrote {args.out}\n")
    else:
        sys.stderr.write(f"gen_joy_golden.py: {args.out} up to date\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
