#!/usr/bin/env python3
"""Generate a posture golden trace from the real Python reference (plan part 08).

Imports the untouched pure ``hexa_posture`` animation stack (``pose`` +
``animations`` — both rclpy-free) straight from the ROS2 source tree, builds the
same animation stacks the firmware bakes (from ``posture.yaml``), and replays a
scripted *recorded engine trace* (per-leg foot targets + stance flags, master
phase, walking flag, engine state, user pose, animation-mode selection) through
the full posture pipeline. Each frame's resulting ``BodyPose`` target is emitted
into ``posture_golden_generated.hpp`` so ``test_posture.cpp`` can drive the float
C++ ``PostureController`` over identical inputs and assert the same output — the
part-08 host verification called out in the plan.

The node's signal-derivation + low-pass math (``_stance_centroid_xy``,
``_max_swing_lift_z``, ``_lpf_step_*``, the POSTURE_ACTIVE_STATES gate) is
transcribed here rather than imported, because ``posture_node`` pulls in rclpy /
hexa_interfaces. The transcription is a line-for-line mirror of posture_node.py;
the C++ port of the *same* helpers is additionally checked against the literal
expected values from the Python node's own unit tests in test_posture.cpp, so a
transcription slip cannot pass silently on both sides.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import yaml

LEG_NAMES = ["l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear"]
MIN_STANCE_FOR_CENTROID = 3
POSTURE_ACTIVE_STATES = {
    "stand", "engaging", "gait", "pausing", "paused", "resuming", "reseating"
}
DT = 0.02  # matches the firmware tick / PUBLISH_RATE_HZ


# ── node signal pipeline (mirror of posture_node.py) ────────────────────────

def stance_centroid_xy(legs):
    xs, ys = [], []
    for name, (x, y, z, stance) in legs.items():
        if stance:
            xs.append(x)
            ys.append(y)
    if len(xs) < MIN_STANCE_FOR_CENTROID:
        return None
    n = float(len(xs))
    return (sum(xs) / n, sum(ys) / n)


def max_swing_lift_z(legs):
    stance_zs, swing_zs = [], []
    for name, (x, y, z, stance) in legs.items():
        (stance_zs if stance else swing_zs).append(z)
    if len(stance_zs) < MIN_STANCE_FOR_CENTROID:
        return None
    if not swing_zs:
        return 0.0
    ground = sum(stance_zs) / len(stance_zs)
    lift = max(swing_zs) - ground
    return lift if lift > 0.0 else 0.0


def lpf_step_xy(prev, raw, tau, dt):
    if raw is None:
        return prev
    if prev is None:
        return raw
    denom = tau + dt
    alpha = dt / denom if denom > 0.0 else 1.0
    px, py = prev
    rx, ry = raw
    return (px + alpha * (rx - px), py + alpha * (ry - py))


def lpf_step_scalar(prev, raw, tau, dt):
    if raw is None:
        return prev
    if prev is None:
        return raw
    denom = tau + dt
    alpha = dt / denom if denom > 0.0 else 1.0
    return prev + alpha * (raw - prev)


def slew_toward(current, target, rate_per_s, dt):
    if rate_per_s <= 0.0 or dt <= 0.0:
        return current
    step = rate_per_s * dt
    if target > current:
        return min(target, current + step)
    return max(target, current - step)


# ── trace ───────────────────────────────────────────────────────────────────

def leg(x, y, z, stance):
    return (float(x), float(y), float(z), bool(stance))


def build_trace():
    """Frames: (legs dict, master_phase, walking, state, gait_name, user_pose,
    animation_mode, dt). Exercises pre-stand gating, pose mode, a tripod walk
    (sway + bounce filters converging), and an animation-mode roll."""
    frames = []

    # Neutral stance polygon (feet at z=0, asymmetric so the centroid is
    # non-zero and GaitSway has something to track).
    def stance_all(z=0.0):
        return {
            "l_front": leg(0.20, 0.16, z, True),
            "l_middle": leg(0.00, 0.20, z, True),
            "l_rear": leg(-0.20, 0.16, z, True),
            "r_front": leg(0.20, -0.16, z, True),
            "r_middle": leg(0.00, -0.20, z, True),
            "r_rear": leg(-0.20, -0.16, z, True),
        }

    zero_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # ── pre-stand: expect IDENTITY regardless of input ──
    frames.append((stance_all(), 0.0, False, "folded", "tripod", zero_pose,
                   "", DT))
    frames.append((stance_all(), 0.0, False, "initialize", "tripod", zero_pose,
                   "", DT))

    # ── STAND / pose mode: translate + tilt, feet planted, not walking ──
    pose = (0.03, -0.02, 0.02, 0.05, -0.04, 0.10)
    for _ in range(4):
        frames.append((stance_all(), 0.0, False, "stand", "tripod", pose,
                       "", DT))
    # Height beyond the envelope to exercise the clamp (z_max 0.04).
    frames.append((stance_all(), 0.0, False, "stand", "tripod",
                   (0.0, 0.0, 0.20, 0.0, 0.0, 0.0), "", DT))

    # Tripod swing pattern: left triad swings on the first half-cycle, right on
    # the second. Foot z follows a simple sine arc.
    def tripod_legs(phi):
        arc = 0.06 * math.sin(math.pi * ((phi * 2.0) % 1.0))  # per half-cycle
        left_swing = phi < 0.5
        legs = {}
        for name in LEG_NAMES:
            base = stance_all()[name]
            bx, by = base[0], base[1]
            is_left = name.startswith("l_")
            swinging = (is_left and left_swing) or (
                (not is_left) and not left_swing)
            if swinging:
                legs[name] = leg(bx, by, arc, False)
            else:
                legs[name] = leg(bx, by, 0.0, True)
        return legs

    # ── GAIT / tripod walk: master phase advances. Feeds GaitSway (centroid) +
    # GaitBounce (swing lift), and ramps the activation crossfade IN (0 -> 1). ──
    n_walk = 24
    for i in range(n_walk):
        phi = (i / 8.0) % 1.0  # ~8 frames per cycle
        frames.append((tripod_legs(phi), phi, True, "gait", "tripod", zero_pose,
                       "", DT))

    # ── ANIMATION mode: vertical_body_roll (still + roll), keep walking so the
    # phase-locked roll is live. ──
    for i in range(8):
        phi = (i / 8.0) % 1.0
        frames.append((stance_all(), phi, True, "gait", "tripod", zero_pose,
                       "vertical_body_roll", DT))

    # ── STAND tail: stop walking while still in vertical_body_roll mode. The
    # phase-locked roll is live only for walking=True, so the activation crossfade
    # fades it OUT (1 -> 0) against the idle stack — exercises the fade-out path.
    for i in range(10):
        phi = (i / 8.0) % 1.0
        frames.append((stance_all(), phi, False, "stand", "tripod", zero_pose,
                       "vertical_body_roll", DT))

    # ── WALK with a dialed-in posture near the envelope: default stack, walking
    # again (activation ramps back IN), user pose beyond the reserved user
    # envelope on several axes. Exercises compose_layered's per-budget split — the
    # old clamp(add(user, animated)) would clip the animation asymmetrically. ──
    posed = (0.045, -0.045, 0.05, 0.25, -0.25, 0.4)
    for i in range(12):
        phi = (i / 8.0) % 1.0
        frames.append((tripod_legs(phi), phi, True, "gait", "tripod", posed,
                       "", DT))

    # ── back to default stack ──
    frames.append((stance_all(), 0.0, True, "gait", "tripod", zero_pose, "", DT))
    return frames


# ── formatting ────────────────────────────────────────────────────────────

def fl(x):
    return repr(float(x)) + "f"


def cstr(s):
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def emit(frames, expected):
    L = []
    w = L.append
    w("// AUTO-GENERATED by test/host/gen_posture_golden.py — DO NOT EDIT.")
    w("// Posture golden trace from the Python hexa_posture reference (part 08).")
    w("#pragma once")
    w("")
    w("#include <cstdint>")
    w("#include <string_view>")
    w("")
    w("namespace posture_golden {")
    w("")
    w("struct LegSample { float x, y, z; bool stance; };  // LEG_NAMES order")
    w("")
    w("struct Frame {")
    w("  LegSample legs[6];")
    w("  float master_phase;")
    w("  bool walking;")
    w("  std::string_view state;")
    w("  std::string_view gait_name;")
    w("  float user_pose[6];  // x, y, z, roll, pitch, yaw")
    w("  std::string_view animation_mode;  // \"\" = default stack")
    w("  float dt;")
    w("};")
    w("")
    w("struct Expected { float x, y, z, roll, pitch, yaw; };")
    w("")
    w("inline constexpr Frame kFrames[] = {")
    for legs, mp, walking, state, gait_name, pose, mode, dt in frames:
        leg_lits = []
        for name in LEG_NAMES:
            x, y, z, st = legs[name]
            leg_lits.append("{" + f"{fl(x)}, {fl(y)}, {fl(z)}, "
                            f"{'true' if st else 'false'}" + "}")
        pose_lit = ", ".join(fl(v) for v in pose)
        w("    {{" + ", ".join(leg_lits) + "}, "
          + f"{fl(mp)}, {'true' if walking else 'false'}, {cstr(state)}, "
          + f"{cstr(gait_name)}, {{{pose_lit}}}, {cstr(mode)}, {fl(dt)}}},")
    w("};")
    w("")
    w("inline constexpr Expected kExpected[] = {")
    for e in expected:
        w("    {" + ", ".join(fl(v) for v in e) + "},")
    w("};")
    w("")
    w("inline constexpr int kNumFrames = "
      "static_cast<int>(sizeof(kFrames) / sizeof(kFrames[0]));")
    w("")
    w("}  // namespace posture_golden")
    w("")
    return "\n".join(L)


# ── stack building (mirror of posture_node._build_animation_stack) ──────────

def build_stacks(pose_mod, anim, pn):
    factories = {
        "still": lambda: anim.Still(),
        "breathing": lambda: anim.Breathing(),
        "gait_sway": lambda: anim.GaitSway(
            gain=pn["gait_sway_gain"], strength=pn["gait_sway_strength"]),
        "gait_bounce": lambda: anim.GaitBounce(
            arc_height=pn["gait_bounce_arc_height"],
            step_height_ref=pn["gait_bounce_step_height_ref"]),
        "vertical_body_roll": lambda: anim.VerticalBodyRoll(
            z_amplitude=pn["vertical_body_roll_z_amplitude"],
            pitch_amplitude=math.radians(
                pn["vertical_body_roll_pitch_amplitude_deg"]),
            pitch_phase_offset=pn["vertical_body_roll_phase_offset"]),
        "horizontal_body_roll": lambda: anim.HorizontalBodyRoll(
            y_amplitude=pn["horizontal_body_roll_y_amplitude"],
            yaw_amplitude=math.radians(
                pn["horizontal_body_roll_yaw_amplitude_deg"]),
            yaw_phase_offset=pn["horizontal_body_roll_phase_offset"]),
        "body_roll_3d": lambda: anim.BodyRoll3D(
            z_amplitude=pn["body_roll_3d_z_amplitude"],
            pitch_amplitude=math.radians(pn["body_roll_3d_pitch_amplitude_deg"]),
            y_amplitude=pn["body_roll_3d_y_amplitude"],
            yaw_amplitude=math.radians(pn["body_roll_3d_yaw_amplitude_deg"]),
            horizontal_phase_offset=pn["body_roll_3d_horizontal_phase_offset"],
            pitch_phase_offset=pn["body_roll_3d_pitch_phase_offset"],
            yaw_phase_offset=pn["body_roll_3d_yaw_phase_offset"]),
    }

    def build(names):
        return anim.Stack(layers=tuple(factories[n]() for n in names))

    default_stack = build(list(pn["enabled_animations"]))
    animation_stacks = {}
    for entry in pn["animation_mode_animations"]:
        names = ["still"] + [n.strip() for n in entry.split(",")]
        animation_stacks[entry] = build(names)
    return default_stack, animation_stacks


def main():
    here = os.path.dirname(os.path.abspath(__file__))  # shared/hexa_pipeline/test
    shared_dir = os.path.dirname(os.path.dirname(here))  # shared/
    default_repo = os.path.dirname(shared_dir)           # workspace root

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo-root", default=default_repo)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    src = os.path.join(args.repo_root, "src")
    # Import the pure reference modules directly (no package __init__ side
    # effects, no rclpy). The animations package __init__ is rclpy-free.
    sys.path.insert(0, os.path.join(src, "hexa_posture"))
    from hexa_posture import pose as pose_mod  # noqa: E402
    from hexa_posture import animations as anim  # noqa: E402

    posture_yaml = yaml.safe_load(
        open(f"{src}/hexa_description/config/tuning.yaml"))
    pn = posture_yaml["posture_node"]["ros__parameters"]

    default_stack, animation_stacks = build_stacks(pose_mod, anim, pn)
    limits = pose_mod.PoseLimits()
    reserve = pose_mod.PoseLimits(
        x=pn["animation_reserve_x"],
        y=pn["animation_reserve_y"],
        z=pn["animation_reserve_z"],
        roll=pn["animation_reserve_roll"],
        pitch=pn["animation_reserve_pitch"],
        yaw=pn["animation_reserve_yaw"],
    )
    activation_slew_rate = pn["gait_activation_slew_rate"]
    centroid_tau = pn["support_centroid_tau"]
    swing_lift_tau = pn["swing_lift_tau"]

    frames = build_trace()
    expected = []

    # Filter state (mirror of the controller members).
    support_centroid = None
    latest_raw_centroid = None
    swing_lift = None
    latest_raw_swing_lift = None
    activation = 0.0
    t = 0.0

    for legs, mp, walking, state, gait_name, pose, mode, dt in frames:
        raw_c = stance_centroid_xy(legs)
        if raw_c is not None:
            latest_raw_centroid = raw_c
        raw_l = max_swing_lift_z(legs)
        if raw_l is not None:
            latest_raw_swing_lift = raw_l
        support_centroid = lpf_step_xy(
            support_centroid, latest_raw_centroid, centroid_tau, dt)
        swing_lift = lpf_step_scalar(
            swing_lift, latest_raw_swing_lift, swing_lift_tau, dt)

        if state not in POSTURE_ACTIVE_STATES:
            activation = 0.0  # reset the crossfade while inactive
            expected.append((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            t += dt
            continue

        # Ramp the gait animations in/out toward the walking flag.
        activation = slew_toward(
            activation, 1.0 if walking else 0.0, activation_slew_rate, dt)

        def eval_stack(is_walking):
            ctx = anim.AnimationContext(
                t=t,
                walking=is_walking,
                gait_name=gait_name,
                support_centroid_xy=support_centroid,
                swing_lift_z=swing_lift,
                master_phase=mp % 1.0,
            )
            stack = animation_stacks[mode] if mode else default_stack
            return stack(ctx)

        gait_out = eval_stack(True)
        idle_out = eval_stack(False)
        animated = pose_mod.lerp(idle_out, gait_out, activation)
        user = pose_mod.BodyPose(x=pose[0], y=pose[1], z=pose[2],
                                 roll=pose[3], pitch=pose[4], yaw=pose[5])
        target = pose_mod.compose_layered(user, animated, limits, reserve)
        expected.append((target.x, target.y, target.z,
                         target.roll, target.pitch, target.yaw))
        t += dt

    header = emit(frames, expected)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    old = None
    if os.path.isfile(args.out):
        old = open(args.out).read()
    if old != header:
        open(args.out, "w").write(header)
        sys.stderr.write(f"gen_posture_golden.py: wrote {args.out}\n")
    else:
        sys.stderr.write(f"gen_posture_golden.py: {args.out} up to date\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
