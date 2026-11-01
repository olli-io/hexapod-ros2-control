#!/usr/bin/env python3
"""Render one Xbox controller diagram per teleop mode from teleop_joy.yaml.

Assets live in scripts/controller/: xbox.svg is the controller body, whose
`anchor-*` circles place the callouts; icons/*.svg are the per-key glyphs.
Output: docs/controller/svg/xbox_<mode>.svg, plus docs/controller/xbox_<mode>.png
when rsvg-convert or inkscape is on the host.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from html import escape
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("generate_controller: needs PyYAML on the host (pip install pyyaml)")

REPO = Path(__file__).resolve().parent.parent
ASSETS = REPO / "scripts" / "controller"
CONFIG = REPO / "src" / "hexa_teleop" / "config" / "teleop_joy.yaml"
OUT_DIR = REPO / "docs" / "controller"

MODES = ("gait", "posture", "animation", "gesture")

LABELS = {
    "gait_mode": "Gait mode",
    "posture_mode": "Posture mode",
    "animation_mode": "Animation mode",
    "gesture_mode": "Gesture mode",
    "init": "Stand up / fold",
    "record": "Save / revert body pose",
    "stabilize_toggle": "Stabilization on / off",
    "yaw_left": "Twist body left",
    "yaw_right": "Twist body right",
    "wiggle_left": "Butt wiggle left",
    "wiggle_right": "Butt wiggle right",
    "height_up": "Lift body up",
    "height_down": "Lower body",
    "gait_prev": "Previous gait",
    "gait_next": "Next gait",
    "animation_prev": "Previous animation",
    "animation_next": "Next animation",
    "quadruped_mode": "Stand up on four legs",
    "drive_x": "Walk forward / back",
    "drive_x_aux": "Walk forward / back",
    "drive_y": "Strafe left / right",
    "drive_yaw": "Turn left / right",
    "pose_x": "Shift body forward / back",
    "pose_y": "Shift body left / right",
    "tilt_roll": "Roll body",
    "tilt_pitch": "Pitch body",
}

# Functions that are bound but do nothing outside these modes.
LIVE_IN = {"record": {"posture"}}

# Callout groups: anchor id in xbox.svg, rows of (key, icon), bracket side and
# position, and the x of the vertical leg of the connector.
GROUPS = [
    dict(anchor="left_shoulder", side="left", bx=470, cy=72, mx=None,
         rows=[("l2", "lt"), ("l1", "lb")]),
    dict(anchor="left_stick", side="left", bx=470, cy=211, mx=None,
         rows=[("left_stick_y", "ls_y"), ("left_stick_x", "ls_x"), ("l3", "l3")]),
    dict(anchor="dpad", side="left", bx=470, cy=368, mx=520,
         rows=[("dpad_up", "dpad_up"), ("dpad_left", "dpad_left"),
               ("dpad_right", "dpad_right"), ("dpad_down", "dpad_down")]),
    dict(anchor="right_shoulder", side="right", bx=1010, cy=72, mx=None,
         rows=[("r2", "rt"), ("r1", "rb")]),
    dict(anchor="face", side="right", bx=1040, cy=211, mx=None,
         rows=[("y", "y"), ("x", "x"), ("b", "b"), ("a", "a")]),
    dict(anchor="right_stick", side="right", bx=1040, cy=368, mx=980,
         rows=[("right_stick_y", "rs_y"), ("right_stick_x", "rs_x"), ("r3", "r3")]),
    dict(anchor="centre", side="below", bx=740, cy=462, mx=None,
         rows=[("start", "start"), ("select", "select")]),
]

HEIGHT = 510
TITLE_H = 32
MARGIN = 24
CHAR_W = 8.2  # mean glyph advance at 15 px, for the crop estimate
BODY_ORIGIN = (540, 100)
ROW = 32
ICON = 24
INK = "#1d1d1f"
FONT = "Helvetica, Arial, sans-serif"


def load_svg(path: Path) -> tuple[str, str]:
    """Return (viewBox, inner markup) of a standalone SVG asset."""
    text = path.read_text()
    root = re.search(r"<svg\b[^>]*>", text)
    view_box = re.search(r'viewBox="([^"]+)"', root.group(0)).group(1)
    inner = text[root.end():text.rindex("</svg>")]
    return view_box, re.sub(r"<!--.*?-->", "", inner, flags=re.S)


def body_anchors(path: Path) -> dict[str, tuple[float, float]]:
    ns = {"s": "http://www.w3.org/2000/svg"}
    root = ET.parse(path).getroot()
    vx, vy, _, _ = (float(v) for v in root.get("viewBox").split())
    ox, oy = BODY_ORIGIN
    return {
        c.get("id").removeprefix("anchor-"):
            (ox + float(c.get("cx")) - vx, oy + float(c.get("cy")) - vy)
        for c in root.iterfind(".//s:circle", ns)
        if c.get("id", "").startswith("anchor-")
    }


def bindings_for(cfg: dict, mode: str) -> dict[str, str]:
    """Key -> label for one mode: base bindings under the mode's section."""
    keys = dict(cfg["base"]["bindings"])
    section = cfg.get(mode, {}).get("bindings", {}) or {}
    if mode == "gesture":
        keys.update({k: f"gesture:{v}" for k, v in section.items()})
    else:
        keys.update(section)
    labels = {}
    for key, fn in keys.items():
        if not fn:
            continue
        if fn.startswith("gesture:"):
            labels[key] = f"Gesture: {fn.split(':', 1)[1].replace('_', ' ')}"
        elif mode in LIVE_IN.get(fn, {mode}):
            if fn not in LABELS:
                sys.exit(f"generate_controller: no label for function '{fn}'")
            labels[key] = LABELS[fn]
    return labels


def icon(name: str, x: float, y: float) -> str:
    view_box, inner = load_svg(ASSETS / "icons" / f"{name}.svg")
    return (f'<svg x="{x:.0f}" y="{y:.0f}" width="{ICON}" height="{ICON}" '
            f'viewBox="{view_box}">{inner.strip()}</svg>')


def render_inline(g: dict, anchor: tuple[float, float], labels: dict[str, str]) -> list[str]:
    """One row of icon + label pairs, centred under the anchor, bracket above."""
    out = []
    gap = 32
    widths = [ICON + 10 + len(labels.get(key, "")) * CHAR_W for key, _ in g["rows"]]
    x = g["bx"] - (sum(widths) + gap * (len(widths) - 1)) / 2
    x0, y = x, g["cy"]
    for (key, icon_name), w in zip(g["rows"], widths):
        out.append(icon(icon_name, x, y - ICON / 2))
        if label := labels.get(key, ""):
            out.append(f'<text x="{x + ICON + 10:.0f}" y="{y + 5:.0f}">{escape(label)}</text>')
        x += w + gap
    ax, ay = anchor
    by = y - ROW / 2 - 6
    out.append(f'<path d="M{x0 - 4:.0f} {by:.0f} H{x - gap + 4:.0f}" class="ln"/>')
    out.append(f'<path d="M{ax:.0f} {by - 12:.0f} V{ay:.0f}" class="ln"/>')
    out.append(f'<circle cx="{ax:.0f}" cy="{by - 12:.0f}" r="5" fill="{INK}"/>')
    out.append(f'<circle cx="{ax:.0f}" cy="{ay:.0f}" r="4" fill="{INK}" '
               f'stroke="#fff" stroke-width="1.5"/>')
    return out


def render_group(g: dict, anchor: tuple[float, float], labels: dict[str, str],
                 active: str) -> list[str]:
    if g["side"] == "below":
        return render_inline(g, anchor, labels)
    out = []
    n = len(g["rows"])
    top = g["cy"] - (n - 1) * ROW / 2
    bx = g["bx"]
    ax, ay = anchor
    for i, (key, icon_name) in enumerate(g["rows"]):
        y = top + i * ROW
        label = labels.get(key, "")
        weight = "700" if label == LABELS.get(f"{active}_mode") else "400"
        if g["side"] == "left":
            out.append(icon(icon_name, bx - 16 - ICON, y - ICON / 2))
            tx, anchor_attr = bx - 28 - ICON, "end"
        else:
            out.append(icon(icon_name, bx + 16, y - ICON / 2))
            tx, anchor_attr = bx + 28 + ICON, "start"
        if label:
            out.append(f'<text x="{tx:.0f}" y="{y + 5:.0f}" text-anchor="{anchor_attr}" '
                       f'font-weight="{weight}">{escape(label)}</text>')
    y0, y1 = top - ROW / 2 + 4, top + (n - 1) * ROW + ROW / 2 - 4
    out.append(f'<path d="M{bx} {y0:.0f} V{y1:.0f}" class="ln"/>')

    dx = bx + 12 if g["side"] == "left" else bx - 12
    dy = g["cy"]
    mx = g["mx"] if g["mx"] is not None else ax
    route = f"M{dx} {dy:.0f} H{mx:.0f} V{ay:.0f} H{ax:.0f}"
    out.append(f'<path d="{route}" class="ln"/>')
    out.append(f'<circle cx="{dx}" cy="{dy:.0f}" r="5" fill="{INK}"/>')
    out.append(f'<circle cx="{ax:.0f}" cy="{ay:.0f}" r="4" fill="{INK}" '
               f'stroke="#fff" stroke-width="1.5"/>')
    return out


def extent(cfg: dict) -> tuple[float, float]:
    """Horizontal crop shared by every mode, from the widest label per side."""
    xmin, xmax = BODY_ORIGIN[0], BODY_ORIGIN[0] + 400
    for mode in MODES:
        labels = bindings_for(cfg, mode)
        for g in GROUPS:
            for key, _ in g["rows"]:
                w = len(labels.get(key, "")) * CHAR_W
                if g["side"] == "below":
                    continue
                if g["side"] == "left":
                    xmin = min(xmin, g["bx"] - 28 - ICON - w)
                else:
                    xmax = max(xmax, g["bx"] + 28 + ICON + w)
    return xmin - MARGIN, xmax + MARGIN


def render(cfg: dict, mode: str, crop: tuple[float, float]) -> str:
    labels = bindings_for(cfg, mode)
    x0, x1 = crop
    width, height = x1 - x0, HEIGHT + TITLE_H
    anchors = body_anchors(ASSETS / "xbox.svg")
    body_vb, body_inner = load_svg(ASSETS / "xbox.svg")
    _, _, bw, bh = body_vb.split()
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{x0:.0f} {-TITLE_H} {width:.0f} {height}" '
        f'width="{width:.0f}" height="{height}" font-family="{FONT}" font-size="15" fill="{INK}">',
        f"<!-- Generated by ./hexa generate-controller from "
        f"src/hexa_teleop/config/teleop_joy.yaml. Do not edit. -->",
        "<style>.ln{fill:none;stroke:%s;stroke-width:1.6}</style>" % INK,
        f'<rect x="{x0:.0f}" y="{-TITLE_H}" width="{width:.0f}" height="{height}" fill="#fff"/>',
        f'<text x="{BODY_ORIGIN[0] + 200}" y="24" text-anchor="middle" font-size="28" '
        f'font-weight="700">{mode.capitalize()} mode</text>',
        f'<svg x="{BODY_ORIGIN[0]}" y="{BODY_ORIGIN[1]}" width="{bw}" height="{bh}" '
        f'viewBox="{body_vb}">{body_inner}</svg>',
    ]
    for g in GROUPS:
        parts += render_group(g, anchors[g["anchor"]], labels, mode)
    parts.append("</svg>\n")
    return "\n".join(parts)


def to_png(svg: Path, png: Path) -> Path | None:
    if shutil.which("rsvg-convert"):
        cmd = ["rsvg-convert", str(svg), "-o", str(png)]
    elif shutil.which("inkscape"):
        cmd = ["inkscape", str(svg), "--export-type=png", f"--export-filename={png}"]
    else:
        return None
    subprocess.run(cmd, check=True, capture_output=True)
    return png


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", type=Path, default=CONFIG)
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    known = {k for g in GROUPS for k, _ in g["rows"]}
    for section in ("base", *MODES):
        for key in (cfg.get(section, {}).get("bindings") or {}):
            if key not in known:
                sys.exit(f"generate_controller: [{section}] key '{key}' has no place on the diagram")

    (args.out / "svg").mkdir(parents=True, exist_ok=True)
    crop = extent(cfg)
    for mode in MODES:
        path = args.out / "svg" / f"xbox_{mode}.svg"
        path.write_text(render(cfg, mode, crop))
        for out in (path, to_png(path, args.out / f"xbox_{mode}.png")):
            if out:
                print(f"wrote {out.relative_to(REPO) if out.is_relative_to(REPO) else out}")
    if not shutil.which("rsvg-convert") and not shutil.which("inkscape"):
        print("generate_controller: no rsvg-convert or inkscape, skipped PNG", file=sys.stderr)


if __name__ == "__main__":
    main()
