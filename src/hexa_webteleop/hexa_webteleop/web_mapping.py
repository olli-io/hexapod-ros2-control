"""Pure mapping from webapp input to high-level commands.

Translates the webapp's two-joystick + nine-button input model into
the ``(axes, buttons)`` sequences that ``hexa_teleop.joy_mapping.map_joy``
consumes, then delegates to ``map_joy`` for the full state machine
(mode switching, init two-press, record, yaw easing, height
integration, gait/animation cycling).

The webapp config (``webteleop.yaml``) produces a ``JoyConfig`` with
webapp-specific virtual key names (``btn_0`` … ``btn_8``,
``left_stick_x/y``, ``right_stick_x/y``) but the same function
namespace and the same ``JoyState`` / ``JoyOutput`` dataclasses as the
gamepad teleop. This lets the non-trivial state machine live in one
place (``map_joy``) rather than being duplicated per input device.

The one validation difference from the gamepad config loader: the
webapp allows ``init`` / ``record`` (BASE_FUNCTIONS) in per-mode
bindings (validated with ``ALL_FUNCTIONS``) so the bottom 6 buttons can
vary per mode including those two functions. The gamepad loader
restricts mode bindings to ``BUTTON_CLASS_FUNCTIONS | AXIS_CLASS_FUNCTIONS``
and keeps ``init`` / ``record`` in ``base.bindings`` only.

Pure-python; rclpy-free so the mapping + config loading are
unit-testable standalone.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from hexa_gait import VelocityCaps, load_velocity_caps
from hexa_gait.gaits import STRATEGIES
from hexa_posture import load_animation_mode_animations

from hexa_teleop.joy_mapping import (
    ALL_FUNCTIONS,
    ANIMATION,
    BASE_FUNCTIONS,
    GAIT,
    POSTURE,
    BaseConfig,
    JoyConfig,
    JoyOutput,
    JoyState,
    ModeConfig,
    PostureConfig,
    cross_section_function_check,
    map_joy,
    resolve_gait_cycle,
    validate_bindings,
)

NUM_BUTTONS = 9


def load_web_config(
    path: str | Path, gait_yaml: str | Path, posture_yaml: str | Path
) -> tuple[JoyConfig, str, str, VelocityCaps]:
    """Load ``webteleop.yaml`` + gait/posture configs into a ``JoyConfig``.

    Returns ``(cfg, initial_mode, default_gait, caps)`` — same shape as
    ``teleop_joy._load_config`` so the node can consume both identically.
    """
    path = Path(path)
    with path.open() as f:
        raw = yaml.safe_load(f)
    caps = load_velocity_caps(gait_yaml)
    animation_list = load_animation_mode_animations(posture_yaml)

    gait_cycle_raw = tuple(str(n) for n in raw["gait_cycle"])
    allow_unstable = bool(raw.get("allow_unstable_gaits", False))
    unstable_gaits = frozenset(
        name for name, factory in STRATEGIES.items() if factory().unstable
    )
    gait_cycle = resolve_gait_cycle(
        gait_cycle_raw, set(STRATEGIES), unstable_gaits, allow_unstable
    )
    default_gait = str(raw["default_gait"])
    if default_gait not in gait_cycle:
        detail = (
            "is excluded by allow_unstable_gaits: false"
            if default_gait in gait_cycle_raw
            else f"must be in gait_cycle={list(gait_cycle_raw)}"
        )
        raise ValueError(f"default_gait={default_gait!r} {detail}")

    base_raw = raw["base"]
    button_index = {str(k): int(v) for k, v in base_raw["buttons"].items()}
    axis_index = {str(k): int(v) for k, v in base_raw["axes"].items()}
    axis_sign = {
        str(k): float(v) for k, v in base_raw.get("axis_signs", {}).items()
    }
    base_bindings = {str(k): str(v) for k, v in base_raw["bindings"].items()}
    validate_bindings(
        "base",
        base_bindings,
        base_buttons=set(button_index),
        base_axes=set(axis_index),
        allowed_functions=BASE_FUNCTIONS,
    )
    base = BaseConfig(
        deadband=float(base_raw["deadband"]),
        trigger_threshold=float(base_raw.get("trigger_threshold", 0.5)),
        button_index=button_index,
        axis_index=axis_index,
        axis_sign=axis_sign,
        bindings=base_bindings,
    )

    def _parse_mode(section: str, raw_section: dict) -> dict[str, str]:
        bindings = {str(k): str(v) for k, v in raw_section["bindings"].items()}
        validate_bindings(
            section,
            bindings,
            base_buttons=set(base.button_index),
            base_axes=set(base.axis_index),
            allowed_functions=ALL_FUNCTIONS,
        )
        return bindings

    gait_bindings = _parse_mode("gait", raw["gait"])
    posture_raw = raw["posture"]
    posture_bindings = _parse_mode("posture", posture_raw)
    animation_bindings = _parse_mode("animation", raw["animation"])
    cross_section_function_check({
        "gait": gait_bindings,
        "posture": posture_bindings,
        "animation": animation_bindings,
    })

    height = posture_raw["height"]
    posture_cfg = PostureConfig(
        bindings=posture_bindings,
        x_max=float(posture_raw["x_max"]),
        y_max=float(posture_raw["y_max"]),
        roll_max=math.radians(float(posture_raw["roll_max_deg"])),
        pitch_max=math.radians(float(posture_raw["pitch_max_deg"])),
        yaw_max=math.radians(float(posture_raw["yaw_max_deg"])),
        yaw_tau=float(posture_raw["yaw_tau_s"]),
        revert_tau=float(posture_raw["revert_tau_s"]),
        wiggle_pivot_forward_m=float(posture_raw["wiggle_pivot_forward_m"]),
        height_max=float(height["max_m"]),
        height_min=float(height["min_m"]),
        height_rate=float(height["rate_m_per_s"]),
    )

    cfg = JoyConfig(
        base=base,
        gait=ModeConfig(bindings=gait_bindings),
        posture=posture_cfg,
        animation=ModeConfig(bindings=animation_bindings),
        gait_cycle=gait_cycle,
        gait_linear_max=caps.linear_max(default_gait),
        gait_angular_z_max=caps.angular_max,
        animation_list=animation_list,
    )

    initial_mode = str(raw.get("initial_mode", GAIT))
    if initial_mode not in (POSTURE, GAIT, ANIMATION):
        raise ValueError(
            f"initial_mode must be one of "
            f"{POSTURE!r}, {GAIT!r}, {ANIMATION!r}; got {initial_mode!r}"
        )
    return cfg, initial_mode, default_gait, caps


def map_web(
    left_stick: tuple[float, float],
    right_stick: tuple[float, float],
    buttons: tuple[int, ...],
    cfg: JoyConfig,
    state: JoyState,
    dt: float,
) -> JoyOutput:
    """Map webapp input to ``JoyOutput`` via the shared ``map_joy``.

    ``left_stick`` / ``right_stick`` are ``(x, y)`` pairs in ``[-1, 1]``,
    REP-103 normalised (x: left = +, y: forward = +). ``buttons`` is a
    ``NUM_BUTTONS``-element tuple of 0/1. The function packs them into
    the ``axes`` / ``buttons`` sequences that ``map_joy`` expects and
    delegates — the full state machine (mode switching, init, record,
    yaw, height, gait/animation cycling) runs inside ``map_joy``.
    """
    axes = (left_stick[0], left_stick[1], right_stick[0], right_stick[1])
    return map_joy(axes, buttons, cfg, state, dt)


def input_is_stale(
    last_input_monotonic: float, now_monotonic: float, timeout_s: float
) -> bool:
    """True if the last webapp input is older than ``timeout_s`` seconds.

    Drives the node's safety watchdog: when input goes stale — the
    WebSocket dropped uncleanly (TCP half-open, no FIN), the phone slept,
    or the tab was backgrounded — the node feeds ``neutral_inputs`` to
    ``map_web`` so ``/cmd_vel`` falls to zero instead of latching the last
    commanded velocity and walking the robot away.
    """
    return (now_monotonic - last_input_monotonic) > timeout_s


def neutral_inputs() -> tuple[tuple[float, float], tuple[float, float], tuple[int, ...]]:
    """Neutral webapp input: centred sticks, all buttons released."""
    return (0.0, 0.0), (0.0, 0.0), (0,) * NUM_BUTTONS


def button_labels_for_mode(cfg: JoyConfig, mode: str) -> tuple[str, ...]:
    """Return ``NUM_BUTTONS`` button labels (function names) for ``mode``.

    Indices 0-2: fixed mode-select buttons from ``base.bindings``
    (``btn_0``, ``btn_1``, ``btn_2``). Indices 3-8: per-mode bindings
    (``btn_3`` … ``btn_8``). Unbound buttons return ``""``.
    """
    if mode == GAIT:
        mode_cfg = cfg.gait
    elif mode == POSTURE:
        mode_cfg = cfg.posture
    else:
        mode_cfg = cfg.animation
    labels: list[str] = []
    for i in range(NUM_BUTTONS):
        key = f"btn_{i}"
        if i < 3:
            labels.append(cfg.base.bindings.get(key, ""))
        else:
            labels.append(mode_cfg.bindings.get(key, ""))
    return tuple(labels)
