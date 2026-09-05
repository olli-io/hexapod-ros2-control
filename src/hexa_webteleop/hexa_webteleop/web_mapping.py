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

import dataclasses
import math
from pathlib import Path

import yaml

from hexa_common import (
    VelocityCaps,
    load_animation_mode_animations,
    load_body_height_offsets,
    load_posture_scalar_limits,
    load_velocity_caps,
    unit_stance_xy,
)

from hexa_teleop.presets import (
    HEXAPOD,
    QUADRUPED,
    PresetRegistry,
    load_presets,
)
from hexa_teleop.presets import resync_gait as _resync_gait

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
    validate_bindings,
)

NUM_BUTTONS = 9


def load_web_config(
    path: str | Path,
    gait_yaml: str | Path,
    posture_yaml: str | Path,
    geometry_yaml: str | Path,
) -> tuple[JoyConfig, str, str, VelocityCaps, PresetRegistry]:
    """Load ``webteleop.yaml`` + gait/posture/geometry configs into a ``JoyConfig``.

    Returns ``(cfg, initial_mode, default_gait, caps, presets)`` — same shape as
    ``teleop_joy._load_config`` so the node can consume both identically.
    ``geometry_yaml`` supplies the leg mounts the angular cap is derived from.
    """
    path = Path(path)
    with path.open() as f:
        raw = yaml.safe_load(f)
    caps = load_velocity_caps(gait_yaml, geometry_yaml)
    animation_list = load_animation_mode_animations(posture_yaml)
    # Shared with the gamepad teleop and the firmware, so tuning.yaml owns
    # them — the webapp poses the body over the same range a gamepad does.
    posture_limits = load_posture_scalar_limits(posture_yaml)
    # Body-height envelope as pose offsets — owned by tuning.yaml, not by
    # webteleop.yaml. Used only to saturate the height integrator.
    height_min, height_max = load_body_height_offsets(gait_yaml, posture_yaml)

    registry = load_presets(raw, gait_yaml, geometry_yaml)
    hexapod_preset = registry.active(HEXAPOD)
    quadruped_preset = registry.active(QUADRUPED)
    if hexapod_preset is None or quadruped_preset is None:
        raise ValueError(
            "presets must declare at least one six-leg and one four-corner "
            "preset: the init buttons stand up on one of each"
        )
    gait_cycle = hexapod_preset.gait_cycle
    default_gait = registry.entry_gait(hexapod_preset.id)
    quadruped_gait_cycle = quadruped_preset.gait_cycle
    default_quadruped_gait = registry.entry_gait(quadruped_preset.id)

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
        # PostureScalarLimits carries PostureConfig's own field names.
        **dataclasses.asdict(posture_limits),
        height_max=height_max,
        height_min=height_min,
        height_rate=float(height["rate_m_per_s"]),
    )

    cfg = JoyConfig(
        base=base,
        gait=ModeConfig(bindings=gait_bindings),
        posture=posture_cfg,
        animation=ModeConfig(bindings=animation_bindings),
        gait_cycle=gait_cycle,
        quadruped_gait_cycle=quadruped_gait_cycle,
        default_quadruped_gait=default_quadruped_gait,
        gait_linear_max=caps.linear_max(default_gait),
        gait_angular_z_max=caps.angular_max(default_gait),
        stance_unit=unit_stance_xy(geometry_yaml, gait_yaml),
        animation_list=animation_list,
    )

    initial_mode = str(raw.get("initial_mode", GAIT))
    if initial_mode not in (POSTURE, GAIT, ANIMATION):
        raise ValueError(
            f"initial_mode must be one of "
            f"{POSTURE!r}, {GAIT!r}, {ANIMATION!r}; got {initial_mode!r}"
        )
    return cfg, initial_mode, default_gait, caps, registry


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


def battery_payload(
    reading: tuple[float, float, float] | None,
    now_monotonic: float,
    stale_after_s: float,
) -> dict[str, float | None]:
    """Pack voltage / current as the webapp's WebSocket fields.

    ``reading`` is ``(volts, amps, monotonic_stamp)`` from the last
    ``BatteryState``, or ``None`` before the first one arrives. A field
    reads ``None`` — a dash in the webapp — when the pack has never been
    heard (the sim publishes no telemetry at all), when the last reading
    is older than ``stale_after_s`` so a dead hardware node cannot leave a
    frozen number on screen, or when the board reported a non-finite
    value. That last case is not only defensive: ``json.dumps`` writes a
    bare ``NaN``, which is not JSON and throws in the client's
    ``JSON.parse``, taking the whole message down with it.
    """
    if reading is None:
        return {"voltage": None, "current": None}
    voltage, current, stamp = reading
    if (now_monotonic - stamp) > stale_after_s:
        return {"voltage": None, "current": None}
    return {
        "voltage": voltage if math.isfinite(voltage) else None,
        "current": current if math.isfinite(current) else None,
    }


def neutral_inputs() -> tuple[tuple[float, float], tuple[float, float], tuple[int, ...]]:
    """Neutral webapp input: centred sticks, all buttons released."""
    return (0.0, 0.0), (0.0, 0.0), (0,) * NUM_BUTTONS


def resync_gait(
    name: str,
    cfg: JoyConfig,
    state: JoyState,
    registry: PresetRegistry,
) -> JoyConfig | None:
    """Resync stick caps and the cycler slot to a commanded gait.

    A thin wrapper over ``hexa_teleop.presets.resync_gait`` for callers that
    only want the rebuilt config. The shared version lives in ``hexa_teleop``
    because the gamepad node needs the same bookkeeping — without it, a gait
    switch made from the web app leaves its D-pad rotating the wrong list. The
    LEG SET is not here: the preset owns it, and ``resync_preset`` follows it.
    """
    result = _resync_gait(name, cfg, state, registry)
    return None if result is None else result.cfg


def preset_payload(
    registry: PresetRegistry,
    leg_set: str | None,
    pending: str | None,
    refused: str | None,
) -> dict:
    """The webapp's ``preset`` message: which preset is actually in force.

    The active row is what ``/gait/preset`` last reported — the preset the
    engine has APPLIED — and nothing else. Never the tap, and never the latched
    ``/cmd_preset``: that topic keeps a refused id forever, so a UI reading it
    would show a preset the robot never took, with nothing to correct it. It
    cannot come off ``/gait/leg_set`` either, now that three of the four presets
    stand on six legs.

    ``None`` before the first ``/gait/preset`` arrives, which is honest — the
    webapp shows no row lit rather than guessing at one. ``leg_set`` rides along
    for the navbar icon, which dims the middle pair on four legs.
    """
    return {
        "active": registry.current_id(),
        "leg_set": leg_set or None,
        "pending": pending,
        "refused": refused,
    }


def preset_descriptors(registry: PresetRegistry) -> list[dict]:
    """The preset list the webapp renders, sent once with ``init``."""
    return [
        {
            "id": p.id,
            "label": p.label,
            "sub": p.sub,
            "leg_set": p.leg_set,
        }
        for p in registry.presets
    ]


def preset_pending_expired(
    deadline_monotonic: float | None, now_monotonic: float
) -> bool:
    """True if a pending preset switch has outlived its deadline.

    ``/gait/preset`` publishes on change only, so a refusal the node could not
    predict — the engine's state moved between the click and the tick, or the
    body pose never came back to neutral — arrives as silence. Silence past the
    deadline is what the webapp reads as "it did not happen".
    """
    return deadline_monotonic is not None and now_monotonic > deadline_monotonic


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
