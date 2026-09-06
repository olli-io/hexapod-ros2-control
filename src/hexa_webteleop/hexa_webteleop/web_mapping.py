"""Pure mapping from webapp input to high-level commands.

The webapp has no buttons and no axes — it has a grid whose slots the
operator taps, and each slot knows which *function* it is. So the client
sends the function name (``init``, ``quadruped_mode``, ``gait_next`` …) and
this module hands it straight to ``hexa_teleop.joy_mapping.map_functions``,
which runs the full state machine (mode switching, init two-press, record,
yaw easing, height integration, gait/animation cycling) — the same one the
gamepad runs. Nothing here builds a ``sensor_msgs/Joy`` index: that layer
belongs to a device that actually reports indices, and this one does not.

The two on-screen sticks are the exception: they are continuous, and which
function each drives depends on the mode, which the *node* is authoritative
for. Resolving them here rather than in the client is what stops a stick
held across a mode switch from leaking drive into pose for the tick before
the client hears about the change. Their per-mode table is
``webteleop.yaml``'s ``sticks:`` blocks, loaded into a ``StickMap``.

Pure-python; rclpy-free so the mapping + config loading are
unit-testable standalone.
"""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path
from typing import Mapping

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
    ANIMATION,
    AXIS_CLASS_FUNCTIONS,
    BASE_FUNCTIONS,
    BUTTON_CLASS_FUNCTIONS,
    GAIT,
    POSTURE,
    BaseConfig,
    FunctionInput,
    JoyConfig,
    JoyOutput,
    JoyState,
    ModeConfig,
    PostureConfig,
    map_functions,
)

# What the client is allowed to name on the wire: every function the state
# machine reads as a press, and nothing else. The axis-class functions are
# the sticks', and the client does not name those.
ACTIONS: frozenset[str] = BASE_FUNCTIONS | BUTTON_CLASS_FUNCTIONS

# The webapp's two sticks, by the names the ``sticks:`` blocks use.
STICKS: tuple[str, ...] = (
    "left_stick_x",
    "left_stick_y",
    "right_stick_x",
    "right_stick_y",
)


@dataclasses.dataclass(frozen=True)
class StickMap:
    """Which axis-class function each stick drives, per mode.

    One dict per config section — ``webteleop.yaml``'s ``sticks:`` blocks
    verbatim, stick name -> function name. Kept out of ``JoyConfig`` because
    it is the webapp's own layout, not something the shared state machine
    reads: ``map_web`` resolves it and hands over functions.
    """

    gait: Mapping[str, str]
    posture: Mapping[str, str]
    animation: Mapping[str, str]

    def for_section(self, section: str) -> Mapping[str, str]:
        if section == POSTURE:
            return self.posture
        if section == ANIMATION:
            return self.animation
        return self.gait


def parse_sticks(section: str, raw_sticks: object) -> dict[str, str]:
    """Validate one ``sticks:`` block: stick name -> axis-class function.

    The webapp's answer to ``validate_bindings``, and much smaller because
    there is nothing here but four sticks and the functions they drive — no
    keys, no indices, no button class.
    """
    if not isinstance(raw_sticks, dict):
        raise ValueError(f"{section}.sticks must be a mapping")
    table: dict[str, str] = {}
    seen: dict[str, str] = {}
    for key, fn in raw_sticks.items():
        stick, function = str(key), str(fn)
        if stick not in STICKS:
            raise ValueError(
                f"{section}.sticks: unknown stick {stick!r} "
                f"(expected one of {', '.join(STICKS)})"
            )
        if function not in AXIS_CLASS_FUNCTIONS:
            raise ValueError(
                f"{section}.sticks: {stick!r} is bound to {function!r}, "
                f"which is not an axis-class function"
            )
        if function in seen:
            raise ValueError(
                f"{section}.sticks: {function!r} is bound to both "
                f"{seen[function]!r} and {stick!r}"
            )
        seen[function] = stick
        table[stick] = function
    return table


def load_web_config(
    path: str | Path,
    gait_yaml: str | Path,
    posture_yaml: str | Path,
    geometry_yaml: str | Path,
) -> tuple[JoyConfig, StickMap, str, str, VelocityCaps, PresetRegistry]:
    """Load ``webteleop.yaml`` + gait/posture/geometry configs into a ``JoyConfig``.

    Returns ``(cfg, sticks, initial_mode, default_gait, caps, presets)`` —
    ``teleop_joy._load_config``'s tuple plus the ``StickMap``, which is the one
    thing the webapp has that a gamepad does not: a layout with no keys in it.
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
    # No key layout: the webapp names its functions, so ``BaseConfig``'s
    # index and binding tables stay at their empty defaults and only the
    # deadband is the webapp's to set.
    base = BaseConfig(
        deadband=float(base_raw["deadband"]),
        trigger_threshold=float(base_raw.get("trigger_threshold", 0.5)),
    )

    posture_raw = raw["posture"]
    sticks = StickMap(
        gait=parse_sticks("gait", raw["gait"]["sticks"]),
        posture=parse_sticks("posture", posture_raw["sticks"]),
        animation=parse_sticks("animation", raw["animation"]["sticks"]),
    )

    height = posture_raw["height"]
    posture_cfg = PostureConfig(
        # PostureScalarLimits carries PostureConfig's own field names.
        **dataclasses.asdict(posture_limits),
        height_max=height_max,
        height_min=height_min,
        height_rate=float(height["rate_m_per_s"]),
    )

    cfg = JoyConfig(
        base=base,
        gait=ModeConfig(),
        posture=posture_cfg,
        animation=ModeConfig(),
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
    return cfg, sticks, initial_mode, default_gait, caps, registry


def map_web(
    left_stick: tuple[float, float],
    right_stick: tuple[float, float],
    actions: frozenset[str],
    sticks: StickMap,
    cfg: JoyConfig,
    state: JoyState,
    dt: float,
) -> JoyOutput:
    """Map webapp input to ``JoyOutput`` via the shared ``map_functions``.

    ``left_stick`` / ``right_stick`` are ``(x, y)`` pairs in ``[-1, 1]``,
    REP-103 normalised (x: left = +, y: forward = +). ``actions`` is the set
    of functions the operator is holding, named outright.

    The state machine asks per section; the held set is the same answer
    whichever it asks for, because a slot that means ``quadruped_mode`` in
    gait mode sends that word and nothing else — the aliasing a key layout
    forces (one key, ``record`` here and ``quadruped_mode`` there) does not
    exist on this device. The sticks do vary, so they are resolved against
    the section asked for.
    """
    values = {
        "left_stick_x": left_stick[0],
        "left_stick_y": left_stick[1],
        "right_stick_x": right_stick[0],
        "right_stick_y": right_stick[1],
    }

    def source(section: str) -> FunctionInput:
        return FunctionInput(
            pressed=actions,
            axes={
                function: values[stick]
                for stick, function in sticks.for_section(section).items()
            },
        )

    return map_functions(source, cfg, state, dt)


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


def neutral_inputs() -> tuple[
    tuple[float, float], tuple[float, float], frozenset[str]
]:
    """Neutral webapp input: centred sticks, nothing held."""
    return (0.0, 0.0), (0.0, 0.0), frozenset()


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
    """The preset list the webapp renders, sent once with ``init``.

    Each row carries its own gait rotation, because the webapp offers the gaits
    of the preset in force as a button apiece. Sent per preset rather than as
    one live list of the gaits currently on offer: the rotations are fixed at
    load, the active preset is already live on ``/gait/preset``, and one static
    list the client indexes by that report cannot disagree with it the way a
    second live message could.
    """
    return [
        {
            "id": p.id,
            "label": p.label,
            "sub": p.sub,
            "leg_set": p.leg_set,
            "gaits": list(p.gait_cycle),
        }
        for p in registry.presets
    ]


def gait_selectable(registry: PresetRegistry, gait: str) -> bool:
    """True if ``gait`` is one the preset in force offers.

    The webapp names a gait outright rather than stepping a cycler, so this is
    what the cycler's arithmetic used to guarantee for free: a gait is only ever
    asked for from the rotation of the preset the engine reports, so the request
    can never carry the other leg set. ``False`` before the first
    ``/gait/preset`` — with no preset in force there is no rotation to be in,
    and the webapp offers no gait buttons either.
    """
    preset = registry.current()
    return preset is not None and gait in preset.gait_cycle


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
