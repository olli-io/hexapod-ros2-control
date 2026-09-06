"""Operator presets: named bundles both teleops pick as one thing.

A **preset** is what the operator selects as one thing — ``normal``, ``fast``,
``offroad`` and ``quad`` ship. It has two halves, owned by different layers:

- the **physical** half — the legs it stands on, where those feet sit, and the
  stride and swing times the walk lays down on it — lives in
  ``hexa_description``'s ``tuning.yaml`` and is read here through
  ``hexa_common.preset_config``. The engine loads it directly.
- the **operator** half — an id, a label, a gait rotation and the gait it
  enters on — is declared in each teleop's own config and is this module's.

Selecting a preset publishes its id on ``/cmd_preset`` and its remembered gait
on ``/cmd_gait``. It needs its own channel because the gait cannot carry it:
``normal``, ``fast`` and ``offroad`` all stand on six legs and can all offer the
same rotation, so a bare gait name says nothing about which is in force. The
engine reports what it has APPLIED on ``/gait/preset``, and that report — never
the tap, never the latched command — is what both teleops follow, which is what
``resync_preset`` is for.

A preset's **leg set is derived**, never declared here: ``tuning.yaml`` owns it,
because it is the physical fact of whether the middle pair stands. Load-time
validation then requires every gait in a rotation to walk that leg set, so a
prev/next press can never ask a standing robot for legs it is not on.

Why the projection, rather than a general N-preset state machine: ``map_joy``
holds exactly two rotations and a ``quadruped`` flag, and it is parity-locked to
``shared/motion_core/joy_mapping.cpp`` by a golden trace. So the registry keeps
the presets and *projects* the active one onto the two ``JoyConfig`` fields
``map_joy`` already reads. A fifth preset is then a config edit: entering it
swaps ``cfg.gait_cycle``, and ``map_joy`` never learns it exists.

Pure Python; rclpy-free, so both nodes' bookkeeping is unit-testable without a
ROS context. It lives here rather than in ``hexa_common`` (a leaf that knows
nothing of ``JoyConfig`` / ``JoyState``), in ``joy_mapping`` (parity-locked), or
in ``hexa_webteleop`` (the dependency runs the other way).
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from hexa_common import VelocityCaps
from hexa_common.gait_catalog import GAIT_DESCRIPTORS
from hexa_common.limits import load_velocity_caps
from hexa_common.preset_config import preset_leg_sets

from .joy_mapping import (
    ANIMATION,
    GAIT,
    JoyConfig,
    JoyState,
    resolve_gait_cycle,
)

HEXAPOD = "hexapod"
QUADRUPED = "quadruped"

# Engine states (/gait/state) a preset change may be asked for from. Narrower
# than the gait-switch set on purpose: a plain gait swap is legal while walking
# or settling because the engine can latch it and apply it at the next stand,
# but a preset change re-plants every foot — and moves the middle pair too where
# the two presets differ in leg set — so it only runs from a stand and is
# refused everywhere else. Pre-gating here (rather than letting the engine
# refuse) matters because /cmd_preset is latched: a refused id would sit on the
# wire forever, and every late subscriber would read a preset the robot never
# took.
PRESET_SWITCH_STATES: frozenset[str] = frozenset({"folded", "fault", "stand"})


def preset_switch_allowed(gait_state: str) -> bool:
    """True if a preset change may be published in ``gait_state``."""
    return gait_state in PRESET_SWITCH_STATES


@dataclass(frozen=True)
class Preset:
    """One operator-facing preset. ``leg_set`` comes from ``tuning.yaml``."""

    id: str
    label: str
    sub: str
    gait_cycle: tuple[str, ...]
    default_gait: str
    leg_set: str


@dataclass
class ResyncResult:
    """What a value on the wire did to this teleop's own bookkeeping."""

    cfg: JoyConfig
    preset: Preset | None
    # The preset that arrived stands on different legs than the one that was in
    # force. The caller eases its posture out on this: a body pose is applied to
    # a planted leg and not to a parked one, so the middle pair may only cross
    # between the two at a neutral pose, and the engine waits for it before
    # moving anything.
    leg_set_changed: bool
    # Quadruped mode has no animations (every one is written for six legs), and
    # ``map_joy`` only blocks *entering* the mode. Arriving on four legs while
    # already in it has to drop out, which the caller then has to tell the
    # engine about by publishing an empty /animation/mode.
    left_animation_mode: bool


class PresetRegistry:
    """The declared presets, each one's caps, and which is in force.

    Two kinds of "active" are tracked, and they are not the same question:

    - ``current`` is the preset the ENGINE reports on ``/gait/preset``. It is
      the honest answer to "which preset is the robot on", and it is ``None``
      until the first report arrives.
    - ``active(leg_set)`` is the last preset selected for each leg set, which is
      what ``project`` writes into ``map_joy``'s two rotations. Two slots rather
      than one, because the six-leg selection an operator was using is still
      theirs when they come back from four legs, exactly as the two cycler
      indexes already behave.
    """

    def __init__(
        self,
        presets: Sequence[Preset],
        default_id: str,
        switch_timeout_s: float,
        caps_by_preset: Mapping[str, VelocityCaps],
    ) -> None:
        self._presets = tuple(presets)
        self.switch_timeout_s = switch_timeout_s
        self._by_id = {p.id: p for p in self._presets}
        if default_id not in self._by_id:
            raise ValueError(
                f"presets.default={default_id!r} names no preset; "
                f"have {sorted(self._by_id)}"
            )
        self.default_id = default_id
        self._caps = dict(caps_by_preset)
        missing = sorted(set(self._by_id) - set(self._caps))
        if missing:
            raise ValueError(f"presets: no velocity caps for {missing}")
        # Which preset is in force for each leg set. Seeded from the default for
        # its own set, and from the first declared preset for the other one.
        self._active: dict[str, str] = {}
        for p in self._presets:
            self._active.setdefault(p.leg_set, p.id)
        self._active[self._by_id[default_id].leg_set] = default_id
        # The last gait seen in each preset's rotation. Read by ``project``
        # alone, and only for the four-corner one: it becomes
        # ``default_quadruped_gait``, the gait the `select` init button stands
        # up on, so coming off the belly lands where the operator last was.
        # NOT what a preset change enters on — see ``entry_gait``.
        self._slot: dict[str, str] = {p.id: p.default_gait for p in self._presets}
        # Nothing until /gait/preset speaks. Deliberately not seeded from the
        # default: showing a preset the robot may not be on is exactly what the
        # report topic exists to prevent.
        self._current: str | None = None

    @property
    def presets(self) -> tuple[Preset, ...]:
        return self._presets

    def get(self, preset_id: str) -> Preset | None:
        return self._by_id.get(preset_id)

    def caps(self, preset_id: str | None = None) -> VelocityCaps:
        """One preset's velocity caps; the one in force when given no id.

        Per preset because three of the four inputs to a cap ride it: the stride
        it lays down, the swing time it lays it down in, and the stance the
        angular cap divides by.
        """
        if preset_id is None:
            preset_id = self._current or self._active[
                self._by_id[self.default_id].leg_set
            ]
        return self._caps[preset_id]

    def current(self) -> Preset | None:
        """The preset the engine reports, or ``None`` before the first report."""
        return self._by_id.get(self._current) if self._current else None

    def current_id(self) -> str | None:
        return self._current

    def active(self, leg_set: str) -> Preset | None:
        pid = self._active.get(leg_set)
        return self._by_id.get(pid) if pid else None

    def active_id(self, leg_set: str) -> str | None:
        return self._active.get(leg_set)

    def entry_gait(self, preset_id: str, current_gait: str | None = None) -> str:
        """The gait selecting ``preset_id`` should publish.

        The one already in force where the new preset offers it: a preset change
        is a change of stance, and an operator who did not ask for a different
        walk should not get one. Its ``default_gait`` where it does not — which
        is also the answer for the leg-set change, where no six-leg gait can
        survive the crossing.

        Not a remembered per-preset slot, which this used to be: it brought back
        a gait the operator picked minutes ago on other ground, and it was fed
        by the /cmd_gait loopback, which lands while the OLD preset is still the
        one in force — so a switch recorded its own entry gait against the
        preset it was leaving, and one preset's default leaked into another's
        memory. ``current_gait`` is optional so a caller with nothing in force
        yet (config load) still gets the cold default.
        """
        preset = self._by_id[preset_id]
        if current_gait is not None and current_gait in preset.gait_cycle:
            return current_gait
        return preset.default_gait

    def for_gait(self, gait: str) -> Preset | None:
        """A preset whose rotation contains ``gait``.

        Ambiguous by design now that rotations may overlap — several six-leg
        presets can offer the same gaits. The preset in force wins when it is
        one of them, which is the answer that matters; otherwise the first
        declared match, which only comes up before ``/gait/preset`` has spoken.
        """
        current = self.current()
        if current is not None and gait in current.gait_cycle:
            return current
        for p in self._presets:
            if gait in p.gait_cycle:
                return p
        return None

    def note_gait(self, gait: str) -> Preset | None:
        """Record that ``gait`` is now in force. Returns its preset, if any.

        Feeds ``default_quadruped_gait`` only; a preset change reads the gait in
        force instead, so the loopback's timing (this lands while the preset
        being left is still the current one) cannot misdirect a selection.
        """
        preset = self.for_gait(gait)
        if preset is None:
            return None
        self._slot[preset.id] = gait
        return preset

    def note_preset(self, preset_id: str) -> Preset | None:
        """Record the preset the engine reports on ``/gait/preset``."""
        preset = self._by_id.get(preset_id)
        if preset is None:
            return None
        self._current = preset.id
        self._active[preset.leg_set] = preset.id
        return preset

    def project(self, cfg: JoyConfig) -> JoyConfig:
        """Write the active presets' rotations into the two ``JoyConfig`` fields.

        This is the whole trick that keeps ``map_joy`` untouched: it still sees
        one six-leg rotation and one four-corner one, and never learns that
        either of them can be swapped underneath it.
        """
        hexapod = self.active(HEXAPOD)
        quadruped = self.active(QUADRUPED)
        changes: dict[str, object] = {}
        if hexapod is not None:
            changes["gait_cycle"] = hexapod.gait_cycle
        if quadruped is not None:
            changes["quadruped_gait_cycle"] = quadruped.gait_cycle
            changes["default_quadruped_gait"] = self._slot[quadruped.id]
        return dataclasses.replace(cfg, **changes)  # type: ignore[arg-type]


def load_presets(
    raw: Mapping,
    tuning_yaml: str | Path,
    geometry_yaml: str | Path,
    *,
    key: str = "presets",
) -> PresetRegistry:
    """Build a registry from a config file's ``presets:`` block.

    ``tuning_yaml`` supplies each preset's leg set and its velocity caps — the
    physical half of a preset, which this file does not get to restate.
    """
    block = raw.get(key)
    if not isinstance(block, Mapping):
        raise ValueError(f"{key}: block is missing")
    entries = block.get("list")
    if not isinstance(entries, Sequence) or not entries:
        raise ValueError(f"{key}.list must be a non-empty list")

    leg_sets = preset_leg_sets(tuning_yaml)
    allow_unstable = bool(raw.get("allow_unstable_gaits", False))
    unstable = frozenset(
        name for name, d in GAIT_DESCRIPTORS.items() if d.unstable
    )

    presets: list[Preset] = []
    seen_ids: set[str] = set()
    for entry in entries:
        pid = str(entry["id"])
        if pid in seen_ids:
            raise ValueError(f"{key}: duplicate preset id {pid!r}")
        seen_ids.add(pid)
        if pid not in leg_sets:
            raise ValueError(
                f"{key}.{pid} has no entry in tuning.yaml gait_node.presets, "
                f"which is where a preset's leg set and stance live; have "
                f"{sorted(leg_sets)}"
            )
        leg_set = leg_sets[pid]

        raw_cycle = tuple(str(n) for n in entry["gait_cycle"])
        # Every gait in a rotation must walk the legs the preset stands on, or a
        # prev/next press could ask a standing robot for a leg set the engine
        # would refuse. Rotations across presets may OVERLAP — several six-leg
        # presets offering the same gaits is the normal case, and it is the
        # /cmd_preset channel, not the gait name, that says which is in force.
        foreign = frozenset(
            name
            for name, d in GAIT_DESCRIPTORS.items()
            if d.leg_set != leg_set
        )
        cycle = resolve_gait_cycle(
            raw_cycle,
            set(GAIT_DESCRIPTORS),
            unstable,
            allow_unstable,
            foreign_gaits=foreign,
            key=f"{key}.{pid}.gait_cycle",
        )

        default_gait = str(entry["default_gait"])
        if default_gait not in cycle:
            detail = (
                "is excluded by allow_unstable_gaits: false"
                if default_gait in raw_cycle
                else f"must be in gait_cycle={list(raw_cycle)}"
            )
            raise ValueError(
                f"{key}.{pid}.default_gait={default_gait!r} {detail}"
            )

        presets.append(
            Preset(
                id=pid,
                label=str(entry.get("label", pid.upper())),
                sub=str(entry.get("sub", "")),
                gait_cycle=cycle,
                default_gait=default_gait,
                leg_set=leg_set,
            )
        )

    caps_by_preset = {
        p.id: load_velocity_caps(tuning_yaml, geometry_yaml, p.id)
        for p in presets
    }
    return PresetRegistry(
        presets,
        str(block.get("default", presets[0].id)),
        float(block.get("switch_timeout_s", 4.0)),
        caps_by_preset,
    )


def resync_gait(
    name: str,
    cfg: JoyConfig,
    state: JoyState,
    registry: PresetRegistry,
) -> ResyncResult | None:
    """Follow a ``/cmd_gait`` value: stick caps and the cycler slot.

    Fed every value on the wire — this teleop's own accepted publishes, heard
    back via loopback, and the other teleop's — so both initiators take one
    bookkeeping path. Returns ``None`` when the name is unknown to the caps
    table (a foreign string on the topic; the caller logs and keeps its caps).

    The LEG SET is not this function's business any more: the preset owns it and
    ``resync_preset`` follows it. A gait valid in the catalog but outside every
    rotation (the other teleop may allow unstable gaits this config excludes)
    still gets its caps; the presets keep their positions, so this teleop's next
    prev/next resumes from where its own selection left off.
    """
    caps = registry.caps()
    try:
        new_linear = caps.linear_max(name)
        new_angular = caps.angular_max(name)
    except KeyError:
        return None

    preset = registry.note_gait(name)
    new_cfg = registry.project(cfg)
    if name in new_cfg.quadruped_gait_cycle:
        state.current_quadruped_gait_idx = new_cfg.quadruped_gait_cycle.index(name)
    if name in new_cfg.gait_cycle:
        state.current_gait_idx = new_cfg.gait_cycle.index(name)

    return ResyncResult(
        cfg=dataclasses.replace(
            new_cfg, gait_linear_max=new_linear, gait_angular_z_max=new_angular
        ),
        preset=preset,
        leg_set_changed=False,
        left_animation_mode=False,
    )


def resync_preset_request(
    preset_id: str,
    state: JoyState,
    registry: PresetRegistry,
) -> Preset | None:
    """Follow a ``/cmd_preset`` value: ease the operator's posture out.

    The REQUEST, not the report — and it has to be the request, because the
    engine will not start the change until the body pose is back at neutral, and
    this decay is the only thing that puts it there. By the time
    ``/gait/preset`` reports the result it is far too late.

    Both teleops subscribe the topic, own publishes included via loopback, so a
    switch made from either one reverts both their postures. Returns ``None`` on
    an id this config does not declare, and does nothing when the request names
    the preset already in force.
    """
    preset = registry.get(preset_id)
    if preset is None or preset_id == registry.current_id():
        return None
    # A body pose is applied to a planted leg and not to a parked one, so the
    # middle pair may only cross between the two at a neutral pose; and even
    # between two six-leg presets the reseat wants a neutral pose to re-plant
    # against. So every preset change reverts, not only a leg-set change.
    state.reverting = True
    return preset


def resync_preset(
    preset_id: str,
    gait: str,
    cfg: JoyConfig,
    state: JoyState,
    registry: PresetRegistry,
) -> ResyncResult | None:
    """Follow a ``/gait/preset`` report: caps, leg set, rotations, posture.

    The engine's own report of the preset it has APPLIED, so this is where the
    leg-set flag ``map_joy`` reads is set, and where the caps swap — every cap
    is derived from the preset's stride, swing time and stance. Returns ``None``
    on an id this config does not declare.

    ``gait`` is the gait currently in force, needed only to re-key the caps
    onto the new preset's table.
    """
    preset = registry.get(preset_id)
    if preset is None:
        return None

    was_quadruped = state.quadruped
    now_quadruped = preset.leg_set == QUADRUPED
    registry.note_preset(preset_id)
    new_cfg = registry.project(cfg)

    caps = registry.caps(preset_id)
    try:
        new_linear = caps.linear_max(gait)
        new_angular = caps.angular_max(gait)
    except KeyError:
        new_linear = cfg.gait_linear_max
        new_angular = cfg.gait_angular_z_max

    # The flag map_joy reads to pick which rotation the D-pad walks and to gate
    # animation mode. Not setting it is the bug this module exists to fix: a
    # gamepad `select` used to leave the web app cycling the six-leg rotation
    # while the robot stood on four legs.
    state.quadruped = now_quadruped
    leg_set_changed = was_quadruped != now_quadruped

    left_animation_mode = now_quadruped and state.mode == ANIMATION
    if left_animation_mode:
        state.mode = GAIT

    return ResyncResult(
        cfg=dataclasses.replace(
            new_cfg, gait_linear_max=new_linear, gait_angular_z_max=new_angular
        ),
        preset=preset,
        leg_set_changed=leg_set_changed,
        left_animation_mode=left_animation_mode,
    )
