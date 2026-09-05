"""Operator presets: named bundles of a gait rotation, shared by both teleops.

A **preset** is what the operator picks as one thing — ``NORMAL`` or ``QUAD``
today, more later. It carries an id, a label, a gait rotation and the gait it
enters on. Its **leg set is derived** from ``hexa_common.gait_catalog``, never
declared: that catalog already owns which legs a gait walks, and a second copy
in ``teleop_joy.yaml`` would be a second copy to keep true.

Selecting a preset publishes its remembered gait on ``/cmd_gait``. It needs no
channel of its own for the same reason the leg set does not — the gait carries
it. That also means the *other* teleop, and this one's own loopback, see every
switch on the same topic, which is what ``resync`` is for.

Why the projection, rather than a general N-preset state machine: ``map_joy``
holds exactly two rotations and a ``quadruped`` flag, and it is parity-locked to
``shared/motion_core/joy_mapping.cpp`` by a golden trace. So the registry keeps
the presets and *projects* the active one onto the two ``JoyConfig`` fields
``map_joy`` already reads. A third preset is then a config edit: entering it
swaps ``cfg.gait_cycle``, and ``map_joy`` never learns it exists.

Pure Python; rclpy-free, so both nodes' bookkeeping is unit-testable without a
ROS context. It lives here rather than in ``hexa_common`` (a leaf that knows
nothing of ``JoyConfig`` / ``JoyState``), in ``joy_mapping`` (parity-locked), or
in ``hexa_webteleop`` (the dependency runs the other way).
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Mapping, Sequence

from hexa_common import VelocityCaps
from hexa_common.gait_catalog import GAIT_DESCRIPTORS

from .joy_mapping import (
    ANIMATION,
    GAIT,
    JoyConfig,
    JoyState,
    resolve_gait_cycle,
)

HEXAPOD = "hexapod"
QUADRUPED = "quadruped"

# Engine states (/gait/state) a leg-set change may be asked for from. Narrower
# than the gait-switch set on purpose: a plain gait swap is legal while walking
# or settling because the engine can latch it and apply it at the next stand,
# but a leg-set change moves the middle pair and re-plants four corners, so it
# only runs from a stand and is refused everywhere else. Pre-gating here (rather
# than letting the engine refuse) matters because /cmd_gait is latched: a
# refused name would sit on the wire forever, and every late subscriber would
# read a leg set the robot never took.
LEG_SET_SWITCH_STATES: frozenset[str] = frozenset({"folded", "fault", "stand"})


def leg_set_switch_allowed(gait_state: str) -> bool:
    """True if a leg-set change may be published in ``gait_state``."""
    return gait_state in LEG_SET_SWITCH_STATES


@dataclass(frozen=True)
class Preset:
    """One operator-facing preset. ``leg_set`` is derived, never declared."""

    id: str
    label: str
    sub: str
    gait_cycle: tuple[str, ...]
    default_gait: str
    leg_set: str


@dataclass
class ResyncResult:
    """What a ``/cmd_gait`` value did to this teleop's own bookkeeping."""

    cfg: JoyConfig
    preset: Preset | None
    # The gait named a different leg set than the one in force. The caller eases
    # its posture out on this: a body pose is applied to a planted leg and not
    # to a parked one, so the middle pair may only cross between the two at a
    # neutral pose, and the engine waits for it before moving anything.
    leg_set_changed: bool
    # Quadruped mode has no animations (every one is written for six legs), and
    # ``map_joy`` only blocks *entering* the mode. Arriving on four legs while
    # already in it has to drop out, which the caller then has to tell the
    # engine about by publishing an empty /animation/mode.
    left_animation_mode: bool


class PresetRegistry:
    """The declared presets, plus which one is active per leg set.

    Two "active" slots rather than one, because the leg set is what the robot is
    physically standing on: the six-leg selection an operator was using is still
    theirs when they come back from four legs, exactly as the two cycler indexes
    already behave.
    """

    def __init__(
        self,
        presets: Sequence[Preset],
        default_id: str,
        switch_timeout_s: float,
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
        # Which preset is in force for each leg set. Seeded from the default for
        # its own set, and from the first declared preset for the other one.
        self._active: dict[str, str] = {}
        for p in self._presets:
            self._active.setdefault(p.leg_set, p.id)
        self._active[self._by_id[default_id].leg_set] = default_id
        # Each preset's remembered slot in its own rotation, seeded from its
        # default_gait so the first selection matches the init button exactly.
        self._slot: dict[str, str] = {p.id: p.default_gait for p in self._presets}

    @property
    def presets(self) -> tuple[Preset, ...]:
        return self._presets

    def get(self, preset_id: str) -> Preset | None:
        return self._by_id.get(preset_id)

    def for_gait(self, gait: str) -> Preset | None:
        """The preset whose rotation contains ``gait``.

        Unique because the rotations are validated pairwise disjoint at load —
        which is what lets a bare gait name on ``/cmd_gait`` say which preset the
        operator is in without a second field on the wire.
        """
        for p in self._presets:
            if gait in p.gait_cycle:
                return p
        return None

    def active(self, leg_set: str) -> Preset | None:
        pid = self._active.get(leg_set)
        return self._by_id.get(pid) if pid else None

    def active_id(self, leg_set: str) -> str | None:
        return self._active.get(leg_set)

    def entry_gait(self, preset_id: str) -> str:
        """The gait selecting ``preset_id`` should publish.

        Its remembered slot, so NORMAL -> QUAD -> NORMAL lands on the gait the
        operator left, and a first selection lands on ``default_gait``.
        """
        return self._slot[preset_id]

    def note_gait(self, gait: str) -> Preset | None:
        """Record that ``gait`` is now in force. Returns its preset, if any."""
        preset = self.for_gait(gait)
        if preset is None:
            return None
        self._slot[preset.id] = gait
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


def _derive_leg_set(preset_id: str, gaits: Sequence[str]) -> str:
    """The one leg set every gait in a rotation walks.

    A mixed rotation is a load-time error rather than a cycler press the engine
    silently refuses: prev/next must never be able to ask a standing robot for a
    leg set it cannot reach without the whole leg-set change running.
    """
    sets = {GAIT_DESCRIPTORS[g].leg_set for g in gaits}
    if len(sets) != 1:
        raise ValueError(
            f"preset {preset_id!r} mixes leg sets {sorted(sets)}; every gait in "
            f"one preset's rotation must walk the same legs"
        )
    return sets.pop()


def load_presets(raw: Mapping, *, key: str = "presets") -> PresetRegistry:
    """Build a registry from a config file's ``presets:`` block."""
    block = raw.get(key)
    if not isinstance(block, Mapping):
        raise ValueError(f"{key}: block is missing")
    entries = block.get("list")
    if not isinstance(entries, Sequence) or not entries:
        raise ValueError(f"{key}.list must be a non-empty list")

    allow_unstable = bool(raw.get("allow_unstable_gaits", False))
    unstable = frozenset(
        name for name, d in GAIT_DESCRIPTORS.items() if d.unstable
    )

    presets: list[Preset] = []
    seen_ids: set[str] = set()
    claimed: dict[str, str] = {}
    for entry in entries:
        pid = str(entry["id"])
        if pid in seen_ids:
            raise ValueError(f"{key}: duplicate preset id {pid!r}")
        seen_ids.add(pid)

        raw_cycle = tuple(str(n) for n in entry["gait_cycle"])
        # Every other preset's gaits are foreign to this one, which generalises
        # the old hexapod/quadruped split: the rotations stay pairwise disjoint,
        # so a gait name maps to exactly one preset.
        foreign = frozenset(
            g for p in presets for g in p.gait_cycle
        ) | frozenset(
            str(n)
            for other in entries
            if str(other["id"]) != pid
            for n in other["gait_cycle"]
        )
        cycle = resolve_gait_cycle(
            raw_cycle,
            set(GAIT_DESCRIPTORS),
            unstable,
            allow_unstable,
            foreign_gaits=foreign,
            key=f"{key}.{pid}.gait_cycle",
        )
        for gait in cycle:
            if gait in claimed:
                raise ValueError(
                    f"{key}: {gait!r} is in both {claimed[gait]!r} and {pid!r}; "
                    f"rotations must be disjoint so a gait names one preset"
                )
            claimed[gait] = pid

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
                leg_set=_derive_leg_set(pid, cycle),
            )
        )

    return PresetRegistry(
        presets,
        str(block.get("default", presets[0].id)),
        float(block.get("switch_timeout_s", 4.0)),
    )


def resync(
    name: str,
    cfg: JoyConfig,
    state: JoyState,
    caps: VelocityCaps,
    registry: PresetRegistry,
) -> ResyncResult | None:
    """Follow a ``/cmd_gait`` value: caps, cycler, leg set, active preset.

    Fed every value on the wire — this teleop's own accepted publishes, heard
    back via loopback, and the other teleop's — so both initiators take one
    bookkeeping path. Returns ``None`` when the name is unknown to the caps
    table (a foreign string on the topic; the caller logs and keeps its caps).

    A gait valid in the catalog but outside every preset's rotation (the other
    teleop may allow unstable gaits this config excludes) still gets its caps;
    the presets keep their positions, so this teleop's next prev/next resumes
    from where its own selection left off.
    """
    try:
        new_linear = caps.linear_max(name)
        new_angular = caps.angular_max(name)
    except KeyError:
        return None

    descriptor = GAIT_DESCRIPTORS.get(name)
    leg_set = descriptor.leg_set if descriptor else HEXAPOD
    was_quadruped = state.quadruped
    now_quadruped = leg_set == QUADRUPED

    preset = registry.note_gait(name)
    new_cfg = registry.project(cfg)

    # The flag map_joy reads to pick which rotation the D-pad walks and to gate
    # animation mode. Not setting it is the bug this module exists to fix: a
    # gamepad `select` used to leave the web app cycling the six-leg rotation
    # while the robot stood on four legs.
    state.quadruped = now_quadruped
    if now_quadruped:
        if name in new_cfg.quadruped_gait_cycle:
            state.current_quadruped_gait_idx = new_cfg.quadruped_gait_cycle.index(name)
    elif name in new_cfg.gait_cycle:
        state.current_gait_idx = new_cfg.gait_cycle.index(name)

    leg_set_changed = was_quadruped != now_quadruped
    if leg_set_changed:
        # Ease the operator's recorded posture out. The engine will not move the
        # middle pair until the body pose is back at neutral, and this is the
        # only thing that puts it there — map_joy's own revert decay, which also
        # takes the body height with it.
        state.reverting = True

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
