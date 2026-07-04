"""Gait engine — the only stateful component in the gait chain.

Strategies stay pure; the engagement/pause/reseat controllers each own
a per-cycle slice of state. The engine routes between modes on commanded
body velocity:

- **STAND**     — cmd zero, nothing to preserve. Emit nominal stance.
- **ENGAGING**  — cmd just went non-zero from STAND. EngagementController
  (engage mode) runs one full master cycle: v_body ramps 0→target on a
  smoothstep over the earliest-touchdown horizon, each leg takes one
  "from NOMINAL" swing, then hands off to GAIT at master=1.0 (≡0.0) with
  every leg already on its strategy curve — a continuous handoff.
- **GAIT**      — cmd non-zero. Advance the clock, evaluate the strategy.
- **PAUSING**   — cmd went zero (debounced from GAIT, immediate from
  ENGAGING). PauseController lowers airborne legs straight down to
  nominal.z (XY frozen); stance legs hold. Master/per-leg phase/β kept
  so re-engage doesn't reset the cycle.
- **PAUSED**    — all airborne legs landed. Hold, tick _paused_elapsed;
  cmd non-zero → RESUMING, else after pause_to_reseat_delay → RESEATING.
- **RESUMING**  — cmd non-zero from PAUSING/PAUSED. EngagementController
  (resume mode) seeded from paused phase + last_targets: previously
  airborne legs merge-arc from lowered Z up to live AEP, previously
  stance legs integrate then swing once. Hands off once every leg is on
  its strategy branch.

GAIT→PAUSING is debounced (pause_debounce_delay) against brief joystick
zero-crossings; ENGAGING→PAUSING is not (see ``update``). PAUSING↔RESUMING
are mutually interruptible.

A gait change while walking (``set_strategy`` in GAIT / PAUSING / PAUSED /
RESEATING) latches a pending name and forces PAUSING → PAUSED (short
gait_change_pause_to_reseat_delay dwell) → RESEATING, suppressing the
resume exits so a held stick can't abort it. The name commits at the
RESEATING→STAND handoff (re-engaging in the new gait if cmd is still
non-zero). Mid-sequence requests overwrite the pending name; ENGAGING
and RESUMING drop requests (gait locked).

``cycle_time`` is derived per GAIT tick from velocity, ``stride_length``,
and the strategy's β: faster ⇒ shorter cycle at constant stride. Bounds
come from swing-time knobs scaled by ``1/(1−β)`` so the swing-phase
foot-velocity envelope is gait-agnostic — only the cycle stretches with β.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Mapping

from hexa_kinematics.body_transform import leg_to_body
from hexa_kinematics.joint_config import (
    StandingPoseDeg,
    load_initial_pose,
    load_standing_pose,
)
from hexa_kinematics.leg_geometry import LegSpec
from hexa_kinematics.leg_ik import forward_kinematics
from hexa_kinematics.leg_specs import LEG_NAMES, load_leg_specs

from .clock import GaitClock
from .engagement import EngagementController, EngagementState
from .fold import FoldController
from .gaits import STRATEGIES
from .gaits.base import (
    LegContext,
    Strategy,
    StrideParams,
    derive_cycle_time,
    identity_y_sign,
    live_aep,
    per_leg_planar_velocity,
    stride_vector,
    swing_arc,
)
from .initialize import InitializeController
from .pause import LegOutput, PauseController, PauseState
from .reseat import ReseatController, ReseatGeometry, reseat_nominal_stance


# "Is the D-pad still moving?" epsilon. Must be well below the teleop's
# 1 mm/tick height slew (0.05 m/s × 50 Hz) — which itself sits on the
# YAML dead-band — so a held D-pad reliably resets the settle timer.
_HEIGHT_NOISE_EPSILON: float = 1e-6

# Inclusive swing→stance boundary tolerance. ``swing_end = 1 - β`` isn't
# exactly representable (1 - 2/3 == 0.3333…37), so a just-landed foot can
# compute one ULP below it and a bare ``phase >= swing_end`` misflags it
# airborne for a tick — dropping the support set onto a lopsided triangle
# when touchdowns coincide with a lift-off (β = 2/3). Far below one phase
# tick (≈0.02), so it absorbs only float noise.
_STANCE_SEAM_EPSILON: float = 1e-9


__all__ = [
    "Engine",
    "EngineConfig",
    "EngineState",
    "LegOutput",
    "StanceIntegrator",
    "SwingPlanner",
    "build_leg_contexts",
    "initial_stance_from_yaml",
    "nominal_stance_from_yaml",
    "reseat_geometry_from_yaml",
]

Vec3 = tuple[float, float, float]


class EngineState(Enum):
    # FOLDED: cold-start — legs at initial_pose, body on belly, awaiting
    #   an operator trigger; cmd_vel ignored so power-on doesn't move the
    #   robot while cables are attached.
    # FOLDING: symmetric operator-gated warm shutdown, STAND → FOLDED.
    # RESEATING: standing-pose restore — after a settled /body/pose.z
    #   change, walk each foot pair to a nominal stance that restores the
    #   YAML default joint angles at the new body height.
    FOLDED = "folded"
    INITIALIZE = "initialize"
    STAND = "stand"
    ENGAGING = "engaging"
    GAIT = "gait"
    PAUSING = "pausing"
    PAUSED = "paused"
    RESUMING = "resuming"
    FOLDING = "folding"
    RESEATING = "reseating"


@dataclass(frozen=True)
class EngineConfig:
    """Engine-internal knobs from ``tuning.yaml`` (``gait_node`` block);
    none are on the wire.

    ``stride_length`` + ``min/max_swing_time`` define the velocity →
    cycle_time map applied each GAIT tick. β lives on the active
    ``Strategy`` (varies with gait); the engine scales the swing-time
    bounds by it into per-gait cycle-time bounds.
    """

    stride_length: float
    min_swing_time: float
    max_swing_time: float
    step_height: float
    swing_width: float
    controller_dt: float
    cmd_zero_tol: float
    # GAIT → PAUSING debounce; brief zero-crossings under it keep GAIT
    # ticking at zero stride.
    pause_debounce_delay: float
    # PAUSED → RESEATING dwell (feet walk back to the nominal footprint
    # so the robot visibly settles).
    pause_to_reseat_delay: float
    # Shorter PAUSED → RESEATING dwell while a gait change is pending, so
    # a mid-walk switch feels immediate.
    gait_change_pause_to_reseat_delay: float
    # Upper clamp on a paused foot's straight-down lowering time
    # (``distance_z / (stride_length / min_swing_time)`` clamped to
    # ``[min_swing_time, max_reset_time]``) — descent speed tracks the
    # fastest gait's foot-velocity ceiling for consistent feel.
    max_reset_time: float
    # INITIALIZE cold-start knobs: PLACE_FEET per-pair duration, LIFT_BODY
    # z-ramp duration, PLACE_FEET arc clearance, and the IK target's
    # above-floor offset. The last must absorb the URDF's vertical-tibia
    # assumption so the foot sphere doesn't penetrate the floor (geometry
    # in the tuning.yaml comment).
    init_pair_swing_time: float
    init_lift_body_time: float
    init_swing_clearance: float
    init_place_feet_clearance: float
    # RESEATING knobs. ``reseat_pose_settle_delay``: dwell pose.z must
    # hold after D-pad release before STAND → RESEATING fires (distinct
    # from the PAUSED → RESEATING dwell above).
    # ``reseat_height_change_threshold``: dead-band for "target differs
    # from applied enough to reseat" (1 mm) — the settle timer itself
    # resets on the much tighter float-noise epsilon so a held D-pad
    # keeps it pinned. ``reseat_pair_swing_time`` matches initialize for
    # symmetry; ``reseat_pair_dwell_time`` lets each pair settle before
    # the next lifts; ``reseat_swing_clearance`` clears ground noise (feet
    # start planted).
    reseat_pose_settle_delay: float
    reseat_height_change_threshold: float
    reseat_pair_swing_time: float
    reseat_pair_dwell_time: float
    reseat_swing_clearance: float


def nominal_stance_from_yaml(
    geometry_yaml: str | Path,
    standing: StandingPoseDeg,
) -> dict[str, Vec3]:
    """Body-frame foot position per leg at the standing pose.

    Segments from ``geometry.yaml``, standing angles from the gait node's
    ros params. Routed through ``leg_to_body`` so the trig lives in one
    place.
    """
    legs = load_leg_specs(geometry_yaml)
    angles = load_standing_pose(standing, geometry_yaml)
    return {n: leg_to_body(forward_kinematics(angles, legs[n]), legs[n]) for n in LEG_NAMES}


def reseat_geometry_from_yaml(
    geometry_yaml: str | Path,
    standing: StandingPoseDeg,
) -> ReseatGeometry:
    """Build the ``ReseatGeometry`` snapshot used by the engine.

    Derives the tibia-from-vertical angle and default foot depth via the
    same FK helper as ``nominal_stance_from_yaml``. Any leg works as the
    reference (all six share segment lengths); LEG_NAMES[0] is picked
    deterministically.
    """
    from .reseat import default_geometry_from_pose

    legs = load_leg_specs(geometry_yaml)
    angles = load_standing_pose(standing, geometry_yaml)
    return default_geometry_from_pose(angles, legs[LEG_NAMES[0]])


def initial_stance_from_yaml(geometry_yaml: str | Path) -> dict[str, Vec3]:
    """Body-frame foot position per leg at the YAML ``initial_pose``.

    Sibling of ``nominal_stance_from_yaml`` with angles from
    ``geometry.yaml``'s ``initial_pose:`` block. Seeds INITIALIZE's
    PLACE_FEET swing origins — the foot positions at power-on with the
    body on its belly and legs folded up.
    """
    legs = load_leg_specs(geometry_yaml)
    angles_per_leg = load_initial_pose(geometry_yaml)
    return {
        n: leg_to_body(forward_kinematics(angles_per_leg[n], legs[n]), legs[n])
        for n in LEG_NAMES
    }


def _cycle_time_bounds(cfg: "EngineConfig", beta: float) -> tuple[float, float]:
    """Per-gait cycle-time bounds derived from swing-phase bounds.

    Both ends scale by ``1/(1−β)`` so the swing-phase foot-velocity
    envelope is gait-agnostic; only the cycle stretches as β grows. The
    ``β >= 1`` branch (degenerate all-stance, never reached) keeps the
    helper total.
    """
    if beta >= 1.0:
        return cfg.max_swing_time, cfg.max_swing_time
    scale = 1.0 / (1.0 - beta)
    return cfg.min_swing_time * scale, cfg.max_swing_time * scale


@dataclass
class StanceIntegrator:
    """Per-leg body-frame stance target integrated from touchdown.

    Problem: strategies rebuild PEP/AEP from live stride each tick, so a
    velocity change off lift-off snaps every stance leg by
    ``(0.5 − s)·Δstride`` — a non-uniform shear that's masked at tripod
    (β = 0.5, shared s) but scrubs feet visibly at ripple/crawl.

    Fix: capture the body-frame foot position at touchdown as a
    world-locked anchor, then decrement it by ``v_leg · dt`` each stance
    tick — history-dependent instead of rebuilt from instantaneous
    stride. Swing is unaffected (body-frame planning curve). Under
    constant velocity this reproduces the closed-form stance Bezier
    exactly (colinear, evenly-spaced nodes → linear interp).
    """

    leg_names: tuple[str, ...]
    anchor: dict[str, Vec3] = field(default_factory=dict)
    is_stance: dict[str, bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for n in self.leg_names:
            self.anchor.setdefault(n, (0.0, 0.0, 0.0))
            self.is_stance.setdefault(n, False)

    def seed(
        self,
        last_targets: Mapping[str, Vec3],
        last_stance: Mapping[str, bool],
    ) -> None:
        """Capture current foot positions as stance anchors.

        Called on every GAIT entry so legs arriving mid-stance (from
        ENGAGING/RESUMING) integrate from where they are, not from their
        next swing.
        """
        for n in self.leg_names:
            self.anchor[n] = tuple(last_targets[n])  # type: ignore[assignment]
            self.is_stance[n] = bool(last_stance[n])

    def step(
        self,
        name: str,
        in_stance: bool,
        swing_target: Vec3,
        v_leg: tuple[float, float],
        dt: float,
    ) -> Vec3 | None:
        """Advance the integrator one tick.

        Returns the integrated target if in stance, else ``None``. On the
        swing → stance edge ``swing_target`` becomes the new anchor and is
        returned as-is; integration starts next tick.
        """
        if not in_stance:
            self.is_stance[name] = False
            return None
        if not self.is_stance[name]:
            self.anchor[name] = tuple(swing_target)  # type: ignore[assignment]
            self.is_stance[name] = True
            return self.anchor[name]
        a = self.anchor[name]
        self.anchor[name] = (a[0] - v_leg[0] * dt, a[1] - v_leg[1] * dt, a[2])
        return self.anchor[name]

    def reset(self) -> None:
        for n in self.leg_names:
            self.is_stance[n] = False


@dataclass
class SwingPlanner:
    """Per-leg swing plan latched at lift-off, held until touchdown.

    Problem (swing-side mirror of ``StanceIntegrator``): strategies
    rebuild PEP/AEP from live stride each tick, so a mid-swing velocity
    change re-evaluates the Bezier against a moved origin/target and the
    airborne foot jumps by a fraction of Δstride. Tripod hides it (short
    strides, 3 swing legs); ripple (β = 5/6) has ~5× the shift on a
    single airborne leg — visible whenever v_y or ω_z is added to v_x.

    Fix: at lift-off capture
      * ``origin`` — actual body-frame position (= last stance anchor),
        for C0 continuity even after a varying-velocity stance;
      * ``target`` — the live AEP (``nominal + 0.5·stride``);
      * ``v_leg``  — used as both swing endpoint velocities so the arc
        launches/lands at stance-frame velocity ``-v_leg``. The
        ``-stride/swing_time`` default is right only at β = 0.5 (2× at
        crawl, 5× at ripple), stepping velocity at every touchdown and
        scrubbing the loaded feet;
      * ``swing_time`` + ``identity_y_sign`` — fix the Bezier nodes.
    ``evaluate`` reads these latched values; at touchdown the integrator
    anchors on ``target`` (exact swing → stance) and the leg is released.

    Reset on every GAIT entry: engagement/resume plan their own swings,
    so a leg still airborne at handoff trips a fresh lift-off next tick.
    """

    leg_names: tuple[str, ...]
    origin: dict[str, Vec3] = field(default_factory=dict)
    target: dict[str, Vec3] = field(default_factory=dict)
    v_leg: dict[str, tuple[float, float]] = field(default_factory=dict)
    swing_time: dict[str, float] = field(default_factory=dict)
    identity_y_sign: dict[str, int] = field(default_factory=dict)
    is_swing: dict[str, bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for n in self.leg_names:
            self.origin.setdefault(n, (0.0, 0.0, 0.0))
            self.target.setdefault(n, (0.0, 0.0, 0.0))
            self.v_leg.setdefault(n, (0.0, 0.0))
            self.swing_time.setdefault(n, 0.0)
            self.identity_y_sign.setdefault(n, 1)
            self.is_swing.setdefault(n, False)

    def liftoff(
        self,
        name: str,
        origin: Vec3,
        target: Vec3,
        v_leg: tuple[float, float],
        swing_time: float,
        identity_y_sign_val: int,
    ) -> None:
        self.origin[name] = tuple(origin)  # type: ignore[assignment]
        self.target[name] = tuple(target)  # type: ignore[assignment]
        self.v_leg[name] = (float(v_leg[0]), float(v_leg[1]))
        self.swing_time[name] = float(swing_time)
        self.identity_y_sign[name] = int(identity_y_sign_val)
        self.is_swing[name] = True

    def touchdown(self, name: str) -> None:
        self.is_swing[name] = False

    def evaluate(
        self,
        name: str,
        phase_in_swing: float,
        swing_clearance: float,
        swing_width: float,
        controller_dt: float,
    ) -> Vec3:
        vx, vy = self.v_leg[name]
        # Stance-frame foot velocity is -v_leg; both endpoints so the
        # Bezier's C1 nodes match it at lift-off and touchdown (the
        # -stride/swing_time default only matches at β = 0.5).
        v_match = (-vx, -vy, 0.0)
        return swing_arc(
            phase_in_swing=phase_in_swing,
            swing_origin=self.origin[name],
            target=self.target[name],
            swing_clearance=swing_clearance,
            swing_width=swing_width,
            identity_y_sign=self.identity_y_sign[name],
            swing_time=self.swing_time[name],
            controller_dt=controller_dt,
            swing_origin_velocity=v_match,
            swing_target_velocity=v_match,
        )

    def reset(self) -> None:
        for n in self.leg_names:
            self.is_swing[n] = False


class Engine:
    """Per-tick gait engine.

    ``update(dt, v_body_xy, omega_z)`` returns one ``LegOutput`` per leg.
    Cold start is FOLDED; ``cycle_time`` is derived per tick from
    velocity, ``stride_length``, β, and the swing-time bounds.
    """

    def __init__(
        self,
        config: EngineConfig,
        strategy: Strategy,
        nominal_stance: Mapping[str, Vec3],
        initial_stance: Mapping[str, Vec3],
        coxa_to_bottom: float,
        leg_contexts: Mapping[str, LegContext],
        leg_specs: Mapping[str, LegSpec] | None = None,
        reseat_geometry: ReseatGeometry | None = None,
    ) -> None:
        missing = set(LEG_NAMES) - set(nominal_stance)
        if missing:
            raise ValueError(f"nominal_stance missing legs: {sorted(missing)}")
        missing = set(LEG_NAMES) - set(initial_stance)
        if missing:
            raise ValueError(f"initial_stance missing legs: {sorted(missing)}")
        missing = set(LEG_NAMES) - set(leg_contexts)
        if missing:
            raise ValueError(f"leg_contexts missing legs: {sorted(missing)}")
        if (leg_specs is None) != (reseat_geometry is None):
            raise ValueError(
                "leg_specs and reseat_geometry must be supplied together "
                "(both None disables reseat, both set enables it)"
            )

        self._config = config
        self._strategy = strategy
        self._nominal: dict[str, Vec3] = {n: tuple(nominal_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._initial: dict[str, Vec3] = {n: tuple(initial_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._coxa_to_bottom = coxa_to_bottom
        self._legs: dict[str, LegContext] = dict(leg_contexts)
        self._leg_specs: dict[str, LegSpec] | None = (
            dict(leg_specs) if leg_specs is not None else None
        )
        self._reseat_geometry: ReseatGeometry | None = reseat_geometry

        self._clock = GaitClock(strategy.phase_offsets)
        self._stance = StanceIntegrator(tuple(LEG_NAMES))
        self._swing = SwingPlanner(tuple(LEG_NAMES))
        self._pause = self._build_pause()
        self._engagement = self._build_engagement()
        self._initialize = self._build_initialize()
        # Built fresh on each operator trigger so every fold gets a clean
        # ladder.
        self._fold: FoldController | None = None
        # Built each time the height settles to a new value in STAND;
        # then run to completion.
        self._reseat: ReseatController | None = None

        # Cold start FOLDED: emit initial_stance, ignore cmd_vel until
        # ``start_initialize``. Power-on must not move the robot.
        self._state = EngineState.FOLDED
        self._last_targets: dict[str, Vec3] = dict(self._initial)
        self._last_stance: dict[str, bool] = {n: True for n in LEG_NAMES}
        # cmd → 0 debounce timer; GAIT → PAUSING only fires once it
        # exceeds ``pause_debounce_delay``. ENGAGING bypasses it (see
        # ``update``).
        self._cmd_zero_elapsed = 0.0
        # PAUSED dwell timer; crossing ``pause_to_reseat_delay`` starts
        # RESEATING. Reset on PAUSING entry.
        self._paused_elapsed: float = 0.0
        # Airborne set at the most-recent PAUSING entry, handed to
        # RESUMING so those legs get merge arcs (lowered Z → live AEP).
        self._last_swing_flags: dict[str, bool] = {n: False for n in LEG_NAMES}

        # Reseat height tracking. ``_applied_height``: pose.z the current
        # ``_nominal`` was computed at (0 = YAML standing pose).
        # ``_target_height``: latest operator command. The stability
        # timer fires the reseat ladder once it passes the settle delay
        # with target ≠ applied.
        self._applied_height: float = 0.0
        self._target_height: float = 0.0
        self._height_stable_elapsed: float = 0.0
        # Latched by ``request_fold``; consumed on the next STAND with
        # height at 0. Drives the teleop two-press scheme (press 1 snaps
        # height → 0 / reseat, press 2 queues the fold).
        self._pending_fold: bool = False
        # Latched by ``set_strategy`` while walking / mid pause-reseat;
        # committed at the RESEATING → STAND handoff. Invariant: non-None
        # only in GAIT (≤1 tick), PAUSING, PAUSED, RESEATING — pause
        # exits are suppressed while pending, and RESEATING clears it
        # before STAND, so RESUMING/ENGAGING never see it.
        self._pending_strategy_name: str | None = None

    @property
    def state(self) -> EngineState:
        return self._state

    @property
    def master_phase(self) -> float:
        """Master phase in ``[0, 1)``; ``0`` = reference-leg lift-off.

        Exposed for posture (a single shared phase signal). During
        ENGAGING/RESUMING ``_clock`` is frozen, so read the controller's
        ``exit_master`` — the same value ``_clock`` is seeded with at the
        handoff, so continuity across the boundary is exact.
        """
        if self._state in (EngineState.ENGAGING, EngineState.RESUMING):
            return self._engagement.exit_master
        return self._clock.master

    @property
    def strategy_name(self) -> str:
        """Active strategy's registry name.

        Matched by type against ``STRATEGIES``; a strategy not from the
        registry falls back to its lower-cased class name.
        """
        for name, factory in STRATEGIES.items():
            if isinstance(self._strategy, factory):  # type: ignore[arg-type]
                return name
        return type(self._strategy).__name__.lower()

    @property
    def pending_strategy_name(self) -> str | None:
        """Gait name latched for commit at the next RESEATING → STAND
        handoff, or ``None`` when no gait change is in flight."""
        return self._pending_strategy_name

    def _apply_strategy(self, name: str) -> None:
        """Install a registry strategy, rebuilding the β-dependent
        engagement controller and the phase clock (offsets change with
        the gait)."""
        self._strategy = STRATEGIES[name]()
        self._clock = GaitClock(self._strategy.phase_offsets)
        self._engagement = self._build_engagement()

    def set_strategy(self, name: str) -> bool:
        """Swap the active gait strategy.

        STAND swaps immediately. Walking / mid pause-reseat (GAIT,
        PAUSING, PAUSED, RESEATING) latches the name pending and forces
        PAUSING → PAUSED (short dwell) → RESEATING → commit at the STAND
        handoff (re-engaging if cmd is still non-zero). Mid-sequence
        calls overwrite pending (even back to the current gait: sequence
        still completes, commit is a no-op). ENGAGING/RESUMING/INITIALIZE/
        FOLDING/FOLDED refuse.

        ``True`` when applied or latched (incl. the no-op match-current
        case), ``False`` on unknown name or locked state.
        """
        if name not in STRATEGIES:
            return False
        if self._state is EngineState.STAND:
            if name != self.strategy_name:
                self._apply_strategy(name)
            return True
        if self._state in (
            EngineState.GAIT,
            EngineState.PAUSING,
            EngineState.PAUSED,
            EngineState.RESEATING,
        ):
            # Nothing to do if we'd latch the already-active gait with
            # none pending; otherwise overwrite unconditionally (cycle-
            # back still completes the sequence, commit becomes a no-op).
            if self._pending_strategy_name is None and name == self.strategy_name:
                return True
            # No transition: the next tick picks the name up (single
            # executor, no race).
            self._pending_strategy_name = name
            return True
        return False

    def start_initialize(self) -> bool:
        """Operator-gated trigger: FOLDED → INITIALIZE.

        ``True`` if transitioned, ``False`` from any other state (stray
        triggers are a safe no-op). Rebuilds the controller so a second
        cold-start after a fold gets a clean ladder.
        """
        if self._state is not EngineState.FOLDED:
            return False
        self._initialize = self._build_initialize()
        self._state = EngineState.INITIALIZE
        return True

    def start_fold(self) -> bool:
        """Operator-gated trigger: STAND → FOLDING.

        Symmetric to ``start_initialize``: ``True`` only from STAND,
        fresh ``FoldController`` each time. Prefer ``request_fold`` from
        the ROS layer (it handles the two-rapid-press-while-lifted case);
        this is kept for tests wanting the unconditional transition.
        """
        if self._state is not EngineState.STAND:
            return False
        self._fold = self._build_fold()
        self._state = EngineState.FOLDING
        return True

    def request_fold(self) -> bool:
        """Idempotent fold request.

        Latches ``_pending_fold``, consumed on the next STAND with both
        heights at zero — so the teleop two-press scheme works (press 1
        snaps height → 0 / reseat, press 2 queues the fold to fire when
        reseat finishes). ``True`` if queued (not already FOLDED/FOLDING),
        else ``False``.
        """
        if self._state is EngineState.FOLDED or self._state is EngineState.FOLDING:
            return False
        self._pending_fold = True
        return True

    def set_target_height(self, target_height: float) -> None:
        """Update the operator-commanded body height (``pose.z``).

        Any change above the float-noise epsilon resets the settle timer,
        so a held D-pad (1 mm/tick) never lets it accrue; on release the
        value stops and the timer runs to ``reseat_pose_settle_delay``.

        The YAML dead-band ``reseat_height_change_threshold`` is
        deliberately not applied here — it gates the reseat-worthiness
        check in ``update``; applying it per tick would race the slew.
        """
        if abs(target_height - self._target_height) > _HEIGHT_NOISE_EPSILON:
            self._height_stable_elapsed = 0.0
        self._target_height = float(target_height)

    def _build_initialize(self) -> InitializeController:
        cfg = self._config
        return InitializeController(
            initial_stance=self._initial,
            nominal_stance=self._nominal,
            coxa_to_bottom=self._coxa_to_bottom,
            pair_swing_time=cfg.init_pair_swing_time,
            lift_body_time=cfg.init_lift_body_time,
            swing_clearance=cfg.init_swing_clearance,
            place_feet_clearance=cfg.init_place_feet_clearance,
            swing_width=cfg.swing_width,
            controller_dt=cfg.controller_dt,
        )

    def _build_fold(self) -> FoldController:
        cfg = self._config
        return FoldController(
            initial_stance=self._initial,
            nominal_stance=self._nominal,
            coxa_to_bottom=self._coxa_to_bottom,
            pair_swing_time=cfg.init_pair_swing_time,
            lift_body_time=cfg.init_lift_body_time,
            swing_clearance=cfg.init_swing_clearance,
            place_feet_clearance=cfg.init_place_feet_clearance,
            swing_width=cfg.swing_width,
            controller_dt=cfg.controller_dt,
        )

    def _build_pause(self) -> PauseController:
        cfg = self._config
        return PauseController(
            nominal_stance=self._nominal,
            swing_clearance=cfg.step_height,
            swing_width=cfg.swing_width,
            controller_dt=cfg.controller_dt,
            descent_speed=cfg.stride_length / cfg.min_swing_time,
            min_reset_time=cfg.min_swing_time,
            max_reset_time=cfg.max_reset_time,
        )

    def _build_engagement(self) -> EngagementController:
        cfg = self._config
        beta = self._strategy.duty_factor
        min_cycle_time, max_cycle_time = _cycle_time_bounds(cfg, beta)
        return EngagementController(
            nominal_stance=self._nominal,
            stride_length=cfg.stride_length,
            min_cycle_time=min_cycle_time,
            max_cycle_time=max_cycle_time,
            duty_factor=beta,
            swing_clearance=cfg.step_height,
            swing_width=cfg.swing_width,
            controller_dt=cfg.controller_dt,
        )

    def _build_reseat(
        self,
        target_stance: Mapping[str, Vec3],
    ) -> ReseatController:
        cfg = self._config
        # Always reseat from where the feet actually are. ``_last_targets``
        # is rewritten every tick so it carries the live foot position
        # for STAND (= nominal), PAUSED (= post-pause lowered XY), or
        # any other state that delegates here.
        return ReseatController(
            current_stance=self._last_targets,
            target_stance=target_stance,
            pair_swing_time=cfg.reseat_pair_swing_time,
            pair_dwell_time=cfg.reseat_pair_dwell_time,
            swing_clearance=cfg.reseat_swing_clearance,
            controller_dt=cfg.controller_dt,
        )

    def _commit_new_nominal(
        self, new_nominal: Mapping[str, Vec3], applied_height: float
    ) -> None:
        """Adopt a new nominal stance as the engine's standing pose.

        Rebuilds the pause / engagement controllers (each caches its
        own snapshot of the nominal stance) and the per-leg
        ``LegContext`` map (the strategy reads ``leg.nominal_stance``
        from there), so subsequent ENGAGING / GAIT / PAUSING cycles
        run against the new posture.
        """
        self._nominal = {n: tuple(new_nominal[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._legs = {
            n: dataclasses.replace(self._legs[n], nominal_stance=self._nominal[n])
            for n in LEG_NAMES
        }
        self._pause = self._build_pause()
        self._engagement = self._build_engagement()
        self._applied_height = applied_height

    def update(
        self,
        dt: float,
        v_body_xy: tuple[float, float],
        omega_z: float,
    ) -> dict[str, LegOutput]:
        cmd_zero = self._cmd_is_zero(v_body_xy, omega_z)
        if cmd_zero:
            self._cmd_zero_elapsed += dt
        else:
            self._cmd_zero_elapsed = 0.0
        # Only commit to PAUSING from GAIT after cmd has stayed zero
        # long enough to be deliberate; a brief joystick zero-crossing
        # keeps GAIT ticking at zero stride. ENGAGING does not use this
        # debounce — it bails to PAUSING on the first zero tick.
        should_pause = cmd_zero and (
            self._cmd_zero_elapsed >= self._config.pause_debounce_delay
        )
        # Height-stability timer: ticks up while the target is within
        # tolerance of its previously-recorded value. ``set_target_height``
        # resets the timer on a significant change.
        self._height_stable_elapsed += dt

        if self._state is EngineState.FOLDED:
            # Operator-gated cold start: emit the folded foot positions
            # and ignore cmd_vel until ``start_initialize`` is called.
            # All legs flagged stance=True so downstream IK treats them
            # as planted (the chassis is resting on its belly with the
            # legs tucked above).
            return {
                n: LegOutput(foot_target=self._initial[n], phase=0.0, stance=True)
                for n in LEG_NAMES
            }

        if self._state is EngineState.INITIALIZE:
            # Cold-start ladder runs to completion regardless of
            # cmd_vel: the chassis is committed to the folded-to-
            # standing sequence (servos may not be reading their own
            # angle on the real robot, and we don't want to abort the
            # only path that bridges initial_pose → nominal). The
            # debounce timer keeps ticking so a cmd_vel that arrives
            # mid-sequence is honoured by ENGAGING right after STAND.
            out = self._initialize.update(dt)
            self._capture_state(out)
            if self._initialize.done:
                self._state = EngineState.STAND
                self._last_targets = dict(self._nominal)
                self._last_stance = {n: True for n in LEG_NAMES}
            return out

        if self._state is EngineState.FOLDING:
            # Symmetric to INITIALIZE: warm-shutdown ladder runs to
            # completion regardless of cmd_vel. The operator
            # explicitly asked to fold; honouring a stray cmd_vel
            # mid-sequence would leave the chassis half-lowered. After
            # the ladder hits FOLDED, cmd_vel is ignored until the
            # operator presses start again.
            assert self._fold is not None
            out = self._fold.update(dt)
            self._capture_state(out)
            if self._fold.done:
                self._state = EngineState.FOLDED
                self._last_targets = dict(self._initial)
                self._last_stance = {n: True for n in LEG_NAMES}
            return out

        if self._state is EngineState.STAND:
            if not cmd_zero:
                # Walking takes priority over a pending reseat / fold:
                # the user is explicitly commanding the body, so honour
                # that immediately. The pending flag stays latched so a
                # later return to STAND consumes it.
                self._engagement.begin(self._strategy, self._legs)
                self._state = EngineState.ENGAGING
                return self._tick_engagement(dt, v_body_xy, omega_z)
            # Pending fold takes priority over reseat at zero height —
            # the user explicitly asked to fold while the chassis was
            # at default, so just fold.
            if (
                self._pending_fold
                and abs(self._applied_height) <= self._config.reseat_height_change_threshold
                and abs(self._target_height) <= self._config.reseat_height_change_threshold
            ):
                self._pending_fold = False
                self._fold = self._build_fold()
                self._state = EngineState.FOLDING
                return self._tick_fold(dt)
            # If the height has settled at a new value, reseat to it.
            # This handles two cases identically:
            #   * the user just released the D-pad after lifting
            #     (target_height != 0, _pending_fold may or may not
            #     be set);
            #   * the user pressed Start while lifted (target_height
            #     just snapped to 0, _pending_fold may be set);
            # In both cases the engine walks the feet to the new
            # nominal, then re-enters STAND, where _pending_fold is
            # consumed on the next tick.
            if (
                self._reseat_geometry is not None
                and self._leg_specs is not None
                and abs(self._target_height - self._applied_height)
                > self._config.reseat_height_change_threshold
                and self._height_stable_elapsed >= self._config.reseat_pose_settle_delay
            ):
                try:
                    target_stance = reseat_nominal_stance(
                        self._target_height,
                        self._reseat_geometry,
                        self._leg_specs,
                    )
                except ValueError:
                    # Geometrically infeasible target — drop the reseat
                    # silently rather than crashing the engine. The
                    # height stays applied via pose.z; the legs just
                    # don't snap back to default joint angles. The
                    # teleop clamps so we should never get here unless
                    # the YAML envelopes are mis-tuned.
                    return self._emit_stand()
                self._reseat = self._build_reseat(target_stance)
                self._state = EngineState.RESEATING
                # Snapshot target so the commit can use it without
                # re-running the geometry.
                self._reseat_target_stance: dict[str, Vec3] = dict(target_stance)
                self._reseat_target_height = self._target_height
                return self._tick_reseat(dt)
            return self._emit_stand()

        if self._state is EngineState.RESEATING:
            # Commit-to-completion ladder. cmd_vel and Start presses
            # may arrive mid-flight; cmd_vel is held until DONE, Start
            # presses latch via ``request_fold`` so the consumer in
            # STAND handles them once the legs are in place.
            return self._tick_reseat(dt)

        if self._state is EngineState.ENGAGING:
            if cmd_zero:
                # Bail straight to PAUSING — no debounce. The debounce
                # exists to ride out brief joystick-through-zero
                # crossings mid-gait without aborting; ENGAGING is a
                # transient ramp state whose body velocity is either
                # still climbing or has only just saturated, so a zero
                # here is far more likely a deliberate release than a
                # stick artefact. Ticking ENGAGING at zero cmd also
                # misbehaves visually: the live AEP collapses to NOMINAL
                # and swing legs lift-off-from-NOMINAL retract back to
                # where they started instead of touching down where the
                # engagement was carrying them.
                self._enter_pausing()
                return self._tick_pause(dt)
            out = self._tick_engagement(dt, v_body_xy, omega_z)
            if self._engagement.state is EngagementState.DONE:
                # Hand off to GAIT. Engagement covers a full master
                # cycle, so the handoff phase wraps to 0 — GAIT picks
                # up at the start of the next cycle with every leg
                # already on its strategy-prescribed curve.
                self._clock.reset(self._engagement.exit_master)
                self._stance.seed(self._last_targets, self._last_stance)
                # Engagement runs its own swing planning; clear the GAIT
                # SwingPlanner so any leg still airborne at handoff trips
                # a fresh lift-off (origin = engagement's last body-frame
                # foot position) on its next swing tick.
                self._swing.reset()
                self._state = EngineState.GAIT
            return out

        if self._state is EngineState.GAIT:
            # A pending gait change pre-empts walking immediately — no
            # pause debounce; the operator explicitly asked to switch.
            if self._pending_strategy_name is not None:
                self._enter_pausing()
                return self._tick_pause(dt)
            if should_pause:
                self._enter_pausing()
                return self._tick_pause(dt)
            return self._tick_gait(dt, v_body_xy, omega_z, cmd_zero)

        if self._state is EngineState.PAUSING:
            # A pending gait change pins the pause sequence: a held
            # stick must not abort to RESUMING before the commit.
            if not cmd_zero and self._pending_strategy_name is None:
                self._enter_resuming()
                return self._tick_engagement(dt, v_body_xy, omega_z)
            out = self._tick_pause(dt)
            if self._pause.state is PauseState.PAUSED:
                self._state = EngineState.PAUSED
                self._paused_elapsed = 0.0
            return out

        if self._state is EngineState.PAUSED:
            if not cmd_zero and self._pending_strategy_name is None:
                self._enter_resuming()
                return self._tick_engagement(dt, v_body_xy, omega_z)
            self._paused_elapsed += dt
            # A pending gait change uses the short dwell so the switch
            # feels immediate; the normal settle keeps the longer one.
            dwell = (
                self._config.gait_change_pause_to_reseat_delay
                if self._pending_strategy_name is not None
                else self._config.pause_to_reseat_delay
            )
            if self._paused_elapsed >= dwell:
                # Reseat the legs back to the current nominal footprint
                # — not a posture-height change, so the
                # _commit_new_nominal at the ladder's end is a no-op
                # rebuild against the unchanged nominal; the ladder
                # just cleans up the lowered positions so the robot
                # looks settled.
                self._reseat = self._build_reseat(self._nominal)
                self._reseat_target_stance = dict(self._nominal)
                self._reseat_target_height = self._applied_height
                self._state = EngineState.RESEATING
                return self._tick_reseat(dt)
            return self._emit_held()

        # RESUMING: drive the engagement controller's resume entry until
        # every leg has crossed into GAIT_LIKE. cmd_zero re-enters
        # PAUSING (interruptible).
        if cmd_zero:
            self._enter_pausing()
            return self._tick_pause(dt)
        out = self._tick_engagement(dt, v_body_xy, omega_z)
        if self._engagement.state is EngagementState.DONE:
            self._clock.reset(self._engagement.exit_master)
            self._stance.seed(self._last_targets, self._last_stance)
            self._state = EngineState.GAIT
        return out

    def _cmd_is_zero(self, v_body_xy: tuple[float, float], omega_z: float) -> bool:
        tol = self._config.cmd_zero_tol
        return abs(v_body_xy[0]) < tol and abs(v_body_xy[1]) < tol and abs(omega_z) < tol

    def _emit_stand(self) -> dict[str, LegOutput]:
        return {
            n: LegOutput(foot_target=self._nominal[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }

    def _tick_gait(
        self,
        dt: float,
        v_body_xy: tuple[float, float],
        omega_z: float,
        cmd_zero: bool,
    ) -> dict[str, LegOutput]:
        # Hold the previous tick's targets verbatim during the cmd-zero
        # debounce window. Freezing only the clock is not enough: the
        # strategy parameterizes its arcs by current stride, so at
        # stride=0 it snaps every leg from its mid-walking arc point to
        # the zero-stride centred arc (PEP=AEP=nominal) on the first
        # cmd_zero tick — a visible discontinuity that looked like an
        # extra pause pass. Skipping the strategy call entirely holds
        # every foot exactly where it was; when cmd resumes, the clock
        # advances from the frozen phase and the strategy picks up;
        # when the debounce expires, PAUSING fires and the pause
        # controller lowers the airborne legs in place.
        if cmd_zero:
            phases = self._clock.phases()
            return {
                n: LegOutput(
                    foot_target=self._last_targets[n],
                    phase=phases[n],
                    stance=self._last_stance[n],
                )
                for n in LEG_NAMES
            }

        duty_factor = self._strategy.duty_factor
        stride_length = self._config.stride_length
        swing_end = 1.0 - duty_factor

        leg_velocities = per_leg_planar_velocity(self._legs, v_body_xy, omega_z)
        max_leg_v = max(
            (math.hypot(vx, vy) for vx, vy in leg_velocities.values()),
            default=0.0,
        )

        cfg = self._config
        min_cycle_time, max_cycle_time = _cycle_time_bounds(cfg, duty_factor)
        cycle_time = derive_cycle_time(
            max_leg_v,
            cfg.stride_length,
            duty_factor,
            min_cycle_time,
            max_cycle_time,
        )
        stance_time = cycle_time * duty_factor
        swing_time = cycle_time * swing_end

        self._clock.advance(dt, cycle_time)
        phases = self._clock.phases()

        out: dict[str, LegOutput] = {}
        for name in LEG_NAMES:
            leg = self._legs[name]
            v_x, v_y = leg_velocities[name]
            stride_vec = stride_vector(v_x, v_y, stance_time, stride_length)
            stride = StrideParams(
                stride_vector=stride_vec,
                cycle_time=cycle_time,
                duty_factor=duty_factor,
                swing_clearance=self._config.step_height,
                swing_width=self._config.swing_width,
                controller_dt=self._config.controller_dt,
            )
            # Strategy is still evaluated unconditionally so test spies
            # and any future strategy-internal bookkeeping see every
            # tick. The result is consumed only as a fallback for stance
            # legs that have never lifted off under the SwingPlanner
            # (e.g. the very first GAIT tick after engagement, where the
            # touchdown edge — and therefore the integrator anchor —
            # comes from the engagement controller's seeded state, not
            # from our latched swing target).
            strategy_target = self._strategy.foot_target(phases[name], stride, leg)
            stance = phases[name] >= (1.0 - duty_factor) - _STANCE_SEAM_EPSILON

            if stance:
                if self._swing.is_swing[name]:
                    # Touchdown edge: adopt the latched swing target as
                    # the new stance anchor. The latched target is the
                    # AEP the swing arc was actually steering toward, so
                    # swing → stance is C0-exact even when v_leg varied
                    # during the airborne phase.
                    touchdown_anchor = self._swing.target[name]
                    self._swing.touchdown(name)
                else:
                    touchdown_anchor = strategy_target
                integrated = self._stance.step(
                    name=name,
                    in_stance=True,
                    swing_target=touchdown_anchor,
                    v_leg=(v_x, v_y),
                    dt=dt,
                )
                # in_stance=True always returns a position.
                assert integrated is not None
                target = integrated
            else:
                if not self._swing.is_swing[name]:
                    # Lift-off edge: capture origin from the foot's
                    # actual current position (= last stance anchor),
                    # target from the live AEP, and velocities from the
                    # current v_leg. Held for the remainder of the
                    # swing so mid-swing velocity changes do not move
                    # the airborne foot in body frame.
                    nominal = self._nominal[name]
                    aep = live_aep(nominal, stride_vec)
                    self._swing.liftoff(
                        name=name,
                        origin=self._last_targets[name],
                        target=aep,
                        v_leg=(v_x, v_y),
                        swing_time=max(swing_time, 1.0e-9),
                        identity_y_sign_val=identity_y_sign(nominal),
                    )
                phase_in_swing = (
                    phases[name] / swing_end if swing_end > 0.0 else 0.0
                )
                target = self._swing.evaluate(
                    name=name,
                    phase_in_swing=phase_in_swing,
                    swing_clearance=self._config.step_height,
                    swing_width=self._config.swing_width,
                    controller_dt=self._config.controller_dt,
                )
                # Keep the stance integrator's per-leg flag in sync so
                # the next stance entry trips its own touchdown edge
                # (StanceIntegrator.step with in_stance=False just clears
                # the flag and returns None).
                self._stance.step(
                    name=name,
                    in_stance=False,
                    swing_target=target,
                    v_leg=(v_x, v_y),
                    dt=dt,
                )

            out[name] = LegOutput(foot_target=target, phase=phases[name], stance=stance)

        self._capture_state(out)
        return out

    def _enter_pausing(self) -> None:
        """Capture swing flags and seed the PauseController.

        Each PAUSING entry recaptures the airborne set from the current
        ``_last_stance`` map — both GAIT → PAUSING (legs in the active
        gait's swing window) and RESUMING → PAUSING (legs mid-merge-arc)
        share this code path. ``_last_swing_flags`` is then handed to
        the next RESUMING entry as the "originally airborne" set, so
        the merge arcs swing from the lowered Z back up to AEP.
        """
        self._last_swing_flags = {
            n: not self._last_stance[n] for n in LEG_NAMES
        }
        self._pause.begin(self._last_targets, self._last_swing_flags)
        self._stance.reset()
        # PauseController owns the airborne legs from here; clear the
        # GAIT SwingPlanner so a subsequent RESUMING → GAIT does not see
        # a stale "is_swing" flag from before the pause.
        self._swing.reset()
        self._state = EngineState.PAUSING

    def _enter_resuming(self) -> None:
        """Seed the EngagementController in resume mode and switch states.

        Uses the stashed ``_last_swing_flags`` from the most-recent
        PAUSING entry so previously-airborne legs get merge arcs and
        previously-stance legs integrate stance from their paused
        position. The engine's ``_clock`` keeps its current master
        phase — engagement.update advances its own master internally,
        and the engine reseats ``_clock`` from ``exit_master`` only on
        the RESUMING → GAIT handoff.
        """
        self._engagement.begin_resume(
            strategy=self._strategy,
            leg_contexts=self._legs,
            last_targets=self._last_targets,
            prev_swing_flags=self._last_swing_flags,
            master_phase=self._clock.master,
        )
        self._state = EngineState.RESUMING

    def _tick_pause(self, dt: float) -> dict[str, LegOutput]:
        out = self._pause.update(dt)
        self._capture_state(out)
        return out

    def _emit_held(self) -> dict[str, LegOutput]:
        return {
            n: LegOutput(foot_target=self._last_targets[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }

    def _tick_reseat(self, dt: float) -> dict[str, LegOutput]:
        assert self._reseat is not None
        out = self._reseat.update(dt)
        self._capture_state(out)
        if self._reseat.done:
            # Commit a pending gait change at the RESEATING → STAND
            # handoff — every path a pending name can take funnels
            # through here. Swapping before _commit_new_nominal means
            # its engagement-controller rebuild already uses the new
            # gait's β, and pending is always None by STAND, so
            # STAND's immediate-swap semantics stay untouched.
            pending = self._pending_strategy_name
            self._pending_strategy_name = None
            if pending is not None and pending != self.strategy_name:
                self._apply_strategy(pending)
            self._commit_new_nominal(
                self._reseat_target_stance, self._reseat_target_height
            )
            self._state = EngineState.STAND
            # Make sure subsequent updates snap to the new nominal even
            # if downstream code reads ``_last_targets`` first.
            self._last_targets = dict(self._nominal)
            self._last_stance = {n: True for n in LEG_NAMES}
        return out

    def _tick_fold(self, dt: float) -> dict[str, LegOutput]:
        assert self._fold is not None
        out = self._fold.update(dt)
        self._capture_state(out)
        if self._fold.done:
            self._state = EngineState.FOLDED
            self._last_targets = dict(self._initial)
            self._last_stance = {n: True for n in LEG_NAMES}
        return out

    def _tick_engagement(
        self,
        dt: float,
        v_body_xy: tuple[float, float],
        omega_z: float,
    ) -> dict[str, LegOutput]:
        """Drive the EngagementController one tick.

        Shared by ENGAGING and RESUMING — the engage / resume distinction
        is internal to ``EngagementController`` and is set by
        ``begin()`` vs ``begin_resume()``. The engine just forwards the
        commanded velocity each tick and snapshots the result.
        """
        out = self._engagement.update(dt, v_body_xy, omega_z)
        self._capture_state(out)
        return out

    def _capture_state(self, out: Mapping[str, LegOutput]) -> None:
        """Snapshot per-leg foot targets and stance flags into ``_last_*``.

        The ``_last_targets`` / ``_last_stance`` maps feed the next
        tick's continuity (StanceIntegrator seeding, PauseController
        airborne snapshot, RESUMING's lift-off positions). Every
        controller's per-tick output flows through here so the
        bookkeeping cannot drift from the emitted trajectory.
        """
        self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
        self._last_stance = {n: out[n].stance for n in LEG_NAMES}


def build_leg_contexts(
    geometry_yaml: str | Path,
    standing: StandingPoseDeg,
) -> dict[str, LegContext]:
    """Build the per-leg ``LegContext`` map the engine needs at init.

    Couples the kinematics' ``LegSpec`` (mount geometry) with the nominal
    stance derived from ``standing``. Kept here rather than in
    ``leg_specs`` because ``LegContext`` is a gait-engine concept.
    """
    legs = load_leg_specs(geometry_yaml)
    nominal = nominal_stance_from_yaml(geometry_yaml, standing)
    return {
        n: LegContext(
            name=n,
            mount_xyz=legs[n].mount_xyz,
            mount_yaw=legs[n].mount_yaw,
            nominal_stance=nominal[n],
        )
        for n in LEG_NAMES
    }
