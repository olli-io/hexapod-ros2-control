"""Gait engine — orchestrates clock, strategy, and the engagement /
pause controllers.

The engine is the only stateful component in the gait chain. Strategies
stay pure; the engagement and pause controllers each own a per-cycle
slice of state. The engine itself routes between modes based on the
commanded body velocity:

- **STAND**     — ``cmd_vel`` is zero (and no gait state to preserve).
  Emit the nominal stance.
- **ENGAGING**  — ``cmd_vel`` just went non-zero from STAND. Run the
  ``EngagementController`` (engage mode) through one full master cycle.
  Body velocity ramps from 0 to ``v_body`` along a smoothstep S-curve
  over the earliest first-touchdown horizon, then holds at ``v_body``.
  Each leg performs exactly one "from NOMINAL" swing during this cycle;
  legs that have already completed their first swing follow the strategy
  directly, so the engagement → GAIT handoff is continuous. Hands off
  to GAIT at master = 1.0 (≡ 0.0 in the modular clock).
- **GAIT**      — ``cmd_vel`` is non-zero. Advance the phase clock and
  evaluate the active strategy.
- **PAUSING**   — ``cmd_vel`` went zero (debounced from GAIT, immediate
  from ENGAGING). The ``PauseController`` lowers the currently-airborne
  legs straight down to ``nominal.z`` (XY frozen); stance legs hold.
  Master phase / per-leg phase / β are preserved so the operator can
  re-engage without resetting the cycle.
- **PAUSED**    — every previously-airborne leg has landed. The engine
  holds positions and ticks ``_paused_elapsed``. On cmd_vel non-zero it
  routes to RESUMING; on ``_paused_elapsed >= pause_to_reseat_delay``
  it kicks off the RESEATING ladder back to the nominal footprint.
- **RESUMING**  — ``cmd_vel`` non-zero from PAUSING / PAUSED. Run the
  ``EngagementController`` (resume mode) seeded from the paused master
  phase and last_targets. Previously-airborne legs sweep custom merge
  arcs from their lowered Z back up to the live AEP; previously-stance
  legs integrate stance then swing through one swing window. Hands off
  to GAIT once every leg has crossed into its strategy-driven branch.

GAIT → PAUSING is debounced by ``pause_debounce_delay`` so brief
joystick zero crossings don't trip a pause; ENGAGING → PAUSING is *not*
debounced (see ``update``). PAUSING ↔ RESUMING are mutually
interruptible — cmd_vel can flip on/off mid-transition without
locking the engine.

``cycle_time`` is not configured directly. The engine derives it each
GAIT tick from the commanded velocity, ``stride_length``, and the
active strategy's ``duty_factor`` (β): faster commands ⇒ shorter cycles
at constant stride. The lower bound comes from a global
``min_swing_time`` — the real physical constraint is on swing-phase
foot velocity, not cycle time, so the per-gait floor is derived as
``min_swing_time / (1 − β)``. ``max_cycle_time`` is a visual slow-end
clamp so the gait stays brisk at zero command.

The nominal-stance helper ``nominal_stance_from_yaml`` reuses
``hexa_kinematics``'s FK and ``leg_to_body`` so the engine never
duplicates the trig that lives in ``body_transform.leg_to_body``.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Mapping

from hexa_kinematics.body_transform import leg_to_body
from hexa_kinematics.joint_config import load_initial_pose, load_standing_pose
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


# Float-noise epsilon for "is the user still moving the D-pad?". The
# teleop's height integrator runs at 0.05 m/s × 50 Hz = exactly 1 mm
# per tick, which sits right on the YAML dead-band
# (``reseat_height_change_threshold``). We need a much tighter bound
# here so a held D-pad reliably resets the settle timer every tick;
# the round-trip float noise is well below 1 µm in practice.
_HEIGHT_NOISE_EPSILON: float = 1e-6


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
    # FOLDED is the cold-start state: legs at initial_pose, body on its
    # belly, awaiting an explicit operator trigger before the
    # INITIALIZE ladder runs. cmd_vel is ignored in this state — the
    # cold-start is operator-gated so the robot does not move on
    # power-on while the user is still attaching the battery / cables.
    # FOLDING is the symmetric warm-shutdown: STAND → FOLDED via the
    # FoldController, also operator-gated.
    # RESEATING is the standing-pose-restoration ladder: after the user
    # lifts/lowers the chassis via /body/pose.z and the height settles,
    # the engine walks each foot pair to a new nominal stance that
    # restores the YAML default joint angles at the new body height.
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
    """Engine-internal knobs, sourced entirely from
    ``hexa_gait/config/gait.yaml``. None of these are on the wire.

    ``stride_length`` and ``min_swing_time`` / ``max_cycle_time`` define
    the velocity → cycle_time relationship the engine applies each
    GAIT tick. ``duty_factor`` lives on the active ``Strategy`` (so it
    can change with the gait); the engine reads it from there.
    """

    stride_length: float
    min_swing_time: float
    max_cycle_time: float
    step_height: float
    swing_width: float
    controller_dt: float
    cmd_zero_tol: float
    # Debounce window before GAIT → PAUSING fires. Brief joystick zero
    # crossings under this window keep the engine in GAIT at zero stride.
    pause_debounce_delay: float
    # PAUSED → RESEATING dwell. Once the engine has been in PAUSED for
    # this long with no cmd_vel, the reseat ladder walks the feet back
    # to the nominal footprint so the operator sees the robot settle.
    pause_to_reseat_delay: float
    # PAUSING adaptive-timing knobs. ``max_foot_speed`` is the body-frame
    # vertical foot-speed cap (m/s) used to derive each descent's
    # duration as ``distance_z / max_foot_speed``, clamped to
    # ``[min_swing_time, max_swing_time]``.
    max_foot_speed: float
    max_swing_time: float
    # INITIALIZE cold-start knobs. ``init_pair_swing_time`` is the
    # per-pair duration during PLACE_FEET; ``init_lift_body_time`` is
    # the LIFT_BODY z-ramp duration; ``init_swing_clearance`` is the
    # arc clearance the PLACE_FEET pair adds above its endpoints;
    # ``init_place_feet_clearance`` is the body-frame offset of the IK
    # target above the floor (with body on belly) at the end of each
    # PLACE_FEET swing; must absorb the URDF's vertical-tibia
    # assumption so the foot sphere does not penetrate the floor (see
    # gait.yaml comment for the geometry behind the value).
    init_pair_swing_time: float
    init_lift_body_time: float
    init_swing_clearance: float
    init_place_feet_clearance: float
    # RESEATING knobs. ``reseat_pose_settle_delay`` is the dwell the
    # target pose.z must stay unchanged after the user lets go of the
    # D-pad before the STAND → RESEATING ladder fires — distinct from
    # ``pause_to_reseat_delay`` above, which covers the gait engine's
    # own PAUSED → RESEATING dwell. ``reseat_height_change_threshold``
    # is the dead-band for "target differs from applied enough to be
    # worth reseating" (1 mm by default); the settle timer is reset by
    # a much tighter float-noise epsilon so a held D-pad
    # (1 mm-per-tick at 50 Hz × 0.05 m/s) keeps resetting it.
    # ``reseat_pair_swing_time`` is the per-pair duration (matches
    # initialize for visual symmetry). ``reseat_pair_dwell_time`` is
    # the hold inserted between successive pair swings so each pair
    # visibly settles before the next lifts. ``reseat_swing_clearance``
    # is the arc clearance above the standing footprint — feet start
    # planted, so a moderate arc just clears ground noise.
    reseat_pose_settle_delay: float
    reseat_height_change_threshold: float
    reseat_pair_swing_time: float
    reseat_pair_dwell_time: float
    reseat_swing_clearance: float


def nominal_stance_from_yaml(
    geometry_yaml: str | Path,
    standing_pose_yaml: str | Path,
) -> dict[str, Vec3]:
    """Body-frame foot position per leg at the YAML-defined standing pose.

    Mirrors the math previously inlined in
    ``hexa_bringup/tools/stub_stance_publisher.py:51-62``, but routed
    through ``hexa_kinematics.body_transform.leg_to_body`` so the trig
    lives in exactly one place.
    """
    legs = load_leg_specs(geometry_yaml)
    angles = load_standing_pose(standing_pose_yaml, geometry_yaml)
    return {n: leg_to_body(forward_kinematics(angles, legs[n]), legs[n]) for n in LEG_NAMES}


def reseat_geometry_from_yaml(
    geometry_yaml: str | Path,
    standing_pose_yaml: str | Path,
) -> ReseatGeometry:
    """Build the ``ReseatGeometry`` snapshot used by the engine.

    Reads ``standing_pose.yaml`` for the default joint angles and
    ``geometry.yaml`` for segment lengths, then derives the
    tibia-from-vertical angle and the default foot depth via the same
    FK helper that ``nominal_stance_from_yaml`` uses. Any leg works as
    the reference (all six share segment lengths); the first one in
    ``LEG_NAMES`` is picked deterministically.
    """
    from .reseat import default_geometry_from_pose

    legs = load_leg_specs(geometry_yaml)
    angles = load_standing_pose(standing_pose_yaml, geometry_yaml)
    return default_geometry_from_pose(angles, legs[LEG_NAMES[0]])


def initial_stance_from_yaml(geometry_yaml: str | Path) -> dict[str, Vec3]:
    """Body-frame foot position per leg at the YAML-defined ``initial_pose``.

    Sibling of ``nominal_stance_from_yaml``: same FK pipeline, but the
    angles come from ``geometry.yaml``'s ``initial_pose:`` block instead
    of ``standing_pose.yaml``. The engine seeds its INITIALIZE state's
    PLACE_FEET swing origins from this map — these are the foot
    positions in the body frame when the hexapod is sitting on its
    belly with legs folded up at power-on.
    """
    legs = load_leg_specs(geometry_yaml)
    angles_per_leg = load_initial_pose(geometry_yaml)
    return {
        n: leg_to_body(forward_kinematics(angles_per_leg[n], legs[n]), legs[n])
        for n in LEG_NAMES
    }


@dataclass
class StanceIntegrator:
    """Per-leg body-frame stance target as an integral from touchdown.

    The standard strategies rebuild PEP/AEP from the current stride each
    tick, so a velocity change that does not coincide with lift-off
    snaps every stance leg by ``(0.5 − s) · (stride_new − stride_old)``
    in the body frame — a non-uniform shear across legs at different
    stance phases. For tripod this is masked (β = 0.5, all stance legs
    share s); for wave and ripple it is visible foot scrubbing.

    The fix: at each leg's touchdown the body-frame foot position is
    captured as the world-locked anchor, and every subsequent stance
    tick decrements that anchor by ``v_leg · dt``. Stance is then
    history-dependent (touchdown anchor + integrated body translation)
    rather than rebuilt from instantaneous stride. Swing keeps using
    the strategy's swing curve — it is a body-frame planning curve and
    is unaffected by the slip.

    Under constant velocity the integrator reproduces the closed-form
    stance Bezier exactly (the Bezier's nodes are colinear and evenly
    spaced, so it degenerates to a linear interpolation).
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
        """Capture current body-frame foot positions as stance anchors.

        Called at every entry to GAIT so legs that arrive mid-stance
        (from ENGAGING or RESUMING) integrate from their current
        position rather than waiting for their next swing.
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
        """Advance the integrator one tick for a leg.

        Returns the integrated body-frame target if the leg is in
        stance, else ``None`` (the caller falls back to the strategy's
        swing curve). On the swing → stance edge, ``swing_target`` is
        adopted as the new anchor and returned unchanged for this tick
        — integration begins on the following tick.
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
    """Per-leg latched swing plan, captured at lift-off and held until touchdown.

    The standard strategies rebuild PEP/AEP from the live stride each
    tick, so a mid-swing velocity change re-evaluates the quartic Bezier
    against a moved swing_origin and target — the airborne foot snaps in
    body frame by a fraction of ``Δstride``. Tripod hides this because
    ``stance_time = min_swing_time`` keeps strides short and there are
    three concurrent swing legs; wave (β = 5/6) has ``stance_time = 5 ·
    min_swing_time`` so the shift is ~5× larger, and only one leg is
    airborne to bear the discontinuity. The result is a visible body-
    frame jump on the swing foot whenever the operator introduces ``v_y``
    or ``ω_z`` on top of a steady ``v_x``.

    The fix mirrors ``StanceIntegrator`` on the swing side: at lift-off
    capture
      * ``origin``       — the foot's actual body-frame position (= the
        last stance integrator anchor), giving C0 continuity into swing
        even when the preceding stance integrated a varying velocity;
      * ``target``       — the live AEP (``nominal + 0.5 · stride``);
      * ``v_leg``        — used as both ``swing_origin_velocity`` and
        ``swing_target_velocity`` so the swing arc launches and lands at
        the stance-frame velocity ``-v_leg``. The default
        ``-stride / swing_time = -v_leg · β / (1−β)`` is correct only at
        β = 0.5; for ripple it is 2× and for wave 5× too fast, producing
        a body-frame velocity step at every lift-off and touchdown that
        scrubs the loaded stance feet;
      * ``swing_time`` and ``identity_y_sign`` — held alongside so the
        Bezier control nodes stay fixed for the full swing.
    During swing the engine evaluates ``swing_arc`` from these latched
    values; at touchdown the integrator's new anchor is ``target`` (so
    swing → stance is exact), and the planner releases the leg.

    Engagement and resume each run their own swing planning, so the
    planner is reset on every entry to GAIT — a leg that arrives
    mid-swing (engagement → GAIT handoff) is then treated as a fresh
    lift-off on its next swing tick, capturing the engagement's last
    body-frame position as the new origin.
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
        # Stance-frame foot velocity is -v_leg; pass it as both endpoints
        # so the Bezier's C1 nodes match the stance-frame velocity at
        # lift-off (origin) and touchdown (target). Defaults derived from
        # ``-stride/swing_time`` only match this at β = 0.5.
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

    ``update(dt, v_body_xy, omega_z)`` returns one ``LegOutput`` per
    leg. Cold start is ``STAND`` with the last-emitted targets seeded
    to ``nominal_stance``, so the first tick matches the previous stub
    publisher's behaviour exactly. ``cycle_time`` is derived per-tick
    from the commanded velocity, ``stride_length``, the active
    strategy's ``duty_factor``, and the engine's
    ``min_swing_time`` / ``max_cycle_time`` bounds.
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
        # Built lazily on the operator trigger so a fresh ladder runs
        # each time the user requests a fold (STAND → FOLDING).
        self._fold: FoldController | None = None
        # Built each time the height settles to a new value while the
        # engine is in STAND. Mid-cycle the engine commits to running
        # this ladder to completion.
        self._reseat: ReseatController | None = None

        # Cold start: assume the operator placed the chassis in the
        # folded initial_pose. The engine emits initial_stance and
        # waits for an operator trigger (``start_initialize``) before
        # running the INITIALIZE ladder to the standing pose. Until
        # then cmd_vel is ignored — power-on must not move the robot.
        self._state = EngineState.FOLDED
        self._last_targets: dict[str, Vec3] = dict(self._initial)
        self._last_stance: dict[str, bool] = {n: True for n in LEG_NAMES}
        # Debounce timer for cmd_vel → 0 while in GAIT. The engine
        # only commits to PAUSING from GAIT after the command has
        # stayed below cmd_zero_tol for ``pause_debounce_delay``
        # seconds, so brief joystick-center crossings don't kick off
        # the pause transition. ENGAGING bypasses this debounce — see
        # ``update`` for why.
        self._cmd_zero_elapsed = 0.0
        # PAUSED dwell timer. Ticks while the engine sits in PAUSED;
        # crossing ``pause_to_reseat_delay`` kicks off RESEATING. Reset
        # on PAUSING entry so the soft-release flow has a fresh window
        # to detect "operator really released the stick".
        self._paused_elapsed: float = 0.0
        # Originally-airborne legs at the most-recent PAUSING entry.
        # Stashed so RESUMING knows which legs need merge arcs (from
        # their lowered Z back up to the live AEP) rather than which
        # are airborne mid-PAUSING.
        self._last_swing_flags: dict[str, bool] = {n: False for n in LEG_NAMES}

        # Reseat state. ``_applied_height`` is the pose.z the current
        # ``_nominal`` was computed at — starts at 0 (the YAML-derived
        # standing pose). ``_target_height`` tracks the latest
        # operator-commanded height, updated via ``set_target_height``.
        # The stability timer measures how long the target has been
        # stable; once it passes the settle delay AND target differs
        # from applied, the engine fires the reseat ladder.
        self._applied_height: float = 0.0
        self._target_height: float = 0.0
        self._height_stable_elapsed: float = 0.0
        # Pending fold flag: latched by ``request_fold`` so a Start
        # press during RESEATING (or any non-STAND state) is consumed
        # when the engine next reaches STAND with the height at 0.
        # Lets the teleop's two-press scheme work: press 1 snaps
        # height → 0 (kicks off reseat); press 2 queues the fold.
        self._pending_fold: bool = False

    @property
    def state(self) -> EngineState:
        return self._state

    @property
    def strategy_name(self) -> str:
        """Name of the currently-active strategy from the registry.

        Lookup is by identity (one entry in ``STRATEGIES`` per gait), so
        a strategy not built from the registry returns its class name
        lower-cased as a best-effort fallback.
        """
        for name, factory in STRATEGIES.items():
            if isinstance(self._strategy, factory):  # type: ignore[arg-type]
                return name
        return type(self._strategy).__name__.lower()

    def set_strategy(self, name: str) -> bool:
        """Swap the active gait strategy.

        Strict: only succeeds when the engine is in ``STAND``. Anywhere
        else (walking, engaging, transitioning, folding, reseating)
        returns ``False`` without queueing. The teleop layer gates the
        publish on ``/gait/state`` so stale intent does not sit on the
        wire.

        Returns ``True`` on a successful swap (including the no-op case
        where ``name`` matches the current strategy), ``False`` on an
        unknown name or a non-STAND state. Rebuilds the engagement
        controller (β-dependent) and the phase clock (offsets change).
        """
        factory = STRATEGIES.get(name)
        if factory is None:
            return False
        if name == self.strategy_name:
            return True
        if self._state is not EngineState.STAND:
            return False
        self._strategy = factory()
        self._clock = GaitClock(self._strategy.phase_offsets)
        self._engagement = self._build_engagement()
        return True

    def start_initialize(self) -> bool:
        """Operator-gated trigger: FOLDED → INITIALIZE.

        Returns ``True`` if the engine actually transitioned, ``False``
        if it was in any other state (idempotent: stray triggers are a
        no-op rather than a fault, so a re-pressed start button after
        the cold-start has already run does not destabilise the gait).

        Rebuilds the controller so the same engine instance can run a
        second cold-start after a fold has returned it to FOLDED.
        """
        if self._state is not EngineState.FOLDED:
            return False
        self._initialize = self._build_initialize()
        self._state = EngineState.INITIALIZE
        return True

    def start_fold(self) -> bool:
        """Operator-gated trigger: STAND → FOLDING.

        Symmetric to ``start_initialize``: returns ``True`` only when
        the engine is in STAND, so a stray press while walking or
        already folded is a safe no-op. Builds a fresh
        ``FoldController`` so repeated fold cycles each get a clean
        ladder.

        Prefer ``request_fold`` from the ROS layer — it handles the
        ``RESEATING`` case where the user has pressed Start twice
        rapidly while the chassis is lifted. ``start_fold`` is kept
        for tests that want the unconditional transition.
        """
        if self._state is not EngineState.STAND:
            return False
        self._fold = self._build_fold()
        self._state = EngineState.FOLDING
        return True

    def request_fold(self) -> bool:
        """Idempotent fold request.

        Latches ``_pending_fold``: the engine consumes the flag the
        next time it lands in STAND with both applied and target
        height at zero. Lets the teleop's two-press Start scheme work
        — press 1 (while chassis lifted) snaps the height to 0 and
        kicks off a reseat ladder; press 2 during that ladder queues
        the fold, which fires automatically when reseat completes.

        Returns ``True`` if the request was queued (engine isn't
        already FOLDED or FOLDING), ``False`` otherwise so the ROS
        layer can keep its existing log line tidy.
        """
        if self._state is EngineState.FOLDED or self._state is EngineState.FOLDING:
            return False
        self._pending_fold = True
        return True

    def set_target_height(self, target_height: float) -> None:
        """Update the operator-commanded body height.

        Called by the ROS layer on every ``/body/pose`` message,
        forwarding only ``pose.z``. Any change above the float-noise
        epsilon resets the settle timer, so while the D-pad is held
        the target slews 1 mm per tick and the timer never accrues;
        once the user lets go the value stops moving and the timer
        ticks up to ``reseat_pose_settle_delay``.

        Note: the YAML dead-band ``reseat_height_change_threshold``
        is intentionally *not* applied here — it gates the "does
        target differ from applied enough to reseat at all?" check
        in ``update``, not the per-tick change detection. Using it
        here would race with the integrator's per-tick step.
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
            max_foot_speed=cfg.max_foot_speed,
            min_swing_time=cfg.min_swing_time,
            max_swing_time=cfg.max_swing_time,
        )

    def _build_engagement(self) -> EngagementController:
        cfg = self._config
        beta = self._strategy.duty_factor
        # Per-gait min_cycle_time = min_swing_time / (1 − β). Wave's
        # short swing window means a much larger min_cycle than
        # tripod's at the same min_swing_time.
        min_cycle_time = (
            cfg.min_swing_time / (1.0 - beta) if beta < 1.0 else cfg.max_cycle_time
        )
        return EngagementController(
            nominal_stance=self._nominal,
            stride_length=cfg.stride_length,
            min_cycle_time=min_cycle_time,
            max_cycle_time=cfg.max_cycle_time,
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
            if should_pause:
                self._enter_pausing()
                return self._tick_pause(dt)
            return self._tick_gait(dt, v_body_xy, omega_z, cmd_zero)

        if self._state is EngineState.PAUSING:
            if not cmd_zero:
                self._enter_resuming()
                return self._tick_engagement(dt, v_body_xy, omega_z)
            out = self._tick_pause(dt)
            if self._pause.state is PauseState.PAUSED:
                self._state = EngineState.PAUSED
                self._paused_elapsed = 0.0
            return out

        if self._state is EngineState.PAUSED:
            if not cmd_zero:
                self._enter_resuming()
                return self._tick_engagement(dt, v_body_xy, omega_z)
            self._paused_elapsed += dt
            if self._paused_elapsed >= self._config.pause_to_reseat_delay:
                # Reseat the legs back to the current nominal footprint.
                # This is *not* a posture-height change — _commit_new_nominal
                # must not fire — we just want the ladder to clean up
                # the lowered positions so the robot looks settled.
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
        min_cycle_time = (
            cfg.min_swing_time / (1.0 - duty_factor)
            if duty_factor < 1.0
            else cfg.max_cycle_time
        )
        cycle_time = derive_cycle_time(
            max_leg_v,
            cfg.stride_length,
            duty_factor,
            min_cycle_time,
            cfg.max_cycle_time,
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
            stance = phases[name] >= (1.0 - duty_factor)

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
    standing_pose_yaml: str | Path,
) -> dict[str, LegContext]:
    """Build the per-leg ``LegContext`` map the engine needs at init.

    Couples the kinematics' ``LegSpec`` (mount geometry) with the
    YAML-derived nominal stance. Kept here rather than in ``leg_specs``
    because ``LegContext`` is a gait-engine concept.
    """
    legs = load_leg_specs(geometry_yaml)
    nominal = nominal_stance_from_yaml(geometry_yaml, standing_pose_yaml)
    return {
        n: LegContext(
            name=n,
            mount_xyz=legs[n].mount_xyz,
            mount_yaw=legs[n].mount_yaw,
            nominal_stance=nominal[n],
        )
        for n in LEG_NAMES
    }
