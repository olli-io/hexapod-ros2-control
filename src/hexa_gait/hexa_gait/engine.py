"""Gait engine — orchestrates clock, strategy, and the engagement /
disengagement controllers.

The engine is the only stateful component in the gait chain. Strategies
stay pure; the engagement and disengagement controllers each own a
per-cycle slice of state. The engine itself routes between four modes
based on the commanded body velocity:

- **STAND**     — ``cmd_vel`` is zero. Emit the nominal stance.
- **ENGAGING**  — ``cmd_vel`` just went non-zero from STAND. Run the
  ``EngagementController`` through one full master cycle. Body velocity
  ramps from 0 to ``v_body`` along a smoothstep S-curve over the
  earliest first-touchdown horizon, then holds at ``v_body``. Each leg
  performs exactly one "from NOMINAL" swing during this cycle; legs
  that have already completed their first swing follow the strategy
  directly, so the engagement → GAIT handoff is continuous. Hands off
  to GAIT at master = 1.0 (≡ 0.0 in the modular clock).
- **GAIT**      — ``cmd_vel`` is non-zero. Advance the phase clock and
  evaluate the active strategy.
- **STOPPING**  — ``cmd_vel`` just went zero from a non-zero state. Run
  the ``DisengagementController`` group-swing queue to bring all six
  legs back to nominal in gait-natural lift-off order. If a non-zero
  ``cmd_vel`` arrives mid-stop, complete the disengagement first, then
  restart the gait from ``master = 0``
  (per the velocity-mid-stop contract in ``src/hexa_gait/README.md``).
  GAIT → STOPPING is debounced by ``forced_touchdown_delay`` so brief
  joystick zero crossings don't trip a touchdown; ENGAGING → STOPPING
  is *not* debounced (see ``update``).

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
from dataclasses import dataclass
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
from .gaits.base import LegContext, Strategy, StrideParams
from .initialize import InitializeController
from .reseat import ReseatController, ReseatGeometry, reseat_nominal_stance
from .disengagement import DisengagementController, DisengagementState, LegOutput


__all__ = [
    "Engine",
    "EngineConfig",
    "EngineState",
    "LegOutput",
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
    STOPPING = "stopping"
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
    forced_touchdown_delay: float
    # Disengagement adaptive-timing knobs. ``max_foot_speed`` is the
    # body-frame planar foot-speed cap (m/s) used to derive each
    # swing's landing duration as ``distance_xy / max_foot_speed``,
    # clamped to ``[min_swing_time, max_swing_time]``.
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
    # RESEATING knobs. ``reseat_settle_delay`` is the dwell the target
    # pose.z must stay stable before reseat fires (so brief D-pad
    # taps don't kick off the ladder). ``reseat_height_change_threshold``
    # is the tolerance for "stable" (1 mm by default).
    # ``reseat_pair_swing_time`` is the per-pair duration (matches
    # initialize for visual symmetry). ``reseat_swing_clearance`` is
    # the arc clearance above the standing footprint — feet start
    # planted, so a moderate arc just clears ground noise.
    reseat_settle_delay: float
    reseat_height_change_threshold: float
    reseat_pair_swing_time: float
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
        self._disengagement = self._build_disengagement()
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
        # only commits to STOPPING from GAIT after the command has
        # stayed below cmd_zero_tol for ``forced_touchdown_delay``
        # seconds, so brief joystick-center crossings don't kick off
        # the stop transition. ENGAGING bypasses this debounce — see
        # ``update`` for why.
        self._cmd_zero_elapsed = 0.0

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
        forwarding only ``pose.z``. The stability timer resets when
        the target moves by more than ``reseat_height_change_threshold``
        from the previously-tracked value, so a slow ramp keeps
        re-resetting the timer until the user lets go.
        """
        threshold = self._config.reseat_height_change_threshold
        if abs(target_height - self._target_height) > threshold:
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

    def _build_disengagement(self) -> DisengagementController:
        cfg = self._config
        return DisengagementController(
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

    def _build_reseat(self, target_stance: Mapping[str, Vec3]) -> ReseatController:
        cfg = self._config
        return ReseatController(
            current_stance=self._nominal,
            target_stance=target_stance,
            pair_swing_time=cfg.reseat_pair_swing_time,
            swing_clearance=cfg.reseat_swing_clearance,
            swing_width=cfg.swing_width,
            controller_dt=cfg.controller_dt,
        )

    def _commit_new_nominal(
        self, new_nominal: Mapping[str, Vec3], applied_height: float
    ) -> None:
        """Adopt a new nominal stance as the engine's standing pose.

        Rebuilds the disengagement / engagement controllers (each caches
        its own snapshot of the nominal stance) and the per-leg
        ``LegContext`` map (the strategy reads ``leg.nominal_stance``
        from there), so subsequent ENGAGING / GAIT / STOPPING cycles
        run against the new posture.
        """
        self._nominal = {n: tuple(new_nominal[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._legs = {
            n: dataclasses.replace(self._legs[n], nominal_stance=self._nominal[n])
            for n in LEG_NAMES
        }
        self._disengagement = self._build_disengagement()
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
        # Only commit to STOPPING from GAIT after cmd has stayed zero
        # long enough to be deliberate; a brief joystick zero-crossing
        # keeps GAIT ticking at zero stride. ENGAGING does not use this
        # debounce — it bails to STOPPING on the first zero tick.
        should_stop = cmd_zero and (
            self._cmd_zero_elapsed >= self._config.forced_touchdown_delay
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
            self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
            self._last_stance = {n: out[n].stance for n in LEG_NAMES}
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
            self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
            self._last_stance = {n: out[n].stance for n in LEG_NAMES}
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
                and self._height_stable_elapsed >= self._config.reseat_settle_delay
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
                # Bail straight to STOPPING — no debounce. The debounce
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
                self._state = EngineState.STOPPING
                swing_flags = {n: not self._last_stance[n] for n in LEG_NAMES}
                self._disengagement.begin(
                    self._last_targets,
                    swing_flags,
                    phase_offsets=self._strategy.phase_offsets,
                    duty_factor=self._strategy.duty_factor,
                    master_phase=self._clock.master,
                )
                return self._tick_disengagement(dt)
            out = self._tick_engagement(dt, v_body_xy, omega_z)
            if self._engagement.state is EngagementState.DONE:
                # Hand off to GAIT. Engagement covers a full master
                # cycle, so the handoff phase wraps to 0 — GAIT picks
                # up at the start of the next cycle with every leg
                # already on its strategy-prescribed curve.
                self._clock.reset(self._engagement.exit_master)
                self._state = EngineState.GAIT
            return out

        if self._state is EngineState.GAIT:
            if should_stop:
                self._state = EngineState.STOPPING
                # _last_stance is True when the leg is on the ground;
                # the controller wants the opposite (True = airborne),
                # so invert here.
                swing_flags = {n: not self._last_stance[n] for n in LEG_NAMES}
                self._disengagement.begin(
                    self._last_targets,
                    swing_flags,
                    phase_offsets=self._strategy.phase_offsets,
                    duty_factor=self._strategy.duty_factor,
                    master_phase=self._clock.master,
                )
                return self._tick_disengagement(dt)
            return self._tick_gait(dt, v_body_xy, omega_z, cmd_zero)

        # STOPPING: run the disengagement queue to completion. A
        # non-zero cmd arriving mid-stop is honoured only after the
        # queue drains; the README is explicit about this.
        out = self._tick_disengagement(dt)
        if self._disengagement.state is DisengagementState.STAND:
            self._state = EngineState.STAND
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
        # extra disengagement pass. Skipping the strategy call entirely
        # holds every foot exactly where it was; when cmd resumes, the
        # clock advances from the frozen phase and the strategy picks
        # up; when the debounce expires, STOPPING fires and the
        # disengagement controller lands the held positions to nominal.
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

        leg_velocities = self._per_leg_planar_velocity(v_body_xy, omega_z)
        max_leg_v = max(
            (math.hypot(vx, vy) for vx, vy in leg_velocities.values()),
            default=0.0,
        )

        cycle_time = self._derive_cycle_time(max_leg_v)
        stance_time = cycle_time * duty_factor

        self._clock.advance(dt, cycle_time)
        phases = self._clock.phases()

        out: dict[str, LegOutput] = {}
        for name in LEG_NAMES:
            leg = self._legs[name]
            v_x, v_y = leg_velocities[name]
            stride_vec = self._stride_vector(v_x, v_y, stance_time, stride_length)
            stride = StrideParams(
                stride_vector=stride_vec,
                cycle_time=cycle_time,
                duty_factor=duty_factor,
                swing_clearance=self._config.step_height,
                swing_width=self._config.swing_width,
                controller_dt=self._config.controller_dt,
            )
            target = self._strategy.foot_target(phases[name], stride, leg)
            stance = phases[name] >= (1.0 - duty_factor)
            out[name] = LegOutput(foot_target=target, phase=phases[name], stance=stance)

        self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
        self._last_stance = {n: out[n].stance for n in LEG_NAMES}
        return out

    def _derive_cycle_time(self, max_leg_v: float) -> float:
        """Pick cycle_time so the fastest leg's stride equals stride_length.

        Clamped to ``[min_cycle_time, max_cycle_time]`` where the lower
        bound is derived from the strategy's duty factor:
        ``min_cycle_time = min_swing_time / (1 − β)``. That keeps the
        swing-phase foot velocity bounded as β shrinks (tripod) or
        grows (wave). At zero ``max_leg_v`` the raw quotient diverges,
        so we clamp to the slow end — the resulting stride is zero
        anyway because every ``v_leg`` is zero.
        """
        cfg = self._config
        beta = self._strategy.duty_factor
        min_cycle_time = (
            cfg.min_swing_time / (1.0 - beta) if beta < 1.0 else cfg.max_cycle_time
        )
        if max_leg_v <= 0.0:
            return cfg.max_cycle_time
        raw = cfg.stride_length / (max_leg_v * beta)
        if raw < min_cycle_time:
            return min_cycle_time
        if raw > cfg.max_cycle_time:
            return cfg.max_cycle_time
        return raw

    def _stride_vector(
        self,
        v_x: float,
        v_y: float,
        stance_time: float,
        stride_length: float,
    ) -> Vec3:
        """Per-leg stride displacement, magnitude-clamped to stride_length.

        The clamp matters only when ``max_leg_v`` exceeds the implied
        ceiling (``min_cycle_time`` has clipped ``cycle_time``); below
        saturation the raw stride is already ``≤ stride_length``.
        """
        sx = v_x * stance_time
        sy = v_y * stance_time
        magnitude = math.hypot(sx, sy)
        if magnitude > stride_length and magnitude > 0.0:
            scale = stride_length / magnitude
            sx *= scale
            sy *= scale
        return (sx, sy, 0.0)

    def _tick_disengagement(self, dt: float) -> dict[str, LegOutput]:
        # Map "in swing during disengagement" to "not stance" so that
        # if the engine drops back to STAND mid-drain it carries the
        # right grounded flags forward.
        out = self._disengagement.update(dt)
        self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
        self._last_stance = {n: out[n].stance for n in LEG_NAMES}
        return out

    def _tick_reseat(self, dt: float) -> dict[str, LegOutput]:
        assert self._reseat is not None
        out = self._reseat.update(dt)
        self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
        self._last_stance = {n: out[n].stance for n in LEG_NAMES}
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
        self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
        self._last_stance = {n: out[n].stance for n in LEG_NAMES}
        if self._fold.done:
            self._state = EngineState.FOLDED
            self._last_targets = dict(self._initial)
            self._last_stance = {n: True for n in LEG_NAMES}
        return out

    def _per_leg_planar_velocity(
        self,
        v_body_xy: tuple[float, float],
        omega_z: float,
    ) -> dict[str, tuple[float, float]]:
        """Linear cmd plus tangential yaw contribution at each hip.

        ``v_leg = v_body + omega × r``, evaluated in the body frame.
        Returned in the same order as ``LEG_NAMES`` so downstream code
        can take a single ``max`` over the speeds.
        """
        out: dict[str, tuple[float, float]] = {}
        for name in LEG_NAMES:
            r_x, r_y, _ = self._legs[name].mount_xyz
            v_x = v_body_xy[0] - omega_z * r_y
            v_y = v_body_xy[1] + omega_z * r_x
            out[name] = (v_x, v_y)
        return out

    def _tick_engagement(
        self,
        dt: float,
        v_body_xy: tuple[float, float],
        omega_z: float,
    ) -> dict[str, LegOutput]:
        out = self._engagement.update(dt, v_body_xy, omega_z)
        self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
        self._last_stance = {n: out[n].stance for n in LEG_NAMES}
        return out


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
