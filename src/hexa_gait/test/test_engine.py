import math

import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.engagement import EngagementState
from hexa_gait.engine import Engine, EngineConfig, EngineState
from hexa_gait.gaits.base import LegContext, StrideParams
from hexa_gait.gaits.ripple import Ripple
from hexa_gait.gaits.tripod import TRIPOD_OFFSETS, Tripod
from hexa_gait.gaits.wave import Wave


# Symmetric six-leg layout. Front/rear sit at 0.18 m from body centre
# (the outer legs in pure rotation); middle legs sit at 0.12 m. Chosen
# so the per-leg radii are easy to reason about in the mixed-motion
# tests below.
_MOUNTS: dict[str, tuple[float, float, float]] = {
    "l_front": (0.15, 0.10, 0.0),
    "r_front": (0.15, -0.10, 0.0),
    "l_middle": (0.0, 0.12, 0.0),
    "r_middle": (0.0, -0.12, 0.0),
    "l_rear": (-0.15, 0.10, 0.0),
    "r_rear": (-0.15, -0.10, 0.0),
}


def _nominal_stance() -> dict[str, tuple[float, float, float]]:
    # Foot directly under each hip at a fixed walk-plane Z.
    return {n: (xyz[0], xyz[1], -0.10) for n, xyz in _MOUNTS.items()}


def _initial_stance() -> dict[str, tuple[float, float, float]]:
    # Folded feet sitting above each hip at body-frame z > 0. Exact
    # value doesn't matter for the post-INITIALIZE behaviours these
    # tests target; only matters that the engine starts in INITIALIZE
    # and runs the ladder to STAND before anything else.
    return {n: (xyz[0], xyz[1], 0.05) for n, xyz in _MOUNTS.items()}


def _leg_contexts() -> dict[str, LegContext]:
    nominal = _nominal_stance()
    return {
        n: LegContext(name=n, mount_xyz=_MOUNTS[n], mount_yaw=0.0, nominal_stance=nominal[n])
        for n in LEG_NAMES
    }


def _config(
    *,
    stride_length: float = 0.10,
    min_swing_time: float = 0.25,
    max_swing_time: float = 1.0,
    pause_debounce_delay: float = 0.0,
    pause_to_reseat_delay: float = 0.2,
    gait_change_pause_to_reseat_delay: float = 0.1,
) -> EngineConfig:
    # min_swing_time=0.25, β=0.5 (tripod) → min_cycle_time = 0.5 s; same
    # for max_swing_time=1.0 → max_cycle_time = 2.0 s (matches the
    # pre-refactor flat scalar). Other gait factors derive their own
    # bounds from swing_time / (1 − β).
    return EngineConfig(
        stride_length=stride_length,
        min_swing_time=min_swing_time,
        max_swing_time=max_swing_time,
        step_height=0.03,
        swing_width=0.0,
        controller_dt=0.02,
        cmd_zero_tol=1.0e-4,
        pause_debounce_delay=pause_debounce_delay,
        pause_to_reseat_delay=pause_to_reseat_delay,
        gait_change_pause_to_reseat_delay=gait_change_pause_to_reseat_delay,
        max_reset_time=0.6,
        # Compact INITIALIZE timings: 3*0.04 + 0.04 = 0.16 s. Keeps the
        # cold-start ladder short enough that drive-past-initialize
        # finishes in well under 20 ticks at dt=0.02. test_initialize.py
        # covers the production-time behaviour separately.
        init_pair_swing_time=0.04,
        init_lift_body_time=0.04,
        init_swing_clearance=0.01,
        init_place_feet_clearance=0.001,
        # Reseat knobs (used only by tests that opt in by passing
        # leg_specs + reseat_geometry to the Engine). The tests below
        # construct Engine without those, so these values are inert.
        reseat_pose_settle_delay=0.1,
        reseat_height_change_threshold=0.001,
        reseat_pair_swing_time=0.04,
        reseat_pair_dwell_time=0.0,
        reseat_swing_clearance=0.01,
    )


class _SpyStrategy:
    """Records every (phase, StrideParams, leg) call from the engine."""

    phase_offsets = TRIPOD_OFFSETS
    duty_factor = 0.5

    def __init__(self) -> None:
        self.calls: list[tuple[str, float, StrideParams]] = []

    def foot_target(self, phase, stride, leg):
        self.calls.append((leg.name, phase, stride))
        return leg.nominal_stance

    def last_stride(self, leg_name: str) -> StrideParams:
        for name, _phase, stride in reversed(self.calls):
            if name == leg_name:
                return stride
        raise AssertionError(f"no recorded stride for {leg_name}")

    def clear(self) -> None:
        self.calls.clear()


def _engine(strategy: _SpyStrategy, config: EngineConfig | None = None) -> Engine:
    return Engine(
        config=config or _config(),
        strategy=strategy,
        nominal_stance=_nominal_stance(),
        initial_stance=_initial_stance(),
        coxa_to_bottom=0.02,
        leg_contexts=_leg_contexts(),
    )


def _drive_past_initialize(engine: Engine, dt: float = 0.02) -> None:
    """Trigger the cold-start ladder and tick until it reaches STAND.

    Existing tests were written before the cold-start FOLDED /
    INITIALIZE states existed and assume the engine begins at STAND.
    The compact init timings in ``_config`` keep this loop short
    (≈8 ticks at dt=0.02).
    """
    engine.start_initialize()
    for _ in range(200):
        if engine.state is EngineState.STAND:
            return
        engine.update(dt=dt, v_body_xy=(0.0, 0.0), omega_z=0.0)
    raise AssertionError("engine did not reach STAND within 200 ticks")


def _drive_to_gait(
    engine: Engine,
    v_body_xy: tuple[float, float],
    omega_z: float,
    dt: float = 0.02,
) -> int:
    """Run the engine from FOLDED through INITIALIZE / STAND / ENGAGING into GAIT.

    Returns the number of ticks consumed. Used by the cycle_time /
    stride tests that target steady-state GAIT behaviour and were
    written before engagement existed — engagement freezes its snapshot
    at entry, so they need a steady-state tick to inspect.
    """
    engine.start_initialize()
    for i in range(200):
        engine.update(dt=dt, v_body_xy=v_body_xy, omega_z=omega_z)
        if engine.state is EngineState.GAIT:
            return i + 1
    raise AssertionError("engine did not reach GAIT within 200 ticks")


def test_below_saturation_derives_cycle_time_from_velocity():
    # v = 0.20 m/s straight forward, stride_length = 0.10, duty = 0.5
    # → cycle_time_raw = 0.10 / (0.20 × 0.5) = 1.0 s, comfortably inside
    # [min, max]. stance_time = 0.5 s. Per-leg stride = 0.20 × 0.5 = 0.10 m.
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_to_gait(engine, v_body_xy=(0.20, 0.0), omega_z=0.0)
    spy.clear()
    engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)

    assert engine.state is EngineState.GAIT
    stride = spy.last_stride("l_front")
    assert stride.cycle_time == pytest.approx(1.0)
    sx, sy, sz = stride.stride_vector
    assert sx == pytest.approx(0.10)
    assert sy == pytest.approx(0.0)
    assert sz == 0.0


def test_saturation_clamps_cycle_time_and_per_leg_stride():
    # v = 0.80 m/s straight; raw = 0.10 / 0.40 = 0.25 < min_cycle_time
    # so cycle_time clamps to 0.5 s. stance_time = 0.25 s. Raw stride
    # would be 0.80 × 0.25 = 0.20 m, which the per-leg clamp must cap
    # at stride_length (0.10 m).
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_to_gait(engine, v_body_xy=(0.80, 0.0), omega_z=0.0)
    spy.clear()
    engine.update(dt=0.02, v_body_xy=(0.80, 0.0), omega_z=0.0)

    stride = spy.last_stride("l_front")
    assert stride.cycle_time == pytest.approx(0.5)
    magnitude = math.hypot(stride.stride_vector[0], stride.stride_vector[1])
    assert magnitude == pytest.approx(0.10)


def test_slow_command_clamps_cycle_time_to_max_and_stride_shrinks_linearly():
    # v = 0.02 m/s; raw = 0.10 / 0.01 = 10 s > max_cycle_time so
    # cycle_time clamps to 2.0 s. stance_time = 1.0 s. Per-leg stride =
    # 0.02 × 1.0 = 0.02 m — well under stride_length, so no further clamp.
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_to_gait(engine, v_body_xy=(0.02, 0.0), omega_z=0.0)
    spy.clear()
    engine.update(dt=0.02, v_body_xy=(0.02, 0.0), omega_z=0.0)

    stride = spy.last_stride("l_front")
    assert stride.cycle_time == pytest.approx(2.0)
    assert stride.stride_vector[0] == pytest.approx(0.02)


def test_pure_rotation_outer_leg_dictates_cycle_time():
    # Pure omega_z = 1.0 rad/s. Outer legs (front/rear) have radius
    # sqrt(0.15² + 0.10²); middle legs have radius 0.12. max_leg_v is
    # the outer-leg radius, so cycle_time = stride_length /
    # (outer_r × duty). Outer-leg stride magnitude saturates at
    # stride_length; middle-leg stride is proportionally shorter.
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_to_gait(engine, v_body_xy=(0.0, 0.0), omega_z=1.0)
    spy.clear()
    engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=1.0)

    outer_r = math.hypot(0.15, 0.10)
    inner_r = 0.12
    expected_cycle = 0.10 / (outer_r * 0.5)

    outer = spy.last_stride("l_front")
    inner = spy.last_stride("l_middle")
    assert outer.cycle_time == pytest.approx(expected_cycle)
    assert inner.cycle_time == pytest.approx(expected_cycle)  # shared

    outer_mag = math.hypot(outer.stride_vector[0], outer.stride_vector[1])
    inner_mag = math.hypot(inner.stride_vector[0], inner.stride_vector[1])
    assert outer_mag == pytest.approx(0.10)
    assert inner_mag == pytest.approx(0.10 * (inner_r / outer_r))


def test_zero_command_stays_in_stand_and_skips_cycle_time_math():
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_past_initialize(engine)
    spy.clear()
    out = engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)

    assert engine.state is EngineState.STAND
    # Strategy was never invoked: STAND emits nominal directly.
    assert spy.calls == []
    for name in LEG_NAMES:
        assert out[name].stance is True


def test_phase_advances_faster_at_higher_velocity():
    # Two engines, identical except for the commanded velocity: the
    # faster command must accumulate phase faster across a fixed dt
    # because cycle_time shrinks proportionally. Drive both through
    # engagement first so this test isolates GAIT phase advance.
    spy_a = _SpyStrategy()
    spy_b = _SpyStrategy()
    engine_a = _engine(spy_a)
    engine_b = _engine(spy_b)

    _drive_to_gait(engine_a, v_body_xy=(0.10, 0.0), omega_z=0.0)
    _drive_to_gait(engine_b, v_body_xy=(0.30, 0.0), omega_z=0.0)
    spy_a.clear()
    spy_b.clear()

    # A handful of GAIT ticks; not enough for the faster engine's phase
    # to lap the slower one, so the direct phase comparison is well-defined.
    for _ in range(3):
        engine_a.update(dt=0.02, v_body_xy=(0.10, 0.0), omega_z=0.0)
        engine_b.update(dt=0.02, v_body_xy=(0.30, 0.0), omega_z=0.0)

    last_phase_a = next(phase for name, phase, _ in reversed(spy_a.calls) if name == "l_front")
    last_phase_b = next(phase for name, phase, _ in reversed(spy_b.calls) if name == "l_front")

    # The faster engine should be further along its (shorter) cycle.
    assert last_phase_b > last_phase_a


def test_stand_to_first_nonzero_cmd_routes_through_engaging():
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_past_initialize(engine)
    engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.ENGAGING
    assert engine._engagement.state is EngagementState.ENGAGING


def test_no_first_tick_position_jump_from_stand():
    # The reported bug: at STAND -> first non-zero cmd tick, no foot
    # should jump. Every leg must still be near NOMINAL.
    spy = _SpyStrategy()
    engine = _engine(spy)
    nominal = _nominal_stance()
    _drive_past_initialize(engine)
    out = engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
    for name in LEG_NAMES:
        dx = abs(out[name].foot_target[0] - nominal[name][0])
        dy = abs(out[name].foot_target[1] - nominal[name][1])
        # Pre-fix this would have been ≈ 0.05 m (half the steady-state
        # stride) for both tripods. With engagement it's bounded by a
        # fraction of swing clearance / smoothstep progress.
        assert dx < 0.01, f"{name} jumped {dx:.4f} m on first tick"
        assert dy < 0.01


def test_engaging_to_gait_at_exit_master():
    # After one full engagement cycle the engine must reach GAIT and the
    # clock must be seeded at exit_master, which wraps to 0.0 (engagement
    # covers a full master cycle).
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_to_gait(engine, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.GAIT
    # The handoff tick resets the clock to exit_master = 0.0;
    # _drive_to_gait returns immediately after that tick, so the clock
    # has not yet been advanced by a GAIT step.
    assert engine._clock.master == pytest.approx(0.0, abs=1e-9)
    # First GAIT tick advances by dt / cycle_time = 0.02 / 1.0.
    engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine._clock.master == pytest.approx(0.02, abs=1e-9)


def test_master_phase_tracks_engagement_during_engaging():
    # Posture animations (vertical/horizontal body roll) read master_phase
    # off /legs/targets. During ENGAGING the engine's _clock is frozen —
    # only the engagement controller advances. The published master_phase
    # property must therefore mirror engagement.exit_master, not the
    # stale clock, otherwise phase-locked animations sit at a constant
    # phase throughout the engagement cycle.
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_past_initialize(engine)

    engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.ENGAGING

    last_phase = engine.master_phase
    assert last_phase == pytest.approx(engine._engagement.exit_master, abs=1e-12)
    assert engine._clock.master == pytest.approx(0.0, abs=1e-12)

    saw_progress = False
    for _ in range(80):
        engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
        if engine.state is not EngineState.ENGAGING:
            break
        phase = engine.master_phase
        assert phase == pytest.approx(engine._engagement.exit_master, abs=1e-12)
        if phase > last_phase:
            saw_progress = True
        last_phase = phase
    assert saw_progress, "master_phase did not advance during ENGAGING"


def test_master_phase_continuous_across_engaging_to_gait_handoff():
    # The engagement controller seeds _clock with exit_master at handoff,
    # so the master_phase value the engine publishes on the last ENGAGING
    # tick and the first GAIT tick (before _clock.advance) must be equal.
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_past_initialize(engine)

    prev_phase = 0.0
    for _ in range(200):
        engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
        phase = engine.master_phase
        if engine.state is EngineState.GAIT:
            # First GAIT tick: _clock was just reset to exit_master (the
            # modular wrap of the last ENGAGING tick's master_phase) and
            # then advanced once by dt / cycle_time. The modular wrap is
            # not a real discontinuity, so measure step mod 1.
            step = (phase - prev_phase) % 1.0
            assert step < 0.05, (
                f"master_phase stepped by {step} across ENGAGING -> GAIT"
            )
            return
        prev_phase = phase
    raise AssertionError("engine did not reach GAIT within 200 ticks")


def test_master_phase_continuous_across_resuming_to_gait_handoff():
    # Same continuity requirement on the PAUSED -> RESUMING -> GAIT path:
    # begin_resume() seats the engagement controller from the paused
    # phase, and the engine seeds _clock with exit_master at the
    # RESUMING -> GAIT handoff. Posture must see a continuous master_phase
    # across both legs of the transition.
    spy = _SpyStrategy()
    engine = _engine(spy, config=_config(pause_debounce_delay=0.0))
    _drive_to_gait(engine, v_body_xy=(0.20, 0.0), omega_z=0.0)

    # Tick a few GAIT cycles so the paused master_phase is not trivially 0.
    for _ in range(20):
        engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)

    # Park at zero cmd until PAUSED.
    for _ in range(300):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.PAUSED:
            break
    assert engine.state is EngineState.PAUSED

    # First non-zero tick enters RESUMING and runs one engagement tick.
    engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.RESUMING
    prev_phase = engine.master_phase
    assert prev_phase == pytest.approx(engine._engagement.exit_master, abs=1e-12)

    for _ in range(400):
        engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
        phase = engine.master_phase
        if engine.state is EngineState.GAIT:
            # Wrap-aware step across the handoff. exit_master wraps modulo
            # 1, and so does _clock.master, so the smallest forward step
            # is what we want to measure.
            step = (phase - prev_phase) % 1.0
            assert step < 0.05, (
                f"master_phase stepped by {step} across RESUMING -> GAIT"
            )
            return
        assert phase == pytest.approx(
            engine._engagement.exit_master, abs=1e-12
        )
        prev_phase = phase
    raise AssertionError("engine did not reach GAIT within 400 RESUMING ticks")


def test_engaging_to_pausing_on_zero_cmd():
    # cmd zeros mid-engagement: bail out to PAUSING via the
    # PauseController on the very first zero tick, regardless of
    # pause_debounce_delay. The debounce exists for joystick
    # zero-crossings during GAIT; ENGAGING is a transient state whose
    # body velocity has barely ramped, and ticking it at zero cmd would
    # snap mid-flight swing legs back to NOMINAL (AEP collapses to
    # NOMINAL when stride is zero).
    spy = _SpyStrategy()
    engine = _engine(spy, config=_config(pause_debounce_delay=0.8))
    _drive_past_initialize(engine)
    engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.ENGAGING
    engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.PAUSING


def test_brief_zero_cmd_under_debounce_stays_in_gait():
    # Right-joystick passing through center sends cmd_vel to zero for a
    # handful of ticks before swinging back to the new yaw direction.
    # With pause_debounce_delay set, those zero ticks must not trip
    # the pause transition — the engine has to keep ticking GAIT so the
    # cycle resumes seamlessly when cmd_vel returns.
    spy = _SpyStrategy()
    engine = _engine(
        spy, config=_config(pause_debounce_delay=0.15)
    )
    _drive_to_gait(engine, v_body_xy=(0.0, 0.0), omega_z=0.5)
    assert engine.state is EngineState.GAIT

    # 5 zero ticks at dt=0.02 ⇒ 0.10 s of dwell, well under the 0.15 s
    # debounce. The engine must stay in GAIT throughout.
    for _ in range(5):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        assert engine.state is EngineState.GAIT

    # Stick re-engages on the other side of center: still GAIT, no
    # transition controller in the loop.
    engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=-0.5)
    assert engine.state is EngineState.GAIT


def test_sustained_zero_cmd_past_debounce_enters_pausing():
    # If cmd_vel really does stay zero, the debounce expires and the
    # engine commits to PAUSING as before.
    spy = _SpyStrategy()
    engine = _engine(
        spy, config=_config(pause_debounce_delay=0.10)
    )
    _drive_to_gait(engine, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.GAIT

    # 4 ticks × 0.02 = 0.08 s < 0.10 s ⇒ still in GAIT.
    for _ in range(4):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.GAIT

    # The 6th zero tick puts elapsed at 0.12 s ≥ 0.10 s ⇒ engine
    # leaves GAIT. With every leg already at nominal (stride was zero
    # during the debounce ticks) the pause controller has no airborne
    # legs to lower, so it flips to PAUSED immediately. Accept either
    # PAUSING (just entered, still on the same tick) or PAUSED.
    for _ in range(2):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state in (EngineState.PAUSING, EngineState.PAUSED)


def test_debounce_resets_on_nonzero_cmd():
    # A near-miss zero crossing must fully reset the timer so the next
    # zero burst gets its own full window — not whatever was left over
    # from the previous one.
    spy = _SpyStrategy()
    engine = _engine(
        spy, config=_config(pause_debounce_delay=0.10)
    )
    _drive_to_gait(engine, v_body_xy=(0.20, 0.0), omega_z=0.0)

    # Burn most of the window at zero, then bounce back to non-zero.
    for _ in range(4):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.GAIT

    # New zero burst: 4 ticks (0.08 s) must still be inside the window.
    for _ in range(4):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.GAIT


def test_repeat_engagement_after_full_stop():
    # Walk -> stop -> walk: the second walk must re-engage cleanly
    # (no first-tick jump on the second engagement either).
    spy = _SpyStrategy()
    engine = _engine(spy)
    nominal = _nominal_stance()

    _drive_to_gait(engine, v_body_xy=(0.20, 0.0), omega_z=0.0)
    # Stop the engine: drive cmd_vel to zero until STAND is reached.
    for _ in range(500):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        if engine.state is EngineState.STAND:
            break
    assert engine.state is EngineState.STAND

    # Re-engage: first tick should not jump.
    out = engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.ENGAGING
    for name in LEG_NAMES:
        dx = abs(out[name].foot_target[0] - nominal[name][0])
        assert dx < 0.01, f"{name} jumped {dx:.4f} m on second engagement"


def test_gait_swing_liftoff_velocity_matches_body_velocity():
    # Regression for the 2× swing-trajectory bug. Engagement now covers
    # one full cycle and exits at master = 0, so the legs at PEP and
    # about to swing are Tripod A (l_front, r_middle, l_rear — offset 0,
    # phase = 0 at GAIT entry). Tripod B (offset 0.5) sits at AEP and
    # begins GAIT in stance. We trace l_front to capture the swing
    # lift-off velocity.
    engine = Engine(
        config=_config(),
        strategy=Tripod(),
        nominal_stance=_nominal_stance(),
        initial_stance=_initial_stance(),
        coxa_to_bottom=0.02,
        leg_contexts=_leg_contexts(),
    )
    engine.start_initialize()
    dt = 0.001
    cmd_v = 0.20

    trace_gait: list[float] = []
    # Engagement is one full cycle (≈ 1.0 s at v=0.20) plus the short
    # INITIALIZE ladder, so we need well over 1000 ticks at dt=0.001 to
    # collect a useful slice of GAIT.
    for _ in range(1600):
        out = engine.update(dt=dt, v_body_xy=(cmd_v, 0.0), omega_z=0.0)
        if engine.state is EngineState.GAIT:
            trace_gait.append(out["l_front"].foot_target[0])
            if len(trace_gait) >= 20:
                break

    # The engine flips state to GAIT *before* returning the engagement
    # controller's last output, so trace_gait[0] still carries that
    # value. Skip it and measure the velocity from index 1 onward, when
    # GAIT's own swing_arc is producing every tick.
    v_gait_liftoff = (trace_gait[11] - trace_gait[1]) / (10 * dt)
    # Sampled at phase_in_swing ∈ [0.002, 0.022] — essentially the
    # lift-off endpoint of the primary Bezier where dB/dt = -v_in. With
    # the trajectory fix v_in = -cmd_v; pre-fix it was -2·cmd_v, which
    # would land here near -0.40 m/s.
    assert v_gait_liftoff == pytest.approx(-cmd_v, abs=5e-3)


# ---- set_strategy --------------------------------------------------------


def test_set_strategy_swap_in_stand_succeeds():
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_past_initialize(engine)
    assert engine.state is EngineState.STAND
    assert engine.set_strategy("ripple") is True
    assert engine.strategy_name == "ripple"
    # Engagement controller is rebuilt with the new β so a subsequent
    # walk uses ripple's duty factor.
    assert engine._strategy.duty_factor == pytest.approx(2.0 / 3.0)


def test_set_strategy_unknown_name_returns_false():
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_past_initialize(engine)
    assert engine.set_strategy("gallop") is False
    # Strategy must not change.
    assert engine._strategy is spy


def test_set_strategy_same_name_is_no_op():
    spy = _SpyStrategy()
    engine = _engine(spy)
    _drive_past_initialize(engine)
    # The spy resolves to "_spystrategy" via the fallback so use
    # the real Tripod for this check.
    engine2 = Engine(
        config=_config(),
        strategy=Tripod(),
        nominal_stance=_nominal_stance(),
        initial_stance=_initial_stance(),
        coxa_to_bottom=0.02,
        leg_contexts=_leg_contexts(),
    )
    _drive_past_initialize(engine2)
    strategy_before = engine2._strategy
    assert engine2.set_strategy("tripod") is True
    # No swap performed — strategy instance is the same object.
    assert engine2._strategy is strategy_before


@pytest.mark.parametrize(
    "name,expected_duty",
    [("tripod", 0.5), ("ripple", 2.0 / 3.0), ("wave", 5.0 / 6.0)],
)
def test_derive_cycle_time_reads_strategy_duty_factor(name, expected_duty):
    # The engine derives min/max cycle_time = swing_time / (1 − β),
    # then clamps cycle_time = stride / (v * β). Verify both the floor
    # and the divisor track the active strategy.
    from hexa_gait.gaits.base import derive_cycle_time

    spy = _SpyStrategy()
    engine = _engine(spy, config=_config(stride_length=0.10, min_swing_time=0.25))
    _drive_past_initialize(engine)
    assert engine.set_strategy(name) is True
    cfg = engine._config
    expected_min_cycle = 0.25 / (1.0 - expected_duty)
    expected_max_cycle = cfg.max_swing_time / (1.0 - expected_duty)

    def cycle_time(max_leg_v: float) -> float:
        return derive_cycle_time(
            max_leg_v,
            cfg.stride_length,
            expected_duty,
            expected_min_cycle,
            expected_max_cycle,
        )

    # Slow command well under saturation: cycle_time ~= stride/(v*β).
    # Use v small enough that the raw quotient exceeds every gait's
    # max_cycle floor; the engine should clamp to that per-gait max.
    v_small = 0.001
    raw = 0.10 / (v_small * expected_duty)
    assert raw > expected_max_cycle
    assert cycle_time(v_small) == pytest.approx(expected_max_cycle)

    # Fast command above saturation: cycle_time clamped to min_cycle.
    v_fast = 10.0
    assert cycle_time(v_fast) == pytest.approx(expected_min_cycle)


# ---- Gait change while walking -------------------------------------------
#
# set_strategy while walking latches a pending name and runs PAUSING →
# PAUSED (short gait_change_pause_to_reseat_delay dwell) → RESEATING,
# committing the new gait at the RESEATING → STAND handoff. With cmd
# still held, the next STAND tick re-engages in the new gait. ENGAGING
# and RESUMING lock the gait: requests are dropped, not queued.


_CMD = (0.20, 0.0)


def _tripod_engine(config: EngineConfig | None = None) -> Engine:
    return Engine(
        config=config or _config(),
        strategy=Tripod(),
        nominal_stance=_nominal_stance(),
        initial_stance=_initial_stance(),
        coxa_to_bottom=0.02,
        leg_contexts=_leg_contexts(),
    )


def _trace_states(
    engine: Engine,
    v_body_xy: tuple[float, float],
    ticks: int,
    dt: float = 0.02,
) -> list[EngineState]:
    trace: list[EngineState] = []
    for _ in range(ticks):
        engine.update(dt=dt, v_body_xy=v_body_xy, omega_z=0.0)
        trace.append(engine.state)
    return trace


def _assert_ordered_subsequence(
    trace: list[EngineState], expected: list[EngineState]
) -> None:
    it = iter(trace)
    for state in expected:
        assert any(s is state for s in it), (
            f"{state.name} missing or out of order in "
            f"{[s.name for s in trace]}"
        )


def _drive_to_state(
    engine: Engine,
    v_body_xy: tuple[float, float],
    target: EngineState,
    max_ticks: int = 500,
    dt: float = 0.02,
) -> None:
    for _ in range(max_ticks):
        engine.update(dt=dt, v_body_xy=v_body_xy, omega_z=0.0)
        if engine.state is target:
            return
    raise AssertionError(
        f"engine did not reach {target.name} within {max_ticks} ticks "
        f"(ended in {engine.state.name})"
    )


def test_set_strategy_during_gait_runs_pause_reseat_engage_sequence():
    engine = _tripod_engine()
    _drive_to_gait(engine, v_body_xy=_CMD, omega_z=0.0)

    assert engine.set_strategy("ripple") is True
    assert engine.pending_strategy_name == "ripple"

    trace = _trace_states(engine, _CMD, ticks=400)
    _assert_ordered_subsequence(
        trace,
        [
            EngineState.PAUSING,
            EngineState.PAUSED,
            EngineState.RESEATING,
            EngineState.STAND,
            EngineState.ENGAGING,
        ],
    )
    assert EngineState.RESUMING not in trace
    assert engine.strategy_name == "ripple"
    assert engine._strategy.duty_factor == pytest.approx(2.0 / 3.0)
    assert engine.pending_strategy_name is None


def test_gait_change_uses_short_pause_to_reseat_dwell():
    dt = 0.02
    cfg = _config(
        pause_to_reseat_delay=5.0, gait_change_pause_to_reseat_delay=0.1
    )

    # Pending gait change: PAUSED must hand off to RESEATING after the
    # short dwell, nowhere near the 5 s normal settle.
    engine = _tripod_engine(cfg)
    _drive_to_gait(engine, v_body_xy=_CMD, omega_z=0.0)
    engine.set_strategy("ripple")
    _drive_to_state(engine, _CMD, EngineState.PAUSED)
    paused_ticks = 0
    for _ in range(50):
        engine.update(dt=dt, v_body_xy=_CMD, omega_z=0.0)
        if engine.state is not EngineState.PAUSED:
            break
        paused_ticks += 1
    assert engine.state is EngineState.RESEATING
    assert paused_ticks * dt <= 0.1 + dt

    # Control: a normal zero-cmd pause with no pending change must
    # stay PAUSED well past the short-dwell window.
    control = _tripod_engine(cfg)
    _drive_to_gait(control, v_body_xy=_CMD, omega_z=0.0)
    _drive_to_state(control, (0.0, 0.0), EngineState.PAUSED)
    for _ in range(25):  # 0.5 s — 5× the short dwell
        control.update(dt=dt, v_body_xy=(0.0, 0.0), omega_z=0.0)
        assert control.state is EngineState.PAUSED


def test_pending_gait_updates_mid_sequence():
    engine = _tripod_engine()
    _drive_to_gait(engine, v_body_xy=_CMD, omega_z=0.0)

    assert engine.set_strategy("ripple") is True
    engine.update(dt=0.02, v_body_xy=_CMD, omega_z=0.0)
    assert engine.state is EngineState.PAUSING
    assert engine.set_strategy("wave") is True
    assert engine.pending_strategy_name == "wave"

    _drive_to_state(engine, _CMD, EngineState.PAUSED)
    assert engine.set_strategy("wave") is True

    _drive_to_state(engine, _CMD, EngineState.GAIT)
    assert engine.strategy_name == "wave"
    assert engine._strategy.duty_factor == pytest.approx(5.0 / 6.0)


def test_cycle_back_to_original_gait_still_completes_sequence():
    engine = _tripod_engine()
    _drive_to_gait(engine, v_body_xy=_CMD, omega_z=0.0)

    assert engine.set_strategy("ripple") is True
    engine.update(dt=0.02, v_body_xy=_CMD, omega_z=0.0)
    assert engine.state is EngineState.PAUSING
    # Cycle back to the originally-active gait: still latched, and the
    # sequence completes deterministically rather than aborting.
    assert engine.set_strategy("tripod") is True
    assert engine.pending_strategy_name == "tripod"

    trace = _trace_states(engine, _CMD, ticks=400)
    _assert_ordered_subsequence(
        trace,
        [
            EngineState.PAUSED,
            EngineState.RESEATING,
            EngineState.STAND,
            EngineState.ENGAGING,
        ],
    )
    assert EngineState.RESUMING not in trace
    assert engine.strategy_name == "tripod"


def test_set_strategy_locked_during_engaging():
    engine = _tripod_engine()
    _drive_past_initialize(engine)
    engine.update(dt=0.02, v_body_xy=_CMD, omega_z=0.0)
    assert engine.state is EngineState.ENGAGING

    assert engine.set_strategy("ripple") is False
    assert engine.pending_strategy_name is None

    _drive_to_state(engine, _CMD, EngineState.GAIT)
    assert engine.strategy_name == "tripod"


def test_set_strategy_locked_during_resuming():
    engine = _tripod_engine()
    _drive_to_gait(engine, v_body_xy=_CMD, omega_z=0.0)
    _drive_to_state(engine, (0.0, 0.0), EngineState.PAUSED)
    engine.update(dt=0.02, v_body_xy=_CMD, omega_z=0.0)
    assert engine.state is EngineState.RESUMING

    assert engine.set_strategy("ripple") is False
    assert engine.pending_strategy_name is None

    _drive_to_state(engine, _CMD, EngineState.GAIT)
    assert engine.strategy_name == "tripod"


def test_resume_suppressed_while_gait_change_pending():
    engine = _tripod_engine()
    _drive_to_gait(engine, v_body_xy=_CMD, omega_z=0.0)
    engine.set_strategy("ripple")

    # cmd held non-zero the whole way: without the suppression the
    # first PAUSING tick would route straight back to RESUMING.
    for _ in range(400):
        engine.update(dt=0.02, v_body_xy=_CMD, omega_z=0.0)
        assert engine.state is not EngineState.RESUMING
        if engine.state is EngineState.GAIT:
            break
    assert engine.state is EngineState.GAIT
    assert engine.strategy_name == "ripple"


def test_pending_gait_commits_when_cmd_released_mid_sequence():
    engine = _tripod_engine()
    _drive_to_gait(engine, v_body_xy=_CMD, omega_z=0.0)
    engine.set_strategy("ripple")
    engine.update(dt=0.02, v_body_xy=_CMD, omega_z=0.0)
    assert engine.state is EngineState.PAUSING

    # Operator releases the stick mid-sequence: the sequence still
    # completes and commits, then settles in STAND.
    _drive_to_state(engine, (0.0, 0.0), EngineState.STAND)
    assert engine.strategy_name == "ripple"
    assert engine.pending_strategy_name is None
    for _ in range(25):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
        assert engine.state is EngineState.STAND


def test_set_strategy_accepted_during_normal_pause():
    engine = _tripod_engine()
    _drive_to_gait(engine, v_body_xy=_CMD, omega_z=0.0)
    _drive_to_state(engine, (0.0, 0.0), EngineState.PAUSED)

    # Request during an ordinary zero-cmd pause: latched, committed by
    # the same short-dwell reseat handoff.
    assert engine.set_strategy("ripple") is True
    assert engine.pending_strategy_name == "ripple"
    _drive_to_state(engine, (0.0, 0.0), EngineState.STAND)
    assert engine.strategy_name == "ripple"

    # The next walk engages the new gait.
    engine.update(dt=0.02, v_body_xy=_CMD, omega_z=0.0)
    assert engine.state is EngineState.ENGAGING
    assert engine._strategy.duty_factor == pytest.approx(2.0 / 3.0)


# ---- Stance integrator: world-frame foot invariance ---------------------
#
# Wave (β = 5/6) spreads its five stance legs across stance phases
# s ∈ {0, 1/5, …, 4/5}. A velocity change mid-stance must not shift any
# stance foot in the world frame — otherwise the IK pulls loaded feet
# across the ground. Tripod (β = 0.5, three stance legs lockstep) hides
# this; wave is the strict test.


def _wave_engine() -> Engine:
    return Engine(
        config=_config(),
        strategy=Wave(),
        nominal_stance=_nominal_stance(),
        initial_stance=_initial_stance(),
        coxa_to_bottom=0.02,
        leg_contexts=_leg_contexts(),
    )


def test_wave_stance_world_invariant_at_constant_velocity():
    # Sanity check: under constant velocity each stance leg's world
    # position holds across its stance window. Confirms the integrator
    # is the closed-form's equivalent at constant v.
    engine = _wave_engine()
    v_x = 0.05
    _drive_to_gait(engine, v_body_xy=(v_x, 0.0), omega_z=0.0)

    dt = 0.005
    body_x = 0.0
    last_world: dict[str, tuple[float, float] | None] = {n: None for n in LEG_NAMES}
    prev_stance: dict[str, bool] = {n: False for n in LEG_NAMES}

    for _ in range(800):
        out = engine.update(dt=dt, v_body_xy=(v_x, 0.0), omega_z=0.0)
        body_x += v_x * dt
        for name in LEG_NAMES:
            if not out[name].stance:
                last_world[name] = None
                prev_stance[name] = False
                continue
            wx = body_x + out[name].foot_target[0]
            wy = out[name].foot_target[1]
            if prev_stance[name] and last_world[name] is not None:
                dx = abs(wx - last_world[name][0])
                dy = abs(wy - last_world[name][1])
                assert dx < 1e-6, f"{name} world drift dx={dx}"
                assert dy < 1e-6, f"{name} world drift dy={dy}"
            last_world[name] = (wx, wy)
            prev_stance[name] = True


def test_wave_mid_stance_velocity_step_keeps_feet_planted():
    # The headline test: at a mixed stance-phase moment on wave, step
    # the commanded velocity from (0.10, 0) to (0.05, 0.05). Every leg
    # that stays in stance across the step must hold its world-frame
    # position. Pre-fix this would have produced a ~25 mm step for the
    # legs at the extreme stance phases (s ≈ 0 and s ≈ 4/5).
    engine = _wave_engine()
    v_x = 0.10
    _drive_to_gait(engine, v_body_xy=(v_x, 0.0), omega_z=0.0)

    dt = 0.005
    body_x = 0.0
    body_y = 0.0
    out = None
    # Tick into a steady-state stance window so phases have spread.
    for _ in range(60):
        out = engine.update(dt=dt, v_body_xy=(v_x, 0.0), omega_z=0.0)
        body_x += v_x * dt

    assert out is not None
    before: dict[str, tuple[float, float]] = {}
    for name in LEG_NAMES:
        if out[name].stance:
            before[name] = (
                body_x + out[name].foot_target[0],
                body_y + out[name].foot_target[1],
            )
    assert len(before) >= 4, f"expected ≥4 stance legs, got {len(before)}"

    # Single-tick velocity step. The integrator must shift each stance
    # foot's body-frame target by exactly -v_new·dt so the world frame
    # position is preserved.
    v_new = (0.05, 0.05)
    out = engine.update(dt=dt, v_body_xy=v_new, omega_z=0.0)
    body_x += v_new[0] * dt
    body_y += v_new[1] * dt

    for name, (wx_before, wy_before) in before.items():
        if not out[name].stance:
            # Leg may have flipped to swing on this tick; skip.
            continue
        wx_after = body_x + out[name].foot_target[0]
        wy_after = body_y + out[name].foot_target[1]
        dx = abs(wx_after - wx_before)
        dy = abs(wy_after - wy_before)
        # 1 mm tolerance — well above the integrator's exact arithmetic,
        # well below the ≥25 mm slip the closed form would inject.
        assert dx < 1e-3, f"{name} world dx={dx*1000:.2f} mm"
        assert dy < 1e-3, f"{name} world dy={dy*1000:.2f} mm"


def test_engagement_to_gait_seed_world_invariant_under_velocity_step():
    # Drive through engagement → GAIT, then step velocity on the first
    # GAIT cycle while legs are still in their initial stance window.
    # Proves the seed at handoff carries the correct world-locked
    # anchors (not the strategy's closed-form stance target, which
    # would already differ from the engagement controller's integrated
    # position).
    engine = _wave_engine()
    v_x = 0.10
    _drive_to_gait(engine, v_body_xy=(v_x, 0.0), omega_z=0.0)
    assert engine.state is EngineState.GAIT

    dt = 0.005
    # The engine returned out of _drive_to_gait at the handoff tick;
    # the body's world position at that moment is unknown to us.
    # Track from here forward.
    body_x = 0.0
    body_y = 0.0
    # One steady-state GAIT tick to populate _last_targets / outputs.
    out = engine.update(dt=dt, v_body_xy=(v_x, 0.0), omega_z=0.0)
    body_x += v_x * dt

    before: dict[str, tuple[float, float]] = {}
    for name in LEG_NAMES:
        if out[name].stance:
            before[name] = (
                body_x + out[name].foot_target[0],
                body_y + out[name].foot_target[1],
            )
    assert len(before) >= 4

    v_new = (0.05, 0.05)
    out = engine.update(dt=dt, v_body_xy=v_new, omega_z=0.0)
    body_x += v_new[0] * dt
    body_y += v_new[1] * dt

    for name, (wx_before, wy_before) in before.items():
        if not out[name].stance:
            continue
        wx_after = body_x + out[name].foot_target[0]
        wy_after = body_y + out[name].foot_target[1]
        dx = abs(wx_after - wx_before)
        dy = abs(wy_after - wy_before)
        assert dx < 1e-3, f"{name} world dx={dx*1000:.2f} mm"
        assert dy < 1e-3, f"{name} world dy={dy*1000:.2f} mm"


# ---- Swing planner: body-frame foot continuity under mid-swing velocity step
#
# The stance integrator world-locks loaded feet. The symmetric airborne
# requirement is that the swing leg's body-frame trajectory does not
# discontinuously jump when v_y or ω_z is introduced on top of a steady
# v_x. Pre-fix the strategy rebuilt PEP/AEP from the live stride each
# tick, so a mid-swing v_y step shifted the swing foot in body frame by
# a fraction of Δstride — visible on wave (β = 5/6, one airborne leg,
# stance_time ≈ 5× tripod's) and a known source of foot tip slipping.


def test_wave_mid_swing_velocity_step_keeps_swing_foot_continuous():
    engine = _wave_engine()
    v_x = 0.10
    _drive_to_gait(engine, v_body_xy=(v_x, 0.0), omega_z=0.0)

    dt = 0.005
    # Tick into steady-state until a leg is mid-swing (phase well inside
    # [0, 1 − β)). Wave has one airborne leg at a time, so we scan all
    # six and pick whichever is far enough into its swing window that a
    # single-tick velocity step is clearly mid-arc.
    swing_name: str | None = None
    last_out = None
    for _ in range(400):
        last_out = engine.update(dt=dt, v_body_xy=(v_x, 0.0), omega_z=0.0)
        for n in LEG_NAMES:
            # Mid-swing window: phase ∈ (0.04, 0.12) on wave's [0, 1/6)
            # swing — past the lift-off endpoint, before touchdown.
            if not last_out[n].stance and 0.04 < last_out[n].phase < 0.12:
                swing_name = n
                break
        if swing_name is not None:
            break
    assert swing_name is not None, "no leg landed mid-swing inside scan window"
    assert last_out is not None

    pos_before = last_out[swing_name].foot_target

    # Single-tick velocity step: add v_y and ω_z on top of steady v_x.
    # The body-frame swing foot must shift by at most the latched arc's
    # own per-tick progression (≪ 1 mm at dt = 5 ms on wave) — pre-fix
    # the rebuilt PEP/AEP would have jumped this by O(Δv · stance_time)
    # = O(0.05 · 1 s) · 0.5 = O(25 mm) in y, plus an ω_z contribution.
    out = engine.update(dt=dt, v_body_xy=(v_x, 0.05), omega_z=0.3)
    # Sanity: the leg we were tracking must still be in swing.
    assert not out[swing_name].stance

    pos_after = out[swing_name].foot_target
    dx = abs(pos_after[0] - pos_before[0])
    dy = abs(pos_after[1] - pos_before[1])
    dz = abs(pos_after[2] - pos_before[2])
    # Latched-arc per-tick motion is bounded by the foot's body-frame
    # swing velocity ~ |stride| / swing_time = stride_length ≈ 0.10 m /
    # min_swing_time 0.25 s = 0.4 m/s. dt = 5 ms ⇒ ≤ 2 mm/tick. 3 mm
    # leaves headroom for the Bezier's mid-arc curvature.
    assert dx < 3.0e-3, f"swing dx={dx*1000:.2f} mm"
    assert dy < 3.0e-3, f"swing dy={dy*1000:.2f} mm"
    assert dz < 3.0e-3, f"swing dz={dz*1000:.2f} mm"


def test_wave_swing_liftoff_velocity_matches_body_velocity():
    # Wave's β = 5/6 means the default ``swing_origin_velocity =
    # -stride/swing_time`` resolves to -5·v_leg, a 5× velocity step at
    # lift-off. The SwingPlanner overrides this with -v_leg so swing
    # launches at the stance-frame velocity. Sampling the foot tip a
    # few ticks past lift-off measures the primary Bezier's endpoint
    # velocity directly.
    engine = _wave_engine()
    cmd_v = 0.10
    _drive_to_gait(engine, v_body_xy=(cmd_v, 0.0), omega_z=0.0)
    dt = 0.001

    # Find a fresh lift-off and trace the airborne leg's x position
    # over the first ~20 swing ticks. Skip the leg whose lift-off
    # happens at the engagement→GAIT handoff (origin = engagement's
    # mid-swing position, not -0.5·stride) — only steady-state GAIT
    # lift-offs use the integrator's PEP as the swing origin.
    prev_stance: dict[str, bool] = {n: True for n in LEG_NAMES}
    trace: list[float] = []
    tracked: str | None = None
    # First pass: let the engagement-mid-swing leg complete its swing
    # so subsequent lift-offs are pure GAIT.
    for _ in range(int(2.0 / dt)):
        out = engine.update(dt=dt, v_body_xy=(cmd_v, 0.0), omega_z=0.0)
        if tracked is None:
            for n in LEG_NAMES:
                if prev_stance[n] and not out[n].stance:
                    tracked = n
                    trace.append(out[n].foot_target[0])
                    break
        elif not out[tracked].stance:
            trace.append(out[tracked].foot_target[0])
            if len(trace) >= 20:
                break
        else:
            # Tracked leg already touched down — restart search.
            tracked = None
            trace.clear()
        for n in LEG_NAMES:
            prev_stance[n] = out[n].stance

    assert tracked is not None and len(trace) >= 20, "no clean lift-off seen"
    # Sample dB/dt at the lift-off endpoint of the primary Bezier:
    # average finite difference over the first ~15 ticks, well inside
    # phase_in_swing ≪ 0.5. Pre-fix this would be ≈ -5·cmd_v = -0.50.
    v_liftoff = (trace[15] - trace[1]) / (14 * dt)
    assert v_liftoff == pytest.approx(-cmd_v, abs=2.0e-2)
