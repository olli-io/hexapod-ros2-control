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
    max_cycle_time: float = 2.0,
    forced_touchdown_delay: float = 0.0,
) -> EngineConfig:
    # min_swing_time=0.25, β=0.5 (tripod) → min_cycle_time = 0.5 s, same
    # as the pre-refactor default. Other gait factors derive their own
    # floor from min_swing_time / (1 − β).
    return EngineConfig(
        stride_length=stride_length,
        min_swing_time=min_swing_time,
        max_cycle_time=max_cycle_time,
        step_height=0.03,
        swing_width=0.0,
        controller_dt=0.02,
        cmd_zero_tol=1.0e-4,
        forced_touchdown_delay=forced_touchdown_delay,
        max_foot_speed=0.333,
        max_swing_time=0.6,
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
        reseat_settle_delay=0.1,
        reseat_height_change_threshold=0.001,
        reseat_pair_swing_time=0.04,
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


def test_engaging_to_stopping_on_zero_cmd():
    # cmd zeros mid-engagement: bail out to STOPPING via the
    # DisengagementController on the very first zero tick, regardless of
    # forced_touchdown_delay. The debounce exists for joystick
    # zero-crossings during GAIT; ENGAGING is a transient state whose
    # body velocity has barely ramped, and ticking it at zero cmd would
    # snap mid-flight swing legs back to NOMINAL (AEP collapses to
    # NOMINAL when stride is zero).
    spy = _SpyStrategy()
    engine = _engine(spy, config=_config(forced_touchdown_delay=0.8))
    _drive_past_initialize(engine)
    engine.update(dt=0.02, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.ENGAGING
    engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.STOPPING


def test_brief_zero_cmd_under_debounce_stays_in_gait():
    # Right-joystick passing through center sends cmd_vel to zero for a
    # handful of ticks before swinging back to the new yaw direction.
    # With forced_touchdown_delay set, those zero ticks must not trip
    # the stop transition — the engine has to keep ticking GAIT so the
    # cycle resumes seamlessly when cmd_vel returns.
    spy = _SpyStrategy()
    engine = _engine(
        spy, config=_config(forced_touchdown_delay=0.15)
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


def test_sustained_zero_cmd_past_debounce_enters_stopping():
    # If cmd_vel really does stay zero, the debounce expires and the
    # engine commits to STOPPING as before.
    spy = _SpyStrategy()
    engine = _engine(
        spy, config=_config(forced_touchdown_delay=0.10)
    )
    _drive_to_gait(engine, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.GAIT

    # 4 ticks × 0.02 = 0.08 s < 0.10 s ⇒ still in GAIT.
    for _ in range(4):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state is EngineState.GAIT

    # The 6th zero tick puts elapsed at 0.12 s ≥ 0.10 s ⇒ engine
    # leaves GAIT. With every leg already at nominal (stride was zero
    # during the debounce ticks) the stop transition completes inside
    # the same tick it begins, so the engine can be observed in either
    # STOPPING (just entered) or STAND (already drained).
    for _ in range(2):
        engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)
    assert engine.state in (EngineState.STOPPING, EngineState.STAND)


def test_debounce_resets_on_nonzero_cmd():
    # A near-miss zero crossing must fully reset the timer so the next
    # zero burst gets its own full window — not whatever was left over
    # from the previous one.
    spy = _SpyStrategy()
    engine = _engine(
        spy, config=_config(forced_touchdown_delay=0.10)
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


def test_set_strategy_outside_stand_rejected():
    # Set up a walking engine: the swap must be refused. The engine
    # stays on its current strategy.
    engine = Engine(
        config=_config(),
        strategy=Tripod(),
        nominal_stance=_nominal_stance(),
        initial_stance=_initial_stance(),
        coxa_to_bottom=0.02,
        leg_contexts=_leg_contexts(),
    )
    _drive_to_gait(engine, v_body_xy=(0.20, 0.0), omega_z=0.0)
    assert engine.state is EngineState.GAIT
    assert engine.set_strategy("ripple") is False
    assert engine.strategy_name == "tripod"


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
    # The engine derives min_cycle_time = min_swing_time / (1 − β),
    # then clamps cycle_time = stride / (v * β). Verify both the floor
    # and the divisor track the active strategy.
    spy = _SpyStrategy()
    engine = _engine(spy, config=_config(stride_length=0.10, min_swing_time=0.25))
    _drive_past_initialize(engine)
    assert engine.set_strategy(name) is True
    expected_min_cycle = 0.25 / (1.0 - expected_duty)

    # Slow command well under saturation: cycle_time ~= stride/(v*β).
    # Use v small enough that all gaits stay below their min_cycle
    # floor; the engine should report the floor.
    v_small = 0.01
    raw = 0.10 / (v_small * expected_duty)
    assert raw > expected_min_cycle
    out_cycle = engine._derive_cycle_time(v_small)
    # raw should be clamped to max_cycle_time = 2.0
    assert out_cycle == pytest.approx(2.0)

    # Fast command above saturation: cycle_time clamped to min_cycle.
    v_fast = 10.0
    out_cycle = engine._derive_cycle_time(v_fast)
    assert out_cycle == pytest.approx(expected_min_cycle)
