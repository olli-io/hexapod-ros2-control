import math

import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.engine import Engine, EngineConfig, EngineState
from hexa_gait.gaits.base import LegContext, StrideParams
from hexa_gait.gaits.tripod import TRIPOD_OFFSETS


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


def _leg_contexts() -> dict[str, LegContext]:
    nominal = _nominal_stance()
    return {
        n: LegContext(name=n, mount_xyz=_MOUNTS[n], mount_yaw=0.0, nominal_stance=nominal[n])
        for n in LEG_NAMES
    }


def _config(
    *,
    stride_length: float = 0.10,
    min_cycle_time: float = 0.5,
    max_cycle_time: float = 2.0,
    duty_factor: float = 0.5,
) -> EngineConfig:
    return EngineConfig(
        stride_length=stride_length,
        min_cycle_time=min_cycle_time,
        max_cycle_time=max_cycle_time,
        duty_factor=duty_factor,
        step_height=0.03,
        swing_width=0.0,
        controller_dt=0.02,
        recenter_swing_time=0.4,
        cmd_zero_tol=1.0e-4,
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
        leg_contexts=_leg_contexts(),
    )


def test_below_saturation_derives_cycle_time_from_velocity():
    # v = 0.20 m/s straight forward, stride_length = 0.10, duty = 0.5
    # → cycle_time_raw = 0.10 / (0.20 × 0.5) = 1.0 s, comfortably inside
    # [min, max]. stance_time = 0.5 s. Per-leg stride = 0.20 × 0.5 = 0.10 m.
    spy = _SpyStrategy()
    engine = _engine(spy)
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
    out = engine.update(dt=0.02, v_body_xy=(0.0, 0.0), omega_z=0.0)

    assert engine.state is EngineState.STAND
    # Strategy was never invoked: STAND emits nominal directly.
    assert spy.calls == []
    for name in LEG_NAMES:
        assert out[name].stance is True


def test_phase_advances_faster_at_higher_velocity():
    # Two engines, identical except for the commanded velocity: the
    # faster command must accumulate phase faster across a fixed dt
    # because cycle_time shrinks proportionally.
    spy_a = _SpyStrategy()
    spy_b = _SpyStrategy()
    engine_a = _engine(spy_a)
    engine_b = _engine(spy_b)

    for _ in range(5):
        engine_a.update(dt=0.05, v_body_xy=(0.10, 0.0), omega_z=0.0)
        engine_b.update(dt=0.05, v_body_xy=(0.30, 0.0), omega_z=0.0)

    # Use the last recorded phase for a representative leg.
    last_phase_a = next(phase for name, phase, _ in reversed(spy_a.calls) if name == "l_front")
    last_phase_b = next(phase for name, phase, _ in reversed(spy_b.calls) if name == "l_front")

    # The faster engine should be further along its (shorter) cycle.
    assert last_phase_b > last_phase_a
