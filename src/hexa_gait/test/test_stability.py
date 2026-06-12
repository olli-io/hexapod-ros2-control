"""Quasi-static stability margins of the gait strategies.

Evaluates each registered strategy closed-form at constant command:
per-leg phases from the strategy's own offsets, stance feet from its
pure ``foot_target``, and ``support_polygon_margin`` sampled across a
full master cycle for a sweep of commands at fractions of the gait's
velocity cap. At constant cmd_vel the engine's StanceIntegrator
reproduces the strategy's stance Bezier exactly, so this pins the
steady-state stability of (phase offsets, β) without engine state.

CoM is the body origin (no posture offsets in this layer). Sampling
uses a half-offset grid plus points just before and just after every
lift-off / touchdown seam; the seam instants themselves are
measure-zero hand-offs where the foot is grounded on both sides.

Per-gait floors are pinned a few mm under the measured worst case so
an offsets / β edit that degrades stability fails loudly. Measured
worst margins with this geometry (commands up to 80 % of each gait's
cap):

- **tripod**   — +0.018 m
- **tetrapod** — +0.026 m
- **crawl**   — +0.043 m
- **surf**     — +0.044 m
- **ripple**     — +0.054 m

Surf with evenly spread metachronal offsets is quasi-statically
unstable at every duty factor below tetrapod's (≈ −0.013 m at
β = 5/8); the tripod-grouped offsets in ``gaits/surf.py`` are what
make it positive. If a floor fails, look at the offset tables first.
"""

import math

import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.gaits import STRATEGIES
from hexa_gait.gaits.base import (
    LegContext,
    StrideParams,
    derive_cycle_time,
    per_leg_planar_velocity,
    stride_vector,
)
from hexa_gait.stability import support_polygon_margin


# ─── support_polygon_margin unit behaviour ──────────────────────────


def test_margin_square_centre():
    feet = [(0.1, 0.1), (0.1, -0.1), (-0.1, -0.1), (-0.1, 0.1)]
    assert support_polygon_margin(feet) == pytest.approx(0.1)


def test_margin_negative_outside():
    feet = [(0.1, 0.1), (0.1, 0.2), (0.2, 0.15)]
    m = support_polygon_margin(feet)
    assert m < 0.0
    # Nearest hull point is (0.1, 0.1).
    assert m == pytest.approx(-math.hypot(0.1, 0.1))


def test_margin_degenerate_support_never_positive():
    assert support_polygon_margin([]) == -math.inf
    assert support_polygon_margin([(0.05, 0.0)]) == pytest.approx(-0.05)
    # Two feet: distance to the segment, negated.
    assert support_polygon_margin([(0.1, 0.1), (0.1, -0.1)]) == pytest.approx(-0.1)
    # Collinear triple is still a degenerate hull.
    assert support_polygon_margin([(0.1, -0.1), (0.1, 0.0), (0.1, 0.1)]) == pytest.approx(
        -0.1
    )


def test_margin_com_offset():
    feet = [(0.1, 0.1), (0.1, -0.1), (-0.1, -0.1), (-0.1, 0.1)]
    assert support_polygon_margin(feet, com_xy=(0.05, 0.0)) == pytest.approx(0.05)


# ─── steady-state strategy sweep ─────────────────────────────────────

_NOMINAL = {
    "l_front": (0.15, 0.10, -0.10),
    "r_front": (0.15, -0.10, -0.10),
    "l_middle": (0.0, 0.12, -0.10),
    "r_middle": (0.0, -0.12, -0.10),
    "l_rear": (-0.15, 0.10, -0.10),
    "r_rear": (-0.15, -0.10, -0.10),
}
_CTX = {
    n: LegContext(name=n, mount_xyz=(x, y, 0.0), mount_yaw=0.0, nominal_stance=(x, y, z))
    for n, (x, y, z) in _NOMINAL.items()
}
_STRIDE_LENGTH = 0.10
_MIN_SWING_TIME = 0.30
_MAX_SWING_TIME = 0.40
_R_OUTER = max(math.hypot(x, y) for x, y, _ in _NOMINAL.values())
_N_GRID = 480


def _commands(beta: float) -> list[tuple[float, float, float]]:
    """Commands at fractions of the gait's own velocity cap.

    The cap mirrors the engine's envelope:
    ``stride_length · (1 − β) / (min_swing_time · β)``; yaw commands
    scale the same per-leg budget by the outer nominal-foot radius.
    """
    cap = _STRIDE_LENGTH * (1.0 - beta) / (_MIN_SWING_TIME * beta)
    return [
        (0.8 * cap, 0.0, 0.0),
        (0.0, 0.8 * cap, 0.0),
        (0.55 * cap, 0.55 * cap, 0.0),
        (0.0, 0.0, 0.8 * cap / _R_OUTER),
        (0.4 * cap, 0.0, 0.4 * cap / _R_OUTER),
    ]


def _stride_params(strategy, cmd) -> dict[str, StrideParams]:
    """Per-leg StrideParams at constant cmd, mirroring a GAIT tick."""
    beta = strategy.duty_factor
    v_legs = per_leg_planar_velocity(_CTX, (cmd[0], cmd[1]), cmd[2])
    max_v = max(math.hypot(vx, vy) for vx, vy in v_legs.values())
    cycle_time = derive_cycle_time(
        max_v,
        _STRIDE_LENGTH,
        beta,
        _MIN_SWING_TIME / (1.0 - beta),
        _MAX_SWING_TIME / (1.0 - beta),
    )
    stance_time = cycle_time * beta
    return {
        name: StrideParams(
            stride_vector=stride_vector(vx, vy, stance_time, _STRIDE_LENGTH),
            cycle_time=cycle_time,
            duty_factor=beta,
            swing_clearance=0.03,
            swing_width=0.0,
            controller_dt=0.02,
        )
        for name, (vx, vy) in v_legs.items()
    }


def _masters(strategy):
    """Half-offset grid plus samples either side of every seam."""
    beta = strategy.duty_factor
    seams = set()
    for o in strategy.phase_offsets.offsets.values():
        seams.add((1.0 - o) % 1.0)  # lift-off
        seams.add((1.0 - o - (1.0 - beta)) % 1.0)  # touchdown
    yield from ((i + 0.5) / _N_GRID for i in range(_N_GRID))
    for e in seams:
        yield (e + 1e-6) % 1.0
        yield (e - 1e-6) % 1.0


def _worst_margin(gait_name: str) -> float:
    strategy = STRATEGIES[gait_name]()
    beta = strategy.duty_factor
    offsets = strategy.phase_offsets.offsets
    worst = math.inf
    for cmd in _commands(beta):
        params = _stride_params(strategy, cmd)
        for master in _masters(strategy):
            feet = []
            for name in LEG_NAMES:
                phase = (master + offsets[name]) % 1.0
                if phase < 1.0 - beta:
                    continue  # swing — foot airborne
                x, y, _ = strategy.foot_target(phase, params[name], _CTX[name])
                feet.append((x, y))
            worst = min(worst, support_polygon_margin(feet))
    return worst


_MARGIN_FLOORS = {
    "tripod": 0.012,
    "tetrapod": 0.020,
    "crawl": 0.038,
    "surf": 0.038,
    "ripple": 0.048,
}


def test_every_registered_gait_has_a_pinned_floor():
    assert set(_MARGIN_FLOORS) == set(STRATEGIES)


@pytest.mark.parametrize("gait_name", sorted(_MARGIN_FLOORS))
def test_gait_keeps_com_inside_support_polygon(gait_name):
    worst = _worst_margin(gait_name)
    assert worst > _MARGIN_FLOORS[gait_name], (
        f"{gait_name}: worst quasi-static margin {worst:.4f} m "
        f"(pinned floor {_MARGIN_FLOORS[gait_name]} m)"
    )


def test_unstable_marks_pin_surf_and_crawl():
    # The ``unstable`` class attribute is what teleop's
    # ``allow_unstable_gaits: false`` filters out of the D-pad gait
    # rotation. Pin the set so a new or edited strategy makes an
    # explicit choice here.
    unstable = {n for n, factory in STRATEGIES.items() if factory().unstable}
    assert unstable == {"surf", "crawl"}
