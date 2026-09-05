"""Operator presets: loading, validation, and following /cmd_gait.

The bookkeeping both teleops share. What matters most here is the resync: it is
the one path a gait switch takes whoever initiated it, so a fact it forgets to
update is a fact one teleop has and the other does not.
"""

from __future__ import annotations

import math

import pytest
import yaml

from hexa_common.gait_catalog import GAIT_DESCRIPTORS
from hexa_common.limits import VelocityCaps

from hexa_teleop.joy_mapping import (
    ANIMATION,
    GAIT,
    BaseConfig,
    JoyConfig,
    JoyState,
    ModeConfig,
    PostureConfig,
)
from hexa_teleop.presets import (
    HEXAPOD,
    QUADRUPED,
    leg_set_switch_allowed,
    load_presets,
    resync,
)

_YAML = """
allow_unstable_gaits: true
presets:
  default: normal
  switch_timeout_s: 4.0
  list:
    - id: normal
      label: NORMAL
      sub: six legs
      gait_cycle: [tripod, surf, tetrapod, crawl, ripple]
      default_gait: tripod
    - id: quad
      label: QUAD
      sub: four corners, middle pair parked
      gait_cycle: [quad_canter, quad_walk]
      default_gait: quad_canter
"""


def _registry(text: str = _YAML):
    return load_presets(yaml.safe_load(text))


@pytest.fixture
def registry():
    return _registry()


@pytest.fixture
def caps():
    # Only the lookups resync makes; the real table comes from tuning.yaml. The
    # values are per-gait so a test can tell one switch's caps from another's.
    return VelocityCaps(
        {name: 0.1 + 0.01 * i for i, name in enumerate(GAIT_DESCRIPTORS)},
        {name: 0.5 + 0.01 * i for i, name in enumerate(GAIT_DESCRIPTORS)},
        {name: 0.0 for name in GAIT_DESCRIPTORS},
    )


@pytest.fixture
def joy_cfg(registry):
    """A JoyConfig with only the fields resync reads and rewrites.

    The bindings and the posture envelope are map_joy's business, not this
    module's — everything here is the two rotations and the two caps.
    """
    base = BaseConfig(
        deadband=0.1,
        trigger_threshold=0.5,
        button_index={},
        axis_index={},
        axis_sign={},
        bindings={},
    )
    posture = PostureConfig(
        bindings={},
        x_max=0.04,
        y_max=0.04,
        roll_max=0.2,
        pitch_max=0.2,
        yaw_max=0.3,
        yaw_tau=0.1,
        revert_tau=0.25,
        wiggle_pivot_forward_m=0.06,
        height_max=0.05,
        height_min=-0.04,
        height_rate=0.05,
    )
    cfg = JoyConfig(
        base=base,
        gait=ModeConfig(bindings={}),
        posture=posture,
        animation=ModeConfig(bindings={}),
        gait_cycle=(),
        quadruped_gait_cycle=(),
        default_quadruped_gait="quad_canter",
        gait_linear_max=0.1,
        gait_angular_z_max=0.5,
        stance_unit=(1.0, 0.0),
        animation_list=("vertical_body_roll",),
    )
    return registry.project(cfg)


# ─── Loading and validation ────────────────────────────────────────

def test_leg_set_is_derived_from_the_catalog(registry):
    # Never declared in the config: the catalog already owns which legs a gait
    # walks, and a second copy is a second copy to keep true.
    for preset in registry.presets:
        for gait in preset.gait_cycle:
            assert GAIT_DESCRIPTORS[gait].leg_set == preset.leg_set

    assert registry.get("normal").leg_set == HEXAPOD
    assert registry.get("quad").leg_set == QUADRUPED


def test_a_preset_mixing_leg_sets_is_a_load_error():
    bad = _YAML.replace(
        "      gait_cycle: [quad_canter, quad_walk]",
        "      gait_cycle: [quad_canter, tetrapod]",
    ).replace("      gait_cycle: [tripod, surf, tetrapod, crawl, ripple]",
              "      gait_cycle: [tripod, surf, crawl, ripple]")
    with pytest.raises(ValueError, match="leg set"):
        _registry(bad)


def test_two_presets_sharing_a_gait_is_a_load_error():
    # Disjointness is what makes a bare name on /cmd_gait name one preset.
    bad = _YAML.replace(
        "      gait_cycle: [quad_canter, quad_walk]",
        "      gait_cycle: [quad_canter, quad_walk, tripod]",
    )
    with pytest.raises(ValueError):
        _registry(bad)


def test_a_default_gait_outside_its_own_rotation_is_a_load_error():
    bad = _YAML.replace("      default_gait: quad_canter",
                        "      default_gait: tripod")
    with pytest.raises(ValueError, match="default_gait"):
        _registry(bad)


def test_an_unknown_default_preset_is_a_load_error():
    with pytest.raises(ValueError, match="default"):
        _registry(_YAML.replace("  default: normal", "  default: sideways"))


def test_unstable_gaits_are_filtered_per_preset():
    reg = _registry(_YAML.replace("allow_unstable_gaits: true",
                                  "allow_unstable_gaits: false"))
    normal = reg.get("normal")
    assert "surf" not in normal.gait_cycle
    assert "crawl" not in normal.gait_cycle
    assert "tripod" in normal.gait_cycle


def test_entry_gait_starts_at_the_preset_default(registry):
    assert registry.entry_gait("normal") == "tripod"
    assert registry.entry_gait("quad") == "quad_canter"


def test_for_gait_finds_the_owning_preset(registry):
    assert registry.for_gait("ripple").id == "normal"
    assert registry.for_gait("quad_walk").id == "quad"
    assert registry.for_gait("moonwalk") is None


# ─── Projection onto the two JoyConfig rotations ───────────────────

def test_project_writes_the_active_rotations(registry, joy_cfg):
    # The whole trick that keeps map_joy untouched: it still sees one six-leg
    # rotation and one four-corner one, and never learns either can be swapped.
    assert joy_cfg.gait_cycle == registry.get("normal").gait_cycle
    assert joy_cfg.quadruped_gait_cycle == registry.get("quad").gait_cycle
    assert joy_cfg.default_quadruped_gait == "quad_canter"


# ─── resync ────────────────────────────────────────────────────────

def _state(**kw) -> JoyState:
    return JoyState(mode=GAIT, **kw)


def test_resync_sets_the_quadruped_flag(registry, caps, joy_cfg):
    # The regression this module exists for. map_joy reads state.quadruped to
    # pick which rotation the D-pad walks and to gate animation mode, and the
    # old resync_gait never touched it — so a gamepad `select` left the web app
    # cycling the six-leg list while the robot stood on four.
    state = _state()
    assert state.quadruped is False

    result = resync("quad_walk", joy_cfg, state, caps, registry)
    assert result is not None
    assert state.quadruped is True
    assert result.leg_set_changed is True
    assert result.preset.id == "quad"

    back = resync("tripod", result.cfg, state, caps, registry)
    assert back is not None
    assert state.quadruped is False
    assert back.leg_set_changed is True


def test_resync_updates_the_right_cycler_index(registry, caps, joy_cfg):
    state = _state(current_gait_idx=0)
    result = resync("ripple", joy_cfg, state, caps, registry)
    assert state.current_gait_idx == result.cfg.gait_cycle.index("ripple")
    # The six-leg slot is untouched by a four-corner switch: it is still the
    # operator's, waiting for them to come back to it.
    quad = resync("quad_walk", result.cfg, state, caps, registry)
    assert state.current_gait_idx == result.cfg.gait_cycle.index("ripple")
    assert state.current_quadruped_gait_idx == (
        quad.cfg.quadruped_gait_cycle.index("quad_walk")
    )


def test_resync_remembers_each_presets_slot(registry, caps, joy_cfg):
    state = _state(current_gait_idx=0)
    cfg = joy_cfg
    cfg = resync("ripple", cfg, state, caps, registry).cfg
    cfg = resync("quad_walk", cfg, state, caps, registry).cfg
    # Selecting NORMAL again should offer the gait it was left on, not the
    # preset's cold default — the round trip lands where the operator was.
    assert registry.entry_gait("normal") == "ripple"
    assert registry.entry_gait("quad") == "quad_walk"


def test_resync_updates_the_caps(registry, caps, joy_cfg):
    state = _state()
    result = resync("ripple", joy_cfg, state, caps, registry)
    assert math.isclose(result.cfg.gait_linear_max, caps.linear_max("ripple"))
    assert math.isclose(result.cfg.gait_angular_z_max, caps.angular_max("ripple"))


def test_resync_starts_the_posture_revert_on_a_leg_set_change(
    registry, caps, joy_cfg
):
    # The engine will not move the middle pair until the body pose is neutral,
    # and map_joy's revert decay is the only thing that puts it there.
    state = _state()
    state.recorded_x = 0.02
    result = resync("quad_walk", joy_cfg, state, caps, registry)
    assert result.leg_set_changed is True
    assert state.reverting is True


def test_resync_does_not_revert_on_a_plain_gait_switch(registry, caps, joy_cfg):
    state = _state()
    state.recorded_x = 0.02
    result = resync("ripple", joy_cfg, state, caps, registry)
    assert result.leg_set_changed is False
    assert state.reverting is False


def test_resync_drops_out_of_animation_mode_on_four_legs(
    registry, caps, joy_cfg
):
    # Every animation is written for six legs, and map_joy only blocks
    # *entering* the mode — arriving there on four has to leave.
    state = JoyState(mode=ANIMATION)
    result = resync("quad_walk", joy_cfg, state, caps, registry)
    assert result.left_animation_mode is True
    assert state.mode == GAIT


def test_resync_leaves_animation_mode_alone_on_six_legs(
    registry, caps, joy_cfg
):
    state = JoyState(mode=ANIMATION)
    result = resync("ripple", joy_cfg, state, caps, registry)
    assert result.left_animation_mode is False
    assert state.mode == ANIMATION


def test_resync_unknown_name_returns_none(registry, joy_cfg):
    empty = VelocityCaps({}, {}, {})
    state = _state(current_gait_idx=1)
    assert resync("moonwalk", joy_cfg, state, empty, registry) is None
    assert state.current_gait_idx == 1
    assert state.quadruped is False


def test_resync_of_an_own_loopback_is_idempotent(registry, caps, joy_cfg):
    # The node hears its own accepted publish back; landing somewhere else
    # would make every switch jump two slots.
    state = _state(current_gait_idx=0)
    first = resync("tetrapod", joy_cfg, state, caps, registry)
    idx = state.current_gait_idx
    second = resync("tetrapod", first.cfg, state, caps, registry)
    assert state.current_gait_idx == idx
    assert second.leg_set_changed is False


# ─── The publish gate ──────────────────────────────────────────────

def test_leg_set_switch_allowed_only_where_the_engine_takes_one():
    for state in ("stand", "folded", "fault"):
        assert leg_set_switch_allowed(state), state
    # Narrower than the gait-switch set on purpose: a plain gait swap latches
    # and applies at the next stand, a leg-set change does not.
    for state in (
        "gait",
        "initialize",
        "engaging",
        "folding",
        "settling",
        "reseating",
        "folding_pair",
        "unfolding_pair",
        "",
    ):
        assert not leg_set_switch_allowed(state), state
