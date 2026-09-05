"""Operator presets: loading, validation, and following the wire.

The bookkeeping both teleops share. What matters most here are the three resync
paths — a gait on /cmd_gait, a preset REQUEST on /cmd_preset, and the engine's
applied preset on /gait/preset. Each is the one path its event takes whoever
initiated it, so a fact one of them forgets to update is a fact one teleop has
and the other does not.

The leg set is NOT derived from the gait any more: tuning.yaml declares it per
preset, because normal, fast and offroad all stand on six legs and can all offer
the same gaits, so a gait name cannot say which preset is in force.
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
    load_presets,
    preset_switch_allowed,
    resync_gait,
    resync_preset,
    resync_preset_request,
)

# The operator half. Ids must match the tuning.yaml block below, which owns the
# physical half — the leg set most of all.
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
    - id: fast
      label: FAST
      sub: low and long-striding
      # Overlaps normal's rotation, which is the point: two six-leg presets can
      # offer the same gaits, and /cmd_preset is what tells them apart.
      gait_cycle: [tripod, surf]
      default_gait: tripod
    - id: quad
      label: QUAD
      sub: four corners, middle pair parked
      gait_cycle: [quad_canter, quad_walk]
      default_gait: quad_canter
"""

# The physical half, in tuning.yaml's shape. Only the fields the preset loader
# reads: the leg set, and the stride/swing bundle the caps are derived from —
# `fast` gets a longer stride at a quicker cadence so a test can tell its caps
# from normal's.
_TUNING = """
gait_node:
  ros__parameters:
    yaw_bias: 0.6
    swing_phase_margin: 0.12
    quadruped_swing_phase_margin: 0.25
    default_preset: normal
    presets:
      - id: normal
        leg_set: hexapod
        standing_pose:
          body_height: 0.06
          front: {tip_reach: 0.135, coxa_deg: 25}
          middle: {tip_reach: 0.135, coxa_deg: 0}
          rear: {tip_reach: 0.135, coxa_deg: 0}
        stride_length: 0.105
        stride_length_radial: 0.085
        min_swing_time: 0.6
        max_swing_time: 0.8
        step_height: 0.04
      - id: fast
        leg_set: hexapod
        standing_pose:
          body_height: 0.055
          front: {tip_reach: 0.138, coxa_deg: 25}
          middle: {tip_reach: 0.138, coxa_deg: 0}
          rear: {tip_reach: 0.138, coxa_deg: 0}
        stride_length: 0.125
        stride_length_radial: 0.100
        min_swing_time: 0.45
        max_swing_time: 0.60
        step_height: 0.035
      - id: quad
        leg_set: quadruped
        standing_pose:
          body_height: 0.06
          front: {tip_reach: 0.119, coxa_deg: 25}
          rear: {tip_reach: 0.119, coxa_deg: 25}
        stride_length: 0.105
        stride_length_radial: 0.085
        min_swing_time: 0.6
        max_swing_time: 0.8
        step_height: 0.04
"""

_GEOMETRY = """
mounts:
  l_front: {x: 0.06, y: 0.05, yaw_deg: 30}
  l_middle: {x: 0.0, y: 0.07, yaw_deg: 90}
"""


@pytest.fixture(scope="module")
def yaml_paths(tmp_path_factory):
    d = tmp_path_factory.mktemp("presets")
    (d / "tuning.yaml").write_text(_TUNING)
    (d / "geometry.yaml").write_text(_GEOMETRY)
    return d / "tuning.yaml", d / "geometry.yaml"


@pytest.fixture
def _registry_factory(yaml_paths):
    tuning, geometry = yaml_paths

    def make(text: str = _YAML, tuning_text: str | None = None):
        t = tuning
        if tuning_text is not None:
            t = tuning.parent / "tuning_override.yaml"
            t.write_text(tuning_text)
        return load_presets(yaml.safe_load(text), t, geometry)

    return make


@pytest.fixture
def registry(_registry_factory):
    return _registry_factory()


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

def test_leg_set_is_read_from_tuning_yaml(registry):
    # Never declared in the teleop config: tuning.yaml owns whether the middle
    # pair stands, because that is a physical fact about the robot and not a
    # property of the gaits a preset happens to offer.
    assert registry.get("normal").leg_set == HEXAPOD
    assert registry.get("fast").leg_set == HEXAPOD
    assert registry.get("quad").leg_set == QUADRUPED


def test_every_gait_in_a_rotation_walks_the_presets_legs(registry):
    for preset in registry.presets:
        for gait in preset.gait_cycle:
            assert GAIT_DESCRIPTORS[gait].leg_set == preset.leg_set


def test_a_rotation_mixing_leg_sets_is_a_load_error(_registry_factory):
    # A prev/next press must never be able to ask a standing robot for legs it
    # is not on.
    bad = _YAML.replace(
        "      gait_cycle: [quad_canter, quad_walk]",
        "      gait_cycle: [quad_canter, tetrapod]",
    )
    with pytest.raises(ValueError):
        _registry_factory(bad)


def test_two_presets_may_share_a_gait(_registry_factory):
    # The rule the preset channel replaced: rotations used to have to be
    # disjoint so a bare gait name could name one preset. It cannot any more —
    # normal and fast both offer tripod — and that is now legal.
    reg = _registry_factory()
    assert "tripod" in reg.get("normal").gait_cycle
    assert "tripod" in reg.get("fast").gait_cycle


def test_a_preset_missing_from_tuning_yaml_is_a_load_error(_registry_factory):
    bad = _YAML.replace("    - id: fast", "    - id: offroad")
    with pytest.raises(ValueError, match="tuning.yaml"):
        _registry_factory(bad)


def test_a_default_gait_outside_its_own_rotation_is_a_load_error(
    _registry_factory
):
    bad = _YAML.replace("      default_gait: quad_canter",
                        "      default_gait: tripod")
    with pytest.raises(ValueError, match="default_gait"):
        _registry_factory(bad)


def test_an_unknown_default_preset_is_a_load_error(_registry_factory):
    with pytest.raises(ValueError, match="default"):
        _registry_factory(_YAML.replace("  default: normal", "  default: sideways"))


def test_unstable_gaits_are_filtered_per_preset(_registry_factory):
    reg = _registry_factory(_YAML.replace("allow_unstable_gaits: true",
                                          "allow_unstable_gaits: false"))
    normal = reg.get("normal")
    assert "surf" not in normal.gait_cycle
    assert "crawl" not in normal.gait_cycle
    assert "tripod" in normal.gait_cycle


def test_entry_gait_starts_at_the_preset_default(registry):
    assert registry.entry_gait("normal") == "tripod"
    assert registry.entry_gait("quad") == "quad_canter"


def test_caps_are_per_preset(registry):
    # Three of the four inputs to a cap ride the preset: the stride it lays
    # down, the swing time it lays it down in, and the stance the angular cap
    # divides by. fast is the longer stride at the quicker cadence.
    normal = registry.caps("normal").linear_max("tripod")
    fast = registry.caps("fast").linear_max("tripod")
    assert fast > normal
    # The ratio is exactly the stride-over-swing-time one; nothing else moves.
    assert math.isclose(fast / normal, (0.125 / 0.45) / (0.105 / 0.6), rel_tol=1e-6)


def test_caps_default_to_the_preset_in_force(registry):
    # Before any /gait/preset the boot preset's table is the honest guess.
    assert registry.caps() is registry.caps("normal")
    registry.note_preset("fast")
    assert registry.caps() is registry.caps("fast")


def test_current_is_empty_until_the_engine_reports(registry):
    # Never seeded from the default: showing a preset the robot may not be on is
    # exactly what the report topic exists to prevent.
    assert registry.current_id() is None
    assert registry.current() is None
    registry.note_preset("quad")
    assert registry.current_id() == "quad"


def test_for_gait_prefers_the_preset_in_force(registry):
    # Ambiguous by design now that rotations overlap; the preset actually in
    # force is the answer that matters.
    assert registry.for_gait("tripod").id == "normal"
    registry.note_preset("fast")
    assert registry.for_gait("tripod").id == "fast"
    assert registry.for_gait("moonwalk") is None


# ─── Projection onto the two JoyConfig rotations ───────────────────

def test_project_writes_the_active_rotations(registry, joy_cfg):
    # The whole trick that keeps map_joy untouched: it still sees one six-leg
    # rotation and one four-corner one, and never learns either can be swapped.
    assert joy_cfg.gait_cycle == registry.get("normal").gait_cycle
    assert joy_cfg.quadruped_gait_cycle == registry.get("quad").gait_cycle
    assert joy_cfg.default_quadruped_gait == "quad_canter"


def test_project_follows_the_six_leg_preset_in_force(registry, joy_cfg):
    # Entering `fast` swaps the one rotation map_joy reads; the mapping's state
    # machine never learns a third preset exists.
    resync_preset("fast", "tripod", joy_cfg, _state(), registry)
    assert registry.project(joy_cfg).gait_cycle == registry.get("fast").gait_cycle


# ─── resync ────────────────────────────────────────────────────────

def _state(**kw) -> JoyState:
    return JoyState(mode=GAIT, **kw)


def test_resync_preset_sets_the_quadruped_flag(registry, joy_cfg):
    # The regression this module exists for. map_joy reads state.quadruped to
    # pick which rotation the D-pad walks and to gate animation mode, and it is
    # the PRESET that moves it now — a gait name cannot, since three of the four
    # presets stand on six legs.
    state = _state()
    assert state.quadruped is False

    result = resync_preset("quad", "quad_walk", joy_cfg, state, registry)
    assert result is not None
    assert state.quadruped is True
    assert result.leg_set_changed is True
    assert result.preset.id == "quad"

    back = resync_preset("normal", "tripod", result.cfg, state, registry)
    assert back is not None
    assert state.quadruped is False
    assert back.leg_set_changed is True


def test_resync_preset_between_two_six_leg_presets_keeps_the_flag(
    registry, joy_cfg
):
    state = _state()
    result = resync_preset("fast", "tripod", joy_cfg, state, registry)
    assert state.quadruped is False
    assert result.leg_set_changed is False
    assert registry.current_id() == "fast"


def test_resync_preset_swaps_the_caps(registry, joy_cfg):
    state = _state()
    result = resync_preset("fast", "tripod", joy_cfg, state, registry)
    assert math.isclose(
        result.cfg.gait_linear_max, registry.caps("fast").linear_max("tripod")
    )
    assert math.isclose(
        result.cfg.gait_angular_z_max,
        registry.caps("fast").angular_max("tripod"),
    )


def test_resync_preset_unknown_id_returns_none(registry, joy_cfg):
    state = _state()
    assert resync_preset("sideways", "tripod", joy_cfg, state, registry) is None
    assert registry.current_id() is None
    assert state.quadruped is False


def test_resync_gait_updates_the_right_cycler_index(registry, joy_cfg):
    state = _state(current_gait_idx=0)
    result = resync_gait("ripple", joy_cfg, state, registry)
    assert state.current_gait_idx == result.cfg.gait_cycle.index("ripple")
    # The six-leg slot is untouched by a four-corner switch: it is still the
    # operator's, waiting for them to come back to it.
    resync_preset("quad", "quad_walk", result.cfg, state, registry)
    quad = resync_gait("quad_walk", result.cfg, state, registry)
    assert state.current_gait_idx == result.cfg.gait_cycle.index("ripple")
    assert state.current_quadruped_gait_idx == (
        quad.cfg.quadruped_gait_cycle.index("quad_walk")
    )


def test_resync_remembers_each_presets_slot(registry, joy_cfg):
    state = _state(current_gait_idx=0)
    cfg = resync_gait("ripple", joy_cfg, state, registry).cfg
    resync_preset("quad", "quad_canter", cfg, state, registry)
    cfg = resync_gait("quad_walk", cfg, state, registry).cfg
    # Selecting NORMAL again should offer the gait it was left on, not the
    # preset's cold default — the round trip lands where the operator was.
    assert registry.entry_gait("normal") == "ripple"
    assert registry.entry_gait("quad") == "quad_walk"


def test_resync_gait_updates_the_caps(registry, joy_cfg):
    state = _state()
    caps = registry.caps()
    result = resync_gait("ripple", joy_cfg, state, registry)
    assert math.isclose(result.cfg.gait_linear_max, caps.linear_max("ripple"))
    assert math.isclose(result.cfg.gait_angular_z_max, caps.angular_max("ripple"))


def test_resync_gait_leaves_the_leg_set_alone(registry, joy_cfg):
    # The split this refactor is about: a gait is a gait, and only the preset
    # moves the robot between the leg sets.
    state = _state()
    result = resync_gait("ripple", joy_cfg, state, registry)
    assert result.leg_set_changed is False
    assert state.quadruped is False


def test_a_preset_request_starts_the_posture_revert(registry, joy_cfg):
    # The engine will not start the change until the body pose is neutral, and
    # map_joy's revert decay is the only thing that puts it there. It has to
    # follow the REQUEST: by the time /gait/preset answers it is far too late.
    state = _state()
    state.recorded_x = 0.02
    assert resync_preset_request("fast", state, registry) is not None
    assert state.reverting is True


def test_a_request_for_the_preset_in_force_does_not_revert(registry, joy_cfg):
    state = _state()
    resync_preset("fast", "tripod", joy_cfg, state, registry)
    state.recorded_x = 0.02
    assert resync_preset_request("fast", state, registry) is None
    assert state.reverting is False


def test_an_unknown_preset_request_does_not_revert(registry):
    state = _state()
    assert resync_preset_request("sideways", state, registry) is None
    assert state.reverting is False


def test_a_gait_switch_does_not_revert(registry, joy_cfg):
    state = _state()
    state.recorded_x = 0.02
    resync_gait("ripple", joy_cfg, state, registry)
    assert state.reverting is False


def test_resync_preset_drops_out_of_animation_mode_on_four_legs(
    registry, joy_cfg
):
    # Every animation is written for six legs, and map_joy only blocks
    # *entering* the mode — arriving there on four has to leave.
    state = JoyState(mode=ANIMATION)
    result = resync_preset("quad", "quad_walk", joy_cfg, state, registry)
    assert result.left_animation_mode is True
    assert state.mode == GAIT


def test_resync_preset_leaves_animation_mode_alone_on_six_legs(
    registry, joy_cfg
):
    state = JoyState(mode=ANIMATION)
    result = resync_preset("fast", "tripod", joy_cfg, state, registry)
    assert result.left_animation_mode is False
    assert state.mode == ANIMATION


def test_resync_gait_unknown_name_returns_none(registry, joy_cfg):
    state = _state(current_gait_idx=1)
    assert resync_gait("moonwalk", joy_cfg, state, registry) is None
    assert state.current_gait_idx == 1
    assert state.quadruped is False


def test_resync_of_an_own_loopback_is_idempotent(registry, joy_cfg):
    # The node hears its own accepted publish back; landing somewhere else
    # would make every switch jump two slots.
    state = _state(current_gait_idx=0)
    first = resync_gait("tetrapod", joy_cfg, state, registry)
    idx = state.current_gait_idx
    resync_gait("tetrapod", first.cfg, state, registry)
    assert state.current_gait_idx == idx


# ─── The publish gate ──────────────────────────────────────────────

def test_preset_switch_allowed_only_where_the_engine_takes_one():
    for state in ("stand", "folded", "fault"):
        assert preset_switch_allowed(state), state
    # Narrower than the gait-switch set on purpose: a plain gait swap latches
    # and applies at the next stand, a preset change re-plants every foot.
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
        assert not preset_switch_allowed(state), state
