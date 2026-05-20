"""Trace disengagement queue and state transitions with control_node-like limiter reset."""
from pathlib import Path
import sys

sys.path.insert(0, "/workspace/src/hexa_gait")
sys.path.insert(0, "/workspace/src/hexa_kinematics")
sys.path.insert(0, "/workspace/src/hexa_control")

import yaml
from hexa_gait.engine import Engine, EngineConfig, build_leg_contexts, initial_stance_from_yaml, nominal_stance_from_yaml, reseat_geometry_from_yaml
from hexa_gait.gaits import STRATEGIES
from hexa_kinematics.leg_specs import load_leg_specs
from hexa_gait.clock import LEG_NAMES
import hexa_gait.engine as eng_mod
from hexa_control.body_velocity_limiter import BodyVelocityLimiter

gait_yaml = Path("/workspace/src/hexa_gait/config/gait.yaml")
geom_yaml = Path("/workspace/src/hexa_description/config/geometry.yaml")
stand_yaml = Path("/workspace/src/hexa_description/config/standing_pose.yaml")

with gait_yaml.open() as f:
    raw = yaml.safe_load(f)
init_cfg = raw["initialize"]
reseat_cfg = raw["reseat"]
cfg = EngineConfig(
    stride_length=float(raw["stride_length"]),
    min_swing_time=float(raw["min_swing_time"]),
    max_cycle_time=float(raw["max_cycle_time"]),
    step_height=float(raw["step_height"]),
    swing_width=float(raw["swing_width"]),
    controller_dt=float(raw["controller_dt"]),
    cmd_zero_tol=float(raw["cmd_zero_tol"]),
    forced_touchdown_delay=float(raw["forced_touchdown_delay"]),
    max_foot_speed=float(raw["max_foot_speed"]),
    max_swing_time=float(raw["max_swing_time"]),
    init_pair_swing_time=float(init_cfg["pair_swing_time"]),
    init_lift_body_time=float(init_cfg["lift_body_time"]),
    init_swing_clearance=float(init_cfg["swing_clearance"]),
    init_place_feet_clearance=float(init_cfg["place_feet_clearance"]),
    reseat_settle_delay=float(reseat_cfg["settle_delay"]),
    reseat_height_change_threshold=float(reseat_cfg["height_change_threshold"]),
    reseat_pair_swing_time=float(reseat_cfg["pair_swing_time"]),
    reseat_swing_clearance=float(reseat_cfg["swing_clearance"]),
)
nominal = nominal_stance_from_yaml(geom_yaml, stand_yaml)
initial = initial_stance_from_yaml(geom_yaml)
with geom_yaml.open() as f:
    coxa_to_bottom = float(yaml.safe_load(f)["body"]["coxa_to_bottom"])
leg_contexts = build_leg_contexts(geom_yaml, stand_yaml)
leg_specs = load_leg_specs(geom_yaml)
reseat_geom = reseat_geometry_from_yaml(geom_yaml, stand_yaml)

engine = Engine(
    config=cfg,
    strategy=STRATEGIES["tripod"](),
    nominal_stance=nominal,
    initial_stance=initial,
    coxa_to_bottom=coxa_to_bottom,
    leg_contexts=leg_contexts,
    leg_specs=leg_specs,
    reseat_geometry=reseat_geom,
)
engine._state = eng_mod.EngineState.STAND
engine._last_targets = dict(engine._nominal)
engine._last_stance = {n: True for n in LEG_NAMES}

# Hook disengagement.begin to log queue
disengagement = engine._disengagement
orig_begin = disengagement.begin
def logged_begin(last_targets, swing_flags, phase_offsets, duty_factor, master_phase):
    print(f"  >>> disengagement.begin() master_phase={master_phase:.3f}")
    print(f"      swing_flags={[n for n in LEG_NAMES if swing_flags.get(n, False)]}")
    print(f"      stance_legs={[n for n in LEG_NAMES if not swing_flags.get(n, False)]}")
    orig_begin(last_targets, swing_flags, phase_offsets, duty_factor, master_phase)
    print(f"      queue length: {len(disengagement._queue)}")
    for i, grp in enumerate(disengagement._queue):
        print(f"        group {i}: {sorted(grp)}")
disengagement.begin = logged_begin

limiter = BodyVelocityLimiter(tau_linear=0.25, tau_angular=0.40)
dt = 0.02

t = 0.0
prev_state = engine.state
WALKING_STATES = {eng_mod.EngineState.ENGAGING, eng_mod.EngineState.GAIT}

# Walk for 5s, then release
target_v = (0.30, 0.0); target_w = 0.0
release_t = 5.0
for _ in range(int(12.0 / dt)):
    if t >= release_t:
        target_v = (0.0, 0.0); target_w = 0.0
    vx, vy, w = limiter.step((target_v[0], target_v[1], target_w), dt)
    out = engine.update(dt=dt, v_body_xy=(vx, vy), omega_z=w)
    # Mimic control_node's limiter reset on state edge
    if engine.state != prev_state:
        print(f"t={t:.3f}  STATE {prev_state.name} -> {engine.state.name}  /gait/params=({vx:.6f},{vy:.6f},{w:.6f})")
        if prev_state in WALKING_STATES and engine.state not in WALKING_STATES:
            limiter.reset((0.0, 0.0, 0.0))
            print(f"          -> limiter reset (walking edge)")
        prev_state = engine.state
    t += dt

