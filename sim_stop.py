"""Simulate the engine flow during cmd_vel -> 0 to count leg group movements."""
from pathlib import Path
import sys

sys.path.insert(0, "/workspace/src/hexa_gait")
sys.path.insert(0, "/workspace/src/hexa_kinematics")
sys.path.insert(0, "/workspace/src/hexa_control")

# Need to load YAML configs from share dirs; use raw paths instead
import yaml
from hexa_gait.engine import Engine, EngineConfig, build_leg_contexts, initial_stance_from_yaml, nominal_stance_from_yaml, reseat_geometry_from_yaml
from hexa_gait.gaits import STRATEGIES
from hexa_kinematics.leg_specs import load_leg_specs
from hexa_gait.clock import LEG_NAMES

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

# Skip initialize ladder: just push state forward
import hexa_gait.engine as eng_mod
engine._state = eng_mod.EngineState.STAND
engine._last_targets = dict(engine._nominal)
engine._last_stance = {n: True for n in LEG_NAMES}

# Now mimic gait_node's behavior with body velocity limiter
from hexa_control.body_velocity_limiter import BodyVelocityLimiter
limiter = BodyVelocityLimiter(tau_linear=0.25, tau_angular=0.40)

dt = 0.02
# Drive forward at full speed
target_v = (0.30, 0.0)
target_w = 0.0

t = 0.0
state_log = []
prev_state = engine.state
prev_swing = {n: False for n in LEG_NAMES}
lift_events = []  # (time, state, legs that just lifted)

def step(target_lin, target_ang):
    global t, prev_state, prev_swing, lift_events
    vx, vy, w = limiter.step((target_lin[0], target_lin[1], target_ang), dt)
    out = engine.update(dt=dt, v_body_xy=(vx, vy), omega_z=w)
    if engine.state != prev_state:
        print(f"t={t:.3f}  state: {prev_state.name} -> {engine.state.name}  v_body=({vx:.4f},{vy:.4f}) w={w:.4f}")
        prev_state = engine.state
    cur_swing = {n: not out[n].stance for n in LEG_NAMES}
    just_lifted = [n for n in LEG_NAMES if cur_swing[n] and not prev_swing[n]]
    if just_lifted:
        lift_events.append((t, engine.state.name, tuple(just_lifted)))
    prev_swing = cur_swing
    t += dt
    return vx, vy, w

# Walk forward for 2 seconds
print("--- Walking phase ---")
for _ in range(int(2.0 / dt)):
    step(target_v, target_w)

# Release stick
print("--- Stick released ---")
target_v = (0.0, 0.0)
target_w = 0.0
# Run until STAND
for i in range(int(8.0 / dt)):
    step(target_v, target_w)
    if engine.state == eng_mod.EngineState.STAND and i > 50:
        # Confirm stable for a few ticks
        if all(e[0] != t-dt for e in lift_events[-3:]):
            break

print("\n--- Lift-off events (after t=2.0, when stick was released) ---")
for ev in lift_events:
    if ev[0] >= 2.0:
        print(f"t={ev[0]:.3f}  state={ev[1]:9s}  legs={ev[2]}")
