# Foot tip trajectory: baseline vs Syropod (OpenSHC)

A reference comparison between this repo's gait trajectory generation
(`shared/motion_core/gait/`) and the CSIRO `syropod_highlevel_controller`'s
`walk_controller.cpp` (OpenSHC), captured 2026-08-09. Use it when weighing
whether to adopt an OpenSHC behaviour or to understand why the two stacks
shape a step differently.

## Architectural summary

- **Syropod** — every phase segment is a quartic (4th-order) Bezier with 5
  control nodes. Swing is *two* chained Beziers (primary + secondary, split at
  mid-swing); stance is one. The controller advances the foot by integrating
  the Bezier *derivative* each iteration (`delta_pos = delta_t *
  quarticBezierDot(...)`); position is accumulated state.
- **This repo** — swing is a single closed-form curve (`swing_arc()` in
  `gaits/base.cpp`): a septic-eased horizontal blend between two moving ground
  lines, plus an independently shaped vertical profile and lateral bump.
  Position is a pure function of `(phase, origin, target, profile)` — nothing
  is integrated during swing. Stance during walking is a velocity integrator
  (`StanceIntegrator`), not a curve at all; the evenly-spaced quartic-Bezier
  stance track (`trajectory.cpp`, node-for-node the Syropod formula) survives
  only in the strategies' closed forms and the engagement pass.

The deepest difference is *derivative-integrated state* vs *closed-form
position*. Syropod's foot position drifts by integration error and absorbs
mid-swing target changes implicitly (the derivative just changes); ours hits
the latched endpoints exactly by construction, and absorbs target changes by
re-aiming the touchdown end of the closed form every tick
(`SwingPlanner::retarget`).

## Swing shape

- **Curve family** — Syropod: two quartic Beziers, C1 (velocity) and
  effectively C2 (acceleration) at the seams via equal node spacing. Ours: one
  `ease7` (septic smoothstep) blend whose first *three* derivatives vanish at
  both ends — the foot departs from ground velocity only as O(t^4), matching
  position, velocity, acceleration and jerk against the stance motion at both
  lift-off and touchdown.
- **Ground-velocity continuity** — Syropod encodes the stance exit/entry
  velocity into node separations (`stance_node_seperation = 0.25 *
  swing_origin_tip_velocity_ ...`). Ours blends between two *moving ground
  lines* — where the planted foot would have got to, and where the landing
  foot will have come from — with the live per-leg stance velocities passed in
  at both ends, so the same continuity falls out of the blend itself.
- **Apex placement** — Syropod's apex is a control node (`swing_1_nodes_[4] =
  mid_tip_position`), at the *time* midpoint of the swing. Ours pins the apex
  over the *spatial* midpoint of the travel via a perspective time warp
  (`apex_warp`), and the apex time is derived from the probe share, not
  configured.
- **Clearance** — both lift `max(origin_z, target_z)` plus a clearance. Syropod
  applies clearance along the estimated walk-plane normal; ours is body-frame
  vertical (flat-ground assumption, no walk plane).
- **Lateral shift** — both have one. Syropod adds a signed `mid_lateral_shift`
  to the midpoint node; ours adds a `bump()` profile evaluated on the blend
  (not on a clock), signed by which body side the leg is on.

## Touchdown handling — the biggest behavioural divergence

- **Syropod** — touchdown speed is whatever the secondary Bezier's node
  spacing produces (stride-derived); the descent only *approaches* it. Terrain
  robustness comes from sensing: rough-terrain mode retargets mid-swing from a
  step-plane estimate, clamps the secondary nodes on detected ground contact,
  and `forceNormalTouchdown` reshapes the curve for a vertical approach.
- **This repo** — fully open loop, no contact sensing. Robustness comes from
  the trajectory itself: the descent ends in a straight constant-velocity
  *probe* at exactly `touchdown_velocity`, spanning
  `touchdown_probe_fraction` of the swing. Any early contact inside the probe
  band (`touchdown_velocity x fraction x swing_time` — ~5.6–6.7 mm at current
  tuning: v = 0.028 m/s, f = 0.4, swing 0.5–0.6 s) lands at the intended
  speed. Syropod has no equivalent of the probe; we have no equivalent of
  its touchdown detection.
- **Touchdown ride (added 2026-08-10)** — the horizontal travel can finish
  early so the foot *rides the touchdown ground line* through the probe:
  world-frame stationary over its landing point, so an early contact in the
  ridden part of the band also lands without horizontal slip. The grant is
  metered two ways (`granted_ride_time`): by overshoot past the AEP
  (`ride_headroom`, the stance band's grace, so a parked foot never leaves
  the stance envelope) and by the slip it prevents (proportional to ground
  speed, so slow and rest-to-rest swings keep the un-ridden schedule). This
  combines the halves each stack got separately: Syropod's tail matches
  ground speed horizontally but decays to zero vertically; the pre-ride arc
  nailed the vertical probe but slid horizontally on early contact.

## Stance

- **Syropod** — quartic Bezier with 5 evenly spaced nodes along
  `-stride_vector`, i.e. a constant-velocity straight line, advanced by the
  derivative. Nodes regenerate as the stride changes.
- **This repo** — the walking stance target is an *anchor* integrated at the
  live per-leg velocity each tick (`StanceIntegrator::step`), seeded from the
  swing's latched AEP at touchdown. An `ease_outward` soft bound brakes only
  the outward radial component once the anchor leaves its half-stride band —
  Syropod has no such workspace bound inside stance (it bounds speed globally
  from workspace instead). Our `generate_stance_control_nodes` is a direct
  port of Syropod's and is used by the strategy closed forms, so the two
  stacks agree exactly on the ideal stance track; they differ in that ours
  integrates commanded velocity directly and so follows a turning command
  mid-stance with no node regeneration.

## Stride vector and speed control

- **Stride composition** — both compute `v_leg = v_linear + omega x r` per
  leg. Syropod takes `r` from the *current* tip position (rejection off the z
  axis); ours takes the leg's `nominal_stance` as the lever arm, fixed per
  leg.
- **Speed modulation** — inverted between the stacks. Syropod walks at a
  *configured step frequency* and scales the stride with velocity
  (`stride_vector *= on_ground_ratio / frequency`). Ours fixes
  `stride_length` and derives `cycle_time` so the fastest leg's stride fills
  it (`derive_cycle_time`), clamped to the swing-time band; only past
  `max_swing_time` does the stride shrink instead. Cadence adapts to speed
  here; stride adapts to speed there.
- **AEP** — ours is `nominal + stride/2` (`live_aep`), symmetric about the
  nominal stance; Syropod's default stance is likewise centred on the
  identity tip pose, so the steady-state envelopes match in spirit.

## Phase and timing

- **Syropod** — discrete iteration counting: periods normalised to even
  integer iteration counts, per-leg integer phase advanced each control loop,
  swing/stance iteration counts and `delta_t` precomputed per curve.
- **This repo** — continuous float master phase in `[0, 1)` (`GaitClock`),
  projected through per-leg offsets; `cycle_time` may change every tick and
  the clock just integrates `dt / cycle_time`. `swing_phase_margin` shortens
  the swing window at the touchdown end so handovers overlap — Syropod has no
  analogous margin; overlap there is a property of the chosen gait phase
  parameters.

## Features with no counterpart

- **Syropod-only** — walk-plane estimation, rough-terrain step-plane
  retargeting, touchdown detection, `forceNormalTouchdown`, odometry-frame
  external targets, per-leg workspace generation feeding a global max-speed
  envelope.
- **This-repo-only** — the constant-velocity touchdown probe, the derived
  lift-off velocity (`2 x clearance / climb_time`, largest monotone climb),
  the apex-over-spatial-midpoint warp, the stance excursion band, the
  settle/reseat machinery around the gait (Syropod stops by ramping velocity
  to zero, not by walking feet home to nominal).
