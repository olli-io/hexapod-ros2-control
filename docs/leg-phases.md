# Leg phases and gait terminology

Canonical vocabulary for `hexa_locomotion` and `shared/motion_core`. Use these
names in code, logs and docstrings.

## 1. Phases and events

- **stance** — foot on the ground, bearing weight. In the body frame it moves
  opposite to the body velocity at the contact point. Not *support*,
  *retraction*, *power stroke*.
- **swing** — foot in the air, moving to its next touchdown. Not *transfer*,
  *protraction*, *return stroke*, *recovery*.
- **lift-off** — stance → swing.
- **touchdown** — swing → stance.
- **PEP** — Posterior Extreme Position, where lift-off happens (rear-most for
  forward walking).
- **AEP** — Anterior Extreme Position, where touchdown happens (front-most for
  forward walking).
- **nominal stance** — default foot placement at standing. At zero velocity, AEP
  and PEP collapse onto it.

```
PEP --[swing]--> AEP --[stance]--> PEP
```

- **foot target** — PEP, AEP and nominal stance are foot targets, not contact
  points. IK reaches the centre of the spherical foot tip; contact is
  `foot_radius` below it. Only places that state a ground contact height apply
  the offset (`kin::ik_z_for_contact`): nominal stance and the fold / unfold
  ramp endpoints. All else is relative to nominal stance and inherits it.
- **stance bound** — AEP..PEP also bounds a planted foot. When the command turns
  under it, the foot eases to a halt a short grace band past PEP and slides
  there. Reversing faster than one stride costs a few millimetres of slip.

## 2. Settle

- **settle** — the stop: the gait run at exactly zero command. Each swing carries
  its foot home; each planted foot stops. For a tripod this is the whole stop.
- Gaits with smaller swing groups settle slower, and crawl (end-to-end swings)
  never has all six down. Alternative stop: no planted leg starts a new swing,
  airborne legs land home, and the remaining legs go to the **reseat** ladder
  (three mirrored pairs, skipping feet already home). The engine picks whichever
  is quicker.
- A release during the engagement ladder also goes to the reseat: the
  engagement can only freeze feet at zero command, not walk them home. A
  reversed command is not a release (see below).

## 3. Reversal

- **reversal ladder** — handles a stick turned around under a walking robot.
  Without it, recently landed feet have no runway in the new direction and drag.
- **mirror** — reflect each leg's stance progress *s* to *1 - s* about the swing
  end. The old AEP becomes the new PEP, so all six legs re-register at once.
  Requires all feet down and an unchanged stride.
- **knee** — the speed where `derive_cycle_time` stops stretching the cycle and
  starts shortening the stride. From the knee to the velocity cap, stride is
  pinned at `stride_length` and the mirror is exact. Below it, feet bunch toward
  nominal and the mirror over-credits runway.
- **Sequence** — hold the command at the knee, wait for the next all-down
  window, mirror, release.
- **crossing** — the shaped command's pass through zero after the mirror (about
  0.5 s). The gait clock waits it out, so the set mapped to lift-off leaves only
  once the body moves the new way.
- **Not held** — reversals already below the knee, and crawl / surf (never all
  down). The ladder still latches them, so the engine does not read the brief
  pass through zero as a release.
- **During engagement** — the ladder holds but cannot mirror there; the hold
  carries across the handoff into the walk. On gaits other than tripod, the
  mirror waits one swing under the walk, because engagement touchdowns
  under-travel their phase. Declined reversals are absorbed by the engagement,
  whose feet ride the same stance bound.

## 4. Cycle parameters

- **cycle time** — duration of one PEP → PEP cycle, in seconds.
- **phase** — `0 <= phase < 1`, 0 at lift-off. Swing is `[0, swing_end)`,
  stance `[swing_end, 1)`, with
  `swing_end = (1 - duty_factor) * (1 - swing_phase_margin)`.
- **swing phase margin** — share of the nominal swing window given back to
  stance at the touchdown end. Gives every handover an all-down overlap. Costs
  top speed. Set per preset as `swing_phase_margin`: 0.12 on six legs (jitter
  insurance), 0.30 on the quadruped preset (window for the support shift).
- **duty factor** (β) — fraction of the cycle in stance. Higher β is more stable
  but slower. Tripod 1/2 (3 down), crawl 2/3 (4 down), ripple 5/6 (5 down).
- **phase offset** — a leg's cycle start relative to the reference leg. This is
  what distinguishes the gaits.

## 5. Foot trajectory

All in `shared/motion_core/gait/`.

- **Stride composition** — per-leg velocity `v_leg = v_linear + omega x r`, with
  `r` the leg's `nominal_stance` (fixed lever arm).
- **Speed control** — `stride_length` is fixed; `derive_cycle_time` sets the
  cycle so the fastest leg's stride fills it, clamped to the swing-time band.
  Only past `max_swing_time` does the stride shrink. Cadence adapts to speed,
  not stride.
- **AEP** — `nominal + stride/2` (`live_aep`), symmetric about nominal stance.
- **Clock** — `GaitClock` integrates `dt / cycle_time` into a continuous master
  phase; per-leg offsets project it. `cycle_time` may change every tick.
- **Swing** — `swing_arc()` (`gaits/base.cpp`), one closed-form curve. Position
  is a pure function of `(phase, origin, target, profile)`; nothing integrates
  during swing, so it hits its latched endpoints exactly.
  - Horizontal: `ease7` blend between two moving ground lines (where the planted
    foot would be, where the landing foot will come from), fed the live stance
    velocities. Position, velocity, acceleration and jerk match stance at both
    ends.
  - Vertical: lifts `max(origin_z, target_z)` plus clearance, body-frame
    vertical (flat ground, no walk plane). Lift-off velocity is derived:
    `2 x clearance / climb_time`, the largest monotone climb.
  - Apex: over the spatial midpoint of the travel via a time warp (`apex_warp`).
    Apex time follows from the probe share, not config.
  - Lateral: `bump()` on the blend (not the clock), signed by body side.
  - Target changes mid-swing re-aim the touchdown end every tick
    (`SwingPlanner::retarget`).
- **Touchdown probe** — open loop, no contact sensing. The descent ends in a
  straight constant-velocity probe at `touchdown_velocity`, over
  `touchdown_probe_fraction` of the swing. Early contact within
  `touchdown_velocity x fraction x swing_time` lands at the intended speed.
- **Touchdown ride** — horizontal travel can finish early so the foot rides the
  touchdown ground line through the probe: world-stationary over its landing
  point, so early contact also lands without slip. `granted_ride_time` meters it
  by overshoot past AEP (`ride_headroom`, the stance grace band) and by the slip
  it prevents (scales with ground speed; slow swings keep the un-ridden
  schedule).
- **Stance** — an anchor integrated at the live per-leg velocity each tick
  (`StanceIntegrator::step`), seeded from the swing's latched AEP. Follows a
  turning command mid-stance. `ease_outward` brakes only the outward radial
  component past the half-stride band (the stance bound, section 1).
- **Ideal stance track** — `generate_stance_control_nodes` (`trajectory.cpp`):
  quartic Bezier, five evenly spaced nodes, i.e. a constant-velocity line. Used
  only by the strategy closed forms and the engagement.

## 6. Stability

- **support polygon** — convex hull of the grounded feet in the ground plane.
- **static stability** — CoG projection inside the support polygon.
- **static stability margin** — distance from the CoG projection to the nearest
  polygon edge.
- Crawl and ripple are always statically stable on flat ground. Tripod is too on
  this layout, with a smaller margin.

## 7. Presets and leg sets

- **preset** — leg set + standing pose + stride and swing times, selected as one
  on `/cmd_preset`. `normal`, `fast`, `offroad`, `quad` ship. The applied one is
  reported on `/gait/preset`.
- **leg set** — `hexapod` or `quadruped`, declared by the preset. A `/cmd_gait`
  naming a gait of the other leg set is refused.
- **park** — middle pair held at the folded pose (`geometry.yaml folded_pose`):
  no weight, no phase. **fold the pair** / **unfold the pair** move it there and
  back.
- **choosing the set** — from the belly: start stands on six legs (last six-leg
  preset), select on four. From a stand: a **preset change**. Refused from any
  other state (mid-ladder, engagement, walking, settling).

### Quadruped gaits

Both are creeps at β = 3/4: lift-offs a quarter cycle apart, swing window
0.1875, so one foot airborne and a 0.0625-cycle all-down handover. A leg lifts
at master phase `pymod(-offset, 1)`, so each offset table runs the mirror of its
walking order.

- **`quad_walk`** — lateral sequence (LR, LF, RR, RF). Offsets `l_rear 0,
  r_front 1/4, r_rear 1/2, l_front 3/4`. The diagonal sequence has negative
  worst-case margin on this chassis.
- **`quad_canter`** — perimeter sequence (RF, LF, LR, RR). Offsets `r_front 0,
  r_rear 1/4, l_rear 1/2, l_front 3/4`.
- D-pad cycles `quadruped_gait_cycle`; select stands on `default_quadruped_gait`.

### Support shift

- **Why** — the four "drop one vertex" triangles of a quadrilateral meet in one
  point only, so no fixed body position is stable for a four-leg creep. The body
  must move into the next triangle before lift-off.
- **support shift** — posture animation targeting the stance-leg centroid,
  weighted by each leg's time to lift-off. Always strictly inside the support
  polygon.
- `support_shift_gain` < 1 trades margin for less body motion.
- The target is filtered by a critically damped spring in polar, not per
  axis, so the body arcs through handovers instead of cutting corners, and
  leaves each touchdown's target jump from rest. `PoseSmoother` does the same.
  `support_shift_tau` keeps the mean lag of a first-order filter
  (omega_n = 2/tau).
- On four feet, teleop refuses the posture **record** (it would eat the x-y
  budget in gait-active). Live pose mode and body height are allowed.

### Preset change

- Every preset change reseats the feet onto the new footprint. Same leg set
  (e.g. `normal` → `offroad`): reseat + plane ramp only.
- **plane ramp** — after the reseat, all six planted feet ease to the new
  `body_height`. The reseat cannot carry height: it would step the body down
  pair by pair, or drop all six at once. Both halves run in `RESEATING`.
- **leg-set change** — order is fixed:
  - **hexapod → quadruped** — reseat corners to the four-corner footprint with
    six down, then fold the pair.
  - **quadruped → hexapod** — unfold the pair to the ground, then reseat corners
    outward.
- The engine leg set stays `hexapod` until the pair arrives. This gives the
  reseat mirrored pairs, no shift hold, and keeps the folded middle out of the
  landing stage.
- The pair moves on one eased chord (111 mm, near-vertical, inside limits). No
  clearance arc: the folded femur sits on its lower limit. The unfold ends with
  the braked descent at `touchdown_velocity`.
- Operator posture reverts to neutral first. Planted feet solve through the body
  pose, parked feet do not, so a non-neutral pose would mismatch the two ends.

## 8. Cold start and fold

- **folded pose** — `geometry.yaml folded_pose`. Power-on pose and end of a fold.
  Sim spawns in it; on hardware it is the assumed pose for servos without
  position feedback.
- **initialized pose** — `geometry.yaml initialized_pose`. Belly-resting, legs
  deployed, feet above their standing targets. Both directions pass through it.
- **folded** (state) — engine emits the folded pose, ignores `cmd_vel`.
  `/gait/initialize` rising edge advances to INITIALIZE.
- **initialize** (state) — folded to standing, in three rungs:
  - **unfold** — folded → initialized, one eased chord, all six together
    (`initialize.unfold_time`).
  - **place feet** — pair-wise to the standing footprint, parked
    `initialize.place_clearance` above the floor. No pair takes load early.
  - **lift body** — septic ramp from place-feet z to standing z. The feet meet
    the floor during the first `place_clearance` of travel.
- **folding** (state) — standing to folded:
  - **lower body** — time-reverse of lift body.
  - **lift feet** — time-reverse of place feet, reverse `PAIR_ORDER`, no
    clearance.
  - **tuck** — unfold in reverse, initialized → folded.
