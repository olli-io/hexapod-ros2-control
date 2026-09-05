# Leg phases and gait terminology

Shared vocabulary for `hexa_locomotion` and the control brain in
`shared/motion_core` (gait / kinematics / control / posture).
This is the source for the names that appear in code (state enums,
variables, docstrings, log messages).

## 1. The two phases

Every leg in every gait alternates between two phases:

- **Stance** — foot on the ground, bearing weight. In the body frame
  the foot translates opposite to the body's velocity at the contact
  point (straight back for pure forward walking; along an arc when the
  body yaws). This is what propels the body.
- **Swing** — foot in the air, moving from where it last touched down to
  where it will touch down next.

Synonyms in the wider literature — all mean the same thing. We use the
names on the left everywhere in this codebase:

- **stance** — also called *support*, *retraction*, *power stroke*.
- **swing** — also called *transfer*, *protraction*, *return stroke*,
  *recovery*.

## 2. Transition events

Two events bracket each swing:

- **Lift-off** (a.k.a. take-off) — stance → swing transition.
- **Touchdown** (a.k.a. foot placement) — swing → stance transition.

The body-frame *points* where these events happen have established names
from biology (Cruse's stick-insect work) that are also standard in the
hexapod-robotics literature:

- **PEP** — *Posterior Extreme Position*, where lift-off happens. For
  forward walking, this is the rear-most foot position in the body
  frame.
- **AEP** — *Anterior Extreme Position*, where touchdown happens. For
  forward walking, the front-most.

A full cycle for a single leg is therefore:

```
PEP --[swing]--> AEP --[stance]--> PEP
```

For zero body velocity, AEP and PEP both collapse onto the leg's
**nominal stance position** — the default foot placement when standing
still.

All three are **foot targets**, and a foot target is not a contact
point. The IK reaches to the *centre* of the spherical foot tip (see the
foot link in `hexa_description`'s `hexapod.urdf.xacro`), and a sphere on
a flat floor touches `foot_radius` directly below its centre — at any
tibia angle, which is what makes the offset a constant rather than a
lean-dependent correction. Only the two places that state a **ground
contact height** apply it (`kin::ik_z_for_contact`): the nominal stance,
where `body_height` is belly clearance off the floor, and the fold /
unfold ramp endpoints. Everything else — stride, swing clearance,
reseat, the posture height envelope — is expressed relative to the
nominal stance and inherits the offset rather than re-applying it.

AEP..PEP is also a **bound**, not just a description. A planted foot
tracks the ground by integrating the commanded velocity, which under a
steady walk carries it from AEP to PEP and no further. When the command
turns *under* a planted foot — a stick reversal, say — that integral
would otherwise run on past PEP with nothing to stop it, so the engine
eases it to a halt a short grace band beyond. The trade is that a foot
held at the bound slides against the ground for as long as the command
keeps pushing it outward: reversing faster than one stride costs a few
millimetres of slip, in exchange for a foot that can never be commanded
outside the stance envelope its leg was sized for.

The collapse is also how the robot stops. **Settling** is the gait run
at an exactly zero command: AEP, PEP and the nominal stance are then the
same point, so each foot's next swing carries it home and each planted
foot stops dead. For a tripod that is the whole stop — the walk is its
own re-plant, and it keeps the swing clearance, the touchdown probe and
the gait's own guarantee about how many feet are down at once. Once all
six are home, at most a cycle later, the engine says it is standing.

A gait that swings its legs in smaller groups takes longer over the same
thing, and one whose swings run end to end — a crawl — never has the
instant with all six feet down that standing means, so it could not
finish at all. Those stop differently: no leg that is down is allowed to
start another swing, whatever is already in the air lands home, and the
legs still out are handed to the **reseat** ladder, which places three
mirrored pairs and skips any foot already standing where it is being
sent. The engine picks between the two by which is quicker, so retuning
either the settle or the ladder moves the line on its own.

Releasing the stick before the walk has fully started — during the
engagement ladder, which is one whole cycle long and so is where a short
drive on a slow gait usually ends — goes to the reseat too. The
engagement carries its planted feet at the commanded velocity, so a zero
command only freezes them where they were; it has no way to walk them
home, and the ladder is what puts them back. A command *withdrawn*, that
is. A command merely turned around is not a release, however close to
zero the velocity shaper takes it on the way — see below.

### Reversing

Turning the stick around under a walking robot is not a velocity change like
any other. Every planted foot is somewhere between its AEP and its PEP, and
the instant travel reverses, the ones that landed most recently have no
excursion left in the new direction at all: their anchors pin against the
stance ceiling and stop tracking the ground for the rest of their stance
window. Half the legs drag.

The **reversal ladder** answers it by re-registering the schedule against the
feet rather than re-planting them. Reflected about the swing end, a leg's stance
progress *s* becomes *1 - s*, so the runway it has left in the new direction is
the runway it just consumed in the old one — which is exactly where its foot is
standing. A leg at touchdown maps to lift-off, because the old AEP it stands on
is the new PEP. One reflection — the **mirror** — fixes all six at once, and on
the gaits that carry a wave down the body it turns the wave around with them.

That identity holds only where the foot is where the schedule says it is, and
two things have to be true for that. Nothing may be in the air, since the
reflection reverses swing progress against a latched lift-off end. And the
stride has to be the one the legs have been walking *and* will go on walking.
Stride is pinned at `stride_length` across the whole band from the **knee** —
the speed at which `derive_cycle_time` stops stretching the cycle and starts
shortening the stride instead — up to the velocity cap, and the phase clock is
locked to distance travelled throughout it, so the identity is exact there to
within a millimetre. Below the knee the cycle time saturates, the clock outruns
the travel and the feet bunch toward nominal: 24-29 mm off at 40% of the knee
speed. Reflecting there hands every leg tens of millimetres of runway it does
not have, and it ploughs into its excursion ceiling — inward, under the body,
for a middle leg walking sideways.

So the ladder does not stop the walk before mirroring. It **holds** it at the
knee, waits for the gait's next all-down window, reflects there, and releases.
What is left afterwards is the stopping distance from the knee against the
stance band's grace: the robot is still travelling the old way as the reflection
lands, and no reflection can give back ground the robot has yet to stop
covering.

A reversal that is already below the knee is not held — its feet are canonical
for its own shorter stride, but the walk being asked for is a longer one, so the
reflection would over-credit them just the same, and its excursions are smaller
in proportion anyway. Neither are crawl and surf, whose swings run end to end so
that all six feet are never down at once.

Not held is not the same as not noticed, and the difference matters most before
the walk has started. The velocity shaper slews the commanded velocity as a
vector, so a stick turned around drags the command straight through zero and
leaves it inside the zero tolerance for a tenth of a second on a tripod and four
tenths on a ripple. Read as a released stick that is a stop: in the walk it bleeds
a little speed, and in the engagement it re-plants the robot outright, one whole
reseat ladder and a fresh engagement to resume the other way. So the ladder
latches every reversal it recognises, including the ones it declines to hold, and
the engine asks it rather than the command whether the stick was let go.

The ladder therefore spans the engagement as well as the walk. It holds there —
the engagement re-plans off the live command every tick, so a hold at the knee
simply walks that ladder out at the knee — but it cannot reflect there: the
engagement runs its own master clock, and the handoff into the walk reseeds the
gait clock the reflection acts on. The hold carries across the handoff and the
reflection lands on the other side.

Not at the walk's first all-down window, though, on any gait but a tripod. The
engagement eases its body velocity in under a smoothstep whose window outlasts the
first touchdowns everywhere else, and a foot that landed under a half-open
envelope has covered less ground than its phase has spent — a third of a stride on
a ripple's rear leg, which is exactly the over-credit the whole ladder exists to
avoid. Those legs need one swing under the walk to square up, and the reflection
waits for the last of them. A tripod lands every leg on the window exactly and
waits for nothing.

A turn the ladder declines is absorbed by the engagement instead, which is why
every planted foot there rides the same excursion wall the walk's do: a foot that
has just landed on the old AEP starts the turn already at what has become its PEP,
and unbounded it would walk another whole stride the same way, half as far again
as the leg can reach.

## 3. Cycle-level parameters

Properties of the gait cycle (the synchronized motion of all six legs):

- **Cycle time** — duration of one complete PEP → PEP cycle, in seconds.
- **Phase** — a leg's progress through its own cycle, normalized to
  `0 <= phase < 1`, with `phase = 0` at lift-off (PEP). Swing then
  occupies `[0, swing_end)` and stance `[swing_end, 1)`, where
  `swing_end = (1 - duty_factor) * (1 - swing_phase_margin)`.
- **Swing phase margin** — the share of a gait's nominal swing window
  `(1 - β)` handed back to stance, taken at the touchdown end. It makes a
  leg land slightly before the leg it is handing over to lifts off, so
  every handover has a stretch with all six feet planted instead of a
  knife-edge swap that timing jitter could turn into too few legs on the
  ground. Taking it all at the touchdown end keeps `phase = 0` meaning
  lift-off; splitting it across both ends would give an identical
  stance-count profile, since shifting the window inside the cycle only
  relabels master phase. It costs top speed: stance grows, so the same
  stride takes longer to cover. It is configured **per leg set**, not per
  gait: on six feet the overlap is insurance against jitter, on four it is
  the window the support shift has to carry the body across into the next
  triangle, and it is worth much more of the top speed there
  (`swing_phase_margin` 0.12 vs `quadruped_swing_phase_margin` 0.25).
- **Duty factor** (β) — fraction of the cycle a leg spends in stance.
  Higher β means more legs on the ground at any instant — more stable,
  but slower: the body advances only during stance, and per-leg swing
  rate caps how fast the cycle can run, so faster gaits require lower
  β. For the three standard hexapod gaits, the phase offsets are
  chosen so that exactly 6β legs are in stance at every instant:
  - Tripod: β = 1/2 → 3 stance legs
  - Crawl:  β = 2/3 → 4 stance legs
  - Ripple: β = 5/6 → 5 stance legs
- **Phase offset** — each leg's cycle start relative to a reference leg.
  This is what distinguishes the three gaits — they share the same
  per-leg cycle, but offset the six legs differently.

## 4. Stability

- **Support polygon** — convex hull of the currently-grounded feet,
  projected to the ground plane.
- **Static stability** — the projection of the body's centre of gravity
  onto the ground lies inside the support polygon. A statically-stable
  gait keeps the body upright even if motion halts mid-cycle.
- **Static stability margin** — shortest in-plane distance from the
  CoG projection to the nearest edge of the support polygon. Larger
  margin = more robust to perturbations and to CoG-estimation error.

Crawl (4 legs down) and ripple (5 legs down) are always statically stable
on flat ground. Tripod (3 legs down) is statically stable only when the
three stance legs form a triangle enclosing the CoG projection — which
our standard leg layout achieves, but with a smaller margin than the
other two gaits.

### Quadruped mode

**Quadruped mode** parks the middle pair (`l_middle` / `r_middle`) in the air and
walks the four corner legs one at a time. The vocabulary:

- **leg set** — which legs the strategy walks. A property of the strategy, so
  selecting one of the four-corner gaits (`quad_walk`, `quad_canter`) *is*
  selecting the mode.
- **park** — a leg held clear of the walk. Neither stance nor swing: it carries
  no weight and takes no phase. The parked pose is the **folded** pose
  (`geometry.yaml folded_pose`) — the same angles the robot powers up in. From
  the belly the middle pair is already there and the stand ladder simply skips
  it, and the fold ladder finds it already home. From a stand it is put there
  and taken back by the **fold the pair** / **unfold the pair** moves, the two
  halves of a leg-set change below.
- **choosing the set** — from the belly by the init buttons, or from a stand by
  a leg-set change. Start stands the robot up on six legs, select on four; off
  the belly either one is still a fold. What has changed is that folding is no
  longer the *only* way between the two: a `/cmd_gait` naming a gait of the
  other leg set, sent from a stand, runs the change in place. The engine still
  refuses one from anywhere else — mid-ladder, mid-engagement, walking or
  settling — so nothing downstream has to guard against a middle pair caught in
  the wrong place.

Both gaits are creeps at duty factor 3/4. Lift-offs are a quarter cycle apart
and the swing window is
`0.25 * (1 - quadruped_swing_phase_margin) = 0.1875`, so exactly one foot is ever
airborne and every handover has a 0.0625-cycle window with all four down. They
differ only in footfall order, and because a leg lifts at master phase
`pymod(-offset, 1)` each table runs the mirror of the order it walks:

- **`quad_walk`** — the lateral sequence (left rear, left front, right rear,
  right front), each hind followed by the fore on its own side. Offsets
  `l_rear 0, r_front 1/4, r_rear 1/2, l_front 3/4`. Reading that table as the
  lift order instead gives the diagonal sequence, whose worst static margin is
  negative on this chassis.
- **`quad_canter`** — the perimeter sequence (right front, left front, left
  rear, right rear), round the chassis rather than up one side, so the two fores
  lift back to back. Offsets `r_front 0, r_rear 1/4, l_rear 1/2, l_front 3/4`.

The operator rotates between them on the teleop D-pad, which walks
`quadruped_gait_cycle` while the robot stands on four legs; `select` always
stands up on `default_quadruped_gait`.

**Why the body has to move.** With four feet down the support polygon is a convex
quadrilateral, and lifting one leg leaves one of its four "drop a vertex"
triangles. Those four triangles intersect only where the diagonals cross — a
single point, an intersection with empty interior. So no fixed body position is
inside all four, and a four-legged creep cannot be statically stable at a standing
posture. The body must carry itself into the next triangle before the foot leaves
it. That is the **support shift** animation: a stance-leg centroid weighted by
each leg's time to lift-off, so the target is always a positive-weight convex
combination of the grounded feet and therefore always strictly inside the current
support polygon.

Two things shape how the body actually rides that target. `support_shift_gain`
scales it: below 1 the body stops short of the point and stands nearer the middle
of the four-foot rectangle, which is less body movement and less static margin —
one knob, both effects. And the target is low-passed **in polar**, not per axis:
it walks a rough circle around the body as each handover passes it to the next
triangle, and lagging `x` and `y` independently cuts every one of those turns
into a corner (both axes cross their midpoints together, collapsing the reach to
`1/sqrt(2)` at the crossing). Lagged in polar the radius holds while the angle
sweeps, so the body arcs through the handover — which is what makes a lag slow
enough to be smooth usable at all. `PoseSmoother` eases the commanded body pose
the same way and for the same reason.

The operator's own body pose shares that envelope, and is welcome to: posture
mode is not walking, so the support shift is not spending its travel while a
pose is being held. What the teleop refuses on four feet is the posture
**record** — a recorded pose bleeds through into gait mode, where the same x-y
budget is what carries the body into the next triangle, and the static margin
there is millimetres. Body height is exempt; it never competed for the x-y
budget in the first place.

### Changing the leg set from a stand

Folding to the belly is no longer the only way between the two stands. A
`/cmd_gait` naming a gait of the other leg set, sent while the engine is at
`stand`, runs the change in place. It is refused from every other state, so a
walking robot ignores it rather than stopping for it.

The two footprints are not the same. The corners stand at `tip_reach` 0.135 on
six legs and 0.119 on four, and the rear pair's splay goes from 0° to 25° — 16 mm
of travel at the front and 57 mm at the rear. So the corners have to be
**reseated** onto the new footprint, and the middle pair has to travel the 111 mm
between the ground and the folded pose. The ordering is fixed by which of those
two the body can afford at a time:

- **hexapod → quadruped** — reseat the corners onto the four-corner footprint
  first, with all six feet still down, then **fold the pair** up.
- **quadruped → hexapod** — **unfold the pair** down onto the ground first, then
  reseat the corners outward onto the six-leg footprint.

Either way **the reseat runs with all six legs planted**, and the engine's leg
set stays `hexapod` for the whole change — it flips only once the pair has
arrived. That is not bookkeeping tidiness: the reseat ladder reads its rung order
and its pre-lift shift hold off the leg set, so a hexapod leg set is what gives
both reseats mirrored pairs and no shift hold. It is also what keeps a folded
middle out of the ladder's landing stage, which latches any foot above its target
as one to be put down and would sweep it 111 mm straight down with nothing under
it.

The middle pair's own move needs neither a waypoint nor a clearance. Standing, the
path from a middle's stance to the folded pose is a near-vertical 111 mm line well
outboard of the corner feet, and every joint triple along it is inside its limits
— so it is one eased chord, both legs together. A clearance would be actively
wrong: the folded pose's femur sits *on* its lower joint limit, so any arc that
climbs over that end is unreachable. The unfold ends with the same braked descent
at `touchdown_velocity` every other touchdown gets.

**The operator's posture is reverted first.** A planted foot is solved through the
body pose; a parked one is held rigid at the folded angles, deliberately, so a
body offset cannot drag a leg that is stuck out in the air. A middle leg crossing
between the two is therefore in one frame at one end and the other at the other,
and the mismatch is not small — the folded configuration puts the foot 0.102 m
from the femur joint, so 5 mm of body y is 4.5° of femur and a third of the pose
envelope is outside the joint limits outright. Easing the pose to neutral before
the pair moves makes the two frames the same one. Body height rides the same
revert, so it costs the operator nothing extra.

## 5. Cold start

Terminology specific to the cold-start sequence the gait engine runs
once at power-on, bridging the folded shipping pose to the standing
pose. There are **two** belly-rest poses, and both the way up and the
way down pass through the second one:

- **folded pose** — the per-joint angles the robot energizes into,
  before any commanded motion, and the pose a fold ends on. Defined in
  `hexa_description/config/geometry.yaml` under `folded_pose:` and
  applied to ros2_control's `<state_interface name="position">` as
  `<param name="initial_value">`. In sim, gz_ros2_control spawns the
  model in this pose; on the real robot, the hardware plugin uses it as
  the assumed pose for servos that cannot report their own angle (the
  operator is responsible for placing the chassis in roughly this pose
  at power-on). Tucked in tighter than the initialized pose.
- **initialized pose** — `geometry.yaml`'s `initialized_pose:`, the
  same schema. Still belly-resting, feet still clear of the floor, but
  the legs are deployed: every foot is in the air above its standing
  target. Sits between folded and standing in both directions, for
  mechanical reasons — nothing goes straight between folded and
  standing.
- **folded** — pre-INITIALIZE engine state. The engine emits the
  folded-pose foot positions and ignores `cmd_vel`; an operator
  trigger (`/gait/initialize` on rising edge of the joystick start
  button) advances to INITIALIZE.
- **initialize** — engine state covering the whole cold-start sequence,
  folded pose to standing. Its three rungs are unfold, place feet and
  lift body.
- **unfold** — folded pose to initialized pose: one eased move of all
  six feet at once, straight along the chord between the two poses.
  Both ends are airborne and the belly carries the robot the whole way,
  so there is nothing to sequence, nothing to land, and no arc to climb.
  Runs over `initialize.unfold_time`.
- **place feet** — INITIALIZE sub-phase: pair-wise foot placement from
  the initialized-pose footprint to the standing footprint, parking each
  foot `initialize.place_clearance` above the floor the belly is already
  resting on rather than landing it. No pair takes load while later pairs
  are still swinging, so belly-height error cannot be preloaded into a
  leg that has no way to give it back; all six meet the floor together
  on the lift-body ramp. The swing arc still descends the last stretch at
  the gait's touchdown speed, which is the slack absorbing a floor higher
  than the belly says it is.
- **lift body** — INITIALIZE sub-phase that follows place feet: ramp
  foot z in the body frame from the place-feet endpoint (one clearance
  above the floor) down to nominal standing z, raising the body to
  standing height as the legs extend. The first `place_clearance` of that
  travel is the feet reaching the floor and taking the load. A septic,
  not the fold's quintic: the contact is an interior point of this ramp
  rather than an endpoint, and the septic's flatter opening puts it at a
  smaller share of the ramp's peak speed. Only a smaller share — the
  absolute contact speed is `place_clearance` against
  `initialize.lift_body_time`, not the choice of curve.
- **lower body** — first FOLDING sub-phase, the time-reverse of lift
  body: all six feet hold their standing XY while body-frame z ramps up
  to the belly height, so the belly reaches the floor exactly as the
  ramp runs out of speed.
- **lift feet** — FOLDING sub-phase, the time-reverse of place feet:
  pair-wise lift of the feet off the floor to the initialized pose,
  three mirroring pairs in reverse PAIR_ORDER. No counterpart to place
  feet's clearance — these feet leave from where they were actually
  standing, carrying the robot until the belly takes over, so there is
  nothing to hold clear of.
- **tuck** — the unfold run backwards, initialized pose to folded pose.
  The last rung of a fold.
