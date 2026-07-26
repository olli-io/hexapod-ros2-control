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
home, and the ladder is what puts them back.

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
  stride takes longer to cover.
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

## 5. Cold start

Terminology specific to the cold-start sequence the gait engine runs
once at power-on, bridging the folded shipping pose to the standing
pose:

- **initial pose** — the per-joint angles assumed at startup, before
  any commanded motion. Defined in `hexa_description/config/geometry.yaml`
  under `initial_pose:` and applied to ros2_control's
  `<state_interface name="position">` as `<param name="initial_value">`.
  In sim, gz_ros2_control spawns the model in this pose; on the real
  robot, the hardware plugin uses it as the assumed pose for servos
  that cannot report their own angle (the operator is responsible for
  placing the chassis in roughly this pose at power-on).
- **folded** — pre-INITIALIZE engine state. The engine emits the
  initial-pose foot positions and ignores `cmd_vel`; an operator
  trigger (`/gait/initialize` on rising edge of the joystick start
  button) advances to INITIALIZE.
- **initialize** — engine state covering the cold-start sequence from
  the initial pose to standing.
- **place feet** — INITIALIZE sub-phase: pair-wise foot placement from
  the folded initial-pose footprint to the standing footprint, with
  the foot held `place_feet_clearance` (~1 mm) above the floor at
  touchdown so the swing arc doesn't scuff the ground.
- **lift body** — INITIALIZE sub-phase that follows place feet: ramp
  foot z in the body frame from the place-feet endpoint (1 mm above
  the floor) down to nominal standing z, raising the body to standing
  height as the legs extend and the feet make ground contact.
