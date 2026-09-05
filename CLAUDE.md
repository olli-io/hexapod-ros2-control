# Repository conventions for Claude

Ground rules for AI assistants working in this hexapod ROS2 codebase.

## Stack

- ROS2 Jazzy (Ubuntu 24.04), Gazebo Harmonic, colcon workspace.
- All build/run commands execute **inside the Docker sim container**, driven by the `hexa` host script in the repo root. The sim stack runs as the container's PID 1, so lifecycle is docker-native: `hexa sim up` (bring the stack up detached), `hexa sim down`, `hexa sim logs -f`. Build the workspace with `hexa sim build` (ephemeral `compose run --rm`, which runs `colcon build --symlink-install` in the container), and run one-off commands with `hexa sim <cmd>` (e.g. `hexa sim ros2 topic list`). Do not assume native ROS2 on the host.
- Tests: `./hexa sim python3 -m pytest src/<pkg>/test -q` from the repo root (pytest only exists inside the container).
- ROS2 packages live under `src/hexa_*/`. The top-level `README.md` documents the dependency graph and runtime data flow.
- **Leg count is fixed at 6.** Do not parameterise it.

## Architectural rules

- Locomotion is a **single node**, `hexa_locomotion`, which compiles the shared
  control brain (`shared/motion_core`) and runs the whole velocity → gait →
  posture → compose/IK pipeline in one 200 Hz tick. It is the sole locomotion
  path (the former `hexa_control → hexa_gait → hexa_posture → hexa_kinematics`
  topic-wired node chain has been deleted). `hexa_teleop → /cmd_vel` (+ the
  discrete command topics) is the entry point; `hexa_locomotion` publishes joint
  commands. See the section below for `shared/motion_core`.
- `hexa_interfaces` depends on nothing hexapod-specific (leaf).
- `hexa_description` is the **single source of truth** for URDF, joint limits, and leg geometry. Never duplicate these values elsewhere — load them at runtime.
- `hexa_simulation` owns **all** Gazebo-specific code. The real-robot bringup must not import it.
- The face is a single-node sink (`hexa_display`, ament_cmake C++), launched only by `hexa_bringup`. It subscribes to topics `hexa_locomotion` publishes (`/gait/state`, etc.), maps robot state through an expression/gaze policy, and rasterizes the eyes on a Pi-attached SH1122 OLED (headless in sim) — all in one process, no intermediate `/display/*` topic hop. It is a **pure sink** of robot state; nothing imports it or subscribes to it. Its one non-robot-state input is `/display/text` (`std_msgs/String`, transient_local): a non-empty message swaps the eyes for a text screen (vendored Pixel Operator font + `text_screen.*` in `shared/display_core`), an empty one returns the face. Its other non-robot-state inputs are `/bluetooth/scanning` and `/display/busy` (both `std_msgs/Bool`, transient_local): either one true wears the `SCANNING` expression, the only animated one — two spinners on the neutral eyes' ring, driven by `AnimFrame::phase`. Two topics because they mean different things (a pairing scan; a network-mode switch) and one expression because the face's answer to both is "wait, I am working"; the node ORs them into `PolicyInputs::busy`, which is named after neither. All three come from `hexa_buttons` (below); the dependency is one-way and topic-only. It owns the Linux SH1122 panel/SPI/GPIO driver (`src/hexa_display/src/Sh1122Panel.*`) and the rclcpp glue (`display_node.cpp`, `face_sim.cpp`). The pure expression/gaze policy (`expression_policy`, `face_animation`, `face_animation_runner`) and the vendored firmware eye core (`EyeAnim`/`EyeRaster`) live in `shared/display_core/` — see the shared-core bullet below.
- `hexa_buttons` (ament_python, real robot only) owns the two front-panel GPIO buttons: one puts pack percentage + the web teleop address on the face's panel and, held 3 s, switches the Pi between wifi station and hotspot mode; the other shows connected-controller status and, held 3 s, requests a Bluetooth pairing scan. It is a **producer into** the display, never part of it — it publishes `/display/text`, `/bluetooth/scanning` and `/display/busy` and imports nothing from `hexa_display`, which is what keeps that node a pure sink. Lines are read **event-driven** via gpiozero on an explicitly pinned lgpio pin factory (never gpiozero's default factory search, whose `native` tail reads garbage on a Pi 5's RP1); gpiozero owns debounce and the hold clock, so there is no polling loop here. Its GPIO callbacks only timestamp an event onto a queue that the housekeeping tick drains before advancing the timeout clocks — so the pure state machine is single-threaded and needs no lock. All timing is `time.monotonic()`, never the node clock (a Pi has no RTC; an NTP step would expire a live screen). The **network-mode switch** cannot happen in-process: the container is unprivileged with no D-Bus socket, so it cannot run `nmcli`. It uses a container→host escape hatch — a request word into the bind-mounted log volume (`log/network`), a host `systemd .path` unit running `systemd/network-mode.sh`, and a reply in `log/network.state`. It is now the only such hatch: the buzzer used the same shape until it was given a writable bind mount of the PWM tree and became `hexa_buzzer` (below), but NetworkManager has no equivalent. Never both directions on one file: the host writing the file its own `.path` unit watches would retrigger forever. The host is the authority on the current mode, so the button sends `toggle`, not a target; SSID/password live in the shell script and are reported back, never duplicated in `buttons.yaml`. Enabled by `hexa robot install-network`; inert without it. The Bluetooth scanning utility is **not yet written**: this node owns the pairing *session* (`/bluetooth/scanning`, published here), the utility will own the *execution* and report the controller name on `/bluetooth/status` (`std_msgs/String`, transient_local, empty = none); a non-empty status during a scan ends it. Pure logic (screen state machine, screen strings, address ranking) lives in its own modules and is pytest-covered without hardware.
- `hexa_buzzer` (ament_python, real robot only) owns the passive buzzer on the Pi's hardware PWM (GPIO12), the robot's only audible channel. It is a **pure sink**, like the face: it subscribes to `/buzzer/play` (`std_msgs/String`, transient_local — `up` is published from `hexa_hardware::on_activate`, which can beat the node's subscription matching, and a volatile reader would drop it) and nothing imports it. `hexa_hardware` is the sole publisher (`up`, `fault`, `undervolt`). The container reaches the PWM through a **writable bind mount at `/pwm`** (`docker-compose.buzzer.yaml`, added by `scripts/robot.sh` only when the host has the tree — a bind with a missing source stops the whole stack from starting). Do not reintroduce the old `log/buzzer` spool: it existed only because `/sys` is mounted read-only. Tunes are named RTTTL strings in `config/tunes.yaml`; `config/buzzer.yaml`'s `events:` map couples an event name — the word on `/buzzer/play`, and what `play-tune` takes — to one of them, so no tune is ever named inline in code. The `boot` and `shutdown` events stay **host systemd units** because no container is running that early or that late; they run the *same* player, so `tunes.py`/`catalog.py`/`pwm.py`/`player.py` must stay stdlib-only and rclpy-free — PyYAML included, since a Pi OS Lite host is not guaranteed to have it, which is why `catalog.py` carries its own small YAML-subset reader (`hexa deploy` ships those files plus `config/` to `~/hexa-robot/hexa_buzzer/`, and `test_package_purity.py` enforces the rclpy half). The channel is claimed per tune and released after, so `hexa robot play-tune` keeps working while the stack is up. Failure is never fatal anywhere: no buzzer, no overlay, no mount all mean silence and nothing else.
- The control brain in `shared/motion_core/` is pure C++, importable without `rclcpp` (unit-testable standalone via `shared/motion_core/test/`). ROS glue lives only in `hexa_locomotion`'s node files (e.g. `locomotion_node.cpp`, `pipeline_config_loader.cpp`).
- The face policy in `shared/display_core/` is the target-agnostic **display** analog of `shared/motion_core`: the pure expression/gaze policy (`hexa::display` namespace) plus the vendored eye core (`core/`) and the u8g2 C core (`u8g2/`). It is compiled directly — a link-time source swap, no `#ifdef` — by `hexa_display` (via `hexa_display_support`), the Pi Pico firmware, and the firmware host test (`pi-pico-firmware/test/host`), so the eyes rasterize bit-identically across targets. It depends on nothing hexapod-specific and imports no rclcpp/hardware. The Pico supplies its own panel seam (`Sh1122PanelPico`); `hexa_display` supplies the Linux `Sh1122Panel`.
- Gait strategies are pure functions: `(phase, params) → foot_target`. No state, no I/O, no clocks. The phase clock and per-leg transition state live in the gait engine, not in strategies.
- Posture animations are pure functions: `AnimationContext → BodyPose`. No state outside the animation instance, no I/O, no clocks. The clock and walking-vs-idle state live in the posture stack, not in animations.

## Configurability

- Gait choice, body geometry, leg dimensions, joystick mapping, **preset** stance and stride bundles: load from YAML in `config/`. No magic numbers in node code.
- **Sim-first**: every feature must run against the Gazebo model before any servo code is touched.

## Frames, units, conventions

- REP-103 body frame: right-handed, `+x` forward, `+y` left, `+z` up.
- Linear in m/s, angular in rad/s, angles in radians throughout code. Convert only at UI/teleop boundaries.
- `cmd_vel` (`geometry_msgs/Twist`) is the high-level entry point. Stay plug-compatible with `teleop_twist_*`, `twist_mux`, and Nav2 — do not introduce adapter topics.

## Canonical vocabulary

Use exactly these names in identifiers, log messages, and docstrings — not the literature synonyms:

- **stance** — foot on ground (not *support* as a synonym for a leg's phase, *retraction*, *power stroke*). *support polygon*, *support centroid* and *support shift* are correct — they name the ground the feet enclose, not the phase one leg is in.
- **swing** — foot in air (not *transfer*, *protraction*, *recovery*).
- **lift-off** — stance → swing transition.
- **touchdown** — swing → stance transition.
- **PEP** — Posterior Extreme Position (lift-off point in body frame).
- **AEP** — Anterior Extreme Position (touchdown point in body frame).
- **phase** — float in `[0, 1)`, `phase = 0` at lift-off.
- **duty factor** (β) — fraction of cycle in stance.
- **cycle time** — duration of one full PEP → PEP cycle, in seconds.
- **phase offset** — leg's cycle start relative to a reference leg.
- **posture** — body pose state and the subsystem that controls it. Covers both static positioning (feet grounded, body translates/yaws/tilts) and gait-coupled body animation (sway, lean, bob). Not *body trim*, *body control*, *body animation* as standalone terms.
- **animation** — a pure function from `AnimationContext` to a `BodyPose` offset; one ingredient in the posture stack. Use this word only for the posture animation-stack layers (`shared/motion_core/posture`), never for gait or kinematic motion.
- **pose mode** — `/cmd_vel` is zero, body posture changes while feet stay planted.
- **gait-active** — `/cmd_vel` is non-zero; posture animations run on top of the walking gait.
- **settle** — how the robot stops: the gait keeps running at an exactly zero command, so every AEP collapses onto the leg's nominal stance and the walk re-plants its own feet. Ends when all six are home. Not *pause*, *stop sequence*, *re-plant*.
- **reseat** — the mirrored-pair re-plant ladder. Five callers: a body-height change from a stand, a settle on a gait too slow (or, like crawl, unable) to walk its own legs home, a command withdrawn mid-engagement (the engagement cannot re-plant its own feet at a zero command), and a **preset change** — the corners onto the new footprint, on its own where both presets stand on the same legs, and before the pair folds or after it unfolds where they do not. It carries the feet across a **ground plane**, never between two: the height the feet stand at is not a rung's to change, and a target plane handed to the rungs would be carried a third at a time below the feet, or read as six airborne feet and landed in one move above them. Where a **preset change** moves the body height, that half is the **plane ramp**.
- **plane ramp** — the body's own half of a **preset change**: all six feet planted, every foot eased from the plane it stands on to the new preset's, so the height moves as one body translation. Runs inside `RESEATING`, after the ladder has the feet on the new footprint and before the pair folds — the body crosses between two presets' heights on six feet, not four. Absent when the two presets stand at the same height. Not *height reseat*, *body lift*.
- **preset** — the bundle the operator selects as one thing: a **leg set**, a standing pose, and the `stride_length`, `stride_length_radial`, `min_swing_time`, `max_swing_time` and `step_height` the walk lays down on it. `normal`, `fast`, `offroad` (all six-leg) and `quad` ship. Selected on the latched `/cmd_preset`; the one the engine has *applied* is reported on the latched `/gait/preset`, a report topic like `/gait/state`, never a command. Its physical half is declared in `tuning.yaml`'s `gait_node.presets` list, its operator half — label, gait rotation, entry gait — in the two teleop configs under the same ids, and the two halves never restate each other. Orthogonal to the **gait**: several presets can offer the same rotation, so a gait name never identifies one. Not *mode*, *profile*, *stance set*.
- **leg set** — which legs the robot stands on: `hexapod` (all six) or `quadruped` (the four corners, middle pair parked). A property of the **preset**, which declares it, and of each **strategy**, which must match the preset in force — a `/cmd_gait` naming a gait of the other set is refused, never read as a request to change it. The set the engine has actually *applied* is reported on the latched `/gait/leg_set`, a report topic like `/gait/state`, never a command; it is the coarser half of the pair with `/gait/preset`, and cannot tell two six-leg presets apart. Not *leg subset*, *active legs*.
- **park** — the middle pair held at the **folded** pose while the corners walk. Neither stance nor swing; a parked leg is emitted with `parked = true` and its `stance` flag is meaningless. It reaches that pose two ways: from the belly the pair powers up folded and the stand ladder simply skips it (the fold ladder then finds it already home), or from a stand the **fold-the-pair** move puts it there and the **unfold-the-pair** move takes it back down — the two halves of a **leg-set change**. Not *tuck*, *stow*. **fold** on its own still means the whole-robot belly rest; *fold the pair* / *unfold the pair* are the only compounds that name this move, and they are exact, because the parked pose is literally the folded pose.
- **quadruped mode** — the operator-facing shape of the `quad` preset: quadruped leg set + one of the four-corner gaits (`quad_walk`, `quad_canter`) + the `support_shift` animation. From the belly the leg set is chosen by the init buttons: **start** stands the robot up on six legs, **select** on four, and off the belly either button folds it — that never changes. The init edge carries the leg set itself, not a gait: **start** resolves to the last six-leg preset applied (so an operator who was on `offroad` gets up on `offroad`) and **select** to the four-corner one. Between the two stands the ladder is the same one, minus the middle pair's rungs. Off the belly the set is changed by a **preset change** instead. The **gait** is still the operator's to change once standing: the teleop D-pad walks `quadruped_gait_cycle` there instead of `gait_cycle`, and every gait in a preset's rotation walks that preset's legs, so a press can never ask for the other stand. Animation mode is unavailable while quadruped (every animation is written for six legs); gait and posture stay available. Posture mode poses the body on four feet as it does on six — only the posture **record** is inert there, so no offset bleeds into gait mode and spends the x-y envelope the support shift needs.
- **preset change** — moving between two **presets** from a stand, without folding to the belly. Always a **reseat** of the walking feet onto the new preset's footprint with all six legs planted, plus a **plane ramp** behind it where the two presets stand at different body heights: the footprint and the height are separate moves, in that order. Requested on `/cmd_preset`, from a stand only, and refused while walking. The operator's held or recorded posture is reverted first — a body pose is applied to a planted leg and not to a parked one, so the pair may only cross between the two at a neutral pose, and even on one leg set the reseat wants a neutral pose to re-plant against. Not *mode switch*, *transition*.
- **leg-set change** — the preset change whose two presets differ in **leg set**, and the only one that moves the middle pair. It carries a **fold the pair** or **unfold the pair** on the other side of the reseat, ordered so the reseat always runs on six feet: unfold-then-reseat toward hexapod, reseat-then-fold toward quadruped. A preset change within one leg set — `normal` → `offroad`, say — is the reseat and its **plane ramp**, and the pair never moves.
- **support shift** — the posture animation that carries the body into the next support triangle before the foot leaves it. Not *CoM shift*, *weight transfer*.
- **reversal ladder** — how the robot turns around: the walk — or the engagement it is still climbing — is held at the knee until the gait has all six feet down *and* every one of them stands where its phase says, the phase circle is mirrored there, and the command released. A reversal the ladder declines to hold is still recognised as one, and is never read as a released stick. Not *flip*, *turnaround*.
- **mirror** — the phase circle reflected about the swing end, so every stance leg's progress `s` becomes `1 - s` and its remaining runway matches where its foot actually stands. Only exact with all six planted *and* at a stride the legs both have been and will be walking. Not *phase flip*, *phase reverse*.
- **knee** — the leg speed at which `derive_cycle_time` stops stretching the cycle and starts shortening the stride: `stride_length · swing_end / (max_swing_time · (1 - swing_end))`. Above it the phase clock is locked to distance travelled and a foot sits exactly where its stance progress says; below it the clock outruns the travel and the feet bunch toward nominal. Not *saturation point*.

Full definitions in `docs/leg-phases.md`. Do not introduce new synonyms.

## Consolidated single-node locomotion (`hexa_locomotion` + `shared/motion_core`)

`hexa_locomotion` is the locomotion controller: one node running the whole
velocity → gait → posture → compose/IK pipeline in a single 200 Hz loop, mirroring
the Pi Pico firmware's single loop. It replaced the old multi-node chain
(`hexa_control` / `hexa_gait` / `hexa_posture` / `hexa_kinematics`, all now
deleted, along with their `*_cpp` ports).

- **`shared/motion_core/`** is the target-agnostic **float** control brain
  (`hexa::pipeline::Pipeline` + `hexa::gait`/`hexa::posture`/`hexa::control`/
  `hexa::supervisor`), extracted from `pi-pico-firmware/` (which now keeps only its
  Pico hardware seams). It is compiled directly — a link-time seam swap, no
  `#ifdef` — by the Pico firmware, `hexa_pico_bridge`, and `hexa_locomotion`. One
  brain, shared bug-for-bug across firmware and sim. Its host test harness lives in
  `shared/motion_core/test/`. It is the **sole** locomotion implementation; the
  earlier double-precision `hexa_gait_cpp`/`hexa_posture_cpp`/`hexa_kinematics_cpp`
  libraries and the Python originals have been removed.
- **Seams** (caller-owned, the only things that differ per target): the Pipeline
  `tick` has a joy overload (Pico/bridge run `map_joy`) and a core
  `tick(CommandIntent, TickInput)` (the ROS node builds `CommandIntent` from
  `/cmd_vel` + `/cmd_gait` + `/cmd_preset` + `/gait/initialize` + `/body/pose` +
  `/animation/mode`, bypassing `map_joy`); config comes from a `PipelineConfig`
  (baked constexpr on the Pico, loaded from `geometry.yaml` + `tuning.yaml` at
  startup in `hexa_locomotion` via `pipeline_config_loader.cpp`, scope =
  geometry+tuning only). `map_joy` never sets `preset_select` — the pad has no
  preset control, and the golden trace it is parity-locked to would move if it
  did — so the Pico boots on the baked default preset and stays there; its init
  buttons still reach both stands, because those carry a **leg set** rather than
  a preset. The runtime loader is parity-tested field-by-field against
  `PipelineConfig::baked()` (`hexa_locomotion/test/test_config_loader.cpp`) so a
  YAML edit can never silently drift from the baked codegen.
- **`hexa_locomotion`** folds `ik_node` + `joint_command_bridge` (publishes
  `Float64MultiArray` on `/joint_group_position_controller/commands` directly) and
  re-publishes `/gait/state` for the face sink, plus `/gait/leg_set` and
  `/gait/preset` — the engine's own reports of what it has applied. Because it composes both the
  velocity/gait and body-pose halves of the pipeline **in one process**, the node
  is the in-code composition point (there is no longer a separate chain for
  `hexa_bringup` to wire via launch). `/cmd_vel` stays the entry point
  (Nav2/twist_mux compatible); the runtime-YAML load honors the
  load-config-at-runtime rule.

## Documentation formatting

- **No markdown tables in `.md` files.** Anywhere — package READMEs, `/docs/`, top-level README.
- Use bullet lists with `**term** — definition` pairs instead. Example:

      - **stance** — also called *support*, *retraction*.
      - **swing** — also called *transfer*, *protraction*.

- Broader principle: prefer documentation formats equally readable to humans and AI agents (bulleted lists, definition pairs). Keep comments succinct.
