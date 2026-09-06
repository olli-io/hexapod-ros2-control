import { useCallback } from "react";
import { createFileRoute } from "@tanstack/react-router";
import { MODES, modeLocked } from "../components/ModeStack";
import Spinner from "../components/Spinner";
import { useModal } from "../providers/ModalProvider";
import { animationAvailable, useTeleop } from "../providers/TeleopProvider";
import { animationLabel, buzz, gaitLabel } from "../utils/labels";
import type { ActionName } from "../types/protocol";

// How long a tap here holds the function down before letting go. This view has
// no keepalive — it is not the one a thumb sits on — so a press sent from it
// would otherwise stay in the node's action set until the input watchdog
// cleared it. Long enough that the 60 Hz tick cannot miss the edge; short
// enough that the window is no press at all.
const TAP_PRESS_MS = 150;

// The two whole-robot moves the belly button starts, and what the modal calls
// each while it runs. Both take a few seconds of ladder that no press can
// shorten, and the engine refuses a second one anyway — the same situation a
// preset change puts the view in, so it gets the same answer.
const TRANSITION_LABELS: Record<string, string> = {
  initialize: "Standing up",
  folding: "Folding down",
};

export const Route = createFileRoute("/preset")({ component: PresetRoute });

function PresetRoute() {
  const { state, send } = useTeleop();
  const Modal = useModal();

  const folded = state.gaitState === "folded";
  // The one engine state a preset change runs from. Narrower than what the node
  // accepts ("folded" and "fault" latch one for the next stand): a tile that
  // takes a press while the robot is on its belly, engaging or walking says the
  // move happened, and every one of those presses either does nothing visible
  // for a while or is refused outright.
  const standing = state.gaitState === "stand";
  const pending = state.pendingPreset;
  // The label of the preset in flight, for the modal below. Falls back to the
  // id, which is what a preset the descriptor list does not carry would be
  // called anywhere else on this view.
  const pendingLabel =
    state.presets.find((p) => p.id === pending)?.label ?? pending;
  // Non-null only while the robot is between the belly and a stand.
  const transition = TRANSITION_LABELS[state.gaitState] ?? null;
  const animationAllowed = animationAvailable(state);
  const animating = state.mode === "animation";

  // Every gait any preset declares, in declaration order, deduped. The row is
  // the whole vocabulary rather than the preset's own slice of it, so it never
  // changes shape: a tile stays where it is across a preset change, and a gait
  // the robot cannot walk right now is dimmed in place instead of vanishing —
  // which also says where it lives, since picking the preset that offers it is
  // the thing to do next.
  const allGaits = [...new Set(state.presets.flatMap((p) => p.gaits ?? []))];
  // The slice the preset in force actually offers. Empty before the first
  // /gait/preset report, which dims the lot — honest, with no preset in force.
  const offered = new Set(
    state.presets.find((p) => p.id === state.activePreset)?.gaits ?? [],
  );

  // Animation mode lives on one preset, named by the config. While the mode is
  // in force the others are inert — the other half of the same rule the ANIM
  // button enforces on the way in, so the two can never be true at once: the
  // way to another preset is to leave the mode first.
  const presetLocked = (id: string) =>
    animating && state.animationPreset !== null && id !== state.animationPreset;

  // Every button on this view is a tap, never a hold: press, then let go a
  // moment later. The state machine reads all of these on the rising edge.
  const tap = useCallback(
    (action: ActionName) => {
      send({ type: "action", action, pressed: true });
      window.setTimeout(
        () => send({ type: "action", action, pressed: false }),
        TAP_PRESS_MS,
      );
    },
    [send],
  );

  const onStand = useCallback(() => tap("init"), [tap]);

  return (
    <div id="preset-view">
      {/* Two stacks, each a column of boxes: what the robot stands as (MODE
          over PRESET) and what it walks (GAIT over ANIM). One under the other
          in portrait, side by side in landscape — where the screen is short and
          a single stack of four boxes does not fit one. A box is sized by what
          it holds either way, so the one-row ANIM box stays a one-row box. */}
      <div id="preset-stacks">
        <div className="preset-column">
          {/* The same three modes the Control view's column offers, from the
              same table, so the two can never disagree about which functions
              exist. Here because the animation row below is inert outside
              animation mode, and the way into that mode was on the other view.
              ANIM is locked wherever the robot cannot be in it — on four legs,
              and on a six-leg preset the animations are not written for — and
              the way out of that is the preset grid right underneath. */}
          <section className="preset-section" data-section="mode">
            <h2 className="preset-section-title">MODE</h2>
            <div id="preset-modes">
              {MODES.map((slot) => (
                <button
                  key={slot.action}
                  className={[
                    "preset-mode",
                    slot.mode === state.mode && "active",
                  ]
                    .filter(Boolean)
                    .join(" ")}
                  data-mode={slot.mode}
                  disabled={
                    pending !== null || modeLocked(slot.mode, animationAllowed)
                  }
                  onClick={() => {
                    buzz(15);
                    tap(slot.action);
                  }}
                >
                  {slot.label}
                </button>
              ))}

              {/* One button for both halves of the same press: `init` stands
                  the robot from the belly and folds it from a stand, so a
                  second button would be a second name for one action — and the
                  label is read off /gait/state, never off what was last
                  pressed, so it cannot claim a stand the robot did not make. In
                  the mode box, on its own row across all three slots: it is the
                  press the modes above are worth nothing without, and on the
                  belly — where the preset box below is dimmed — it is the
                  view's one live action. */}
              <button
                id="preset-stand"
                className={folded ? "folded" : undefined}
                disabled={pending !== null}
                onClick={() => {
                  buzz(15);
                  onStand();
                }}
              >
                <span className="preset-label">
                  {folded ? "STAND" : "FOLD"}
                </span>
              </button>
            </div>
          </section>

          {/* Live only from a stand, dimmed everywhere else rather than hidden:
              a preset change re-plants every foot — and moves the middle pair
              where the two presets differ in leg set — so it is a stand-only
              move. The grid is what the view is for, though, and one that came
              and went would move the gait box under a thumb already reaching
              for it; dimmed, it still reads as which preset the robot is on.
              */}
          <section className="preset-section" data-section="preset">
            {/* Why the tiles below are dimmed, said in the box rather than left
                to be worked out from the robot's state, and on the heading's
                own line rather than under the tiles — the heading is the one
                row a box always has, so the line costs the tiles no height.
                Present only while the gate is shut. */}
            <h2 className="preset-section-title">
              PRESET
              {!standing && (
                <span className="preset-hint">Stand to activate</span>
              )}
            </h2>
            <div id="preset-list">
              {state.presets.map((preset) => (
                <button
                  key={preset.id}
                  className={[
                    "preset-item",
                    preset.id === state.activePreset && "active",
                    preset.id === pending && "pending",
                  ]
                    .filter(Boolean)
                    .join(" ")}
                  data-preset={preset.id}
                  // Everything is inert while a switch is in flight: the robot
                  // is moving its legs, and a second request would be refused
                  // anyway. And everything but the animation preset is inert in
                  // animation mode, which pins the robot to it.
                  disabled={
                    !standing || pending !== null || presetLocked(preset.id)
                  }
                  onClick={() => {
                    buzz(15);
                    send({ type: "select_preset", preset: preset.id });
                  }}
                >
                  <span className="preset-label">{preset.label}</span>
                </button>
              ))}
            </div>
          </section>
        </div>

        <div className="preset-column">
          {/* Every declared gait, with the ones the preset in force does not
              walk dimmed in place. Live on the belly too: no gait is running
              there, but /cmd_gait is latched, so the one lit is the one the
              next stand will come up walking — which is worth being able to
              pick before standing rather than after. Also inert while a preset
              change is in flight, which has already published the new preset's
              entry gait. */}
          {allGaits.length > 0 && (
            <section className="preset-section" data-section="gait">
              <h2 className="preset-section-title">GAIT</h2>
              <div id="gait-row">
                {allGaits.map((gait) => (
                  <button
                    key={gait}
                    className={["gait-item", gait === state.gait && "active"]
                      .filter(Boolean)
                      .join(" ")}
                    data-gait={gait}
                    disabled={pending !== null || !offered.has(gait)}
                    onClick={() => {
                      buzz(15);
                      send({ type: "select_gait", gait });
                    }}
                  >
                    {gaitLabel(gait)}
                  </button>
                ))}
              </div>
            </section>
          )}

          {/* The animation rotation, under the gaits and read the same way: the
              one lit is the one latched on /animation/mode. Dimmed rather than
              hidden outside animation mode — nothing is driving the body there,
              so there is nothing to select, but a box that came and went would
              move the gaits under a thumb already reaching for them. The node
              refuses a selection from another mode anyway; this is the same
              answer, given earlier. */}
          {state.animations.length > 0 && (
            <section className="preset-section" data-section="anim">
              <h2 className="preset-section-title">
                ANIM
                {!animating && (
                  <span className="preset-hint">Use ANIM mode to activate</span>
                )}
              </h2>
              <div id="anim-row">
                {state.animations.map((animation) => (
                  <button
                    key={animation}
                    className={[
                      "anim-item",
                      animation === state.animation && "active",
                    ]
                      .filter(Boolean)
                      .join(" ")}
                    data-animation={animation}
                    disabled={!animating || pending !== null}
                    onClick={() => {
                      buzz(15);
                      send({ type: "select_animation", animation });
                    }}
                  >
                    {animationLabel(animation)}
                  </button>
                ))}
              </div>
            </section>
          )}
        </div>
      </div>

      {/* Every tile on the view is already inert while a change is in flight,
          so the modal takes nothing away — it says why, in one place, instead
          of leaving the operator to find the word under one dashed tile. Up
          only for the couple of seconds the move takes: the node clears the
          pending preset on the engine's report, and expires it on its own
          deadline if none arrives. Written here, where the state that raises it
          lives, but rendered above every view — this box is a grid in
          landscape, which sizes a fixed child to a grid area instead of the
          screen. */}
      {pending !== null && (
        <Modal id="preset-switching">
          <Spinner />
          <p>Switching preset</p>
          <p className="dialog-sub">{pendingLabel}</p>
        </Modal>
      )}

      {/* The stand and the fold, said the same way: the ladder takes a few
          seconds, nothing on the view can be pressed through it, and the
          backdrop is what keeps a second press off the button that started it.
          Only one of the two boxes is ever up — a preset change runs from a
          stand and never passes through either of these states. */}
      {pending === null && transition !== null && (
        <Modal id="preset-transition">
          <Spinner />
          <p>{transition}</p>
        </Modal>
      )}

      {/* Empty until the node refuses something, so it reserves no space. */}
      <p id="preset-note" className={state.refusal ? "refused" : undefined}>
        {state.refusal?.text ?? ""}
      </p>
    </div>
  );
}
