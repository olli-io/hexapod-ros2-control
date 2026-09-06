import { useCallback } from "react";
import { createFileRoute } from "@tanstack/react-router";
import { buzz } from "../utils/labels";
import { useTeleop } from "../session";

// How long the Mode view's STAND holds the init button down (ms). Several node
// ticks, and well under the input watchdog.
const STAND_PRESS_MS = 150;

// The Mode view: the operator presets, and the switch between them. A route of
// its own, but not a page of its own — pending and refused are live states that
// only the WebSocket carries, and the socket outlives every navigation. Reachable
// in every state, including while the take-control prompt has replaced the button
// grid, which is what makes a mode switch possible while a controller drives.
//
// The active row is driven by /gait/preset alone. A tap sets nothing locally: an
// optimistic row would show a mode the robot may refuse, and on a latched command
// topic that lie has nothing to correct it.
export const Route = createFileRoute("/preset")({ component: PresetRoute });

function PresetRoute() {
  const { state, send } = useTeleop();

  const folded = state.gaitState === "folded";
  const pending = state.pendingPreset;

  // The belly's one action, in place of the inert mode rows. It asks for the
  // same `init` function the grid's stand button does rather than talking to the
  // node directly: standing up IS that function, two-press revert and all, and
  // duplicating it here would be a second path into the same state machine. Held
  // for a beat rather than released in the same breath — the node samples the
  // held set on its 60 Hz tick, so a press and release inside one tick window is
  // no press at all.
  const onStand = useCallback(() => {
    send({ type: "action", action: "init", pressed: true });
    window.setTimeout(
      () => send({ type: "action", action: "init", pressed: false }),
      STAND_PRESS_MS,
    );
  }, [send]);

  return (
    <div id="preset-view">
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
            // Everything is inert while a switch is in flight: the robot is
            // moving its legs, and a second request would be refused anyway.
            // Inert on the belly too — a stand carries its own LEG SET, which
            // the engine resolves to a preset, so a four-corner mode picked here
            // would be overwritten the moment the robot got up.
            disabled={folded || pending !== null}
            onClick={() => {
              buzz(15);
              send({ type: "select_preset", preset: preset.id });
            }}
          >
            <span className="preset-label">{preset.label}</span>
            {preset.sub && <span className="preset-sub">{preset.sub}</span>}
          </button>
        ))}
      </div>

      {/* Folded only. On the belly the leg set the next stand comes up on is
          chosen by the stand itself, so the mode rows there would move a
          highlight the stand immediately overrides. The rows go inert and the
          view offers the one thing that is meaningful on the belly. */}
      {folded && (
        <button id="preset-stand" className="preset-item" onClick={onStand}>
          <span className="preset-label">STAND</span>
          <span className="preset-sub">stand up to choose a mode</span>
        </button>
      )}

      {/* Empty until the node refuses something, so it reserves no space. */}
      <p id="preset-note" className={state.refusal ? "refused" : undefined}>
        {state.refusal?.text ?? ""}
      </p>
    </div>
  );
}
