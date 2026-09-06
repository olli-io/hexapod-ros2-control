import { useCallback } from "react";
import { createFileRoute } from "@tanstack/react-router";
import { buzz } from "../utils/labels";
import { useTeleop } from "../session";

const STAND_PRESS_MS = 150;

export const Route = createFileRoute("/preset")({ component: PresetRoute });

function PresetRoute() {
  const { state, send } = useTeleop();

  const folded = state.gaitState === "folded";
  const pending = state.pendingPreset;
  const gaits =
    state.presets.find((p) => p.id === state.activePreset)?.gaits ?? [];

  // window is no press at all.
  const onStand = useCallback(() => {
    send({ type: "action", action: "init", pressed: true });
    window.setTimeout(
      () => send({ type: "action", action: "init", pressed: false }),
      STAND_PRESS_MS,
    );
  }, [send]);

  return (
    <div id="preset-view">
      {folded ? (
        <button id="preset-stand" onClick={onStand}>
          <span className="preset-label">STAND</span>
          <span className="preset-sub">stand up to choose a mode</span>
        </button>
      ) : (
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
              disabled={pending !== null}
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
      )}

      {/* The gaits of the preset in force. Live on the belly too: no gait is
          running there, but /cmd_gait is latched, so the one lit is the one the
          next stand will come up walking — which is worth being able to pick
          before standing rather than after. Inert only while a preset change is
          in flight, which has already published the new preset's entry gait. */}
      {gaits.length > 0 && (
        <div id="gait-row">
          {gaits.map((gait) => (
            <button
              key={gait}
              className={["gait-item", gait === state.gait && "active"]
                .filter(Boolean)
                .join(" ")}
              data-gait={gait}
              disabled={pending !== null}
              onClick={() => {
                buzz(15);
                send({ type: "select_gait", gait });
              }}
            >
              {gait}
            </button>
          ))}
        </div>
      )}

      {/* Empty until the node refuses something, so it reserves no space. */}
      <p id="preset-note" className={state.refusal ? "refused" : undefined}>
        {state.refusal?.text ?? ""}
      </p>
    </div>
  );
}
