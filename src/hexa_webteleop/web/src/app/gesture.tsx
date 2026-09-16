import { createFileRoute, useNavigate } from "@tanstack/react-router";
import Spinner from "../components/Spinner";
import { useModal } from "../providers/ModalProvider";
import { useTeleop } from "../providers/TeleopProvider";
import { buzz, presetLabel } from "../utils/labels";
import { VIEW_PATHS } from "../utils/views";

// The gestures: one tile per id in gestures.yaml, a tap sends the id on
// /cmd_gesture and the lit tile is the engine's own report on /gait/gesture.
// The engine plays a gesture only from a stand on the default preset, so the
// view is gated the same way — off the engine's reports, never the tap.
export const Route = createFileRoute("/gesture")({ component: GestureRoute });

function GestureRoute() {
  const { state, send } = useTeleop();
  const Modal = useModal();
  const navigate = useNavigate();

  const standing = state.gaitState === "stand";
  const playing = state.gesture !== "";
  const pending = state.pendingPreset;
  // Gestures are written for one preset's stance. Before the first
  // /gait/preset report nothing is in force, and the answer is no — the same
  // honesty the unlit preset tiles show.
  const onGesturePreset =
    state.gesturePreset === null || state.activePreset === state.gesturePreset;
  const gesturePresetLabel = presetLabel(state.presets, state.gesturePreset);
  const live = standing && onGesturePreset && pending === null;

  return (
    <div id="gesture-view">
      <section className="preset-section" data-section="gesture">
        {/* Why the tiles are dimmed, on the heading's own line as the Mode view
            says it. Absent while a gesture plays: the lit tile says what is
            happening, and the robot is not waiting for a stand. */}
        <h2 className="preset-section-title">
          GESTURE
          {!standing && !playing && (
            <span className="preset-hint">Stand to activate</span>
          )}
        </h2>
        <div id="gesture-list">
          {state.gestures.map((gesture) => (
            <button
              key={gesture}
              className={[
                "gesture-item",
                gesture === state.gesture && "active",
              ]
                .filter(Boolean)
                .join(" ")}
              data-gesture={gesture}
              // Inert while one plays too: the engine refuses a second, and the
              // lit tile keeps its fill through the dimming, like a running
              // animation on the Mode view.
              disabled={!live || playing}
              onClick={() => {
                buzz(15);
                send({ type: "select_gesture", gesture });
              }}
            >
              <span className="preset-label">{gesture}</span>
            </button>
          ))}
        </div>
      </section>

      {/* Off the gesture preset the view cannot be used, so instead of a box of
          dimmed tiles the operator meets the fix: the switch to that preset,
          which is a stand-only move — on the belly or mid-walk the button is
          dimmed and the way out is the Mode view, where STAND lives. The
          switch itself is the Mode view's own request, so it lands under the
          same rules: refused mid-walk, pending until /gait/preset reports it.
          Hidden while the switch is in flight, where the spinner below takes
          over. */}
      {!onGesturePreset && pending === null && (
        <Modal id="gesture-preset">
          <p>Gestures need the {gesturePresetLabel} preset</p>
          <p className="dialog-sub">
            {standing
              ? "Switch to it to continue."
              : "Stand up first, on the Mode view."}
          </p>
          <div className="dialog-actions">
            <button
              className="panel-btn"
              disabled={!standing}
              onClick={() => {
                buzz(15);
                if (state.gesturePreset !== null) {
                  send({ type: "select_preset", preset: state.gesturePreset });
                }
              }}
            >
              Switch to {gesturePresetLabel}
            </button>
            <button
              className="panel-btn secondary"
              onClick={() => void navigate({ to: VIEW_PATHS.preset })}
            >
              Mode view
            </button>
          </div>
        </Modal>
      )}

      {/* The switch in flight, said the way the Mode view says it. */}
      {pending !== null && (
        <Modal id="preset-switching">
          <Spinner />
          <p>Switching preset</p>
          <p className="dialog-sub">{presetLabel(state.presets, pending)}</p>
        </Modal>
      )}

      {/* Empty until the node refuses something, so it reserves no space. */}
      <p id="preset-note" className={state.refusal ? "refused" : undefined}>
        {state.refusal?.text ?? ""}
      </p>
    </div>
  );
}
