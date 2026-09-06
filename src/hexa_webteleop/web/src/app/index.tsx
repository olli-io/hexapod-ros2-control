import { useCallback, useEffect, useRef, useState } from "react";
import { createFileRoute } from "@tanstack/react-router";
import Joystick from "../components/Joystick";
import type { JoystickHandle } from "../components/Joystick";
import StatusBar from "../components/StatusBar";
import ButtonGrid from "../components/ButtonGrid";
import ControlPrompt from "../components/ControlPrompt";
import { controllerActive, useTeleop } from "../session";
import { GRID } from "../utils/actions";
import type { ActionName } from "../types/protocol";

// Re-send held input this often (ms) so the server's input watchdog
// (safety.input_timeout_s, default 500 ms) doesn't zero /cmd_vel while a stick or
// button is held stationary — move/press events only fire on change, so without
// this the command latches for one tick then drops. Kept well under the watchdog
// window (~10 sends per timeout); stops when the tab suspends (the timer suspends
// too), so the watchdog still guards a dropped link.
const KEEPALIVE_MS = 50;

// The home route, and the way back from every other one.
export const Route = createFileRoute("/")({ component: ControlRoute });

function ControlRoute() {
  const { state, send } = useTeleop();
  const [pressed, setPressed] = useState<ReadonlySet<ActionName>>(new Set());

  const leftJoyRef = useRef<JoystickHandle | null>(null);
  const rightJoyRef = useRef<JoystickHandle | null>(null);
  const pressedRef = useRef(pressed);
  pressedRef.current = pressed;

  const pressAction = useCallback(
    (action: ActionName) => {
      setPressed((prev) => new Set(prev).add(action));
      send({ type: "action", action, pressed: true });
    },
    [send],
  );

  const releaseAction = useCallback(
    (action: ActionName) => {
      setPressed((prev) => {
        if (!prev.has(action)) return prev;
        const next = new Set(prev);
        next.delete(action);
        return next;
      });
      send({ type: "action", action, pressed: false });
    },
    [send],
  );

  // Input keepalive to ensure continuous input stream
  useEffect(() => {
    const id = window.setInterval(() => {
      leftJoyRef.current?.resend();
      rightJoyRef.current?.resend();
      for (const action of pressedRef.current) {
        send({ type: "action", action, pressed: true });
      }
    }, KEEPALIVE_MS);
    return () => clearInterval(id);
  }, [send]);

  // Button hygiene on tab nav
  useEffect(() => {
    return () => {
      for (const action of pressedRef.current) {
        send({ type: "action", action, pressed: false });
      }
    };
  }, [send]);

  // ... and on a mode change, which swaps six of the nine slots. A slot the
  // new mode does not offer is gone from the screen, so its release event
  // never arrives — and the keepalive would hold that function down for good.
  useEffect(() => {
    const offered = new Set(GRID[state.mode].map((slot) => slot.action));
    const stale = [...pressedRef.current].filter((a) => !offered.has(a));
    if (stale.length === 0) return;
    setPressed((prev) => {
      const next = new Set(prev);
      for (const action of stale) next.delete(action);
      return next;
    });
    for (const action of stale) send({ type: "action", action, pressed: false });
  }, [state.mode, send]);

  // Safety stop: if the page is hidden, reset joys to zero
  useEffect(() => {
    const onVisibility = () => {
      if (document.visibilityState === "hidden") {
        leftJoyRef.current?.reset();
        rightJoyRef.current?.reset();
      }
    };
    document.addEventListener("visibilitychange", onVisibility);
    return () => document.removeEventListener("visibilitychange", onVisibility);
  }, []);

  const folded = state.gaitState === "folded";
  const controlled = controllerActive(state);
  const presetLabel =
    state.presets.find((p) => p.id === state.activePreset)?.label ??
    state.activePreset ??
    "";

  return (
    <div id="control-area">
      <Joystick
        side="left"
        label="Left"
        disabled={controlled}
        send={send}
        handleRef={leftJoyRef}
      />

      {/* Center column: status strip over the button grid. */}
      <div id="center-panel">
        <StatusBar
          presetLabel={presetLabel}
          // No gait is running on the belly, so the strategy the next stand will
          // use is not a status.
          gait={folded ? "" : state.gait}
          animation={state.animation}
          voltage={state.packVoltage}
          current={state.packCurrent}
        />
        {controlled ? (
          <ControlPrompt
            onTakeControl={() => send({ type: "request_control" })}
          />
        ) : (
          <ButtonGrid
            mode={state.mode}
            pressed={pressed}
            onPress={pressAction}
            onRelease={releaseAction}
          />
        )}
      </div>

      <Joystick
        side="right"
        label="Right"
        disabled={controlled}
        send={send}
        handleRef={rightJoyRef}
      />
    </div>
  );
}
