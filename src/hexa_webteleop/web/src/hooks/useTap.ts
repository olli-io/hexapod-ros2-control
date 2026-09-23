import { useCallback } from "react";
import type { ActionName, SendFn } from "../types/protocol";

// How long a tap holds the function down before letting go. The views that use
// this have no keepalive — none is the one a thumb sits on — so a press sent
// from them would otherwise stay in the node's action set until the input
// watchdog cleared it. Long enough that the 60 Hz tick cannot miss the edge;
// short enough that the window is no press at all.
const TAP_PRESS_MS = 150;

// A tap, never a hold: press, then let go a moment later. The state machine
// reads every function sent this way on the rising edge.
export function useTap(send: SendFn) {
  return useCallback(
    (action: ActionName) => {
      send({ type: "action", action, pressed: true });
      window.setTimeout(
        () => send({ type: "action", action, pressed: false }),
        TAP_PRESS_MS,
      );
    },
    [send],
  );
}
