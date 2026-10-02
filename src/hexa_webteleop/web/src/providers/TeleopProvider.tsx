import { createContext, useContext, useEffect } from "react";
import type { ReactNode } from "react";
import { useTeleopSocket } from "../hooks/useTeleopSocket";
import type { TeleopSocket, TeleopState } from "../hooks/useTeleopSocket";

// How long a refusal note stays on the Mode view before clearing itself (ms).
const REFUSAL_MS = 4000;

// The one socket, held above the router.
//
// Routes mount and unmount as the operator switches tabs; the link must not.
// The server has a single client slot and hands it to whoever reconnects, so a
// socket owned by a route would drop the robot every time somebody looked at
// the log. Hence a provider outside the RouterProvider rather than a hook in
// each view.
const SessionContext = createContext<TeleopSocket | null>(null);

export function TeleopProvider({ children }: { children: ReactNode }) {
  const session = useTeleopSocket();
  const { clearRefusal } = session;

  // The refusal note clears itself. Keyed on the nonce so the node refusing the
  // same thing twice re-arms the clock instead of letting the first one expire.
  // Here rather than in the Mode view, which is unmounted on every other tab: a
  // note armed there would stop its own clock on the way out and still be
  // sitting under the list on the way back.
  const refusalNonce = session.state.refusal?.nonce;
  useEffect(() => {
    if (refusalNonce === undefined) return;
    const id = window.setTimeout(clearRefusal, REFUSAL_MS);
    return () => clearTimeout(id);
  }, [refusalNonce, clearRefusal]);

  return <SessionContext value={session}>{children}</SessionContext>;
}

export function useTeleop(): TeleopSocket {
  const session = useContext(SessionContext);
  if (!session) throw new Error("useTeleop() outside <TeleopProvider>");
  return session;
}

// The engine states a preset change runs from — hexa_teleop's
// PRESET_SWITCH_STATES.
const PRESET_SWITCH_STATES = new Set(["folded", "fault", "stand"]);

// Whether animation mode may be entered now. Never on four legs: every
// animation is written for six, and the shared mapping refuses the mode there.
// Entering it switches to the preset `presets.animation` names, so off that
// preset the engine has to be in a state a preset change runs from. Both read
// the ENGINE's reports, never the tap.
export function animationAvailable(state: TeleopState): boolean {
  if (state.activeLegSet === "quadruped") return false;
  if (state.animationPreset === null) return true;
  if (state.activePreset === state.animationPreset) return true;
  return PRESET_SWITCH_STATES.has(state.gaitState);
}

// A controller is active whenever arbitration is on and the web app does not
// own /cmd_vel. Read from two routes: the Control tab turns green — it is the
// tab whose contents change — and the view itself swaps the button grid for the
// take-control prompt.
export function controllerActive(state: TeleopState): boolean {
  return state.arbitrationEnabled && state.owner !== "web";
}
