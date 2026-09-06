import { useCallback, useEffect, useReducer, useRef, useState } from "react";
import type {
  ClientMessage,
  LegSet,
  Mode,
  Owner,
  PresetDescriptor,
  ServerMessage,
} from "../types/protocol";

// Everything the server tells us, in one place. The old client kept these as
// eighteen module-level `let`s, each with its own hand-written fan-out into the
// DOM; here one reducer case per message type feeds the render.
export interface TeleopState {
  // False until the first "init", so nothing renders a state the node has not
  // spoken to yet — and the battery poll does not start before it knows its period.
  initialized: boolean;
  arbitrationEnabled: boolean;
  owner: Owner;
  mode: Mode;
  // Names as latched on /cmd_gait and /animation/mode; an empty animation means
  // nothing latched yet (pipeline startup default) -> placeholder.
  gait: string;
  animation: string;
  // Latest /gait/state. Only the folded case is read: no gait is running on the
  // belly, so the strategy the next stand will use is not a status.
  gaitState: string;
  // The animation-mode rotation, fixed at connect like `presets`. The Mode
  // view's animation row is one button per entry; `animation` above says which
  // of them the pipeline is on.
  animations: string[];
  // `presets` is fixed at connect; `activePreset` / `activeLegSet` come from the
  // engine's report topics and NOTHING else — never from the tap, never from the
  // latched /cmd_gait, which keeps a refused name forever. `pendingPreset` is
  // what was asked for and has not landed yet.
  presets: PresetDescriptor[];
  // The preset animation mode is pinned to; null where the config names none.
  animationPreset: string | null;
  activePreset: string | null;
  activeLegSet: LegSet | null;
  pendingPreset: string | null;
  // The nonce re-arms the four-second clear when the node refuses the same thing
  // twice — the text alone would not change, so the effect would not re-run.
  refusal: { text: string; nonce: number } | null;
  // Pack telemetry, polled over the socket. Null means unknown — never heard,
  // stale, or a non-finite reading — and shows as a dash.
  packVoltage: number | null;
  packCurrent: number | null;
  batteryPollS: number;
  // Another device holds the server's single client slot.
  busy: boolean;
}

const INITIAL: TeleopState = {
  initialized: false,
  arbitrationEnabled: false,
  owner: "gamepad",
  mode: "gait",
  gait: "",
  animation: "",
  gaitState: "",
  animations: [],
  presets: [],
  animationPreset: null,
  activePreset: null,
  activeLegSet: null,
  pendingPreset: null,
  refusal: null,
  packVoltage: null,
  packCurrent: null,
  batteryPollS: 1,
  busy: false,
};

type Action =
  | { kind: "message"; msg: ServerMessage }
  | { kind: "closed" }
  | { kind: "clearRefusal" };

function reduce(state: TeleopState, action: Action): TeleopState {
  if (action.kind === "clearRefusal") {
    return state.refusal === null ? state : { ...state, refusal: null };
  }
  if (action.kind === "closed") {
    // A dropped link makes the last reading meaningless, not merely old.
    return {
      ...state,
      initialized: false,
      packVoltage: null,
      packCurrent: null,
    };
  }
  const msg = action.msg;
  switch (msg.type) {
    case "init":
      return {
        ...state,
        initialized: true,
        busy: false,
        arbitrationEnabled: msg.arbitration_enabled,
        owner: msg.owner,
        mode: msg.mode,
        gait: msg.gait,
        animation: msg.animation,
        gaitState: msg.gait_state,
        animations: msg.animations ?? [],
        presets: msg.presets ?? [],
        animationPreset: msg.preset_animation ?? null,
        activePreset: msg.preset_active ?? null,
        activeLegSet: msg.preset_leg_set ?? null,
        pendingPreset: msg.preset_pending ?? null,
        batteryPollS: msg.battery_poll_s ?? 1,
      };
    case "busy":
      // The server closes the socket right after. Keep the overlay up across
      // reconnect attempts until a slot frees and a real "init" arrives.
      return { ...state, busy: true };
    case "mode":
      return { ...state, mode: msg.mode };
    case "owner":
      return { ...state, owner: msg.owner };
    case "gait":
      return { ...state, gait: msg.gait };
    case "animation":
      return { ...state, animation: msg.animation };
    case "gait_state":
      return { ...state, gaitState: msg.state };
    case "preset":
      return {
        ...state,
        activePreset: msg.active ?? null,
        activeLegSet: msg.leg_set ?? null,
        pendingPreset: msg.pending ?? null,
        refusal: msg.refused
          ? { text: msg.refused, nonce: (state.refusal?.nonce ?? 0) + 1 }
          : null,
      };
    case "battery":
      return {
        ...state,
        packVoltage: typeof msg.voltage === "number" ? msg.voltage : null,
        packCurrent: typeof msg.current === "number" ? msg.current : null,
      };
  }
}

const RECONNECT_MS = 2000;

export interface TeleopSocket {
  state: TeleopState;
  connected: boolean;
  // A connect attempt has failed, or an open link dropped. NOT the same as
  // `!connected`, which is also true for the few milliseconds before the first
  // socket opens — a fallback keyed on that fires on every page load, and lands
  // the operator on the Network view before the link has had a chance.
  linkDown: boolean;
  send: (msg: ClientMessage) => void;
  connect: () => void;
  disconnect: () => void;
  clearRefusal: () => void;
}

export function useTeleopSocket(): TeleopSocket {
  const [state, dispatch] = useReducer(reduce, INITIAL);
  const [connected, setConnected] = useState(false);
  const [linkDown, setLinkDown] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);
  // A manual disconnect stays down until the user reconnects.
  const manualRef = useRef(false);
  const retryRef = useRef<number | null>(null);
  // The reconnect fires from inside onclose, so the handler needs the freshest
  // connect without connect having to depend on itself.
  const connectRef = useRef<() => void>(() => {});

  const connect = useCallback(() => {
    if (retryRef.current !== null) {
      clearTimeout(retryRef.current);
      retryRef.current = null;
    }
    manualRef.current = false;
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const ws = new WebSocket(`${proto}//${location.host}/ws`);
    wsRef.current = ws;

    ws.onopen = () => {
      setConnected(true);
      setLinkDown(false);
    };

    ws.onclose = () => {
      if (wsRef.current === ws) wsRef.current = null;
      setConnected(false);
      setLinkDown(true);
      dispatch({ kind: "closed" });
      if (!manualRef.current) {
        retryRef.current = window.setTimeout(
          () => connectRef.current(),
          RECONNECT_MS,
        );
      }
    };

    ws.onerror = () => ws.close();

    ws.onmessage = (ev: MessageEvent<string>) => {
      let msg: ServerMessage;
      try {
        msg = JSON.parse(ev.data) as ServerMessage;
      } catch {
        return;
      }
      dispatch({ kind: "message", msg });
    };
  }, []);

  connectRef.current = connect;

  const disconnect = useCallback(() => {
    manualRef.current = true;
    if (retryRef.current !== null) {
      clearTimeout(retryRef.current);
      retryRef.current = null;
    }
    wsRef.current?.close();
  }, []);

  const send = useCallback((msg: ClientMessage) => {
    const ws = wsRef.current;
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
  }, []);

  const clearRefusal = useCallback(() => {
    dispatch({ kind: "clearRefusal" });
  }, []);

  useEffect(() => {
    connect();
    return () => {
      manualRef.current = true;
      if (retryRef.current !== null) clearTimeout(retryRef.current);
      wsRef.current?.close();
    };
  }, [connect]);

  // Pack telemetry. Polled rather than pushed because the pack is sampled at
  // 10 Hz on the robot and the strip needs it about once a second.
  useEffect(() => {
    if (!connected || !state.initialized) return;
    const poll = () => send({ type: "battery" });
    poll();
    const id = window.setInterval(
      poll,
      Math.max(200, state.batteryPollS * 1000),
    );
    return () => clearInterval(id);
  }, [connected, state.initialized, state.batteryPollS, send]);

  return { state, connected, linkDown, send, connect, disconnect, clearRefusal };
}
