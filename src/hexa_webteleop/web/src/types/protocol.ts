// The /ws wire contract, as `webteleop_node.py` speaks it. Server messages are
// discriminated on `type`; the reducer has one case per member, so adding a
// message to the node and forgetting the client is a type error here.

export type Owner = "web" | "gamepad";

// Every function the node's state machine reads as a press — the button half
// of `joy_mapping.ALL_FUNCTIONS`, which is what the node validates an incoming
// action against. The grid asks for these by name; nothing on this wire knows
// what a button index is.
export type ActionName =
  | "gait_mode"
  | "posture_mode"
  | "animation_mode"
  | "init"
  | "record"
  | "quadruped_mode"
  | "gait_prev"
  | "gait_next"
  | "animation_prev"
  | "animation_next"
  | "yaw_left"
  | "yaw_right"
  | "wiggle_left"
  | "wiggle_right"
  | "height_up"
  | "height_down";
export type Mode = "gait" | "posture" | "animation";
export type LegSet = "hexapod" | "quadruped";

// One row of the Mode view, as `presets.list` in webteleop.yaml declares it.
// `gaits` is that preset's rotation, which the Mode view offers as a button
// apiece once the preset is the one in force.
export interface PresetDescriptor {
  id: string;
  label: string;
  sub?: string;
  gaits?: string[];
}

export interface InitMessage {
  type: "init";
  arbitration_enabled: boolean;
  owner: Owner;
  mode: Mode;
  gait: string;
  animation: string;
  gait_state: string;
  presets?: PresetDescriptor[];
  preset_active?: string | null;
  preset_leg_set?: LegSet | null;
  preset_pending?: string | null;
  battery_poll_s?: number;
}

export interface BusyMessage {
  type: "busy";
  message?: string;
}

export interface ModeMessage {
  type: "mode";
  mode: Mode;
}

export interface OwnerMessage {
  type: "owner";
  owner: Owner;
}

export interface GaitMessage {
  type: "gait";
  gait: string;
}

export interface AnimationMessage {
  type: "animation";
  animation: string;
}

export interface GaitStateMessage {
  type: "gait_state";
  state: string;
}

export interface PresetMessage {
  type: "preset";
  active?: string | null;
  leg_set?: LegSet | null;
  pending?: string | null;
  refused?: string | null;
}

// Either field is null when the node has no fresh reading to give.
export interface BatteryMessage {
  type: "battery";
  voltage: number | null;
  current: number | null;
}

export type ServerMessage =
  | InitMessage
  | BusyMessage
  | ModeMessage
  | OwnerMessage
  | GaitMessage
  | AnimationMessage
  | GaitStateMessage
  | PresetMessage
  | BatteryMessage;

export type StickSide = "left" | "right";

export type ClientMessage =
  | { type: "stick"; stick: StickSide; x: number; y: number }
  | { type: "action"; action: ActionName; pressed: boolean }
  | { type: "battery" }
  | { type: "select_preset"; preset: string }
  // A gait by name, not a step through a rotation. The node still checks it
  // against the preset in force before it reaches /cmd_gait.
  | { type: "select_gait"; gait: string }
  | { type: "request_control" }
  | { type: "release_control" };

export type SendFn = (msg: ClientMessage) => void;
