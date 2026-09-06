import type { ActionName, Mode } from "../types/protocol";

// One grid slot: the function it asks for, and what it says on the button.
export interface Slot {
  action: ActionName;
  // Set on the three mode-select slots, naming the mode the press selects;
  // absent on the six action slots, which is what "action slot" means here.
  selects?: Mode;
  label: string;
}

// The three mode-select slots, the same in every mode.
const MODE_ROW: readonly Slot[] = [
  { action: "gait_mode", selects: "gait", label: "Gait" },
  { action: "posture_mode", selects: "posture", label: "Posture" },
  { action: "animation_mode", selects: "animation", label: "Anim" },
];

// Body height, the same in every mode, and the two slots that close the grid.
const HEIGHT_ROW: readonly Slot[] = [
  { action: "height_up", label: "Body\nup" },
  { action: "height_down", label: "Body\ndown" },
];

// The 3x3 grid per mode, in reading order: three mode-select slots, then the
// six that do something. Each slot names the FUNCTION it asks for, so this
// table is the whole answer to "what does that button do" — there is no key
// layout behind it to keep in sync, and the label sits next to the function it
// describes rather than in a config the client never sees.
export const GRID: Record<Mode, readonly Slot[]> = {
  gait: [
    ...MODE_ROW,
    { action: "init", label: "Stand\n(Hexa)" },
    // The select half of the init button: stands the robot up on four legs
    // with the middle pair folded. Offered in gait mode only — that is what
    // the node's state machine acts on, and `record` takes this slot in the
    // other two modes.
    { action: "quadruped_mode", label: "Stand\n(Quad)" },
    { action: "gait_prev", label: "Prev\nGait" },
    { action: "gait_next", label: "Next\nGait" },
    ...HEIGHT_ROW,
  ],
  posture: [
    ...MODE_ROW,
    { action: "init", label: "Stand\n(Hexa)" },
    { action: "record", label: "Save\nposture" },
    { action: "yaw_left", label: "Yaw\nleft" },
    { action: "yaw_right", label: "Yaw\nright" },
    ...HEIGHT_ROW,
  ],
  animation: [
    ...MODE_ROW,
    { action: "init", label: "Stand\n(Hexa)" },
    { action: "record", label: "Save\nposture" },
    { action: "animation_prev", label: "Prev\nAnim" },
    { action: "animation_next", label: "Next\nAnim" },
    ...HEIGHT_ROW,
  ],
};
