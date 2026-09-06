import type { ActionName, Mode } from "../types/protocol";
import HoldButton from "./HoldButton";

// The three modes, top to bottom, each naming the FUNCTION its button asks for
// and what it says on itself. One table, read by the JSX below and by the pass
// in `app/index.tsx` that releases what a mode change takes off the screen.
//
// Nothing else lives in the middle of the screen. Every other function is
// either at a joystick corner (`CORNERS` in `app/index.tsx`) — height, yaw,
// stand, record, animation prev/next, all things a thumb needs without letting
// go of its stick — or on the Mode view, which owns preset and gait selection.
export const MODES: readonly { action: ActionName; mode: Mode; label: string }[] =
  [
    { action: "gait_mode", mode: "gait", label: "Gait" },
    { action: "posture_mode", mode: "posture", label: "Posture" },
    { action: "animation_mode", mode: "animation", label: "Anim" },
  ];

interface Props {
  mode: Mode;
  pressed: ReadonlySet<ActionName>;
  onPress: (action: ActionName) => void;
  onRelease: (action: ActionName) => void;
}

// The mode selector: three buttons on one line — a column between the circles
// in landscape, a row between them in portrait, which is the CSS's to decide.
// Offered in every mode, so a press here can never leave a held function on
// screen in a mode that dropped it.
export default function ModeStack({
  mode,
  pressed,
  onPress,
  onRelease,
}: Props) {
  return (
    <div id="mode-stack">
      {MODES.map((slot) => (
        <HoldButton
          key={slot.action}
          className={`mode-btn${slot.mode === mode ? " active" : ""}`}
          label={slot.label}
          pressed={pressed.has(slot.action)}
          onPress={() => onPress(slot.action)}
          onRelease={() => onRelease(slot.action)}
        />
      ))}
    </div>
  );
}
