import { useEffect, useRef } from "react";
import type { ActionName, Mode } from "../types/protocol";
import { GRID } from "../utils/actions";
import type { Slot } from "../utils/actions";

interface Props {
  mode: Mode;
  pressed: ReadonlySet<ActionName>;
  onPress: (action: ActionName) => void;
  onRelease: (action: ActionName) => void;
}

function GridButton({
  slot,
  mode,
  pressed,
  onPress,
  onRelease,
}: {
  slot: Slot;
  mode: Mode;
  pressed: boolean;
  onPress: () => void;
  onRelease: () => void;
}) {
  const ref = useRef<HTMLButtonElement>(null);
  // Refs so the effect can stay mounted for the button's life: re-attaching the
  // listeners whenever a label changed would drop a press mid-hold.
  const cbs = useRef({ onPress, onRelease, pressed });
  cbs.current = { onPress, onRelease, pressed };

  useEffect(() => {
    const el = ref.current;
    if (!el) return;

    // preventDefault on the touch pair is what stops the browser's synthetic
    // mouse events, which would otherwise fire a second press per tap. React's
    // synthetic touch handlers are passive and cannot do it.
    const press = (e: Event) => {
      e.preventDefault();
      cbs.current.onPress();
    };
    const release = (e: Event) => {
      e.preventDefault();
      cbs.current.onRelease();
    };
    const leave = (e: Event) => {
      if (cbs.current.pressed) release(e);
    };

    el.addEventListener("touchstart", press, { passive: false });
    el.addEventListener("touchend", release, { passive: false });
    el.addEventListener("touchcancel", release, { passive: false });
    el.addEventListener("mousedown", press);
    el.addEventListener("mouseup", release);
    el.addEventListener("mouseleave", leave);
    return () => {
      el.removeEventListener("touchstart", press);
      el.removeEventListener("touchend", release);
      el.removeEventListener("touchcancel", release);
      el.removeEventListener("mousedown", press);
      el.removeEventListener("mouseup", release);
      el.removeEventListener("mouseleave", leave);
    };
  }, []);

  const className = [
    slot.selects ? "mode-btn" : "action-btn",
    slot.selects === mode && "active",
    !slot.selects && pressed && "pressed",
  ]
    .filter(Boolean)
    .join(" ");

  return (
    <button ref={ref} className={className}>
      {slot.label}
    </button>
  );
}

// 3x3 grid, `GRID[mode]` in reading order. A slot asks for a function by name,
// so what a button does and what it says are the same entry in one table
// (`utils/actions.ts`) — there is no binding elsewhere for the text to drift
// from, and swapping a slot is one line there.
export default function ButtonGrid({
  mode,
  pressed,
  onPress,
  onRelease,
}: Props) {
  const slots = GRID[mode];
  // A literal 3x3 table: the rows are the rows on screen, in slot order, so the
  // markup is the layout and no CSS decides where a button lands.
  return (
    <table id="button-grid">
      <tbody>
        {[0, 3, 6].map((start) => (
          <tr key={start}>
            {slots.slice(start, start + 3).map((slot) => (
              <td key={slot.action}>
                <GridButton
                  slot={slot}
                  mode={mode}
                  pressed={pressed.has(slot.action)}
                  onPress={() => onPress(slot.action)}
                  onRelease={() => onRelease(slot.action)}
                />
              </td>
            ))}
          </tr>
        ))}
      </tbody>
    </table>
  );
}
