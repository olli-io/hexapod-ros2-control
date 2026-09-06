import { useEffect, useRef } from "react";

interface Props {
  className: string;
  // May carry a newline (\n): the CSS breaks it onto a second line.
  label: string;
  pressed: boolean;
  // Inert and dimmed. The listeners stay attached — a disabled button fires no
  // events — so the caller must not disable one that is currently held: the
  // release would never arrive. Callers that can are careful about it.
  disabled?: boolean;
  onPress: () => void;
  onRelease: () => void;
}

// A button that reports the press and the release, not the click: every
// function the operator holds — height, yaw, drive — is commanded for as long
// as the thumb is down. Used by the grid slots and by the corner buttons around
// the joysticks, so the touch handling below is written once.
export default function HoldButton({
  className,
  label,
  pressed,
  disabled = false,
  onPress,
  onRelease,
}: Props) {
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

  return (
    <button ref={ref} className={className} disabled={disabled}>
      {label}
    </button>
  );
}
