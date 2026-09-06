// Status-strip names for the animation-mode animations. Display only — the wire
// names on /animation/mode stay the long ones.
const ANIM_LABELS: Record<string, string> = {
  vertical_body_roll: "wave",
  horizontal_body_roll: "snake",
  body_roll_3d: "spiral",
};

export function animationLabel(name: string): string {
  return ANIM_LABELS[name] ?? name;
}

// Short haptic tick on button press (no-op where unsupported, e.g. iOS).
export function buzz(ms: number): void {
  navigator.vibrate?.(ms);
}
