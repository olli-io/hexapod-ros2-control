// Tile names for the gaits. The four-corner two lose their `quad_` prefix
// rather than carry it as a suffix: what the operator picks is a canter or a
// walk, no six-leg gait shares either name, and which legs they walk is the
// preset row right above. `tetrapod` is shortened for width alone — it is the
// one name that will not fit a third of a row. Display only — the wire names on
// /cmd_gait stay the ones the catalog declares.
const GAIT_LABELS: Record<string, string> = {
  quad_canter: "canter",
  quad_walk: "walk",
  tetrapod: "tetra",
};

export function gaitLabel(name: string): string {
  return GAIT_LABELS[name] ?? name;
}

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
