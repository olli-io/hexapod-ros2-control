import type { PresetDescriptor } from "../types/protocol";

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

// The operator-facing name of a preset id, as `presets.list` declares it.
// Falls back to the id — the label is config, and a preset the descriptor list
// does not carry has to be called something. Here rather than at each call site
// because the Control strip, the Mode view's own strip and the switching modal
// all name the same preset and must agree on what it is called.
export function presetLabel(
  presets: PresetDescriptor[],
  id: string | null,
): string {
  if (id === null) return "";
  return presets.find((preset) => preset.id === id)?.label ?? id;
}

// Short haptic tick on button press (no-op where unsupported, e.g. iOS).
export function buzz(ms: number): void {
  navigator.vibrate?.(ms);
}
