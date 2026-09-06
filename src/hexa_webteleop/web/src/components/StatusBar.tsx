import { Bug, BugPlay, CarBattery, Settings2, Zap } from "lucide-react";
import type { LucideIcon } from "lucide-react";
import { animationLabel, gaitLabel } from "../utils/labels";

interface Props {
  presetLabel: string;
  gait: string;
  animation: string;
  voltage: number | null;
  current: number | null;
}

// One item: an icon for what the value is, the value itself, and nothing spelt
// out. The icon is the key — a word like MODE or PACK costs a chunk of a strip
// held to the mode column's width to say something the reader learns once.
function Item({ icon: Icon, value }: { icon: LucideIcon; value: string }) {
  return (
    <span className="status-item">
      <Icon className="status-icon" aria-hidden />
      <span className="status-value">{value}</span>
    </span>
  );
}

// A compact strip over the button grid. A sibling of the grid, not inside it, so
// it stays visible while the control prompt replaces the grid — a controller
// owner switching gaits is exactly when the passive observer wants to see it.
export default function StatusBar({
  presetLabel,
  gait,
  animation,
  voltage,
  current,
}: Props) {
  return (
    <div id="status-bar">
      {/* The preset by name — the Mode tab's icon is a symbol for the view and
          cannot tell NORMAL / FAST / OFFROAD / QUAD apart. */}
      <Item icon={Settings2} value={presetLabel || "—"} />
      <Item icon={Bug} value={gaitLabel(gait) || "—"} />
      <Item icon={BugPlay} value={animationLabel(animation) || "—"} />
      {/* Pack voltage and current, polled over the WebSocket. Real robot only;
          a dash on each where nothing publishes telemetry — two items rather
          than one, so the dash says which half is missing. */}
      <Item
        icon={CarBattery}
        value={voltage === null ? "—" : `${voltage.toFixed(2)}V`}
      />
      <Item icon={Zap} value={current === null ? "—" : `${current.toFixed(2)}A`} />
    </div>
  );
}
