import { animationLabel } from "../utils/labels";

interface Props {
  presetLabel: string;
  gait: string;
  animation: string;
  voltage: number | null;
  current: number | null;
}

// Pack readout: both values, or a single dash while neither is known.
function packText(voltage: number | null, current: number | null): string {
  if (voltage === null && current === null) return "—";
  const v = voltage === null ? "— V" : `${voltage.toFixed(2)} V`;
  const a = current === null ? "— A" : `${current.toFixed(2)} A`;
  return `${v}  ${a}`;
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
      {/* The preset, because the tab icon cannot carry it: that icon shows the
          LEG SET, and NORMAL / FAST / OFFROAD all stand on six legs. */}
      <span className="status-item">
        <span className="status-key">Mode</span>
        <span className="status-value">{presetLabel || "—"}</span>
      </span>
      <span className="status-item">
        <span className="status-key">Gait</span>
        <span className="status-value">{gait || "—"}</span>
      </span>
      <span className="status-item">
        <span className="status-key">Anim</span>
        <span className="status-value">{animationLabel(animation) || "—"}</span>
      </span>
      {/* Pack voltage + current, polled over the WebSocket. Real robot only; a
          dash where nothing publishes telemetry. */}
      <span className="status-item">
        <span className="status-key">Pack</span>
        <span className="status-value">{packText(voltage, current)}</span>
      </span>
    </div>
  );
}
