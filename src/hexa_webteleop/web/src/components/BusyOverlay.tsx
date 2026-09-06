// The one modal left: another device holds the server's single client slot,
// which is a state the operator cannot act on from any tab.
export default function BusyOverlay() {
  return (
    <div id="busy-overlay" className="overlay">
      <div className="dialog">
        <p>Another device is already controlling the hexapod.</p>
        <p className="dialog-sub">Waiting for it to disconnect…</p>
      </div>
    </div>
  );
}
