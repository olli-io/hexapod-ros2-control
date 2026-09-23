import { CircleFadingArrowUp } from "lucide-react";
import Spinner from "./Spinner";
import { useTap } from "../hooks/useTap";
import { useTeleop } from "../providers/TeleopProvider";
import { buzz } from "../utils/labels";

// What the stand and the fold are called while they run. Both take a few
// seconds of ladder that no press can shorten, and the engine refuses a second
// one anyway.
const TRANSITION_LABELS: Record<string, string> = {
  initialize: "Standing up",
  folding: "Folding down",
};

// The belly, over the Mode and Gesture views: neither can do anything until
// the robot stands, so instead of a view of dimmed tiles the operator meets
// the one press that is live — STAND. Drawn inside the view rather than
// portalled, so the tab bar beside it stays reachable; the stand itself is the
// same `init` function the Control view's corner sends, and the label is read
// off /gait/state, never off what was last pressed.
//
// Once pressed the same overlay carries the ladder's spinner in the button's
// place, so the view does not flash back between the press and the stand, and
// the overlay is what keeps a second press off the tiles under it.
export default function StandOverlay() {
  const { state, send } = useTeleop();
  const tap = useTap(send);

  const folded = state.gaitState === "folded";
  // A preset change runs from a stand and never passes through either of
  // these states, so this and the switching modal are never up at once.
  const transition =
    state.pendingPreset === null
      ? (TRANSITION_LABELS[state.gaitState] ?? null)
      : null;

  if (!folded && transition === null) return null;

  return (
    <div id="stand-overlay">
      {folded ? (
        <button
          id="stand-overlay-btn"
          className="panel-btn"
          onClick={() => {
            buzz(15);
            tap("init");
          }}
        >
          <CircleFadingArrowUp aria-hidden />
          STAND
        </button>
      ) : (
        <div className="stand-overlay-transition">
          <Spinner />
          <p>{transition}</p>
        </div>
      )}
    </div>
  );
}
