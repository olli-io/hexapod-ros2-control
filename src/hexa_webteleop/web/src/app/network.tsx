import { createFileRoute } from "@tanstack/react-router";
import { useTeleop } from "../providers/TeleopProvider";

// Link state and the controller handover, in one view because they are one
// question: which input the robot is listening to, and whether this device can
// reach it at all. Where a dropped socket puts you. A view rather than the two
// popovers it replaces — the handover is a state worth reading next to the link
// that carries it, and a dialog over the joysticks was a modal in front of a
// control the operator still had a thumb on.
export const Route = createFileRoute("/network")({ component: NetworkRoute });

function NetworkRoute() {
  const { state, connected, send, connect, disconnect } = useTeleop();
  const webOwns = state.owner === "web";

  return (
    <div id="network-view">
      <div className="panel">
        <div className="panel-title">Link</div>
        <p id="conn-state">{connected ? "Connected to" : "Disconnected from"}</p>
        <p id="conn-host" className="panel-sub">
          {location.host}
        </p>
        <button
          id="conn-toggle"
          className="panel-btn"
          onClick={connected ? disconnect : connect}
        >
          {connected ? "Disconnect" : "Reconnect"}
        </button>
      </div>

      {/* Control handover. Lives here rather than on its own tab: it is the same
          question as the link — which input the robot is listening to. */}
      <div className="panel">
        <div className="panel-title">Control</div>
        <p id="controller-status">
          {!state.arbitrationEnabled
            ? "Arbitration disabled — web is always in control."
            : webOwns
              ? "The web app is in control."
              : "A controller is active."}
        </p>
        {state.arbitrationEnabled && (
          <button
            id="controller-toggle"
            className="panel-btn"
            onClick={() =>
              send({ type: webOwns ? "release_control" : "request_control" })
            }
          >
            {webOwns ? "Give control to controller" : "Take control"}
          </button>
        )}
      </div>
    </div>
  );
}
