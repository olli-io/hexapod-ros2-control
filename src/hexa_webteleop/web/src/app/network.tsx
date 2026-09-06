import { useEffect, useState } from "react";
import { createFileRoute } from "@tanstack/react-router";
import {
  Gamepad,
  Gamepad2,
  Maximize,
  Minimize,
  Plug,
  Unplug,
} from "lucide-react";
import { useTeleop } from "../providers/TeleopProvider";

// Whether this page is already running as an installed app. Read once at load
// because it cannot change without one: a home-screen launch is a fresh
// document. `navigator.standalone` is the iOS-only spelling and predates the
// media query, which is why both are asked.
const INSTALLED =
  window.matchMedia("(display-mode: standalone)").matches ||
  (navigator as unknown as { standalone?: boolean }).standalone === true;

// Absent on iPhone Safari, which has no Fullscreen API at all — and needs
// none, since an installed iOS app has no browser chrome to hide in the first
// place. So the control is offered where it does something and omitted where
// it would be a dead button.
const CAN_FULLSCREEN = document.fullscreenEnabled;

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

  // Mirrors the document rather than owning the state: the operator can leave
  // fullscreen with the OS back gesture or Escape, and the label has to follow
  // that as readily as it follows the button.
  const [fullscreen, setFullscreen] = useState(
    () => document.fullscreenElement !== null,
  );
  useEffect(() => {
    const sync = () => setFullscreen(document.fullscreenElement !== null);
    document.addEventListener("fullscreenchange", sync);
    return () => document.removeEventListener("fullscreenchange", sync);
  }, []);

  // Never automatic. Fullscreen needs a gesture, and the only other gesture on
  // this app is a joystick drag — entering fullscreen under a thumb that is
  // steering is exactly the surprise a teleop UI must not spring.
  const toggleFullscreen = () => {
    if (document.fullscreenElement)
      void document.exitFullscreen().catch(() => {});
    else void document.documentElement.requestFullscreen().catch(() => {});
  };

  return (
    <div id="network-view">
      <div className="panel">
        <div className="panel-title">Link</div>
        <p id="conn-state">
          {connected ? "Connected to" : "Disconnected from"}
        </p>
        <p id="conn-host" className="panel-sub">
          {location.host}
        </p>
        <button
          id="conn-toggle"
          className="panel-btn"
          onClick={connected ? disconnect : connect}
        >
          {connected ? <Unplug aria-hidden /> : <Plug aria-hidden />}
          {connected ? "Disconnect" : "Reconnect"}
        </button>
      </div>

      {/* Screen real estate. Here rather than on the Control view because it is
          set once per device and never touched again, and the Control view is
          for things a thumb is on.

          There is no "Install" button: Chrome fires beforeinstallprompt only in
          a secure context, and the robot is plain HTTP (see the README), so the
          handler would never run. Adding to the home screen is the browser's
          own menu item on both platforms, hence a sentence rather than a
          control. */}
      {(CAN_FULLSCREEN || !INSTALLED) && (
        <div className="panel">
          <div className="panel-title">Display</div>
          {!INSTALLED && (
            <p id="install-hint" className="panel-sub">
              Add to Home Screen from the browser menu to launch without browser
              chrome.
            </p>
          )}
          {CAN_FULLSCREEN && (
            <button
              id="fullscreen-toggle"
              className="panel-btn"
              onClick={toggleFullscreen}
            >
              {fullscreen ? <Minimize aria-hidden /> : <Maximize aria-hidden />}
              {fullscreen ? "Leave fullscreen" : "Fullscreen"}
            </button>
          )}
        </div>
      )}

      {/* Control handover. Lives here rather than on its own tab: it is the same
          question as the link — which input the robot is listening to. */}
      <div className="panel">
        <div className="panel-title">Control</div>
        <p id="controller-status">
          {!state.arbitrationEnabled
            ? "Arbitration disabled — web is always in control."
            : webOwns
              ? "This web app is currently in control"
              : "Control defaults to a bt controller"}
        </p>
        {state.arbitrationEnabled && (
          <button
            id="controller-toggle"
            className="panel-btn"
            onClick={() =>
              send({ type: webOwns ? "release_control" : "request_control" })
            }
          >
            {/* The icon names where control is going, not what is pressing the
                button: the pad on the way out, this device on the way in. */}
            {webOwns ? <Gamepad2 aria-hidden /> : <Gamepad aria-hidden />}
            {webOwns ? "Hand over control" : "Control here"}
          </button>
        )}
      </div>
    </div>
  );
}
