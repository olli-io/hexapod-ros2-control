import type { ReactNode } from "react";
import { Link, useRouterState } from "@tanstack/react-router";
import {
  Hand,
  Joystick,
  Maximize,
  Minimize,
  Settings,
  SlidersHorizontal,
  Wifi,
  WifiOff,
} from "lucide-react";
import { CAN_FULLSCREEN, useFullscreen } from "../hooks/useFullscreen";
import { VIEW_PATHS, tabOfPath } from "../utils/views";
import type { ViewName } from "../utils/views";

interface Props {
  // With the socket down the control area commands nothing and the preset rows
  // report a stale robot, so the bar keeps only the one tab that still works
  // (and the log it opens).
  usableOnly: boolean;
  connected: boolean;
  controllerActive: boolean;
  presetPending: boolean;
  gesturePlaying: boolean;
}

// Tab bar: symbols only, evenly spaced. Horizontal across the bottom in
// portrait, vertical down the right edge in landscape (CSS-driven). Every item
// but the last is a route link: it swaps the view above the bar and the bar
// itself never leaves, which is why no view carries a back arrow. Which tab is
// lit comes from the router rather than from a prop, so the bar cannot disagree
// with what is on screen. The last item is the fullscreen toggle — not a view,
// so it lights from the document instead.
export default function NavBar({
  usableOnly,
  connected,
  controllerActive,
  presetPending,
  gesturePlaying,
}: Props) {
  const pathname = useRouterState({ select: (s) => s.location.pathname });
  const current = tabOfPath(pathname);
  const { fullscreen, toggle: toggleFullscreen } = useFullscreen();

  const cls = (view: ViewName, ...extra: (string | false)[]) => {
    const usable = view === "network";
    return [
      "nav-icon",
      view === current && "tab-active",
      usableOnly && !usable && "hidden",
      ...extra,
    ]
      .filter(Boolean)
      .join(" ");
  };

  const tab = (view: ViewName, label: string, className: string, icon: ReactNode) => (
    <Link
      id={`tab-${view}`}
      to={VIEW_PATHS[view]}
      className={className}
      // Empty rather than the default `{ className: "active" }`: which tab is
      // lit is `tab-active` above, and the router's own prefix matching would
      // call the Control tab ("/") active on every route.
      activeProps={{}}
      aria-label={label}
    >
      {icon}
    </Link>
  );

  return (
    <nav id="navbar">
      {/* Control: the joysticks and the button grid. The home tab, and the way
          back from every other one. Green while a controller owns /cmd_vel,
          since that is the tab whose contents change. */}
      {tab(
        "control",
        "Control",
        cls("control", controllerActive && "controlled"),
        <Joystick aria-hidden />,
      )}

      {/* Mode: the operator preset. Accent while a switch is in flight; which
          preset is in force is the status strip's to say, since the tab is a
          symbol and NORMAL / FAST / OFFROAD / QUAD are four of them. */}
      {tab(
        "preset",
        "Mode",
        cls("preset", presetPending && "pending"),
        <SlidersHorizontal aria-hidden />,
      )}

      {/* Gesture: the keyframed moves. Accent while one plays — the view's
          tiles are inert for those seconds, and the tab says why from anywhere. */}
      {tab(
        "gesture",
        "Gesture",
        cls("gesture", gesturePlaying && "playing"),
        <Hand aria-hidden />,
      )}

      {/* Network: link state, who holds control, and the way to the log. Lit
          on the log too, which is what makes it the way back. A grey cog with a
          wifi glyph beside it, green up and red down, so the link is legible
          from any tab without tinting the tab itself. */}
      {tab(
        "network",
        "Network",
        cls("network", connected ? "connected" : "disconnected"),
        <>
          <Settings aria-hidden />
          <span className="nav-badge" aria-hidden>
            {connected ? <Wifi /> : <WifiOff />}
          </span>
        </>,
      )}

      {/* Fullscreen: the browser chrome on or off. A button rather than a link,
          and the one item `usableOnly` does not hide — it is a property of this
          device's screen, so it keeps working with the socket down. Lit while
          fullscreen, by the same class the tabs use: it is the one item whose
          "active" is a state of the document rather than a route. */}
      {CAN_FULLSCREEN && (
        <button
          id="tab-fullscreen"
          type="button"
          className={["nav-icon", fullscreen && "tab-active"]
            .filter(Boolean)
            .join(" ")}
          onClick={toggleFullscreen}
          aria-label={fullscreen ? "Leave fullscreen" : "Fullscreen"}
          aria-pressed={fullscreen}
        >
          {fullscreen ? <Minimize aria-hidden /> : <Maximize aria-hidden />}
        </button>
      )}
    </nav>
  );
}
