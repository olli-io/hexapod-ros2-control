import type { ReactNode } from "react";
import { Link, useRouterState } from "@tanstack/react-router";
import { VIEW_PATHS, viewOfPath } from "../utils/views";
import type { ViewName } from "../utils/views";

interface Props {
  // With the socket down the control area commands nothing and the preset rows
  // report a stale robot, so the bar keeps only the two tabs that still work.
  usableOnly: boolean;
  connected: boolean;
  controllerActive: boolean;
  quad: boolean;
  presetPending: boolean;
}

// Tab bar: symbols only, evenly spaced. Horizontal across the bottom in
// portrait, vertical down the left in landscape (CSS-driven). Every item is a
// route link: it swaps the view above the bar and the bar itself never leaves,
// which is why no view carries a back arrow. Which tab is lit comes from the
// router rather than from a prop, so the bar cannot disagree with what is on
// screen.
export default function NavBar({
  usableOnly,
  connected,
  controllerActive,
  quad,
  presetPending,
}: Props) {
  const pathname = useRouterState({ select: (s) => s.location.pathname });
  const current = viewOfPath(pathname);

  const cls = (view: ViewName, ...extra: (string | false)[]) => {
    const usable = view === "network" || view === "log";
    return [
      "nav-icon",
      view === current && "tab-active",
      usableOnly && !usable && "hidden",
      ...extra,
    ]
      .filter(Boolean)
      .join(" ");
  };

  const tab = (view: ViewName, label: string, className: string, svg: ReactNode) => (
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
      {svg}
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
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"
             strokeLinecap="round" strokeLinejoin="round">
          <circle cx="12" cy="7.5" r="3.5" />
          <line x1="12" y1="11" x2="12" y2="16.5" />
          <ellipse cx="12" cy="18.5" rx="7" ry="2.5" />
        </svg>,
      )}

      {/* Mode: the operator preset (NORMAL / QUAD). The icon doubles as a
          leg-set readout — the two middle legs dim when the robot is standing
          on four — so the current mode is visible without opening the view. */}
      {tab(
        "preset",
        "Mode",
        cls("preset", quad && "quad", presetPending && "pending"),
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"
             strokeLinecap="round" strokeLinejoin="round">
          <rect x="8" y="6" width="8" height="12" rx="2" />
          <line x1="8" y1="8.5" x2="4" y2="6" />
          <line x1="16" y1="8.5" x2="20" y2="6" />
          <line className="preset-mid-leg" x1="8" y1="12" x2="3.5" y2="12" />
          <line className="preset-mid-leg" x1="16" y1="12" x2="20.5" y2="12" />
          <line x1="8" y1="15.5" x2="4" y2="18" />
          <line x1="16" y1="15.5" x2="20" y2="18" />
        </svg>,
      )}

      {/* Network: link state and who holds control. The wifi symbol keeps its
          connection colour (slash shows when disconnected) so the link is
          legible from any tab. */}
      {tab(
        "network",
        "Network",
        cls("network", connected ? "connected" : "disconnected"),
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"
             strokeLinecap="round" strokeLinejoin="round">
          <path d="M4 11.5a12 12 0 0 1 16 0" />
          <path d="M7.5 15a7 7 0 0 1 9 0" />
          <path d="M10.5 18.3a2.5 2.5 0 0 1 3 0" />
          <circle cx="12" cy="20.5" r="0.6" fill="currentColor" stroke="none" />
          <line className="wifi-slash" x1="3" y1="3" x2="21" y2="21" />
        </svg>,
      )}

      {/* Log */}
      {tab(
        "log",
        "Log",
        cls("log"),
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"
             strokeLinecap="round" strokeLinejoin="round">
          <rect x="4" y="3" width="16" height="18" rx="2" />
          <line x1="8" y1="8" x2="16" y2="8" />
          <line x1="8" y1="12" x2="16" y2="12" />
          <line x1="8" y1="16" x2="13" y2="16" />
        </svg>,
      )}
    </nav>
  );
}
