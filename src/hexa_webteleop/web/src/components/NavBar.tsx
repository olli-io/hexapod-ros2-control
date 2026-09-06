import type { ReactNode } from "react";
import { Link, useRouterState } from "@tanstack/react-router";
import { Joystick, ScrollText, SlidersHorizontal, Wifi, WifiOff } from "lucide-react";
import { VIEW_PATHS, viewOfPath } from "../utils/views";
import type { ViewName } from "../utils/views";

interface Props {
  // With the socket down the control area commands nothing and the preset rows
  // report a stale robot, so the bar keeps only the two tabs that still work.
  usableOnly: boolean;
  connected: boolean;
  controllerActive: boolean;
  presetPending: boolean;
}

// Tab bar: symbols only, evenly spaced. Horizontal across the bottom in
// portrait, vertical down the right edge in landscape (CSS-driven). Every item
// is a route link: it swaps the view above the bar and the bar itself never leaves,
// which is why no view carries a back arrow. Which tab is lit comes from the
// router rather than from a prop, so the bar cannot disagree with what is on
// screen.
export default function NavBar({
  usableOnly,
  connected,
  controllerActive,
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

      {/* Network: link state and who holds control. The wifi symbol keeps its
          connection colour (the struck-through glyph shows when disconnected)
          so the link is legible from any tab. */}
      {tab(
        "network",
        "Network",
        cls("network", connected ? "connected" : "disconnected"),
        connected ? <Wifi aria-hidden /> : <WifiOff aria-hidden />,
      )}

      {/* Log */}
      {tab("log", "Log", cls("log"), <ScrollText aria-hidden />)}
    </nav>
  );
}
