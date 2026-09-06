// The four views, in tab-bar order, each with the route that shows it.
//
// Routes, but never page loads: the WebSocket is the session, and the server
// hands its one client slot to whoever reconnects, so a navigation that fetched
// a document would cost the operator control of the robot. The router keeps
// every switch in the client and the socket lives above it (see session.tsx),
// which is what makes these four independent routes instead of one page holding
// four hidden divs.
export const VIEW_NAMES = ["control", "preset", "network", "log"] as const;

export type ViewName = (typeof VIEW_NAMES)[number];

// The path each tab navigates to. Kept here rather than spelled out at the call
// sites so the tab bar, the disconnect fallback and the route files agree by
// construction; `as const` keeps the literals the router type-checks `to`
// against.
export const VIEW_PATHS = {
  control: "/",
  preset: "/preset",
  network: "/network",
  log: "/log",
} as const;

export function viewOfPath(pathname: string): ViewName | null {
  return VIEW_NAMES.find((name) => VIEW_PATHS[name] === pathname) ?? null;
}
