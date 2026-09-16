// The tab bar's four views, in order, plus the one route that opens from a
// view rather than from the bar: the log, reached through the Network view's
// LOGS panel and lighting the Network tab while it is up.
//
// Routes, but never page loads: the WebSocket is the session, and the server
// hands its one client slot to whoever reconnects, so a navigation that fetched
// a document would cost the operator control of the robot. The router keeps
// every switch in the client and the socket lives above it (see
// TeleopProvider.tsx), which is what makes these independent routes instead of
// one page holding hidden divs.
export const VIEW_NAMES = ["control", "preset", "gesture", "network"] as const;

export type ViewName = (typeof VIEW_NAMES)[number];

// The path each tab navigates to, and the log's. Kept here rather than spelled
// out at the call sites so the tab bar, the disconnect fallback and the route
// files agree by construction; `as const` keeps the literals the router
// type-checks `to` against.
export const VIEW_PATHS = {
  control: "/",
  preset: "/preset",
  gesture: "/gesture",
  network: "/network",
  log: "/log",
} as const;

// Which tab is lit on a path: the tab's own, or Network for the log it opens.
export function tabOfPath(pathname: string): ViewName | null {
  if (pathname === VIEW_PATHS.log) return "network";
  return VIEW_NAMES.find((name) => VIEW_PATHS[name] === pathname) ?? null;
}
