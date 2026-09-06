import { useEffect } from "react";
import {
  Outlet,
  createRootRoute,
  useNavigate,
  useRouter,
} from "@tanstack/react-router";
import NavBar from "../components/NavBar";
import BusyOverlay from "../components/BusyOverlay";
import { controllerActive, useTeleop } from "../providers/TeleopProvider";
import { VIEW_PATHS } from "../utils/views";

// The shell every route renders inside: the tab bar, which never leaves the
// screen (so no view carries a back arrow), and the busy overlay, which is the
// one state no view can act on. Mounted once for the life of the page — the
// socket provider is above it, and this component is above the Outlet.
export const Route = createRootRoute({ component: RootLayout });

function RootLayout() {
  const { state, connected, linkDown } = useTeleop();
  const navigate = useNavigate();
  const router = useRouter();

  // With the socket down the control area commands nothing and the preset rows
  // report a stale robot, so the bar keeps only the two tabs that still work:
  // Network, to get the link back, and Log, which is fetched over plain HTTP and
  // is where the reason for the drop shows up. The route follows unless it is
  // already Log, so nobody is left on a dead screen with no way back. Keyed on
  // `linkDown` rather than `!connected`, which is also true while the first
  // socket is still opening — every load would begin on the Network view. The
  // path is read inside the effect rather than depended on, so this fires on the
  // drop and not on every navigation after it.
  useEffect(() => {
    if (!linkDown) return;
    if (router.state.location.pathname === VIEW_PATHS.log) return;
    // Replace, not push: a link that drops and comes back would otherwise
    // leave a trail of Network entries under the operator's back button.
    void navigate({ to: VIEW_PATHS.network, replace: true });
  }, [linkDown, navigate, router]);

  return (
    <>
      <NavBar
        usableOnly={!connected}
        connected={connected}
        controllerActive={controllerActive(state)}
        presetPending={state.pendingPreset !== null}
      />

      {/* Every view takes the same place in the #app flexbox, so they inherit
          one box in both orientations and the tab bar stays put. */}
      <Outlet />

      {state.busy && <BusyOverlay />}
    </>
  );
}
