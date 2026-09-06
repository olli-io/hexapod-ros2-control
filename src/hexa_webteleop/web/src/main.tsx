import { createRoot } from "react-dom/client";
import {
  RouterProvider,
  createHashHistory,
  createRouter,
} from "@tanstack/react-router";
import { routeTree } from "./routeTree.gen";
import { TeleopProvider } from "./session";
import "./styles.css";

const router = createRouter({
  routeTree,
  history: createHashHistory(),
});

declare module "@tanstack/react-router" {
  interface Register {
    router: typeof router;
  }
}

const container = document.getElementById("app");
if (!container) throw new Error("#app not found");
createRoot(container).render(
  <TeleopProvider>
    <RouterProvider router={router} />
  </TeleopProvider>,
);
