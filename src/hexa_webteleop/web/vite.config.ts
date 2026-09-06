import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { tanstackRouter } from "@tanstack/router-plugin/vite";
import { viteSingleFile } from "vite-plugin-singlefile";

// The teleop server serves exactly one page, and it is unforgiving about the
// rest: `captive_portal.static_filename` returns None for anything with a "/"
// in it and `_handle_get` answers that with a 302 to "/" rather than a 404, so
// a bundle under assets/ would fail *silently* — the browser handed the HTML
// page in place of the script it asked for. viteSingleFile() removes the
// question by inlining the JS and the CSS into index.html: one file to install,
// one file to serve, nothing left to resolve. The router runs on a hash history
// for the other half of the same rule — see main.tsx.
export default defineConfig({
  plugins: [
    // File-based routes. `src/app/` is the route directory and the tree it
    // generates is committed, because `npm run build` type-checks *before* Vite
    // runs and a fresh checkout has to type-check. Code splitting is off: a
    // split route is a second file to fetch, and after viteSingleFile there is
    // no second file.
    tanstackRouter({
      target: "react",
      routesDirectory: "./src/app",
      generatedRouteTree: "./src/routeTree.gen.ts",
      autoCodeSplitting: false,
    }),
    // The React Compiler memoizes what this app would otherwise re-render by
    // hand. It is deliberately conservative: the imperative corners here — the
    // joystick canvas and the button grid, which both write a ref during render
    // to keep hand-attached listeners reading live values — are bailed out of
    // rather than rewritten, so the hot paths stay exactly as written.
    react({
      babel: { plugins: [["babel-plugin-react-compiler", { target: "19" }]] },
    }),
    viteSingleFile(),
  ],
  // Absolute, not "./": the app is only ever served from "/" (the captive
  // portal redirect puts every stray request there on purpose), and a
  // relative href loaded from any other path would resolve to a nested URL
  // the server refuses.
  base: "/",
  build: {
    outDir: "dist",
    emptyOutDir: true,
    // Terser rather than the esbuild default: the whole app is one file served
    // over the robot's own hotspot to a phone, and the extra pass is worth the
    // couple of seconds it costs a build that only runs on a developer's host.
    // `console` and `debugger` go with it — the Log view reads the *node's*
    // log, and nothing ships a browser console to the operator.
    minify: "terser",
    terserOptions: {
      compress: { drop_console: true, drop_debugger: true, passes: 2 },
    },
    cssMinify: true,
    // Belt and braces for anything too large for viteSingleFile to inline: it
    // still lands flat and unhashed beside index.html, where the server can
    // serve it. Hashing would buy nothing anyway — every response carries
    // Cache-Control: no-store — and fixed names keep colcon's
    // --symlink-install links valid, so a rebuild refreshes a running sim
    // without a colcon run.
    assetsDir: "",
    rollupOptions: {
      output: {
        inlineDynamicImports: true,
        entryFileNames: "main.js",
        chunkFileNames: "[name].js",
        assetFileNames: "[name][extname]",
      },
    },
  },
  // Dev loop: `npm run dev` on the host against a sim brought up with
  // `./hexa sim up`, which serves the real node on 8080.
  server: {
    proxy: {
      "/ws": { target: "http://localhost:8080", ws: true },
      "/logs": "http://localhost:8080",
    },
  },
});
