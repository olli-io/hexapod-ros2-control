// Hexapod log page — plain JavaScript, no dependencies.
//
// Standalone page (no joysticks, no WebSocket). Shows recent log output
// fetched from GET /logs, with a manual refresh and a back button.

"use strict";

function $(id) {
  const el = document.getElementById(id);
  if (!el) throw new Error("element #" + id + " not found");
  return el;
}

// ── Log viewer ─────────────────────────────────────────────────────

async function loadLogs() {
  const view = $("logs-view");
  view.textContent = "Loading…";
  try {
    const res = await fetch("/logs", { cache: "no-store" });
    const data = await res.json();
    if (data.error) {
      view.textContent = "Error: " + data.error;
      return;
    }
    const lines = data.lines || [];
    view.textContent = lines.length ? lines.join("\n") : "(no log entries)";
    // Pin to newest entry.
    view.scrollTop = view.scrollHeight;
  } catch (e) {
    view.textContent = "Failed to load logs: " + e;
  }
}

// ── Init ───────────────────────────────────────────────────────────

function init() {
  $("close-btn").addEventListener("click", function () {
    location.href = "/";
  });
  $("refresh-btn").addEventListener("click", loadLogs);
  loadLogs();
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}
