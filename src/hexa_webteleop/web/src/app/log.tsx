import { useCallback, useEffect, useRef, useState } from "react";
import { createFileRoute } from "@tanstack/react-router";

interface LogsResponse {
  lines?: string[];
  error?: string;
}

// Recent node output, over plain HTTP. Reachable with the socket down — it is
// where the reason for a drop shows up — so it takes no session state at all.
export const Route = createFileRoute("/log")({ component: LogRoute });

// Loaded when the route mounts — i.e. each time the tab is opened — and on
// demand from the refresh button. Not polled: it is a thing you go and read, and
// the socket next to it is carrying control input.
function LogRoute() {
  const [text, setText] = useState("");
  const preRef = useRef<HTMLPreElement>(null);

  const load = useCallback(async () => {
    setText("Loading…");
    try {
      const res = await fetch("/logs", { cache: "no-store" });
      const data = (await res.json()) as LogsResponse;
      if (data.error) {
        setText(`Error: ${data.error}`);
        return;
      }
      const lines = data.lines ?? [];
      setText(lines.length ? lines.join("\n") : "(no log entries)");
    } catch (e) {
      setText(`Failed to load logs: ${e}`);
    }
  }, []);

  useEffect(() => {
    void load();
  }, [load]);

  // Pin to newest entry.
  useEffect(() => {
    const el = preRef.current;
    if (el) el.scrollTop = el.scrollHeight;
  }, [text]);

  return (
    <div id="log-view">
      {/* No title: the tab bar names the view, and it never leaves. The refresh
          floats over the log's top right corner rather than taking a header row
          of its own — the entries are what the view is for. */}
      <pre ref={preRef} id="logs-view">{text}</pre>
      <button
        id="log-refresh"
        className="nav-icon"
        aria-label="Refresh logs"
        onClick={() => void load()}
      >
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"
             strokeLinecap="round" strokeLinejoin="round">
          <path d="M3.5 12a8.5 8.5 0 1 1 2.5 6" />
          <polyline points="3 19 3 13 9 13" />
        </svg>
      </button>
    </div>
  );
}
