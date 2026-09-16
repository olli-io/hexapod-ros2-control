import { useCallback, useEffect, useRef, useState } from "react";
import { createFileRoute } from "@tanstack/react-router";
import { LOG_LEVELS, filterLines } from "../utils/logs";
import type { LogLevel } from "../utils/logs";

interface LogsResponse {
  lines?: string[];
  error?: string;
}

// Recent node output, over plain HTTP. Reachable with the socket down — it is
// where the reason for a drop shows up — so it takes no session state at all.
// No tab of its own: it opens from the Network view's LOGS panel, and the
// Network tab stays lit while it is up, so that tab is the way back.
export const Route = createFileRoute("/log")({ component: LogRoute });

// Loaded when the route mounts — i.e. each time the view is opened — and on
// demand from the refresh button. Not polled: it is a thing you go and read, and
// the socket next to it is carrying control input.
function LogRoute() {
  const [lines, setLines] = useState<string[] | null>(null);
  const [status, setStatus] = useState("Loading…");
  // Every level on until the operator turns one off; the filter is per visit,
  // like the fetch, since a filter left on across visits hides lines silently.
  const [shown, setShown] = useState<Set<LogLevel>>(() => new Set(LOG_LEVELS));
  const preRef = useRef<HTMLPreElement>(null);

  const load = useCallback(async () => {
    setLines(null);
    setStatus("Loading…");
    try {
      const res = await fetch("/logs", { cache: "no-store" });
      const data = (await res.json()) as LogsResponse;
      if (data.error) {
        setStatus(`Error: ${data.error}`);
        return;
      }
      setLines(data.lines ?? []);
    } catch (e) {
      setStatus(`Failed to load logs: ${e}`);
    }
  }, []);

  useEffect(() => {
    void load();
  }, [load]);

  const toggle = (level: LogLevel) =>
    setShown((prev) => {
      const next = new Set(prev);
      if (next.has(level)) next.delete(level);
      else next.add(level);
      return next;
    });

  let text = status;
  if (lines !== null) {
    const visible = filterLines(lines, shown);
    text = visible.length
      ? visible.join("\n")
      : lines.length
        ? "(no entries at the selected levels)"
        : "(no log entries)";
  }

  // Pin to newest entry.
  useEffect(() => {
    const el = preRef.current;
    if (el) el.scrollTop = el.scrollHeight;
  }, [text]);

  return (
    <div id="log-view">
      {/* No title: the Network tab names the way back, and it never leaves.
          One row over the log: the level chips, which hide rather than
          highlight — the log is read for the one line that matters, and the
          rest is noise around it — and the refresh at the row's end. */}
      <div id="log-filter">
        {LOG_LEVELS.map((level) => (
          <button
            key={level}
            className={["log-level", shown.has(level) && "active"]
              .filter(Boolean)
              .join(" ")}
            data-level={level}
            aria-pressed={shown.has(level)}
            onClick={() => toggle(level)}
          >
            {level}
          </button>
        ))}
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
      <pre ref={preRef} id="logs-view">{text}</pre>
    </div>
  );
}
