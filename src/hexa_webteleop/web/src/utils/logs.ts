// Level filtering for the Log view. ROS 2 lines open with the level in
// brackets — `[INFO] [stamp] [node]: text` — and anything else (a traceback, a
// wrapped line, launch output) is a continuation of the line above it, so it
// follows that line's level rather than vanishing under the filter.
export const LOG_LEVELS = ["DEBUG", "INFO", "WARN", "ERROR"] as const;

export type LogLevel = (typeof LOG_LEVELS)[number];

const LEVEL_RE = /^\[(DEBUG|INFO|WARN|ERROR|FATAL)\]/;

// FATAL is folded into ERROR: it is one chip the operator would never see lit.
export function lineLevel(line: string): LogLevel | null {
  const m = LEVEL_RE.exec(line);
  if (!m) return null;
  return m[1] === "FATAL" ? "ERROR" : (m[1] as LogLevel);
}

export function filterLines(
  lines: readonly string[],
  shown: ReadonlySet<LogLevel>,
): string[] {
  const out: string[] = [];
  // Untagged lines before the first tagged one are launch output; kept.
  let keep = true;
  for (const line of lines) {
    const level = lineLevel(line);
    if (level !== null) keep = shown.has(level);
    if (keep) out.push(line);
  }
  return out;
}
