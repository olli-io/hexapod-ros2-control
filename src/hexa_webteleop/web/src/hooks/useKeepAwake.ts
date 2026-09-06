/** Keep the phone's screen on while the robot is under a thumb.

The Control view is held for minutes at a time with no taps between joystick
drags, so the OS dims and then locks the screen mid-walk. Nothing else in the
app tells it otherwise.

Two mechanisms, because the good one is unavailable where it is most needed.
`navigator.wakeLock` is **secure-context only**, and every address the robot is
actually reached at is plain HTTP — `http://control.hexa/` on its own hotspot,
`http://<ip>:8080` on a home network (see the webteleop README on why there is
no TLS here). So the API is used where it exists (the Vite dev server on
localhost, and anywhere a future HTTPS listener lands) and the pre-API trick —
a muted looping video, which counts as playback and holds the screen on — is
the fallback everywhere else.

The fallback is best-effort by nature: Apple has tightened background media
over the years and no spec guarantees this. Every failure path here is a silent
no-op, because a phone that dims is a nuisance and an exception thrown out of a
teleop view is not.
*/
import { useEffect } from "react";

/** 1 s of flat --bg, H.264 baseline — the smallest clip both iOS and Chrome
 *  will decode. 2x2 rather than 1x1 because H.264's 4:2:0 chroma subsampling
 *  needs even dimensions and x264 refuses an odd one outright; the element is
 *  sized to a single CSS pixel below, so what reaches the screen is one pixel
 *  either way, painted the colour of the page behind it.
 *
 *  Inline rather than a file so `dist/` stays flat and the bundle stays a
 *  single page; at ~2 kB it costs less than the request would. Regenerate with:
 *    ffmpeg -f lavfi -i color=c=0x282828:s=2x2:r=1 -t 1 -c:v libx264 \
 *      -profile:v baseline -level 3.0 -pix_fmt yuv420p -movflags +faststart out.mp4
 */
const KEEP_AWAKE_MP4 = "data:video/mp4;base64,AAAAIGZ0eXBpc29tAAACAGlzb21pc28yYXZjMW1wNDEAAAMQbW9vdgAAAGxtdmhkAAAAAAAAAAAAAAAAAAAD6AAAA+gAAQAAAQAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAgAAAjt0cmFrAAAAXHRraGQAAAADAAAAAAAAAAAAAAABAAAAAAAAA+gAAAAAAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAABAAAAAAAIAAAACAAAAAAAkZWR0cwAAABxlbHN0AAAAAAAAAAEAAAPoAAAAAAABAAAAAAGzbWRpYQAAACBtZGhkAAAAAAAAAAAAAAAAAABAAAAAQABVxAAAAAAALWhkbHIAAAAAAAAAAHZpZGUAAAAAAAAAAAAAAABWaWRlb0hhbmRsZXIAAAABXm1pbmYAAAAUdm1oZAAAAAEAAAAAAAAAAAAAACRkaW5mAAAAHGRyZWYAAAAAAAAAAQAAAAx1cmwgAAAAAQAAAR5zdGJsAAAAunN0c2QAAAAAAAAAAQAAAKphdmMxAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAAAAAIAAgBIAAAASAAAAAAAAAABFExhdmM2My4xLjEwMSBsaWJ4MjY0AAAAAAAAAAAAAAAAGP//AAAAMGF2Y0MBQsAe/+EAGGdCwB7ZH4iIwEQAAAMABAAAAwAIPFi5IAEABWjLg8sgAAAAEHBhc3AAAAABAAAAAQAAABRidHJ0AAAAAAAAFBgAAAAAAAAAGHN0dHMAAAAAAAAAAQAAAAEAAEAAAAAAHHN0c2MAAAAAAAAAAQAAAAEAAAABAAAAAQAAABRzdHN6AAAAAAAAAoMAAAABAAAAFHN0Y28AAAAAAAAAAQAAA0AAAABhdWR0YQAAAFltZXRhAAAAAAAAACFoZGxyAAAAAAAAAABtZGlyYXBwbAAAAAAAAAAAAAAAACxpbHN0AAAAJKl0b28AAAAcZGF0YQAAAAEAAAAATGF2ZjYzLjEuMTAxAAAACGZyZWUAAAKLbWRhdAAAAnAGBf//bNxF6b3m2Ui3lizYINkj7u94MjY0IC0gY29yZSAxNjUgcjMyMjIgYjM1NjA1YSAtIEguMjY0L01QRUctNCBBVkMgY29kZWMgLSBDb3B5bGVmdCAyMDAzLTIwMjUgLSBodHRwOi8vd3d3LnZpZGVvbGFuLm9yZy94MjY0Lmh0bWwgLSBvcHRpb25zOiBjYWJhYz0wIHJlZj0zIGRlYmxvY2s9MTowOjAgYW5hbHlzZT0weDE6MHgxMTEgbWU9aGV4IHN1Ym1lPTcgcHN5PTEgcHN5X3JkPTEuMDA6MC4wMCBtaXhlZF9yZWY9MSBtZV9yYW5nZT0xNiBjaHJvbWFfbWU9MSB0cmVsbGlzPTEgOHg4ZGN0PTAgY3FtPTAgZGVhZHpvbmU9MjEsMTEgZmFzdF9wc2tpcD0xIGNocm9tYV9xcF9vZmZzZXQ9LTIgdGhyZWFkcz0xIGxvb2thaGVhZF90aHJlYWRzPTEgc2xpY2VkX3RocmVhZHM9MCBucj0wIGRlY2ltYXRlPTEgaW50ZXJsYWNlZD0wIGJsdXJheV9jb21wYXQ9MCBjb25zdHJhaW5lZF9pbnRyYT0wIGJmcmFtZXM9MCB3ZWlnaHRwPTAga2V5aW50PTI1MCBrZXlpbnRfbWluPTEgc2NlbmVjdXQ9NDAgaW50cmFfcmVmcmVzaD0wIHJjX2xvb2thaGVhZD00MCByYz1jcmYgbWJ0cmVlPTEgY3JmPTIzLjAgcWNvbXA9MC42MCBxcG1pbj0wIHFwbWF4PTY5IHFwc3RlcD00IGlwX3JhdGlvPTEuNDAgYXE9MToxLjAwAIAAAAALZYiEBXyYoAA3v4A=";

type WakeLockLike = { released: boolean; release: () => Promise<void> };

/** Narrowed through `unknown` rather than declared globally: whether the DOM lib
 *  in use already types `wakeLock` varies by TypeScript version, and a second
 *  declaration would collide with the one that does. */
type WakeLockNavigator = {
  wakeLock?: { request: (type: "screen") => Promise<WakeLockLike> };
};

/** Hold the screen awake for as long as the calling component is mounted.
 *
 *  Mounted-scoped on purpose: the Control route already releases every held
 *  function on the way out, and the screen is one more thing that should not
 *  stay held once nothing is being driven.
 */
export function useKeepAwake(): void {
  useEffect(() => {
    let disposed = false;
    let sentinel: WakeLockLike | null = null;
    let video: HTMLVideoElement | null = null;

    const api = (navigator as unknown as WakeLockNavigator).wakeLock;

    const acquire = async (): Promise<void> => {
      if (!api || sentinel) return;
      try {
        const lock = await api.request("screen");
        // The effect may have been torn down while that was in flight.
        if (disposed) {
          void lock.release().catch(() => {});
          return;
        }
        sentinel = lock;
      } catch {
        // Denied (backgrounded tab, battery saver). The next visibility
        // change tries again; there is nothing to report.
      }
    };

    const startVideo = (): void => {
      if (video) {
        void video.play().catch(() => {});
        return;
      }
      const el = document.createElement("video");
      el.src = KEEP_AWAKE_MP4;
      // Both spellings: the property is what the autoplay policy reads, the
      // attribute is what iOS has historically wanted present for inline
      // playback. Setting one and not the other is the classic way this trick
      // works everywhere except the phone it was written for.
      el.muted = true;
      el.setAttribute("muted", "");
      el.loop = true;
      el.playsInline = true;
      el.setAttribute("aria-hidden", "true");
      // Present and playing, but out of the way of a layout built on flex
      // boxes and a full-screen canvas. `display: none` would let the browser
      // treat it as inert, which is the one thing that must not happen.
      el.style.cssText =
        "position:fixed;top:0;left:0;width:1px;height:1px;opacity:0;pointer-events:none";
      document.body.appendChild(el);
      video = el;
      void el.play().catch(() => {});
    };

    const onVisible = (): void => {
      if (document.visibilityState !== "visible") return;
      // Both mechanisms drop when the page is backgrounded: the sentinel is
      // released by the browser, and playback is suspended.
      if (api) {
        if (sentinel?.released) sentinel = null;
        void acquire();
      } else {
        startVideo();
      }
    };

    // A muted video may autoplay under every current policy, but "may" is not
    // "does" — a gesture is the one condition that always clears it, and this
    // view is nothing but gestures.
    const onGesture = (): void => {
      if (!api) startVideo();
    };

    if (api) void acquire();
    else startVideo();

    document.addEventListener("visibilitychange", onVisible);
    window.addEventListener("pointerdown", onGesture);

    return () => {
      disposed = true;
      document.removeEventListener("visibilitychange", onVisible);
      window.removeEventListener("pointerdown", onGesture);
      if (sentinel) {
        void sentinel.release().catch(() => {});
        sentinel = null;
      }
      if (video) {
        video.pause();
        video.remove();
        video = null;
      }
    };
  }, []);
}
