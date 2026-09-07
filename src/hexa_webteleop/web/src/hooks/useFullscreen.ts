/** The document's fullscreen state, and the one gesture that changes it. */
import { useEffect, useState } from "react";

/** Absent on iPhone Safari, which has no Fullscreen API at all — and needs
 *  none, since an installed iOS app has no browser chrome to hide in the first
 *  place. So the control is offered where it does something and omitted where
 *  it would be a dead button. Read once: a browser does not grow the API
 *  mid-document. */
export const CAN_FULLSCREEN = document.fullscreenEnabled;

/** Mirrors the document rather than owning the state: the operator can leave
 *  fullscreen with the OS back gesture or Escape, and the icon has to follow
 *  that as readily as it follows the button.
 *
 *  The toggle is never automatic. Fullscreen needs a gesture, and the only
 *  other gesture on this app is a joystick drag — entering fullscreen under a
 *  thumb that is steering is exactly the surprise a teleop UI must not spring.
 */
export function useFullscreen(): { fullscreen: boolean; toggle: () => void } {
  const [fullscreen, setFullscreen] = useState(
    () => document.fullscreenElement !== null,
  );

  useEffect(() => {
    const sync = () => setFullscreen(document.fullscreenElement !== null);
    document.addEventListener("fullscreenchange", sync);
    return () => document.removeEventListener("fullscreenchange", sync);
  }, []);

  const toggle = () => {
    if (document.fullscreenElement)
      void document.exitFullscreen().catch(() => {});
    else void document.documentElement.requestFullscreen().catch(() => {});
  };

  return { fullscreen, toggle };
}
