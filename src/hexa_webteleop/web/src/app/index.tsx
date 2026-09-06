import { useCallback, useEffect, useRef, useState } from "react";
import { createFileRoute } from "@tanstack/react-router";
import Joystick from "../components/Joystick";
import type { JoystickHandle } from "../components/Joystick";
import StatusBar from "../components/StatusBar";
import ModeStack, { MODES, modeLocked } from "../components/ModeStack";
import HoldButton from "../components/HoldButton";
import ControlPrompt from "../components/ControlPrompt";
import { useKeepAwake } from "../hooks/useKeepAwake";
import {
  animationAvailable,
  controllerActive,
  useTeleop,
} from "../providers/TeleopProvider";
import type { ActionName, Mode } from "../types/protocol";

// Re-send held input this often (ms) so the server's input watchdog
// (safety.input_timeout_s, default 500 ms) doesn't zero /cmd_vel while a stick or
// button is held stationary — move/press events only fire on change, so without
// this the command latches for one tick then drops. Kept well under the watchdog
// window (~10 sends per timeout); stops when the tab suspends (the timer suspends
// too), so the watchdog still guards a dropped link.
const KEEPALIVE_MS = 50;

// The corner buttons each mode offers, by the function each one asks for. One
// table, read twice below — by the effect that releases what a mode change
// takes off the screen, and by the JSX that renders them — so a corner can
// never be on screen in a mode the release pass thinks it is gone from. That
// drift is not a cosmetic bug: a button that leaves under a thumb never sees
// its own touchend, and the keepalive would hold the function down for good.
//
// Stand is offered everywhere. Height, yaw and wiggle are live offsets the
// state machine adds to the pose it publishes, but animation mode offers none
// of them, and no record either: the animation is driving the body there, so a
// pose trimmed or saved underneath it is a pose fighting the animation. Gait
// mode offers no record for the reason `hexa_teleop`'s README gives. Yaw and
// wiggle being absent in animation mode also keeps the two teleops offering the
// same functions, since that is the one mode where the gamepad leaves
// `l1`/`r1`/`l2`/`r2` unbound. Height is the one place the two teleops now
// differ on purpose: the gamepad keeps it live in animation mode and `map_web`
// would still act on it — the web only stops offering the button.
//
// Animation prev/next take the right circle's two bottom corners in animation
// mode, which is exactly the mode that gives them up: they are the height-down
// and wiggle corners, and neither pair is ever on screen with the other.
const CORNERS: Record<Mode, readonly ActionName[]> = {
  gait: [
    "init",
    "yaw_left",
    "yaw_right",
    "wiggle_left",
    "wiggle_right",
    "height_up",
    "height_down",
  ],
  posture: [
    "init",
    "yaw_left",
    "yaw_right",
    "wiggle_left",
    "wiggle_right",
    "height_up",
    "height_down",
    "record",
  ],
  animation: ["init", "animation_prev", "animation_next"],
};

// The home route, and the way back from every other one.
export const Route = createFileRoute("/")({ component: ControlRoute });

function ControlRoute() {
  const { state, send } = useTeleop();

  // Held for as long as this route is mounted, for the same reason the
  // keepalive and the held functions are: driving is minutes of stick with no
  // taps in between, which every phone reads as an idle screen. Leaving for the
  // Log view releases it — nothing is being driven from there.
  useKeepAwake();
  const [pressed, setPressed] = useState<ReadonlySet<ActionName>>(new Set());

  const animationAllowed = animationAvailable(state);
  const controlled = controllerActive(state);

  const leftJoyRef = useRef<JoystickHandle | null>(null);
  const rightJoyRef = useRef<JoystickHandle | null>(null);
  const pressedRef = useRef(pressed);
  pressedRef.current = pressed;

  const pressAction = useCallback(
    (action: ActionName) => {
      setPressed((prev) => new Set(prev).add(action));
      send({ type: "action", action, pressed: true });
    },
    [send],
  );

  const releaseAction = useCallback(
    (action: ActionName) => {
      setPressed((prev) => {
        if (!prev.has(action)) return prev;
        const next = new Set(prev);
        next.delete(action);
        return next;
      });
      send({ type: "action", action, pressed: false });
    },
    [send],
  );

  // Input keepalive to ensure continuous input stream
  useEffect(() => {
    const id = window.setInterval(() => {
      leftJoyRef.current?.resend();
      rightJoyRef.current?.resend();
      for (const action of pressedRef.current) {
        send({ type: "action", action, pressed: true });
      }
    }, KEEPALIVE_MS);
    return () => clearInterval(id);
  }, [send]);

  // Button hygiene on tab nav
  useEffect(() => {
    return () => {
      for (const action of pressedRef.current) {
        send({ type: "action", action, pressed: false });
      }
    };
  }, [send]);

  // ... and on a mode change, or a controller taking /cmd_vel. A mode change
  // swaps the corners the modes do not share —
  // animation mode keeps stand and takes the height pair off for its own. A
  // button the new mode does not offer is gone from the screen, so its release
  // event never arrives, and the keepalive would hold that function down for
  // good. The mode buttons are offered in every mode, so a mode change can
  // never leave one of them stale — but a preset the animations are not written
  // for disables the animation one, and a disabled button fires no events
  // either, so that is watched here alongside the mode. A controller taking
  // /cmd_vel is the extreme of the same case: the whole view is replaced by the
  // prompt, so nothing at all is offered and everything held is released.
  useEffect(() => {
    const offered = controlled
      ? new Set<ActionName>()
      : new Set<ActionName>([
          ...MODES.filter((m) => !modeLocked(m.mode, animationAllowed)).map(
            (m) => m.action,
          ),
          ...CORNERS[state.mode],
        ]);
    const stale = [...pressedRef.current].filter((a) => !offered.has(a));
    if (stale.length === 0) return;
    setPressed((prev) => {
      const next = new Set(prev);
      for (const action of stale) next.delete(action);
      return next;
    });
    for (const action of stale) send({ type: "action", action, pressed: false });
  }, [state.mode, animationAllowed, controlled, send]);

  // Safety stop: if the page is hidden, reset joys to zero
  useEffect(() => {
    const onVisibility = () => {
      if (document.visibilityState === "hidden") {
        leftJoyRef.current?.reset();
        rightJoyRef.current?.reset();
      }
    };
    document.addEventListener("visibilitychange", onVisibility);
    return () => document.removeEventListener("visibilitychange", onVisibility);
  }, []);

  // A controller owning /cmd_vel owns every input on this view: the sticks, the
  // corners hanging off them and the mode column all feed a /cmd_vel this page
  // is not the source of, and the status strip reports a robot somebody else is
  // driving. So the view is the prompt while it lasts, rather than a prompt in
  // the middle of a screenful of dead controls. The joysticks unmount with it,
  // and their cleanup commands zero on the way out; the effect above releases
  // whatever the corners were holding.
  if (controlled) {
    return (
      <div id="control-area">
        <ControlPrompt onTakeControl={() => send({ type: "request_control" })} />
      </div>
    );
  }

  const folded = state.gaitState === "folded";
  const presetLabel =
    state.presets.find((p) => p.id === state.activePreset)?.label ??
    state.activePreset ??
    "";

  // The functions a thumb needs without letting go of its stick sit at the
  // corners of the circles the thumbs are already on: reaching the middle of
  // the screen for body height means leaving the stick, and the middle now
  // holds nothing but the mode column. Which corners each mode offers is
  // `CORNERS` above.
  const corner = (
    action: ActionName,
    position: string,
    label: string,
    extra = "",
  ) => (
    <HoldButton
      className={`corner-btn ${position}${extra ? ` ${extra}` : ""}${
        pressed.has(action) ? " pressed" : ""
      }`}
      label={label}
      pressed={pressed.has(action)}
      onPress={() => pressAction(action)}
      onRelease={() => releaseAction(action)}
    />
  );
  // Stand/Fold is the only stand on this view: one `init` press, which stands
  // the robot on the last six-leg preset from the belly and folds it from a
  // stand — so it needs no leg-set qualifier, and the four-legged stand is the
  // Mode view's QUAD preset rather than a second button here. The Mode view's
  // own button is the same press under the same two words.
  const offers = (action: ActionName) => CORNERS[state.mode].includes(action);

  return (
    <div id="control-area">
      <Joystick
        side="left"
        label="Left"
        send={send}
        handleRef={leftJoyRef}
      >
        {offers("yaw_left") && corner("yaw_left", "tl", "Yaw\n\u25c0")}
        {offers("record") && corner("record", "tr", "Save\npose")}
        {offers("wiggle_left") && corner("wiggle_left", "bl", "Wiggle\n\u25c0")}
        {/* Red on the belly: the one press that has to happen before anything
            else on screen does anything, and the only state the operator can
            read off the button itself. The word follows /gait/state for the
            same reason — one press does both halves, and a button labelled
            Stand from a stand would fold the robot under a hand reaching for
            the opposite. */}
        {offers("init") &&
          corner(
            "init",
            "br",
            folded ? "Stand" : "Fold",
            folded ? "folded" : "",
          )}
      </Joystick>

      {/* Center column: status strip over the mode selector. */}
      <div id="center-panel">
        <StatusBar
          presetLabel={presetLabel}
          // No gait is running on the belly, so the strategy the next stand will
          // use is not a status.
          gait={folded ? "" : state.gait}
          animation={state.animation}
          voltage={state.packVoltage}
          current={state.packCurrent}
        />
        <ModeStack
          mode={state.mode}
          animationAllowed={animationAllowed}
          pressed={pressed}
          onPress={pressAction}
          onRelease={releaseAction}
        />
      </div>

      <Joystick
        side="right"
        label="Right"
        send={send}
        handleRef={rightJoyRef}
      >
        {offers("yaw_right") && corner("yaw_right", "tr", "Yaw\n\u25b6")}
        {offers("height_up") && corner("height_up", "tl", "Body\n\u25b2")}
        {offers("height_down") && corner("height_down", "bl", "Body\n\u25bc")}
        {offers("wiggle_right") &&
          corner("wiggle_right", "br", "Wiggle\n\u25b6")}
        {/* The height-down and wiggle corners, in the one mode that offers
            neither. */}
        {offers("animation_prev") &&
          corner("animation_prev", "bl", "Prev\nAnim")}
        {offers("animation_next") &&
          corner("animation_next", "br", "Next\nAnim")}
      </Joystick>
    </div>
  );
}
