import { useEffect, useRef } from "react";
import type { RefObject } from "react";
import type { SendFn, StickSide } from "../types/protocol";

export interface JoystickHandle {
  // Re-centre the knob and command zero. Called on release and when the page is
  // hidden (safety stop). Leaving the Control route needs no call: the canvas
  // unmounts, and the cleanup below commands zero on the way out.
  reset: () => void;
  // Re-send the value the knob is sitting at, if a thumb is still down. The
  // keepalive calls this so a stationary hold stays commanded.
  resend: () => void;
}

interface Props {
  side: StickSide;
  label: string;
  disabled: boolean;
  send: SendFn;
  handleRef: RefObject<JoystickHandle | null>;
}

// Canvas rather than DOM, and imperative rather than React state: a touchmove
// fires per frame, so the knob position must never round-trip through a render.
// The listeners are attached by hand because React's synthetic touch handlers
// are passive and cannot preventDefault().
export default function Joystick({
  side,
  label,
  disabled,
  send,
  handleRef,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const sendRef = useRef(send);
  sendRef.current = send;

  const st = useRef({
    active: false,
    touchId: null as number | "mouse" | null,
    knobX: 0,
    knobY: 0,
    lastX: 0,
    lastY: 0,
    centerX: 0,
    centerY: 0,
    radius: 1,
  });

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("2d context unavailable");
    const s = st.current;

    function draw() {
      if (!canvas || !ctx) return;
      const r = s.radius;
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Outer ring — grey1, bright enough to read on a phone outdoors.
      ctx.strokeStyle = "#928374";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(s.centerX, s.centerY, r, 0, Math.PI * 2);
      ctx.stroke();

      // Crosshair
      ctx.strokeStyle = "#5a524c";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(s.centerX - r, s.centerY);
      ctx.lineTo(s.centerX + r, s.centerY);
      ctx.moveTo(s.centerX, s.centerY - r);
      ctx.lineTo(s.centerX, s.centerY + r);
      ctx.stroke();

      // Knob — aqua accent when active
      const knobR = Math.max(r * 0.3, 12);
      ctx.fillStyle = s.active ? "#89b482" : "#665c54";
      ctx.beginPath();
      ctx.arc(s.centerX + s.knobX, s.centerY + s.knobY, knobR, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#89b482";
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    function resize() {
      if (!canvas || !ctx) return;
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      // setTransform, not scale: resize runs more than once per canvas, and
      // scale() compounds on the transform the previous call left behind.
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      s.centerX = rect.width / 2;
      s.centerY = rect.height / 2;
      s.radius = rect.width / 2 - 4;
      draw();
    }

    function sendStick(x: number, y: number) {
      s.lastX = x;
      s.lastY = y;
      sendRef.current({ type: "stick", stick: side, x, y });
    }

    function reset() {
      s.active = false;
      s.touchId = null;
      s.knobX = 0;
      s.knobY = 0;
      sendStick(0, 0);
      draw();
    }

    function update(clientX: number, clientY: number) {
      if (!canvas) return;
      const rect = canvas.getBoundingClientRect();
      const dx = clientX - rect.left - s.centerX;
      const dy = clientY - rect.top - s.centerY;
      const dist = Math.hypot(dx, dy);
      const clampedDist = Math.min(dist, s.radius);
      const angle = Math.atan2(dy, dx);
      s.knobX = Math.cos(angle) * clampedDist;
      s.knobY = Math.sin(angle) * clampedDist;

      // REP-103: x = left = +, y = forward = +
      // Screen: x right = +, y down = +
      // So: stickX = -(knobX / radius) = left = positive
      //     stickY = -(knobY / radius) = up/forward = positive
      sendStick(-(s.knobX / s.radius), -(s.knobY / s.radius));
      draw();
    }

    const onTouchStart = (e: TouchEvent) => {
      e.preventDefault();
      if (s.active) return;
      const t = e.changedTouches[0];
      s.touchId = t.identifier;
      s.active = true;
      update(t.clientX, t.clientY);
    };

    const onTouchMove = (e: TouchEvent) => {
      e.preventDefault();
      if (!s.active || s.touchId === null) return;
      for (const t of Array.from(e.touches)) {
        if (t.identifier === s.touchId) {
          update(t.clientX, t.clientY);
          return;
        }
      }
    };

    const onTouchEnd = (e: TouchEvent) => {
      e.preventDefault();
      if (!s.active || s.touchId === null) return;
      reset();
    };

    const onMouseMove = (e: MouseEvent) => {
      if (!s.active || s.touchId !== "mouse") return;
      update(e.clientX, e.clientY);
    };

    const onMouseUp = () => {
      if (!s.active || s.touchId !== "mouse") return;
      reset();
      window.removeEventListener("mousemove", onMouseMove);
      window.removeEventListener("mouseup", onMouseUp);
    };

    const onMouseDown = (e: MouseEvent) => {
      e.preventDefault();
      if (s.active) return;
      s.active = true;
      s.touchId = "mouse";
      update(e.clientX, e.clientY);
      window.addEventListener("mousemove", onMouseMove);
      window.addEventListener("mouseup", onMouseUp);
    };

    canvas.addEventListener("touchstart", onTouchStart, { passive: false });
    canvas.addEventListener("touchmove", onTouchMove, { passive: false });
    canvas.addEventListener("touchend", onTouchEnd, { passive: false });
    canvas.addEventListener("touchcancel", onTouchEnd, { passive: false });
    canvas.addEventListener("mousedown", onMouseDown);
    window.addEventListener("resize", resize);

    resize();
    handleRef.current = {
      reset,
      resend: () => {
        if (s.active) sendStick(s.lastX, s.lastY);
      },
    };

    return () => {
      canvas.removeEventListener("touchstart", onTouchStart);
      canvas.removeEventListener("touchmove", onTouchMove);
      canvas.removeEventListener("touchend", onTouchEnd);
      canvas.removeEventListener("touchcancel", onTouchEnd);
      canvas.removeEventListener("mousedown", onMouseDown);
      window.removeEventListener("resize", resize);
      window.removeEventListener("mousemove", onMouseMove);
      window.removeEventListener("mouseup", onMouseUp);
      handleRef.current = null;
      // The Control route is going away, and a knob held at that moment never
      // sees its own touchend — the last velocity would stand until the
      // server's input watchdog noticed. Command zero on the way out.
      sendStick(0, 0);
    };
  }, [side, handleRef]);

  return (
    <div className={`joystick${disabled ? " disabled" : ""}`}>
      <canvas ref={canvasRef} />
      <span className="joystick-label">{label}</span>
    </div>
  );
}
