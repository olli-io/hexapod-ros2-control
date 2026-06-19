// Hexapod web teleop client — plain JavaScript, no dependencies.
//
// Connects to the WebSocket server at /ws, renders two touch joysticks
// and 9 buttons, and relays input events. Coexistence with the gamepad
// is handled by a control-request prompt: when the gamepad owns, the
// webapp must explicitly claim control before its inputs take effect.

"use strict";

// ── Button display labels ──────────────────────────────────────────

const LABELS = {
  gait_mode: "Gait",
  posture_mode: "Posture",
  animation_mode: "Anim",
  init: "Init",
  record: "Rec",
  gait_prev: "Gait \u25C0",
  gait_next: "Gait \u25B6",
  animation_prev: "Anim \u25C0",
  animation_next: "Anim \u25B6",
  height_up: "\u25B2",
  height_down: "\u25BC",
  yaw_left: "Yaw \u25C0",
  yaw_right: "Yaw \u25B6",
  "": "",
};

function labelFor(fn) {
  return LABELS[fn] || fn;
}

// ── State ──────────────────────────────────────────────────────────

let ws = null;
let connected = false;
let manualDisconnect = false;
let arbitrationEnabled = false;
let owner = "gamepad";
let currentMode = "gait";

// ── DOM helpers ────────────────────────────────────────────────────

function $(id) {
  const el = document.getElementById(id);
  if (!el) throw new Error("element #" + id + " not found");
  return el;
}

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(msg));
  }
}

// ── WebSocket connection ───────────────────────────────────────────

function connect() {
  manualDisconnect = false;
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const url = proto + "//" + location.host + "/ws";
  ws = new WebSocket(url);

  ws.onopen = function () {
    connected = true;
    $("conn-status").className = "nav-icon connected";
    updateConnOverlay();
  };

  ws.onclose = function () {
    connected = false;
    $("conn-status").className = "nav-icon disconnected";
    setJoysticksEnabled(false);
    updateConnOverlay();
    // A manual disconnect stays down until the user reconnects.
    if (!manualDisconnect) setTimeout(connect, 2000);
  };

  ws.onerror = function () {
    if (ws) ws.close();
  };

  ws.onmessage = function (ev) {
    let msg;
    try {
      msg = JSON.parse(ev.data);
    } catch (e) {
      return;
    }
    handleMessage(msg);
  };
}

function handleMessage(msg) {
  switch (msg.type) {
    case "init":
      hideBusy();
      arbitrationEnabled = msg.arbitration_enabled;
      owner = msg.owner;
      currentMode = msg.mode;
      updateModeDisplay();
      updateButtonLabels(msg.button_labels);
      updateOwnerDisplay();
      break;
    case "busy":
      // Server already has another device; it closes the socket right
      // after. Keep the overlay up across reconnect attempts until a
      // slot frees and we receive a real "init".
      showBusy();
      break;
    case "mode":
      currentMode = msg.mode;
      updateModeDisplay();
      updateButtonLabels(msg.button_labels);
      break;
    case "owner":
      owner = msg.owner;
      updateOwnerDisplay();
      break;
    case "gait_state":
      break;
  }
}

// ── UI updates ─────────────────────────────────────────────────────

function updateModeDisplay() {
  const btns = document.querySelectorAll(".mode-btn");
  btns.forEach(function (btn, i) {
    const isActive =
      (i === 0 && currentMode === "gait") ||
      (i === 1 && currentMode === "posture") ||
      (i === 2 && currentMode === "animation");
    btn.classList.toggle("active", isActive);
  });
}

function updateButtonLabels(labels) {
  for (let i = 0; i < 9; i++) {
    const btn = document.querySelector('button[data-index="' + i + '"]');
    if (btn && i < labels.length) {
      btn.textContent = labelFor(labels[i]);
    }
  }
}

function updateOwnerDisplay() {
  // A controller is active whenever arbitration is on and the web app does
  // not own /cmd_vel. The navbar controller icon turns green in that state,
  // and the button grid is swapped for the inline take-control prompt.
  const controllerActive = arbitrationEnabled && owner !== "web";
  $("controller-btn").classList.toggle("active", controllerActive);
  $("control-prompt").classList.toggle("hidden", !controllerActive);
  $("button-grid").classList.toggle("hidden", controllerActive);
  setJoysticksEnabled(!controllerActive);
  updateControllerOverlay();
}

function setJoysticksEnabled(enabled) {
  $("left-joystick").classList.toggle("disabled", !enabled);
  $("right-joystick").classList.toggle("disabled", !enabled);
}

// \u2500\u2500 Controller status overlay (navbar controller icon) \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500

function updateControllerOverlay() {
  const status = $("controller-status");
  const toggle = $("controller-toggle");
  if (!arbitrationEnabled) {
    status.textContent = "Arbitration disabled \u2014 web is always in control.";
    toggle.classList.add("hidden");
    return;
  }
  toggle.classList.remove("hidden");
  if (owner === "web") {
    status.textContent = "The web app is in control.";
    toggle.textContent = "Switch to controller";
  } else {
    status.textContent = "A controller is active.";
    toggle.textContent = "Take control";
  }
}

function showControllerOverlay() {
  updateControllerOverlay();
  $("controller-overlay").classList.remove("hidden");
}

function hideControllerOverlay() {
  $("controller-overlay").classList.add("hidden");
}

// ── Connection overlay (navbar connection icon) ────────────────────

function updateConnOverlay() {
  $("conn-host").textContent = location.host;
  const state = $("conn-state");
  const toggle = $("conn-toggle");
  if (connected) {
    state.textContent = "Connected to";
    toggle.textContent = "Disconnect";
  } else {
    state.textContent = "Disconnected from";
    toggle.textContent = "Reconnect";
  }
}

function showConnOverlay() {
  updateConnOverlay();
  $("conn-overlay").classList.remove("hidden");
}

function hideConnOverlay() {
  $("conn-overlay").classList.add("hidden");
}

// ── Busy overlay (another device already connected) ────────────────

function showBusy() {
  $("busy-overlay").classList.remove("hidden");
}

function hideBusy() {
  $("busy-overlay").classList.add("hidden");
}

// ── Button handling ────────────────────────────────────────────────

function setupButtons() {
  for (let i = 0; i < 9; i++) {
    const btn = document.querySelector('button[data-index="' + i + '"]');
    if (!btn) continue;

    const press = function (e) {
      e.preventDefault();
      btn.classList.add("pressed");
      send({ type: "button", index: i, pressed: true });
    };
    const release = function (e) {
      e.preventDefault();
      btn.classList.remove("pressed");
      send({ type: "button", index: i, pressed: false });
    };

    btn.addEventListener("touchstart", press, { passive: false });
    btn.addEventListener("touchend", release, { passive: false });
    btn.addEventListener("touchcancel", release, { passive: false });
    btn.addEventListener("mousedown", press);
    btn.addEventListener("mouseup", release);
    btn.addEventListener("mouseleave", function (e) {
      if (btn.classList.contains("pressed")) release(e);
    });
  }

  // Inline take-control prompt (shown in place of the button grid).
  $("take-control-btn").addEventListener("click", function () {
    send({ type: "request_control" });
  });

  // Navbar controller icon → status overlay with a toggle.
  $("controller-btn").addEventListener("click", showControllerOverlay);
  $("controller-close").addEventListener("click", hideControllerOverlay);
  $("controller-toggle").addEventListener("click", function () {
    send({ type: owner === "web" ? "release_control" : "request_control" });
    hideControllerOverlay();
  });

  // Navbar connection icon → host/disconnect popover.
  $("conn-status").addEventListener("click", showConnOverlay);
  $("conn-close").addEventListener("click", hideConnOverlay);
  $("conn-toggle").addEventListener("click", function () {
    if (connected) {
      manualDisconnect = true;
      if (ws) ws.close();
    } else {
      connect();
    }
    hideConnOverlay();
  });

  // Navbar log icon → log page.
  $("log-btn").addEventListener("click", function () {
    location.href = "logs.html";
  });
}

// ── Touch joystick ─────────────────────────────────────────────────

class TouchJoystick {
  constructor(canvasId, stick) {
    this.canvas = $(canvasId);
    const ctx = this.canvas.getContext("2d");
    if (!ctx) throw new Error("2d context unavailable");
    this.ctx = ctx;
    this.stick = stick;
    this.active = false;
    this.touchId = null;
    this.knobX = 0;
    this.knobY = 0;

    this.canvas.addEventListener("touchstart", this.onStart.bind(this), {
      passive: false,
    });
    this.canvas.addEventListener("touchmove", this.onMove.bind(this), {
      passive: false,
    });
    this.canvas.addEventListener("touchend", this.onEnd.bind(this), {
      passive: false,
    });
    this.canvas.addEventListener("touchcancel", this.onEnd.bind(this), {
      passive: false,
    });

    this.canvas.addEventListener("mousedown", this.onMouseDown.bind(this));

    window.addEventListener("resize", this.resize.bind(this));
    this.resize();
    this.draw();
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = rect.width * dpr;
    this.canvas.height = rect.height * dpr;
    this.ctx.scale(dpr, dpr);
    this.centerX = rect.width / 2;
    this.centerY = rect.height / 2;
    this.radius = rect.width / 2 - 4;
    this.draw();
  }

  onStart(e) {
    e.preventDefault();
    if (this.active) return;
    const t = e.changedTouches[0];
    this.touchId = t.identifier;
    this.active = true;
    this.update(t.clientX, t.clientY);
  }

  onMove(e) {
    e.preventDefault();
    if (!this.active || this.touchId === null) return;
    for (let i = 0; i < e.touches.length; i++) {
      if (e.touches[i].identifier === this.touchId) {
        this.update(e.touches[i].clientX, e.touches[i].clientY);
        return;
      }
    }
  }

  onEnd(e) {
    e.preventDefault();
    if (!this.active || this.touchId === null) return;
    this.active = false;
    this.touchId = null;
    this.knobX = 0;
    this.knobY = 0;
    send({ type: "stick", stick: this.stick, x: 0, y: 0 });
    this.draw();
  }

  onMouseDown(e) {
    e.preventDefault();
    if (this.active) return;
    this.active = true;
    this.touchId = "mouse";
    this.update(e.clientX, e.clientY);
    this._mouseMove = this.onMouseMove.bind(this);
    this._mouseUp = this.onMouseUp.bind(this);
    window.addEventListener("mousemove", this._mouseMove);
    window.addEventListener("mouseup", this._mouseUp);
  }

  onMouseMove(e) {
    if (!this.active || this.touchId !== "mouse") return;
    this.update(e.clientX, e.clientY);
  }

  onMouseUp(e) {
    if (!this.active || this.touchId !== "mouse") return;
    this.active = false;
    this.touchId = null;
    this.knobX = 0;
    this.knobY = 0;
    send({ type: "stick", stick: this.stick, x: 0, y: 0 });
    this.draw();
    window.removeEventListener("mousemove", this._mouseMove);
    window.removeEventListener("mouseup", this._mouseUp);
  }

  update(clientX, clientY) {
    const rect = this.canvas.getBoundingClientRect();
    const dx = clientX - rect.left - this.centerX;
    const dy = clientY - rect.top - this.centerY;
    const dist = Math.hypot(dx, dy);
    const clampedDist = Math.min(dist, this.radius);
    const angle = Math.atan2(dy, dx);
    this.knobX = Math.cos(angle) * clampedDist;
    this.knobY = Math.sin(angle) * clampedDist;

    // REP-103: x = left = +, y = forward = +
    // Screen: x right = +, y down = +
    // So: stickX = -(knobX / radius) = left = positive
    //     stickY = -(knobY / radius) = up/forward = positive
    const sx = -(this.knobX / this.radius);
    const sy = -(this.knobY / this.radius);
    send({ type: "stick", stick: this.stick, x: sx, y: sy });
    this.draw();
  }

  draw() {
    const ctx = this.ctx;
    const r = this.radius;
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

    // Outer ring — gruvbox material border
    ctx.strokeStyle = "#45403d";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(this.centerX, this.centerY, r, 0, Math.PI * 2);
    ctx.stroke();

    // Crosshair
    ctx.strokeStyle = "#3c3836";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(this.centerX - r, this.centerY);
    ctx.lineTo(this.centerX + r, this.centerY);
    ctx.moveTo(this.centerX, this.centerY - r);
    ctx.lineTo(this.centerX, this.centerY + r);
    ctx.stroke();

    // Knob — aqua accent when active
    const knobR = Math.max(r * 0.3, 12);
    ctx.fillStyle = this.active ? "#89b482" : "#504945";
    ctx.beginPath();
    ctx.arc(
      this.centerX + this.knobX,
      this.centerY + this.knobY,
      knobR,
      0,
      Math.PI * 2
    );
    ctx.fill();
    ctx.strokeStyle = "#89b482";
    ctx.lineWidth = 2;
    ctx.stroke();
  }
}

// ── Init ───────────────────────────────────────────────────────────

function init() {
  setupButtons();
  new TouchJoystick("left-canvas", "left");
  new TouchJoystick("right-canvas", "right");
  connect();
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}
