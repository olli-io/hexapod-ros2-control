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
  init: "Stand\n(Hexa)",
  record: "Save\nposture",
  gait_prev: "Prev\nGait",
  gait_next: "Next\nGait",
  animation_prev: "Prev\nAnim",
  animation_next: "Next\nAnim",
  height_up: "Body\nup",
  height_down: "Body\ndown",
  yaw_left: "Yaw\nleft",
  yaw_right: "Yaw\nright",
  quadruped_mode: "Stand\n(Quad)",
  "": "",
};

function labelFor(fn) {
  return LABELS[fn] || fn;
}

// Status-strip names for the animation-mode animations. Display only —
// the wire names on /animation/mode stay the long ones.
const ANIM_LABELS = {
  vertical_body_roll: "wave",
  horizontal_body_roll: "snake",
  body_roll_3d: "spiral",
};

function animationLabel(name) {
  return ANIM_LABELS[name] || name;
}

// ── State ──────────────────────────────────────────────────────────

let ws = null;
let connected = false;
let manualDisconnect = false;
let arbitrationEnabled = false;
let owner = "gamepad";
let currentMode = "gait";
// Names as latched on /cmd_gait and /animation/mode; empty animation
// means nothing latched yet (pipeline startup default) → placeholder.
let currentGait = "";
let currentAnimation = "";
// Latest /gait/state. Only the folded case is read here: no gait is running
// on the belly, so the strategy the next stand will use is not a status.
let currentGaitState = "";
// The Mode view. `presets` is fixed at connect; `activePreset` / `activeLegSet`
// come from /gait/leg_set and NOTHING else — never from the tap, never from the
// latched /cmd_gait, which keeps a refused name forever. `pendingPreset` is what
// was asked for and has not landed yet.
let presets = [];
let activePreset = null;
let activeLegSet = null;
let pendingPreset = null;
let refusedTimer = null;
// Pack telemetry, polled over the WebSocket. Null means unknown — never
// heard, stale, or a non-finite reading — and shows as a dash.
let packVoltage = null;
let packCurrent = null;
let packTimer = null;
// The active mode's button functions, as sent with "init" and every "mode".
// Kept so the Mode view's STAND can find the grid slot bound to `init` instead
// of hardcoding an index the config owns.
let buttonLabels = [];
// Set once in init(); module-scoped because the Mode view re-centres them on
// the way in — the sticks leave the screen, and a knob held at that moment
// would otherwise never see its own touchend.
let joysticks = [];
// The tab now showing: "control", "preset", "network" or "log". The tab bar is
// the only navigation — every item swaps this and nothing else — so a view
// never needs a way out of its own.
let currentView = "control";

// Re-send held input this often (ms) so the server's input watchdog
// (safety.input_timeout_s, default 500 ms) doesn't zero /cmd_vel while a
// stick or button is held stationary — move/press events only fire on
// change, so without this the command latches for one tick then drops.
// Kept well under the watchdog window (~10 sends per timeout); stops when
// the tab suspends (the timer suspends too), so the watchdog still guards
// a dropped link.
const KEEPALIVE_MS = 50;

// How long the Mode view's STAND holds the init button down (ms). Several node
// ticks, and well under the input watchdog.
const STAND_PRESS_MS = 150;

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

// Short haptic tick on button press (no-op where unsupported, e.g. iOS).
function buzz(ms) {
  if (navigator.vibrate) navigator.vibrate(ms);
}

// ── WebSocket connection ───────────────────────────────────────────

function connect() {
  manualDisconnect = false;
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const url = proto + "//" + location.host + "/ws";
  ws = new WebSocket(url);

  ws.onopen = function () {
    connected = true;
    applyConnState();
    updateNetworkView();
  };

  ws.onclose = function () {
    connected = false;
    applyConnState();
    setJoysticksEnabled(false);
    stopPackPolling();
    updateNetworkView();
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
      currentGait = msg.gait;
      currentAnimation = msg.animation;
      currentGaitState = msg.gait_state;
      presets = msg.presets || [];
      activePreset = msg.preset_active || null;
      activeLegSet = msg.preset_leg_set || null;
      pendingPreset = msg.preset_pending || null;
      renderPresetList();
      updatePresetDisplay();
      updateModeDisplay();
      updateButtonLabels(msg.button_labels);
      updateOwnerDisplay();
      updateStatusDisplay();
      startPackPolling(msg.battery_poll_s);
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
    case "gait":
      currentGait = msg.gait;
      updateStatusDisplay();
      break;
    case "animation":
      currentAnimation = msg.animation;
      updateStatusDisplay();
      break;
    case "gait_state":
      currentGaitState = msg.state;
      updateStatusDisplay();
      updatePresetDisplay();
      break;
    case "preset":
      activePreset = msg.active || null;
      activeLegSet = msg.leg_set || null;
      pendingPreset = msg.pending || null;
      updatePresetDisplay();
      showPresetRefusal(msg.refused || null);
      break;
    case "battery":
      // Either field is null when the node has no fresh reading to give.
      packVoltage = typeof msg.voltage === "number" ? msg.voltage : null;
      packCurrent = typeof msg.current === "number" ? msg.current : null;
      updateStatusDisplay();
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
  buttonLabels = labels || [];
  for (let i = 0; i < 9; i++) {
    const btn = document.querySelector('button[data-index="' + i + '"]');
    if (btn && i < buttonLabels.length) {
      btn.textContent = labelFor(buttonLabels[i]);
    }
  }
  // The Mode view's STAND presses one of these, and which slot carries `init`
  // is per-mode config.
  updatePresetDisplay();
}

function updateOwnerDisplay() {
  // A controller is active whenever arbitration is on and the web app does
  // not own /cmd_vel. The Control tab turns green in that state — it is the
  // tab whose contents change — and the button grid is swapped for the inline
  // take-control prompt.
  const controllerActive = arbitrationEnabled && owner !== "web";
  $("tab-control").classList.toggle("controlled", controllerActive);
  $("control-prompt").classList.toggle("hidden", !controllerActive);
  $("button-grid").classList.toggle("hidden", controllerActive);
  setJoysticksEnabled(!controllerActive);
  updateNetworkView();
}

function updateStatusDisplay() {
  const folded = currentGaitState === "folded";
  $("status-preset").textContent = presetLabel(activePreset) || "—";
  $("status-gait").textContent = (!folded && currentGait) || "—";
  $("status-animation").textContent = animationLabel(currentAnimation) || "—";
  $("status-battery").textContent = packText();
}

// Pack readout: both values, or a single dash while neither is known.
function packText() {
  if (packVoltage === null && packCurrent === null) return "—";
  const v = packVoltage === null ? "— V" : packVoltage.toFixed(2) + " V";
  const a = packCurrent === null ? "— A" : packCurrent.toFixed(2) + " A";
  return v + "  " + a;
}

// ── Pack telemetry polling ────────────────────────────────

// Ask the node for voltage/current every `periodS` (its config value, sent
// with "init"). Polled rather than pushed because the pack is sampled at
// 10 Hz on the robot and the strip needs it about once a second.
function startPackPolling(periodS) {
  stopPackPolling();
  const poll = function () {
    send({ type: "battery" });
  };
  poll();
  packTimer = setInterval(poll, Math.max(200, (periodS || 1) * 1000));
}

function stopPackPolling() {
  if (packTimer !== null) {
    clearInterval(packTimer);
    packTimer = null;
  }
  // A dropped link makes the last reading meaningless, not merely old.
  packVoltage = null;
  packCurrent = null;
  updateStatusDisplay();
}

function setJoysticksEnabled(enabled) {
  $("left-joystick").classList.toggle("disabled", !enabled);
  $("right-joystick").classList.toggle("disabled", !enabled);
}

// ── Tab navigation ─────────────────────────────────────────────────
//
// Every menu item is a tab: it swaps the view above the bar, the bar stays put,
// and the Control tab is the way back — so no view carries a back arrow. All
// four views live in this page rather than in pages of their own because the
// WebSocket is the page: navigating away drops it, and the server hands its one
// client slot to whoever reconnects.

const VIEWS = {
  control: "control-area",
  preset: "preset-view",
  network: "network-view",
  log: "log-view",
};

function showView(name) {
  if (!VIEWS[name]) return;
  const leavingControl = currentView === "control" && name !== "control";
  currentView = name;

  // The sticks are about to disappear from under the operator's thumbs, and a
  // knob held at that moment would never see its own touchend.
  if (leavingControl) {
    joysticks.forEach(function (j) {
      j.reset();
    });
  }

  Object.keys(VIEWS).forEach(function (key) {
    $(VIEWS[key]).classList.toggle("hidden", key !== name);
  });
  document.querySelectorAll("#navbar .nav-icon").forEach(function (tab) {
    tab.classList.toggle("tab-active", tab.dataset.view === name);
  });

  if (name === "control") {
    // The canvases were display:none, so their measured size was zero.
    joysticks.forEach(function (j) {
      j.resize();
    });
  } else if (name === "preset") {
    updatePresetDisplay();
  } else if (name === "network") {
    updateNetworkView();
  } else if (name === "log") {
    loadLogs();
  }
}


// ── Mode view (Mode tab) ───────────────────────────────────────────
//
// Pending and refused are live states only the WebSocket carries, which is why
// this is a view in this page and not a page of its own.
//
// The active row is driven by /gait/leg_set alone. A tap sets nothing locally:
// an optimistic row would show a mode the robot may refuse, and on a latched
// command topic that lie has nothing to correct it.

function renderPresetList() {
  const list = $("preset-list");
  list.textContent = "";
  presets.forEach(function (preset) {
    const btn = document.createElement("button");
    btn.className = "preset-item";
    btn.dataset.preset = preset.id;

    const label = document.createElement("span");
    label.className = "preset-label";
    label.textContent = preset.label;
    btn.appendChild(label);

    if (preset.sub) {
      const sub = document.createElement("span");
      sub.className = "preset-sub";
      sub.textContent = preset.sub;
      btn.appendChild(sub);
    }

    btn.addEventListener("click", function () {
      buzz(15);
      send({ type: "select_preset", preset: preset.id });
    });
    list.appendChild(btn);
  });
  updatePresetDisplay();
}

function updatePresetDisplay() {
  const folded = currentGaitState === "folded";
  const rows = document.querySelectorAll("#preset-list .preset-item");
  rows.forEach(function (row) {
    const id = row.dataset.preset;
    row.classList.toggle("active", id === activePreset);
    row.classList.toggle("pending", id === pendingPreset);
    // Everything is inert while a switch is in flight: the robot is moving its
    // legs, and a second request would be refused anyway. Inert on the belly
    // too — a stand carries its own LEG SET, which the engine resolves to a
    // preset, so a four-corner mode picked here would be overwritten the moment
    // the robot got up.
    row.disabled = folded || pendingPreset !== null;
  });
  updateStandButton(folded);
  // The navbar icon doubles as the readout, so the current mode is legible
  // without opening anything.
  $("tab-preset").classList.toggle("quad", activeLegSet === "quadruped");
  $("tab-preset").classList.toggle("pending", pendingPreset !== null);
  // The strip carries the name the icon cannot: the icon is the leg set, and
  // three of the four presets share one.
  updateStatusDisplay();
}

// A preset id -> the label the descriptors gave it. A dash until /gait/preset
// has spoken, which is honest — the view lights no row then either.
function presetLabel(id) {
  if (!id) return "";
  for (let i = 0; i < presets.length; i++) {
    if (presets[i].id === id) return presets[i].label;
  }
  return id;
}

// The belly's one action, in place of the inert rows. It presses the grid's
// `init` button rather than talking to the node directly: standing up IS that
// button, two-press revert and all, and duplicating it here would be a second
// path into the same state machine.
//
// Not gated on ownership, like the mode rows either side of it: the node
// publishes an init whoever owns /cmd_vel, because a stand is one discrete
// command and not a stream to fight over. It is also the only stand reachable
// while a controller drives — the grid it presses is a take-control prompt then.
function updateStandButton(folded) {
  const btn = $("preset-stand");
  btn.classList.toggle("hidden", !folded);
  if (!folded) return;
  btn.disabled = standIndex() < 0;
  $("preset-stand-sub").textContent = "stand up to choose a mode";
}

// The grid slot bound to `init` in the mode now showing. -1 if none is.
function standIndex() {
  return buttonLabels.indexOf("init");
}

function showPresetRefusal(reason) {
  const note = $("preset-note");
  if (!reason) {
    note.textContent = "";
    note.classList.remove("refused");
    return;
  }
  note.textContent = reason;
  note.classList.add("refused");
  if (refusedTimer !== null) clearTimeout(refusedTimer);
  refusedTimer = setTimeout(function () {
    note.textContent = "";
    note.classList.remove("refused");
    refusedTimer = null;
  }, 4000);
}

// ── Network view (link + control handover) ─────────────────────────
//
// One view for both, because they are one question: which input the robot is
// listening to, and whether this device can reach it at all. Replaces the two
// navbar popovers — a modal over the joysticks sat in front of a control the
// operator still had a thumb on.

function applyConnState() {
  // classList, not className: the tab-active class lives on this element too.
  const icon = $("tab-network");
  icon.classList.toggle("connected", connected);
  icon.classList.toggle("disconnected", !connected);

  // With the socket down the control area commands nothing and the preset
  // rows report a stale robot, so the bar keeps only the two tabs that still
  // work: Network, to get the link back, and Log, which is fetched over plain
  // HTTP and is where the reason for the drop shows up. The view follows
  // unless it is already one of them, so nobody is left on a dead screen with
  // no way back.
  document.querySelectorAll("#navbar .nav-icon").forEach(function (tab) {
    const usable = tab.dataset.view === "network" || tab.dataset.view === "log";
    tab.classList.toggle("hidden", !connected && !usable);
  });
  if (!connected && currentView !== "log") showView("network");
}

function updateNetworkView() {
  $("conn-host").textContent = location.host;
  $("conn-state").textContent = connected ? "Connected to" : "Disconnected from";
  $("conn-toggle").textContent = connected ? "Disconnect" : "Reconnect";

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
    toggle.textContent = "Give control to controller";
  } else {
    status.textContent = "A controller is active.";
    toggle.textContent = "Take control";
  }
}

// ── Log view ───────────────────────────────────────────────────────
//
// Recent log output from GET /logs, loaded when the tab opens and on demand.
// Not polled: it is a thing you go and read, and the socket next to it is
// carrying control input.

async function loadLogs() {
  const view = $("logs-view");
  view.textContent = "Loading\u2026";
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
      buzz(15);
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

  // The Mode view's STAND (folded only). Held for a beat rather than released
  // in the same breath: the node samples the button state on its 60 Hz tick, so
  // a press and release inside one tick window is no press at all.
  $("preset-stand").addEventListener("click", function () {
    const idx = standIndex();
    if (idx < 0) return;
    buzz(15);
    send({ type: "button", index: idx, pressed: true });
    setTimeout(function () {
      send({ type: "button", index: idx, pressed: false });
    }, STAND_PRESS_MS);
  });

  // Inline take-control prompt (shown in place of the button grid).
  $("take-control-btn").addEventListener("click", function () {
    send({ type: "request_control" });
  });

  // Network view: the link toggle, and the control handover next to it. The
  // handover is deliberately outside the take-control gate — it IS the gate.
  $("conn-toggle").addEventListener("click", function () {
    if (connected) {
      manualDisconnect = true;
      if (ws) ws.close();
    } else {
      connect();
    }
  });
  $("controller-toggle").addEventListener("click", function () {
    send({ type: owner === "web" ? "release_control" : "request_control" });
  });

  // Log view: manual refresh (the tab load is in showView).
  $("log-refresh").addEventListener("click", loadLogs);

  // Tab bar. Nothing here is gated on ownership: every view but Control is
  // supervisory, and the Mode view is meant to work while a controller drives.
  document.querySelectorAll("#navbar .nav-icon").forEach(function (tab) {
    tab.addEventListener("click", function () {
      showView(tab.dataset.view);
    });
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
    // Last stick value sent, re-sent by the keepalive while active.
    this.lastX = 0;
    this.lastY = 0;

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
    this.reset();
  }

  // Re-centre the knob and command zero. Used on touch/mouse release and
  // when the page is hidden (safety stop).
  reset() {
    this.active = false;
    this.touchId = null;
    this.knobX = 0;
    this.knobY = 0;
    this.sendStick(0, 0);
    this.draw();
  }

  // Send a stick value and remember it so the keepalive can re-send it
  // while the knob is held stationary.
  sendStick(x, y) {
    this.lastX = x;
    this.lastY = y;
    send({ type: "stick", stick: this.stick, x: x, y: y });
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
    this.reset();
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
    this.sendStick(sx, sy);
    this.draw();
  }

  draw() {
    const ctx = this.ctx;
    const r = this.radius;
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

    // Outer ring — grey1, bright enough to read on a phone outdoors.
    ctx.strokeStyle = "#928374";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(this.centerX, this.centerY, r, 0, Math.PI * 2);
    ctx.stroke();

    // Crosshair
    ctx.strokeStyle = "#5a524c";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(this.centerX - r, this.centerY);
    ctx.lineTo(this.centerX + r, this.centerY);
    ctx.moveTo(this.centerX, this.centerY - r);
    ctx.lineTo(this.centerX, this.centerY + r);
    ctx.stroke();

    // Knob — aqua accent when active
    const knobR = Math.max(r * 0.3, 12);
    ctx.fillStyle = this.active ? "#89b482" : "#665c54";
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
  joysticks = [
    new TouchJoystick("left-canvas", "left"),
    new TouchJoystick("right-canvas", "right"),
  ];

  // Keepalive: the client is event-driven (stick moves and button presses
  // only fire on change), but the server zeros /cmd_vel if no message
  // arrives within its input-timeout window. Re-send whatever is currently
  // held — active sticks and pressed buttons (height/yaw integrate while
  // held) — so a stationary hold stays commanded.
  setInterval(function () {
    joysticks.forEach(function (j) {
      if (j.active) j.sendStick(j.lastX, j.lastY);
    });
    const held = document.querySelectorAll("#button-grid button.pressed");
    held.forEach(function (btn) {
      send({ type: "button", index: Number(btn.dataset.index), pressed: true });
    });
  }, KEEPALIVE_MS);

  // Safety stop: if the page is hidden (tab switch, screen lock, app
  // backgrounded) re-centre both sticks so the robot doesn't keep the
  // last velocity. The server-side input watchdog is the backstop for
  // when the browser suspends before this can fire.
  document.addEventListener("visibilitychange", function () {
    if (document.visibilityState === "hidden") {
      joysticks.forEach(function (j) {
        j.reset();
      });
    }
  });

  showView("control");
  connect();
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}
