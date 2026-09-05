"""ROS + WebSocket node for the web-app teleop.

Hosts a lightweight HTTP + WebSocket server (``aiohttp``) that serves
the static webapp and relays input events to the same ROS topics the
gamepad teleop publishes: ``/cmd_vel``, ``/body/pose``, ``/cmd_gait``,
``/animation/mode``, ``/gait/initialize``. The webapp is pure HTML +
JavaScript (no build step); the server serves the static files plus a
``/ws`` WebSocket and the ``/logs`` and ``/control/release`` endpoints.
The webapp also polls pack telemetry (``sensor_msgs/BatteryState``) over
the same WebSocket for its status strip.

Coexistence with the gamepad teleop (``hexa_teleop.teleop_joy``) is
mediated by ``/teleop/owner`` (``std_msgs/String``, TRANSIENT_LOCAL).
The web node is the sole writer. Default owner is ``gamepad``; the
webapp must explicitly request control, and the web node releases on
disconnect. See ``hexa_teleop.teleop_arbitration`` for the protocol.

Safety: two independent guards stop the robot if the link to the
webapp dies — important because a dropped phone (TCP half-open, sleep,
backgrounded tab) would otherwise leave the last stick value latched
and republished at 60 Hz.
- WebSocket heartbeat: ``aiohttp`` pings each client and force-closes a
  socket that misses its pong, which triggers the disconnect cleanup.
- Input watchdog: the 60 Hz timer feeds ``neutral_inputs`` to
  ``map_web`` whenever no stick/button message has arrived within
  ``safety.input_timeout_s``, so ``/cmd_vel`` falls to zero rather than
  latching. The disconnect path also zeroes the shared input state.

Architecture:
- Main thread: ``rclpy.spin`` with a 60 Hz timer that calls
  ``map_web`` and publishes (when web owns).
- Server thread: ``asyncio`` event loop running the ``aiohttp`` app.
- Shared state: ``threading.Lock``-protected stick/button values +
  last-input timestamp + client count + ownership flag. The WS handler
  writes; the timer reads. rclpy publishers are thread-safe, so
  ``/teleop/owner`` is published from the WS handler directly. The pack
  reading crosses the other way (executor writes, WS handler reads) as a
  single tuple swapped whole, which needs no lock.
"""

from __future__ import annotations

import asyncio
import json
import threading
import time
from pathlib import Path

import aiohttp
import aiohttp.web
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from hexa_interfaces.msg import BodyPose as BodyPoseMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty, String

from hexa_teleop.joy_mapping import JoyState
from hexa_teleop.teleop_arbitration import (
    GAMEPAD,
    WEB,
    ArbitrationState,
    web_claim,
    web_release,
)

from hexa_teleop.presets import leg_set_switch_allowed, resync

from . import captive_portal
from .web_mapping import (
    NUM_BUTTONS,
    battery_payload,
    button_labels_for_mode,
    input_is_stale,
    load_web_config,
    map_web,
    neutral_inputs,
    preset_descriptors,
    preset_payload,
    preset_pending_expired,
)

# The webapp sends one stick message per ``touchmove``, which browsers
# coalesce to the display refresh rate.
PUBLISH_RATE_HZ = 60.0
TICK_DT_S = 1.0 / PUBLISH_RATE_HZ

# "folded" and "fault" swap the strategy the next stand comes up on, leg
# set included; the rest are as in hexa_teleop's copy.
# "folding_pair" / "unfolding_pair" are deliberately absent alongside
# "initialize", "engaging" and "folding": the middle pair is between the ground
# and the folded pose, and the engine refuses a switch there. A leg-set change
# is narrower still — see presets.LEG_SET_SWITCH_STATES.
_GAIT_SWITCH_STATES: frozenset[str] = frozenset(
    {"folded", "fault", "stand", "gait", "settling", "reseating"}
)


class WebTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("web_teleop")

        default_cfg_path = (
            Path(get_package_share_directory("hexa_webteleop"))
            / "config"
            / "webteleop.yaml"
        )
        # Velocity caps (gait_node block) and the animation-mode list
        # (posture_node block) both come from hexa_description's tuning.yaml —
        # the single source of truth the gait/posture nodes also read.
        description_config = (
            Path(get_package_share_directory("hexa_description")) / "config"
        )
        tuning_yaml_path = description_config / "tuning.yaml"
        # The angular cap is the linear one over the outermost foot's standing
        # radius, so the caps need the leg mounts as well.
        geometry_yaml_path = description_config / "geometry.yaml"
        self.declare_parameter("config_file", str(default_cfg_path))
        cfg_path = Path(
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        (
            self._cfg,
            initial_mode,
            default_gait,
            self._caps,
            self._presets,
        ) = load_web_config(
            cfg_path, tuning_yaml_path, tuning_yaml_path, geometry_yaml_path
        )
        self._state = JoyState(
            mode=initial_mode,
            current_gait_idx=self._cfg.gait_cycle.index(default_gait),
            current_quadruped_gait_idx=self._cfg.quadruped_gait_cycle.index(
                self._cfg.default_quadruped_gait
            ),
        )
        self._active_gait: str = default_gait
        self._latest_gait_state: str = ""
        # The leg set the engine has APPLIED, from /gait/leg_set. Empty until
        # the first message; the Mode view lights no row rather than guessing.
        self._latest_leg_set: str = ""
        # A preset switch the operator asked for, and the monotonic instant past
        # which its silence counts as a refusal. Both cleared by /gait/leg_set
        # changing, which is the only thing that says it happened.
        self._pending_preset: str | None = None
        self._pending_deadline: float | None = None
        # Empty until something is latched on /animation/mode — the
        # pipeline is on its startup default; the UI shows a placeholder.
        self._latest_animation_mode: str = ""

        # Server config
        with cfg_path.open() as f:
            import yaml

            raw = yaml.safe_load(f)
        server_cfg = raw.get("server", {}) or {}
        self._port = int(server_cfg.get("port", 8080))
        self._portal_port = int(server_cfg.get("portal_port", 80))
        self._ws_heartbeat_s = float(server_cfg.get("ws_heartbeat_s", 5.0))
        self._input_timeout_s = float(
            (raw.get("safety", {}) or {}).get("input_timeout_s", 0.5)
        )
        self._arbitration_enabled = bool(
            raw.get("arbitration", {}).get("enabled", True)
        )
        logs_cfg = raw.get("logs", {}) or {}
        self._logs_command = str(logs_cfg.get("command", "")).strip()
        self._logs_lines = int(logs_cfg.get("lines", 200))
        telemetry_cfg = raw.get("telemetry", {}) or {}
        self._battery_topic = str(
            telemetry_cfg.get("battery_topic", "/hexa_hardware_aux/battery_state")
        )
        self._battery_poll_s = float(telemetry_cfg.get("poll_period_s", 2.0))
        self._battery_stale_after_s = float(
            telemetry_cfg.get("stale_after_s", 10.0)
        )
        self._web_dir = str(
            Path(get_package_share_directory("hexa_webteleop")) / "web"
        )

        self.get_logger().info(f"loaded web teleop config from {cfg_path}")
        self.get_logger().info(f"mode={self._state.mode}")
        self.get_logger().info(
            f"gait rotation: {list(self._cfg.gait_cycle)}"
        )
        self.get_logger().info(
            f"animation list: {list(self._cfg.animation_list)}"
        )

        # Shared input state (WS thread writes, timer reads)
        self._lock = threading.Lock()
        self._left_stick: tuple[float, float] = (0.0, 0.0)
        self._right_stick: tuple[float, float] = (0.0, 0.0)
        self._buttons: tuple[int, ...] = (0,) * NUM_BUTTONS
        # Safety watchdog: monotonic time of the last stick/button message.
        # Seeded to 0.0 so input reads stale until the first message lands.
        self._last_input_monotonic = 0.0
        self._input_stale = True

        # Latest pack telemetry as ``(volts, amps, monotonic_stamp)``, or None
        # until the first reading. Written by the executor thread, read by the
        # server thread answering a poll: one tuple swapped whole, so the read
        # is consistent without taking ``_lock``. Stamped off the monotonic
        # clock, like the input watchdog — an NTP step must not age a reading.
        self._battery: tuple[float, float, float] | None = None

        # Arbitration + client tracking
        self._arbitration = ArbitrationState()
        self._client_count = 0
        self._web_owns = False

        # ROS publishers / subscriptions
        self._pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_body_pose = self.create_publisher(BodyPoseMsg, "/body/pose", 10)
        self._pub_init = self.create_publisher(Empty, "/gait/initialize", 10)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_cmd_gait = self.create_publisher(String, "/cmd_gait", latched_qos)
        self._pub_animation_mode = self.create_publisher(
            String, "/animation/mode", latched_qos
        )
        self._pub_owner = self.create_publisher(String, "/teleop/owner", latched_qos)
        # TRANSIENT_LOCAL to match hexa_locomotion's latched publisher:
        # /gait/state publishes on change only, so a late-joining node
        # would otherwise wait for the next state change to learn the state.
        self._sub_gait_state = self.create_subscription(
            String, "/gait/state", self._on_gait_state, latched_qos
        )
        # The latched command topics are the only truth for the current
        # gait / animation selection (locomotion publishes no name
        # feedback), and both teleops write them. Subscribing — own
        # publishes included, via loopback — gives display and the
        # caps/cycler resync one code path whichever teleop switched.
        self._sub_cmd_gait = self.create_subscription(
            String, "/cmd_gait", self._on_cmd_gait, latched_qos
        )
        # The leg set the engine has APPLIED. The Mode view's only source of
        # truth: /cmd_gait is latched, so a request the engine refused would sit
        # on it forever and the view would show a preset the robot never took.
        self._sub_leg_set = self.create_subscription(
            String, "/gait/leg_set", self._on_leg_set, latched_qos
        )
        self._sub_animation_mode = self.create_subscription(
            String, "/animation/mode", self._on_animation_mode, latched_qos
        )
        # Pack telemetry for the status strip. Sensor QoS (best-effort) to
        # match hexa_hardware's publisher — a reliable reader would never
        # match it. Real robot only: in sim nothing publishes here and the
        # readout stays a dash.
        self._sub_battery = self.create_subscription(
            BatteryState, self._battery_topic, self._on_battery,
            qos_profile_sensor_data,
        )

        # Publish "gamepad" on startup so a dormant gamepad from a
        # previous web-node instance is released.
        if self._arbitration_enabled:
            self._pub_owner.publish(String(data=GAMEPAD))

        # Latest gait state for WS broadcast (main thread detects change,
        # schedules broadcast on the asyncio loop)
        self._last_broadcast_gait_state = ""
        self._ws_clients: list = []  # aiohttp WebSocketResponse objects
        self._ws_clients_lock = threading.Lock()

        # Start the aiohttp server in a daemon thread
        self._loop: asyncio.AbstractEventLoop | None = None
        self._server_thread = threading.Thread(
            target=self._run_server, daemon=True, name="webteleop-server"
        )
        self._server_thread.start()

        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

    # ── ROS callbacks (main thread) ──────────────────────────────────

    def _on_gait_state(self, msg: String) -> None:
        self._latest_gait_state = msg.data
        if msg.data != self._last_broadcast_gait_state:
            self._last_broadcast_gait_state = msg.data
            self._broadcast_to_clients({
                "type": "gait_state",
                "state": msg.data,
            })

    def _on_cmd_gait(self, msg: String) -> None:
        """Every gait switch on the wire — the gamepad's, or our own
        accepted publish heard back via loopback. All the bookkeeping
        lives here (``_tick`` only gates and publishes): stick caps,
        cycler index, and the client display. Runs on the same
        single-threaded executor as ``_tick``, so ``_cfg`` / ``_state``
        need no lock.
        """
        name = msg.data
        if name == self._active_gait:
            return
        result = resync(name, self._cfg, self._state, self._caps, self._presets)
        if result is None:
            self.get_logger().warning(
                f"/cmd_gait={name!r} unknown to velocity caps — keeping "
                f"{self._active_gait!r}"
            )
            return
        self._cfg = result.cfg
        self._active_gait = name
        self.get_logger().info(
            f"stick linear_max={result.cfg.gait_linear_max:.3f} m/s, "
            f"angular_max={result.cfg.gait_angular_z_max:.3f} rad/s for gait "
            f"{name!r}"
        )
        if result.left_animation_mode:
            # Animations are six-leg only, and map_joy only blocks *entering*
            # the mode — arriving on four legs has to leave it, and tell the
            # pipeline, which is still holding the last selected animation.
            self.get_logger().info("animation mode left (quadruped leg set)")
            self._pub_animation_mode.publish(String(data=""))
            self._broadcast_to_clients({
                "type": "mode",
                "mode": self._state.mode,
                "button_labels": list(
                    button_labels_for_mode(self._cfg, self._state.mode)
                ),
            })
        self._broadcast_to_clients({"type": "gait", "gait": name})

    def _on_leg_set(self, msg: String) -> None:
        """The engine's applied leg set — what the Mode view actually shows.

        A change here is the only thing that says a preset switch happened, so
        it is also what clears the pending state the view is holding.
        """
        if msg.data == self._latest_leg_set:
            return
        self._latest_leg_set = msg.data
        self._pending_preset = None
        self._pending_deadline = None
        self.get_logger().info(f"leg set is now {msg.data!r}")
        self._broadcast_preset()

    def _broadcast_preset(self, refused: str | None = None) -> None:
        self._broadcast_to_clients({
            "type": "preset",
            **preset_payload(
                self._presets, self._latest_leg_set, self._pending_preset,
                refused,
            ),
        })

    def _on_animation_mode(self, msg: String) -> None:
        if msg.data == self._latest_animation_mode:
            return
        self._latest_animation_mode = msg.data
        self._broadcast_to_clients({
            "type": "animation",
            "animation": msg.data,
        })

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery = (msg.voltage, msg.current, time.monotonic())

    def _tick(self) -> None:
        with self._lock:
            left = self._left_stick
            right = self._right_stick
            buttons = self._buttons
            web_owns = self._web_owns
            last_input = self._last_input_monotonic

        # Safety watchdog: if no input has arrived within the timeout (the
        # WebSocket dropped uncleanly, the phone slept, etc.) feed neutral
        # inputs so /cmd_vel falls to zero instead of latching the last
        # commanded velocity. map_web still runs so map_joy sees the button
        # releases and edge state stays consistent.
        stale = input_is_stale(last_input, time.monotonic(), self._input_timeout_s)
        publishing = web_owns or not self._arbitration_enabled
        if stale and not self._input_stale and publishing:
            self.get_logger().warning(
                "webapp input stale — holding zero velocity (input watchdog)"
            )
        self._input_stale = stale
        if stale:
            left, right, buttons = neutral_inputs()

        out = map_web(left, right, buttons, self._cfg, self._state, TICK_DT_S)

        # Ahead of the ownership gate: a preset switch is published regardless
        # of who owns /cmd_vel, so its deadline has to be watched the same way.
        self._expire_pending_preset()

        if out.mode_changed:
            self.get_logger().info(f"mode={self._state.mode}")
            self._broadcast_to_clients({
                "type": "mode",
                "mode": self._state.mode,
                "button_labels": list(button_labels_for_mode(self._cfg, self._state.mode)),
            })

        # Arbitration: map_web always runs (keeps prev_* fresh), but
        # publishes are gated on ownership.
        if self._arbitration_enabled and not web_owns:
            return

        if out.animation_name is not None:
            self.get_logger().info(
                f"publishing /animation/mode={out.animation_name!r}"
            )
            self._pub_animation_mode.publish(String(data=out.animation_name))
        if out.gait_select is not None:
            if self._latest_gait_state in _GAIT_SWITCH_STATES:
                self.get_logger().info(f"switching gait to {out.gait_select!r}")
                # Caps + cycler bookkeeping happens in _on_cmd_gait when
                # this publish loops back — the path gamepad switches
                # already take. Sticks run on the old cap for the tick or
                # two until then, invisible at 60 Hz.
                self._pub_cmd_gait.publish(String(data=out.gait_select))
            else:
                self.get_logger().info(
                    f"gait switch to {out.gait_select!r} dropped — "
                    f"engine in {self._latest_gait_state!r} (gait locked)"
                )
        # After the gait publish, not before: an init request carries its leg
        # set as the gait it publishes, and hexa_locomotion reads the leg set
        # off the strategy that is applied by the time it starts the ladder.
        if out.init_request:
            which = "quadruped" if out.init_quadruped else "hexapod"
            self.get_logger().info(
                f"webapp init ({which}) — publishing /gait/initialize"
            )
            self._pub_init.publish(Empty())

        stamp = self.get_clock().now().to_msg()
        twist = Twist()
        twist.linear.x = out.linear_x
        twist.linear.y = out.linear_y
        twist.angular.z = out.angular_z
        self._pub_cmd_vel.publish(twist)

        pose = BodyPoseMsg()
        pose.header.stamp = stamp
        pose.x = out.pose_x
        pose.y = out.pose_y
        pose.z = out.pose_z
        pose.yaw = out.pose_yaw
        pose.roll = out.pose_roll
        pose.pitch = out.pose_pitch
        self._pub_body_pose.publish(pose)

    # ── Server thread ─────────────────────────────────────────────────

    def _run_server(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._start_server())
        self._loop.run_forever()

    async def _start_server(self) -> None:
        app = aiohttp.web.Application(client_max_size=1024 * 1024)
        app.router.add_get("/ws", self._handle_ws)
        app.router.add_get("/logs", self._handle_logs)
        app.router.add_post("/control/release", self._handle_release)
        app.router.add_get("/", self._handle_index)
        # Catch-all, registered last so the routes above still win. On the
        # robot's hotspot every hostname resolves here, so an unrecognised path
        # is not an error — see ``captive_portal`` and ``_handle_get``.
        app.router.add_get("/{tail:.*}", self._handle_get)
        runner = aiohttp.web.AppRunner(app, access_log=None)
        await runner.setup()
        site = aiohttp.web.TCPSite(runner, "0.0.0.0", self._port)
        await site.start()
        self.get_logger().info(
            f"web teleop server on port {self._port} (web dir: {self._web_dir})"
        )
        await self._start_portal_site(runner)

    async def _start_portal_site(self, runner: aiohttp.web.AppRunner) -> None:
        """Also answer on port 80, so the hotspot's name needs no port typed.

        ``http://control.hexa/`` and an OS's connectivity probe both arrive on
        port 80, and serving it directly beats depending on the AP's nftables
        redirect (``network-mode.sh``), which needs an ``nft`` binary the host
        may not have.

        Best-effort by necessity: only a root container may bind a privileged
        port, which the robot's is and the sim's is not. A failure is one log
        line — the main port is already up, and the redirect still stands where
        it exists.
        """
        if not self._portal_port or self._portal_port == self._port:
            return
        try:
            await aiohttp.web.TCPSite(runner, "0.0.0.0", self._portal_port).start()
        except OSError as e:
            self.get_logger().info(
                f"not serving port {self._portal_port} ({e.strerror or e}) — "
                f"reach the UI on port {self._port}"
            )
            return
        self.get_logger().info(f"also serving on port {self._portal_port}")

    def _index_response(self) -> aiohttp.web.Response:
        # no-store because a captive-portal browser that cached this page would
        # show it again on the *next* network, where it controls nothing.
        return aiohttp.web.FileResponse(
            Path(self._web_dir) / "index.html",
            headers={"Cache-Control": "no-store"},
        )

    async def _handle_index(self, request: aiohttp.web.Request) -> aiohttp.web.Response:
        return self._index_response()

    async def _handle_get(self, request: aiohttp.web.Request) -> aiohttp.web.Response:
        """A webapp asset, or somebody who needs the controller.

        Nothing here 404s: on the hotspot the AP answers every hostname with
        the robot, so an unrecognised path is a person, not a mistake. See
        ``captive_portal`` for why they are redirected rather than served the
        page where they stand.
        """
        filename = captive_portal.static_filename(
            captive_portal.request_path(request.path)
        )
        if filename:
            filepath = Path(self._web_dir) / filename
            if filepath.is_file():
                return aiohttp.web.FileResponse(filepath)
        # Absolute, and to this same host: the client asked for
        # captive.apple.com (or whichever name it probes), so that is the one
        # its captive-portal browser already treats as the portal.
        raise aiohttp.web.HTTPFound(f"http://{request.host}/")

    async def _handle_logs(self, request: aiohttp.web.Request) -> aiohttp.web.Response:
        """Run the configured log command and return its last N lines."""
        if not self._logs_command:
            return aiohttp.web.json_response(
                {"lines": [], "error": "no logs.command configured"}
            )
        try:
            proc = await asyncio.create_subprocess_shell(
                self._logs_command,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )
            out, _ = await asyncio.wait_for(proc.communicate(), timeout=10.0)
            text = out.decode("utf-8", errors="replace")
        except asyncio.TimeoutError:
            return aiohttp.web.json_response(
                {"lines": [], "error": "log command timed out"}
            )
        except Exception as e:  # pragma: no cover - defensive
            return aiohttp.web.json_response({"lines": [], "error": str(e)})
        lines = text.splitlines()[-self._logs_lines :]
        return aiohttp.web.json_response({"lines": lines})

    async def _handle_release(self, request: aiohttp.web.Request) -> aiohttp.web.Response:
        """Hand control back to the gamepad (settings page action)."""
        self._release_control()
        owner = self._arbitration.owner if self._arbitration_enabled else GAMEPAD
        return aiohttp.web.json_response({"owner": owner})

    async def _handle_ws(self, request: aiohttp.web.Request) -> aiohttp.web.WebSocketResponse:
        # heartbeat: aiohttp pings each client and force-closes a socket
        # that misses its pong, so a half-open link (no clean FIN) still
        # reaches the disconnect cleanup below.
        ws = aiohttp.web.WebSocketResponse(heartbeat=self._ws_heartbeat_s)
        await ws.prepare(request)

        # Single-connection policy: only one webapp may be connected at a
        # time. Check-and-increment under one lock so two simultaneous
        # connections cannot both pass the guard.
        with self._lock:
            if self._client_count >= 1:
                busy = True
            else:
                busy = False
                self._client_count += 1

        if busy:
            self.get_logger().info(
                "webapp connection refused — another device is connected"
            )
            try:
                await ws.send_json({
                    "type": "busy",
                    "message": "Another device is already connected.",
                })
                await ws.close()
            except Exception:  # pragma: no cover - defensive
                pass
            return ws

        with self._ws_clients_lock:
            self._ws_clients.append(ws)

        self.get_logger().info(f"webapp connected ({self._client_count} client(s))")

        # Send initial state
        await ws.send_json({
            "type": "init",
            "gaits": list(self._cfg.gait_cycle),
            "animations": list(self._cfg.animation_list),
            "mode": self._state.mode,
            "button_labels": list(button_labels_for_mode(self._cfg, self._state.mode)),
            "owner": self._arbitration.owner if self._arbitration_enabled else GAMEPAD,
            "arbitration_enabled": self._arbitration_enabled,
            "gait_state": self._latest_gait_state,
            "gait": self._active_gait,
            "animation": self._latest_animation_mode,
            # The Mode view. The list is fixed at load; the rest is live state,
            # so a client reconnecting mid-switch inherits the truth instead of
            # a blank view.
            "presets": preset_descriptors(self._presets),
            **{
                f"preset_{k}": v
                for k, v in preset_payload(
                    self._presets, self._latest_leg_set, self._pending_preset,
                    None,
                ).items()
            },
            # How often the client should poll for pack telemetry. Pushed
            # rather than hard-coded in the webapp so the period stays a
            # config value; see ``_handle_ws_message``.
            "battery_poll_s": self._battery_poll_s,
        })

        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except json.JSONDecodeError:
                        continue
                    await self._handle_ws_message(data, ws)
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    self.get_logger().error(f"ws error: {ws.exception()}")
        finally:
            with self._ws_clients_lock:
                if ws in self._ws_clients:
                    self._ws_clients.remove(ws)
            with self._lock:
                self._client_count -= 1
                last_client = self._client_count == 0
                # Drop stale inputs so a fresh client can't inherit the
                # departed one's stick values, and so the watchdog reads
                # stale immediately.
                self._left_stick = (0.0, 0.0)
                self._right_stick = (0.0, 0.0)
                self._buttons = (0,) * NUM_BUTTONS
                self._last_input_monotonic = 0.0
            if last_client:
                self._release_control()
            self.get_logger().info(
                f"webapp disconnected ({self._client_count} client(s))"
            )

        return ws

    async def _handle_ws_message(self, data: dict, ws) -> None:
        msg_type = data.get("type")
        if msg_type == "stick":
            stick = data.get("stick", "")
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            # Clamp to [-1, 1]
            x = max(-1.0, min(1.0, x))
            y = max(-1.0, min(1.0, y))
            with self._lock:
                if stick == "left":
                    self._left_stick = (x, y)
                elif stick == "right":
                    self._right_stick = (x, y)
                self._last_input_monotonic = time.monotonic()
        elif msg_type == "button":
            idx = int(data.get("index", -1))
            pressed = bool(data.get("pressed", False))
            if 0 <= idx < NUM_BUTTONS:
                with self._lock:
                    btns = list(self._buttons)
                    btns[idx] = 1 if pressed else 0
                    self._buttons = tuple(btns)
                    self._last_input_monotonic = time.monotonic()
        elif msg_type == "battery":
            # Polled rather than pushed: the pack is sampled at 10 Hz on the
            # robot and the strip reads it once a second or so, so a reply per
            # ask is far less traffic than a broadcast per reading — and a
            # client that stops asking (backgrounded tab) stops the traffic.
            await ws.send_json({
                "type": "battery",
                **battery_payload(
                    self._battery, time.monotonic(), self._battery_stale_after_s
                ),
            })
        elif msg_type == "select_preset":
            self._select_preset(str(data.get("preset", "")))
        elif msg_type == "request_control":
            self._claim_control()
        elif msg_type == "release_control":
            self._release_control()

    def _select_preset(self, preset_id: str) -> None:
        """Ask the engine for a preset. Deliberately NOT gated on ownership.

        A preset switch is supervisory, not a drive input: it touches neither
        /cmd_vel nor /body/pose — the two continuous streams arbitration exists
        to stop from fighting — and is one idempotent write to a latched
        selection topic the gamepad already writes without asking anyone. The
        operator asked for it to work while a controller drives, and this is
        what makes that true.
        """
        preset = self._presets.get(preset_id)
        if preset is None:
            self.get_logger().warning(f"unknown preset {preset_id!r} requested")
            self._broadcast_preset(refused="no such mode")
            return
        if preset.leg_set == self._latest_leg_set:
            # Already there. Not an error, and not worth a wire round trip.
            self._broadcast_preset()
            return
        if not leg_set_switch_allowed(self._latest_gait_state):
            # Pre-gated here rather than left to the engine, because /cmd_gait
            # is latched: a name the engine refuses would stay on the wire and
            # every late subscriber would read a leg set the robot never took.
            self.get_logger().info(
                f"preset -> {preset_id!r} refused — engine in "
                f"{self._latest_gait_state!r}"
            )
            self._broadcast_preset(
                refused="not while walking — stop first"
                if self._latest_gait_state == "gait"
                else "the robot is busy"
            )
            return

        gait = self._presets.entry_gait(preset.id)
        owner = "web" if self._web_owns else "gamepad"
        self.get_logger().info(
            f"preset -> {preset_id!r} (gait {gait!r}), requested by the webapp "
            f"while {owner} owns /cmd_vel"
        )
        self._pending_preset = preset.id
        self._pending_deadline = (
            time.monotonic() + self._presets.switch_timeout_s
        )
        self._pub_cmd_gait.publish(String(data=gait))
        self._broadcast_preset()

    def _expire_pending_preset(self) -> None:
        """Silence past the deadline is a refusal.

        /gait/leg_set publishes on change only, so a switch the node could not
        rule out — the engine's state moved between the tap and the tick, or the
        operator's body pose never came back to neutral — arrives as nothing at
        all. This is what turns that into an answer.
        """
        if not preset_pending_expired(self._pending_deadline, time.monotonic()):
            return
        asked = self._pending_preset
        self._pending_preset = None
        self._pending_deadline = None
        self.get_logger().warning(
            f"preset -> {asked!r} never took effect — the engine refused it, "
            f"or the body pose never returned to neutral"
        )
        self._broadcast_preset(refused="the robot did not switch")

    def _claim_control(self) -> None:
        with self._lock:
            if self._web_owns:
                return
            self._web_owns = True
            owner = web_claim(self._arbitration)
        self.get_logger().info("webapp claimed /cmd_vel ownership")
        self._pub_owner.publish(String(data=owner))
        self._broadcast_to_clients({"type": "owner", "owner": WEB})

    def _release_control(self) -> None:
        with self._lock:
            if not self._web_owns:
                return
            self._web_owns = False
            owner = web_release(self._arbitration)
        self.get_logger().info("webapp released /cmd_vel ownership")
        self._pub_owner.publish(String(data=owner))
        self._broadcast_to_clients({"type": "owner", "owner": GAMEPAD})

    def _broadcast_to_clients(self, msg: dict) -> None:
        """Schedule a JSON broadcast to all WS clients on the asyncio loop."""
        if self._loop is None or not self._loop.is_running():
            return
        asyncio.run_coroutine_threadsafe(
            self._async_broadcast(msg), self._loop
        )

    async def _async_broadcast(self, msg: dict) -> None:
        text = json.dumps(msg)
        with self._ws_clients_lock:
            clients = list(self._ws_clients)
        for ws in clients:
            try:
                await ws.send_str(text)
            except Exception:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
