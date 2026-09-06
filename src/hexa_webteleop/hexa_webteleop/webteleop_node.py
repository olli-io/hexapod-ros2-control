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
  ``map_web`` whenever no stick/action message has arrived within
  ``safety.input_timeout_s``, so ``/cmd_vel`` falls to zero rather than
  latching. The disconnect path also zeroes the shared input state.

Architecture:
- Main thread: ``rclpy.spin`` with a 60 Hz timer that calls
  ``map_web`` and publishes (when web owns).
- Server thread: ``asyncio`` event loop running the ``aiohttp`` app.
- Shared state: ``threading.Lock``-protected stick/action values +
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

from hexa_teleop.joy_mapping import ANIMATION, GAIT, JoyState
from hexa_teleop.teleop_arbitration import (
    GAMEPAD,
    WEB,
    ArbitrationState,
    web_claim,
    web_release,
)

from hexa_teleop.presets import (
    preset_switch_allowed,
    resync_gait,
    resync_preset,
    resync_preset_request,
)

from . import captive_portal
from .web_mapping import (
    ACTIONS,
    battery_payload,
    gait_selectable,
    input_is_stale,
    load_animation_preset,
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
            self._sticks,
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
        # The preset animation mode is pinned to. Read from the raw config
        # rather than threaded through ``load_web_config``'s tuple: it is the
        # web node's own policy, not part of the shared JoyConfig the mapping
        # runs on.
        self._animation_preset: str | None = load_animation_preset(
            raw, self._presets
        )
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
        if self._animation_preset is not None:
            self.get_logger().info(
                f"animation mode is available on preset "
                f"{self._animation_preset!r} only"
            )

        # Shared input state (WS thread writes, timer reads)
        self._lock = threading.Lock()
        self._left_stick: tuple[float, float] = (0.0, 0.0)
        self._right_stick: tuple[float, float] = (0.0, 0.0)
        # The functions the operator is holding, named — never an index.
        self._actions: frozenset[str] = frozenset()
        # Safety watchdog: monotonic time of the last stick/action message.
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
        # The operator preset. The Mode view is the only thing that writes it —
        # nothing on either pad asks for a preset change, only the init buttons'
        # leg set — and both teleops read it back for the posture revert.
        self._pub_cmd_preset = self.create_publisher(
            String, "/cmd_preset", latched_qos
        )
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
        self._sub_cmd_preset = self.create_subscription(
            String, "/cmd_preset", self._on_cmd_preset, latched_qos
        )
        # The leg set the engine has APPLIED — the navbar icon's source, and
        # only that: three of the four presets stand on six legs, so it cannot
        # say which preset is in force.
        self._sub_leg_set = self.create_subscription(
            String, "/gait/leg_set", self._on_leg_set, latched_qos
        )
        # The PRESET the engine has applied. The Mode view's only source of
        # truth: /cmd_preset is latched, so a request the engine refused would
        # sit on it forever and the view would show a preset the robot never
        # took.
        self._sub_gait_preset = self.create_subscription(
            String, "/gait/preset", self._on_gait_preset, latched_qos
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
        result = resync_gait(name, self._cfg, self._state, self._presets)
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
        self._broadcast_to_clients({"type": "gait", "gait": name})

    def _on_leg_set(self, msg: String) -> None:
        """The engine's applied leg set.

        The navbar icon's source — it dims the middle pair on four legs — and
        nothing else. Which PRESET is in force is /gait/preset's answer, below:
        three of the four presets stand on six legs, so the leg set cannot tell
        them apart.
        """
        if msg.data == self._latest_leg_set:
            return
        self._latest_leg_set = msg.data
        self.get_logger().info(f"leg set is now {msg.data!r}")
        self._broadcast_preset()

    def _on_cmd_preset(self, msg: String) -> None:
        """A preset REQUEST — ours, heard back, or the gamepad's.

        Eases the operator's recorded posture out. The engine will not start the
        change until the body pose is back at neutral, and this decay is the
        only thing that puts it there — so it has to follow the request, not the
        report.
        """
        if resync_preset_request(msg.data, self._state, self._presets) is None:
            return
        self.get_logger().info(
            f"preset change to {msg.data!r} requested — reverting the recorded "
            f"posture (the engine holds the change until the body is neutral)"
        )

    def _on_gait_preset(self, msg: String) -> None:
        """The engine's applied preset — what the Mode view actually shows.

        A change here is the only thing that says a preset switch happened, so
        it is also what clears the pending state the view is holding. And it is
        where the caps swap: every one is derived from the preset's stride,
        swing time and stance.
        """
        result = resync_preset(
            msg.data, self._active_gait, self._cfg, self._state, self._presets
        )
        if result is None:
            self.get_logger().warning(
                f"/gait/preset={msg.data!r} is not declared in this config — "
                f"keeping preset {self._presets.current_id()!r}"
            )
            return
        self._cfg = result.cfg
        self._pending_preset = None
        self._pending_deadline = None
        self.get_logger().info(
            f"preset is now {msg.data!r}: stick linear_max="
            f"{result.cfg.gait_linear_max:.3f} m/s, angular_max="
            f"{result.cfg.gait_angular_z_max:.3f} rad/s"
        )
        if result.left_animation_mode:
            # Animations are six-leg only, and the mapping only blocks *entering*
            # the mode — arriving on four legs has to leave it, and tell the
            # pipeline, which is still holding the last selected animation.
            self.get_logger().info("animation mode left (quadruped leg set)")
            self._leave_animation_mode()
        elif self._state.mode == ANIMATION and not self._animation_mode_allowed():
            # Same shape, one preset finer: a six-leg preset the animations are
            # not written for. Only reachable from outside — the mode cannot be
            # entered from one of these, and the Mode view will not switch preset
            # while it is in force — but /cmd_preset is a public topic, so
            # arriving on one has to leave the mode rather than sit in a state
            # the two rules above exist to prevent.
            self.get_logger().info(
                f"animation mode left (preset {msg.data!r} is not "
                f"{self._animation_preset!r})"
            )
            self._state.mode = GAIT
            self._leave_animation_mode()
        self._broadcast_preset()

    def _leave_animation_mode(self) -> None:
        """Drop the animation and tell both ends the mode moved.

        The pipeline is still holding the last selected animation, so the empty
        publish is not optional; the client is showing ANIM lit, so neither is
        the broadcast.
        """
        self._pub_animation_mode.publish(String(data=""))
        self._broadcast_to_clients({
            "type": "mode",
            "mode": self._state.mode,
        })

    def _animation_mode_allowed(self) -> bool:
        """True where animation mode may be in force: on its own preset.

        ``presets.animation`` names the one preset the animations are written
        for; without the key nothing is gated and any preset will do. The
        four-legged case is not this function's — the shared mapping already
        refuses to enter the mode there, off ``JoyState.quadruped``.
        """
        if self._animation_preset is None:
            return True
        return self._presets.current_id() == self._animation_preset

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
        # Cycler bookkeeping, the way /cmd_gait's loopback does the gait's: a
        # name selected by the Mode view has to leave the prev/next index where
        # the operator can step off it, or the next animation_next press would
        # jump back to wherever the cycler last was. Here rather than in
        # ``_select_animation`` because this runs on the executor thread, which
        # is the only thread ``_state`` belongs to. An empty name is the mapping
        # leaving animation mode; it has already cleared its own state.
        if msg.data in self._cfg.animation_list:
            self._state.current_animation_idx = self._cfg.animation_list.index(
                msg.data
            )
            self._state.animation_name = msg.data
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
            actions = self._actions
            web_owns = self._web_owns
            last_input = self._last_input_monotonic

        # Safety watchdog: if no input has arrived within the timeout (the
        # WebSocket dropped uncleanly, the phone slept, etc.) feed neutral
        # inputs so /cmd_vel falls to zero instead of latching the last
        # commanded velocity. map_web still runs so the state machine sees
        # the releases and edge state stays consistent.
        stale = input_is_stale(last_input, time.monotonic(), self._input_timeout_s)
        publishing = web_owns or not self._arbitration_enabled
        if stale and not self._input_stale and publishing:
            self.get_logger().warning(
                "webapp input stale — holding zero velocity (input watchdog)"
            )
        self._input_stale = stale
        if stale:
            left, right, actions = neutral_inputs()

        out = map_web(
            left, right, actions, self._sticks, self._cfg, self._state, TICK_DT_S
        )

        # Ahead of the ownership gate: a preset switch is published regardless
        # of who owns /cmd_vel, so its deadline has to be watched the same way.
        self._expire_pending_preset()

        if out.mode_changed:
            self.get_logger().info(f"mode={self._state.mode}")
            self._broadcast_to_clients({
                "type": "mode",
                "mode": self._state.mode,
            })

        # Ahead of the ownership gate, and exempt from it for the reason a
        # Mode-view switch is: a stand or fold is discrete and supervisory — one
        # Empty on /gait/initialize plus the gait naming the leg set it wants —
        # and it touches neither /cmd_vel nor /body/pose, the two continuous
        # streams arbitration exists to stop from fighting. Gated, it would be a
        # dead button in exactly the state the Mode view's STAND exists for: the
        # button grid an init press comes from is swapped for the take-control
        # prompt while a controller drives, so that view is the only way in.
        if out.init_request:
            # Gait first, not after: hexa_locomotion reads the leg set off the
            # strategy that is applied by the time it starts the ladder.
            if out.gait_select is not None:
                self._publish_gait_select(out.gait_select)
            which = "quadruped" if out.init_quadruped else "hexapod"
            self.get_logger().info(
                f"webapp init ({which}) — publishing /gait/initialize"
            )
            self._pub_init.publish(Empty())

        # Arbitration: map_web always runs (keeps prev_* fresh), but the
        # continuous streams and the plain cycler publishes are gated on
        # ownership.
        if self._arbitration_enabled and not web_owns:
            return

        if out.animation_name is not None:
            self.get_logger().info(
                f"publishing /animation/mode={out.animation_name!r}"
            )
            self._pub_animation_mode.publish(String(data=out.animation_name))
        # An init request's own gait already went out above.
        if out.gait_select is not None and not out.init_request:
            self._publish_gait_select(out.gait_select)

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

    def _publish_gait_select(self, name: str) -> bool:
        """Publish a gait, unless the engine has it locked. True if it went.

        The return is for the webapp's Mode view, which asks for a gait by name
        and wants an answer; the mapping's own cycler calls this from the tick
        and ignores it — a D-pad press the engine will not take is already
        answered by the gait on the strip not changing.
        """
        if self._latest_gait_state not in _GAIT_SWITCH_STATES:
            self.get_logger().info(
                f"gait switch to {name!r} dropped — engine in "
                f"{self._latest_gait_state!r} (gait locked)"
            )
            return False
        self.get_logger().info(f"switching gait to {name!r}")
        # Caps + cycler bookkeeping happens in _on_cmd_gait when this publish
        # loops back — the path gamepad switches already take. Sticks run on
        # the old cap for the tick or two until then, invisible at 60 Hz.
        self._pub_cmd_gait.publish(String(data=name))
        return True

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
                # no-store for the same reason index.html gets it, plus one:
                # without it these carry only an ETag, so a phone is free to
                # serve a heuristically cached copy and run last week's UI
                # against today's socket protocol.
                return aiohttp.web.FileResponse(
                    filepath, headers={"Cache-Control": "no-store"}
                )
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
            "owner": self._arbitration.owner if self._arbitration_enabled else GAMEPAD,
            "arbitration_enabled": self._arbitration_enabled,
            "gait_state": self._latest_gait_state,
            "gait": self._active_gait,
            "animation": self._latest_animation_mode,
            # The Mode view. The list is fixed at load; the rest is live state,
            # so a client reconnecting mid-switch inherits the truth instead of
            # a blank view.
            "presets": preset_descriptors(self._presets),
            # Which preset the Mode view leaves selectable in animation mode.
            "preset_animation": self._animation_preset,
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
                self._actions = frozenset()
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
        elif msg_type == "action":
            # The client names the function it wants; anything outside the
            # shared namespace is a client the node does not know, and is
            # dropped rather than guessed at.
            action = str(data.get("action", ""))
            pressed = bool(data.get("pressed", False))
            if action not in ACTIONS:
                self.get_logger().warning(f"unknown action {action!r} ignored")
                return
            # The one action the node refuses outright rather than passing to
            # the mapping. The button is already dimmed on a preset that does
            # not carry animation mode, so this is only ever a stale client —
            # but the mapping has no preset to check against (that gate would
            # have to go in the parity-locked half), and it is easier to not
            # enter the mode than to back out of it: entry snaps an animation
            # and forces tripod on the way through.
            if (
                pressed
                and action == "animation_mode"
                and not self._animation_mode_allowed()
            ):
                self.get_logger().info(
                    f"animation mode refused — preset is "
                    f"{self._presets.current_id()!r}, not "
                    f"{self._animation_preset!r}"
                )
                # The label, not the id: it is the word on the tile the
                # operator has to press. Non-None whenever the guard above
                # says no.
                wanted = self._presets.get(self._animation_preset)
                self._broadcast_preset(
                    refused=f"animation mode needs the {wanted.label} preset"
                )
                return
            with self._lock:
                self._actions = (
                    self._actions | {action}
                    if pressed
                    else self._actions - {action}
                )
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
        elif msg_type == "select_gait":
            self._select_gait(str(data.get("gait", "")))
        elif msg_type == "select_animation":
            self._select_animation(str(data.get("animation", "")))
        elif msg_type == "request_control":
            self._claim_control()
        elif msg_type == "release_control":
            self._release_control()

    def _select_gait(self, gait: str) -> None:
        """Ask the engine for a gait by name, from the Mode view's gait row.

        The webapp names the gait it wants; the node still owns whether that is
        a gait to be asking for. Two guards the cycler did not need: the name
        must be in the rotation of the preset the engine reports (a cycler
        walking that rotation could not leave it), and no preset change may be
        in flight (the switch has already published the new preset's entry gait,
        and a gait from the OLD rotation landing behind it would sit latched on
        /cmd_gait as a gait the engine refuses).

        Not gated on ownership, for the reason ``_select_preset`` is not: this
        is one idempotent write to a latched selection topic, not a drive
        stream, and the gamepad makes the same write without asking anyone.
        """
        if self._pending_preset is not None:
            self._broadcast_preset(refused="switching mode — wait")
            return
        if not gait_selectable(self._presets, gait):
            self.get_logger().warning(f"unknown gait {gait!r} requested")
            self._broadcast_preset(refused="no such gait")
            return
        if gait == self._active_gait:
            # Already there: the latched topic would carry the same name, and a
            # refusal would be a lie. Nothing to say and nothing to publish.
            return
        if not self._publish_gait_select(gait):
            self._broadcast_preset(refused="the robot is busy")

    def _select_animation(self, animation: str) -> None:
        """Ask for an animation by name, from the Mode view's animation row.

        The cycler's counterpart to ``_select_gait``, and guarded the same way:
        the name must be one the config offers, and the state machine must
        actually be in ANIMATION mode — outside it nothing is driving the body,
        ``map_functions`` holds ``animation_name`` empty, and a name published
        anyway would sit latched on /animation/mode as an animation the operator
        cannot see running. The row is already dimmed there; this is the same
        answer for a client that asks regardless.

        Not gated on ownership, for the reason ``_select_gait`` is not: one
        idempotent write to a latched selection topic, not a drive stream. The
        index the prev/next presses walk is resynced by the loopback in
        ``_on_animation_mode``, on the thread that owns ``_state``.
        """
        if animation not in self._cfg.animation_list:
            self.get_logger().warning(f"unknown animation {animation!r} requested")
            self._broadcast_preset(refused="no such animation")
            return
        if self._state.mode != ANIMATION:
            self._broadcast_preset(refused="not in animation mode")
            return
        if animation == self._latest_animation_mode:
            # Already there: the latched topic would carry the same name, and a
            # refusal would be a lie.
            return
        self.get_logger().info(f"publishing /animation/mode={animation!r}")
        self._pub_animation_mode.publish(String(data=animation))

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
        if preset.id == self._presets.current_id():
            # Already there. Not an error, and not worth a wire round trip.
            # Compared on the PRESET, never on the leg set: three of the four
            # stand on six legs, so a leg-set comparison would make every switch
            # between them a silent no-op.
            self._broadcast_preset()
            return
        if not preset_switch_allowed(self._latest_gait_state):
            # Pre-gated here rather than left to the engine, because /cmd_preset
            # is latched: an id the engine refuses would stay on the wire and
            # every late subscriber would read a preset the robot never took.
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

        # The gait in force where the new preset walks it, its default where it
        # does not. The operator asked for a stance, not a different walk.
        gait = self._presets.entry_gait(preset.id, self._active_gait)
        owner = "web" if self._web_owns else "gamepad"
        self.get_logger().info(
            f"preset -> {preset_id!r} (gait {gait!r}), requested by the webapp "
            f"while {owner} owns /cmd_vel"
        )
        self._pending_preset = preset.id
        self._pending_deadline = (
            time.monotonic() + self._presets.switch_timeout_s
        )
        # Preset first, then the gait that walks it. The engine measures a gait
        # against the preset in force, so on the other order a four-corner gait
        # would be refused before its preset had been asked for. Both are
        # latched, and both loop back for the bookkeeping.
        self._pub_cmd_preset.publish(String(data=preset.id))
        self._pub_cmd_gait.publish(String(data=gait))
        self._broadcast_preset()

    def _expire_pending_preset(self) -> None:
        """Silence past the deadline is a refusal.

        /gait/preset publishes on change only, so a switch the node could not
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
