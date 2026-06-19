"""ROS + WebSocket node for the web-app teleop.

Hosts a lightweight HTTP + WebSocket server (``aiohttp``) that serves
the static webapp and relays input events to the same ROS topics the
gamepad teleop publishes: ``/cmd_vel``, ``/body/pose``, ``/cmd_gait``,
``/animation/mode``, ``/gait/initialize``. The webapp is pure HTML +
TypeScript (compiled at image build time); the server only serves
static files and a single ``/ws`` endpoint.

Coexistence with the gamepad teleop (``hexa_teleop.teleop_joy``) is
mediated by ``/teleop/owner`` (``std_msgs/String``, TRANSIENT_LOCAL).
The web node is the sole writer. Default owner is ``gamepad``; the
webapp must explicitly request control, and the web node releases on
disconnect. See ``hexa_teleop.teleop_arbitration`` for the protocol.

Architecture:
- Main thread: ``rclpy.spin`` with a 50 Hz timer that calls
  ``map_web`` and publishes (when web owns).
- Server thread: ``asyncio`` event loop running the ``aiohttp`` app.
- Shared state: ``threading.Lock``-protected stick/button values +
  client count + ownership flag. The WS handler writes; the timer
  reads. rclpy publishers are thread-safe, so ``/teleop/owner`` is
  published from the WS handler directly.
"""

from __future__ import annotations

import asyncio
import dataclasses
import json
import threading
from pathlib import Path

import aiohttp
import aiohttp.web
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from hexa_interfaces.msg import BodyPose as BodyPoseMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String

from hexa_teleop.joy_mapping import JoyState
from hexa_teleop.teleop_arbitration import (
    GAMEPAD,
    WEB,
    ArbitrationState,
    web_claim,
    web_release,
)

from .web_mapping import NUM_BUTTONS, button_labels_for_mode, load_web_config, map_web

PUBLISH_RATE_HZ = 50.0
TICK_DT_S = 1.0 / PUBLISH_RATE_HZ

_GAIT_SWITCH_STATES: frozenset[str] = frozenset(
    {"stand", "gait", "pausing", "paused", "reseating"}
)


class WebTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("web_teleop")

        default_cfg_path = (
            Path(get_package_share_directory("hexa_webteleop"))
            / "config"
            / "webteleop.yaml"
        )
        gait_yaml_path = (
            Path(get_package_share_directory("hexa_gait"))
            / "config"
            / "gait.yaml"
        )
        posture_yaml_path = (
            Path(get_package_share_directory("hexa_posture"))
            / "config"
            / "posture.yaml"
        )
        self.declare_parameter("config_file", str(default_cfg_path))
        cfg_path = Path(
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        self._cfg, initial_mode, default_gait, self._caps = load_web_config(
            cfg_path, gait_yaml_path, posture_yaml_path
        )
        self._state = JoyState(
            mode=initial_mode,
            current_gait_idx=self._cfg.gait_cycle.index(default_gait),
        )
        self._active_gait: str = default_gait
        self._latest_gait_state: str = ""

        # Server config
        with cfg_path.open() as f:
            import yaml

            raw = yaml.safe_load(f)
        self._port = int(raw.get("server", {}).get("port", 8080))
        self._arbitration_enabled = bool(
            raw.get("arbitration", {}).get("enabled", True)
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
        self._sub_gait_state = self.create_subscription(
            String, "/gait/state", self._on_gait_state, 10
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

    def _tick(self) -> None:
        with self._lock:
            left = self._left_stick
            right = self._right_stick
            buttons = self._buttons
            web_owns = self._web_owns

        out = map_web(left, right, buttons, self._cfg, self._state, TICK_DT_S)

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

        if out.init_request:
            self.get_logger().info("webapp init — publishing /gait/initialize")
            self._pub_init.publish(Empty())
        if out.animation_name is not None:
            self.get_logger().info(
                f"publishing /animation/mode={out.animation_name!r}"
            )
            self._pub_animation_mode.publish(String(data=out.animation_name))
        if out.gait_select is not None:
            if self._latest_gait_state in _GAIT_SWITCH_STATES:
                self.get_logger().info(f"switching gait to {out.gait_select!r}")
                self._pub_cmd_gait.publish(String(data=out.gait_select))
                self._active_gait = out.gait_select
                new_cap = self._caps.linear_max(self._active_gait)
                self._cfg = dataclasses.replace(self._cfg, gait_linear_max=new_cap)
                self.get_logger().info(
                    f"stick linear_max={new_cap:.3f} m/s for gait "
                    f"{self._active_gait!r}"
                )
            else:
                self.get_logger().info(
                    f"gait switch to {out.gait_select!r} dropped — "
                    f"engine in {self._latest_gait_state!r} (gait locked)"
                )

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
        app.router.add_get("/", self._handle_index)
        app.router.add_get("/{filename}", self._handle_static)
        runner = aiohttp.web.AppRunner(app, access_log=None)
        await runner.setup()
        site = aiohttp.web.TCPSite(runner, "0.0.0.0", self._port)
        await site.start()
        self.get_logger().info(
            f"web teleop server on port {self._port} (web dir: {self._web_dir})"
        )

    async def _handle_index(self, request: aiohttp.web.Request) -> aiohttp.web.Response:
        return aiohttp.web.FileResponse(Path(self._web_dir) / "index.html")

    async def _handle_static(self, request: aiohttp.web.Request) -> aiohttp.web.Response:
        filename = request.match_info["filename"]
        # Prevent path traversal: only serve flat files from web_dir
        if "/" in filename or ".." in filename or filename == "":
            raise aiohttp.web.HTTPNotFound()
        filepath = Path(self._web_dir) / filename
        if not filepath.is_file():
            raise aiohttp.web.HTTPNotFound()
        return aiohttp.web.FileResponse(filepath)

    async def _handle_ws(self, request: aiohttp.web.Request) -> aiohttp.web.WebSocketResponse:
        ws = aiohttp.web.WebSocketResponse()
        await ws.prepare(request)

        with self._ws_clients_lock:
            self._ws_clients.append(ws)
        with self._lock:
            self._client_count += 1

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
        elif msg_type == "button":
            idx = int(data.get("index", -1))
            pressed = bool(data.get("pressed", False))
            if 0 <= idx < NUM_BUTTONS:
                with self._lock:
                    btns = list(self._buttons)
                    btns[idx] = 1 if pressed else 0
                    self._buttons = tuple(btns)
        elif msg_type == "request_control":
            self._claim_control()
        elif msg_type == "release_control":
            self._release_control()

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
