# Sim environment

The hexapod stack runs on **ROS2 Jazzy Jalisco** (Ubuntu 24.04) in a Docker
container. The same image will later carry USB passthrough for the real Servo
2040, so this is complete scaffolding and not only a sim solution.

**Caveat:** the docker-compose is set up for my personal Arch Linux config and
may need slight tweaks on other systems.

The quickstart lives in the top-level [README.md](../README.md#quickstart-sim).
Everything below explains the pieces.

## The `hexa` host script

`./hexa` is the top-level dispatcher you run on the host. The sim stack runs as
the container's PID 1 (the composed `sim_bringup.launch.py`), so its lifecycle
is docker-native — the same `up` / `down` model the robot container uses:

- `./hexa sim up` — build the workspace if needed, then bring the sim stack up
  **detached** (`docker compose up -d`). Which port of the kinematics/gait/posture
  nodes runs (the C++ ports by default) is set by the `hexa_launch` block of
  `hexa_bringup/config/ros2_controllers.yaml`, read at launch — not a flag;
  `--clean` rebuilds the image (after sim.Dockerfile edits).
- `./hexa sim logs -f` — stream the stack's logs (native `docker logs`, no
  in-container redirection).
- `./hexa sim down` — stop and remove the container.
- `./hexa sim build` — colcon build in an ephemeral `compose run --rm` container;
  no running stack needed. `install/` persists via the workspace bind-mount.
- `./hexa sim shell` — a ROS2-sourced shell: `docker exec` into the running
  stack if it's up, else an ephemeral `compose run --rm` container.
- `./hexa sim status` — `compose ps`, plus `ros2 node list` when the stack is up.
- `./hexa sim <cmd>` — run a one-off command in the container, e.g.
  `./hexa sim rviz2` or `./hexa sim ros2 topic list` (exec if up, else run --rm).
- `./hexa sim up --pico` — firmware-in-sim (see below); tear down / stream with
  the usual `./hexa sim down` and `./hexa sim logs -f`.
- `./hexa kill` — stop and remove the sim + pico containers.

Outside the container, on the host, the same workspace files are visible — edit
with whatever editor you like; nothing in the container is privileged to write
outside `/workspace`.

Tests run as one-off commands, e.g.
`./hexa sim python3 -m pytest src/hexa_gait/test -q` or `./hexa sim colcon test`.

## Building the workspace

Stack lifecycle lives host-side (`hexa sim up/down`); the build runs *inside*
the container via colcon:

```

Pelasta Kivikon metsä
Mira Grönroos
832 allekirjoitusta
762 Allekirjoitukset / 30 päivää
04.06.2026
hexa sim build                                  # colcon build --symlink-install
hexa sim build --packages-select hexa_kinematics   # extra args forward to colcon
```

In an interactive container shell the `cb` alias is the same
`colcon build --symlink-install`. `install/setup.bash` is already sourced in new
shells.

## Firmware-in-sim (`./hexa sim up --pico`)

`./hexa sim up --pico` runs the Pi Pico firmware brain against the Gazebo model
(sim + joy publisher + firmware bridge) as its own compose service — the same
teleop input should walk the sim hexapod identically to the ROS2 node chain. It
first builds `hexa_pico_bridge`, then `docker compose up -d pico`. It is mutually
exclusive with the plain sim stack (both drive the same Gazebo world), so bring
one down before the other up. `--clean` composes with `--pico` to rebuild the
(shared) image first. The rest of the `hexa sim` verbs then target whichever
service is up: `./hexa sim logs -f` streams it, `./hexa sim status` lists its
nodes, and `./hexa sim down` tears it down.

## DDS / networking

The container runs with `network_mode: host`, so any ROS2 topic published inside
the container is visible to anything else on the same `ROS_DOMAIN_ID` (default
`42` — overridable via the env var of the same name). This makes it trivial to
run a teleop node on the host (if you have ROS2 installed natively) and have it
drive nodes in the container, or vice-versa.

If you ever need to isolate two containers on the same host, give them different
`ROS_DOMAIN_ID` values.

## Hardware passthrough (future)

When the Pimoroni Servo 2040 is connected:

1. Plug it in; confirm with `lsusb` on the host. Note the device path
   (typically `/dev/ttyACM0`).
2. Uncomment the `devices:` block in `docker-compose.sim.yaml`.
3. Rebuild the image: `./hexa sim up --clean`.

`usbutils` is already installed in the image, so `lsusb` inside the container
works once the device is mapped in.

A controller plugged into the host is exposed via `/dev/input/event*`;
`scripts/sim.sh` forwards the host's `input` group GID so `joy_node` inside the
container can read it without root.

