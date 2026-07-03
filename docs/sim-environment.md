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
  **detached** (`docker compose up -d`). `--cpp` sets `HEXA_CPP=1` so the launch
  files default to the C++ ports (hexa_kinematics_cpp / hexa_gait_cpp); `--clean`
  rebuilds the image (after sim.Dockerfile edits).
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
- `./hexa pico up` — firmware-in-sim (see below); `./hexa pico down` / `logs -f`.
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
hexa sim build                                  # colcon build --symlink-install
hexa sim build --packages-select hexa_kinematics   # extra args forward to colcon
```

In an interactive container shell the `cb` alias is the same
`colcon build --symlink-install`. `install/setup.bash` is already sourced in new
shells.

## Firmware-in-sim (`./hexa pico`)

`./hexa pico up` runs the Pi Pico firmware brain against the Gazebo model
(sim + joy publisher + firmware bridge) as its own compose service — the same
teleop input should walk the sim hexapod identically to the ROS2 node chain. It
first builds `hexa_pico_bridge`, then `docker compose up -d pico`. It is mutually
exclusive with the sim stack (both drive the same Gazebo world), so bring one
down before the other up. `./hexa pico logs -f` streams it; `./hexa pico down`
tears it down.

## GUI smoke check

These should each pop a window on your desktop:

```
./hexa sim rviz2
./hexa sim gz sim shapes.sdf
```

If you get `cannot open display`, run `xhost +local:docker` once on the host and
try again. The wrapper does this for you, but a fresh login may reset the rule.

## Layout

```
sim.Dockerfile                          # image definition: jazzy-desktop + ros_gz + ros2_control + dev user
docker/entrypoint.sh                    # sources /opt/ros/jazzy/setup.bash, then install/setup.bash if built
docker-compose.sim.yaml                 # sim + pico services; each runs a composed launch as PID 1
hexa                                    # top-level host dispatcher (sim, pico, deploy, robot, kill)
src/hexa_bringup/launch/sim_bringup.launch.py  # the sim container's PID 1: sim + teleop + webteleop
scripts/sim.sh                          # docker compose lifecycle dispatcher (up/down/logs/build/shell/pico)
scripts/kill.sh                         # compose down the sim + pico containers
.dockerignore                           # keeps build/, install/, .git/ out of the build context
```

The sim container's PID 1 is the stack itself (via `sim_bringup.launch.py`),
mirroring the robot container (whose PID 1 is `bringup.launch.py`). Docker owns
supervision, detaching, log capture, and teardown for both.

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

## Production deployment (`./hexa deploy` + `./hexa robot`)

The sim container is x86_64, Gazebo-heavy, and built around a live source
bind-mount — none of that fits the Pi. The robot path is a separate
`robot.Dockerfile` cross-built for `linux/arm64`, shipped to the robot as a saved
image tarball, and run as a long-lived service.

Prerequisites on the workstation:

- `docker buildx` (the `docker-buildx-plugin` apt package, or built into recent
  Docker Desktop).
- `qemu-user-static` for the emulator binfmts. On Arch:
  `sudo pacman -S qemu-user-static qemu-user-static-binfmt`.

Prerequisites on the Pi (Ubuntu Server 24.04, ARM64):

- Docker engine + `docker compose` plugin.
- The deploy user in the `docker` group; an `input` group set up by the distro
  (kernel adds it automatically).
- The Pimoroni Servo 2040 plugged in (`/dev/ttyACM0`) and a joystick if teleop
  is wanted (`/dev/input/js*`).

Build + ship (`./hexa deploy`, workstation-only):

- `./hexa deploy build` — cross-build the ARM64 image, save it to
  `.deploy/hexa-robot_<sha>.tar.gz`. The tag and tarball are stamped with
  `git rev-parse --short HEAD` (with `-dirty` if the tree has changes).
- `./hexa deploy push <user@host>` — `scp` the tarball plus
  `docker-compose.robot.yaml`, `.env.robot.sample`, and the launcher
  (`hexa` + `scripts/robot.sh`) to `~/hexa-robot/` on the Pi, `docker load`, and
  `docker compose up -d`. The service comes up **cold**: the hardware component
  sits at `inactive` and the servo-rail relay stays open — container start does
  **not** energise the robot. Shipping the launcher is what makes `./hexa robot`
  usable on the Pi.

Operate the running container (`./hexa robot`, on the Pi — or from the
workstation with `-H user@host`, which re-dispatches over `ssh` in
`~/hexa-robot`):

- `./hexa robot up` — `compose up -d` (the container boots cold), waits for
  `controller_manager`, then transitions the hardware component to `active` (relay
  click) and spawns `joint_state_broadcaster` + `joint_group_position_controller`.
  After this the robot is drivable. This is the one attended action that energizes.
- `./hexa robot down` — safe-stop: unloads the controllers, drops the hardware
  back to `inactive` (relay opens; the robot goes limp), then `compose down`.
- `./hexa robot {restart|status|logs|shell}` — routine container ops against the
  local `hexa-robot` service. Teleop (gamepad + web) is part of the container's
  launch, so the robot is drivable as soon as `up` finishes — no separate verb.

Because energizing is a CLI step in `up` — never the container's CMD — a
`restart: unless-stopped` auto-restart (crash / power blip) brings the stack back
**cold** (relay open), so the servos never flail unattended. See
[`robot-environment.md`](robot-environment.md).

The cold-start gate is implemented by passing the
`hardware_components_initial_state` parameter to `controller_manager` from
`robot.launch.py` when `engage_on_start:=false`, which is the only non-default
setting in `bringup.launch.py`. No new C++ in `hexa_hardware` — the relay still
toggles in `on_activate` / `on_deactivate`, and the lifecycle state is held back
externally.

## Troubleshooting

- **`cannot open display`** — `xhost +local:docker` on the host. The wrapper
  attempts this but silently ignores failures.
- **Files in `build/` / `install/` owned by root** — your host UID/GID didn't
  match what was baked into the image. Rebuild the image with
  `./hexa sim up --clean`, which forwards `UID`/`GID`/`INPUT_GID` into the build.
  The issue only appears if you call `docker compose` directly without those env
  vars set.
- **`ros2 topic list` empty across containers** — check `ROS_DOMAIN_ID` is the
  same in both, and that no host firewall is dropping DDS multicast on loopback.
- **Gazebo Harmonic complains about OpenGL** — the host's GPU userspace isn't
  reachable inside the container. For most workstation GPUs this works out of
  the box; for NVIDIA, install `nvidia-container-toolkit` and add
  `runtime: nvidia` to the compose service.
