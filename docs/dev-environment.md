# Dev environment

The hexapod stack runs on **ROS2 Jazzy Jalisco** (Ubuntu 24.04) in a Docker
container. The same image will later carry USB passthrough for the real Servo
2040, so this is complete scaffolding and not only a dev solution.

**Caveat:** the docker-compose is set up for my personal Arch Linux config and
may need slight tweaks on other systems.

The quickstart lives in the top-level [README.md](../README.md#quickstart-sim).
Everything below explains the pieces.

## The `hexa` host script

`./hexa` is the top-level dispatcher you run on the host:

- `./hexa dev` — drop into a shell in the dev container. The first call builds
  the image and creates the container; later calls `docker exec` into the same
  one.
- `./hexa dev --launch` — open a shell and immediately run the full sim stack
  (`pod launch`).
- `./hexa dev --tmux` — same dev container, two panes: pane 0 runs the full sim
  stack, pane 1 is an idle shell for ad-hoc commands (add `horizontal` to split
  side-by-side).
- `./hexa dev --test` — build, then run the full colcon test suite.
- `./hexa dev --clean` — rebuild the image from scratch (after Dockerfile
  edits): kills the container, rebuilds, runs `pod build`, drops you into a
  shell.
- `./hexa kill` — stop and remove the dev container.
- `./hexa dev <cmd>` — run a one-off command in the container, e.g.
  `./hexa dev rviz2`.

Outside the container, on the host, the same workspace files are visible — edit
with whatever editor you like; nothing in the container is privileged to write
outside `/workspace`.

## The `pod` CLI (inside the container)

`./hexa dev --launch` is enough for the daily loop, but inside the container the
`pod` CLI gives you finer control:

```
pod build                       # colcon build --symlink-install
pod sim                         # ros2 launch hexa_bringup sim.launch.py
pod teleop                      # ros2 launch hexa_teleop teleop.launch.py
pod webteleop                   # ros2 launch hexa_webteleop webteleop.launch.py
pod launch                      # sim + webteleop + teleop, once /clock is up
```

Extra args are forwarded, e.g. `pod build --packages-select hexa_kinematics`.
`install/setup.bash` is already sourced in new shells.

## GUI smoke check

These should each pop a window on your desktop:

```
./hexa dev rviz2
./hexa dev gz sim shapes.sdf
```

If you get `cannot open display`, run `xhost +local:docker` once on the host and
try again. The wrapper does this for you, but a fresh login may reset the rule.

## Layout

```
Dockerfile               # image definition: jazzy-desktop + ros_gz + ros2_control + dev user
docker/entrypoint.sh     # sources /opt/ros/jazzy/setup.bash, then install/setup.bash if built
docker-compose.yml       # mounts workspace, X11 socket, host network for DDS
hexa                     # top-level host dispatcher (dev, prod, kill)
pod                      # in-container workspace CLI (build / sim / teleop / webteleop / launch)
scripts/dev.sh           # ensure single long-lived dev container, then docker exec into it
scripts/kill.sh          # stop and remove the dev container
scripts/tmux.sh          # tmux session with sim + teleop sharing one container
.dockerignore            # keeps build/, install/, .git/ out of the build context
```

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
2. Uncomment the `devices:` block in `docker-compose.yml`.
3. Rebuild from scratch: `./hexa dev --clean`.

`usbutils` is already installed in the image, so `lsusb` inside the container
works once the device is mapped in.

A controller plugged into the host is exposed via `/dev/input/event*`;
`scripts/dev.sh` forwards the host's `input` group GID so `joy_node` inside the
container can read it without root.

## Production deployment (`./hexa prod`)

The dev container is x86_64, Gazebo-heavy, and built around a live source
bind-mount — none of that fits the Pi. The prod path is a separate
`Dockerfile.prod` cross-built for `linux/arm64`, shipped to the robot as a saved
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

Lifecycle:

- `./hexa prod build` — cross-build the ARM64 image, save it to
  `.deploy/hexa-prod_<sha>.tar.gz`. The tag and tarball are stamped with
  `git rev-parse --short HEAD` (with `-dirty` if the tree has changes).
- `./hexa prod deploy <user@host>` — `scp` the tarball plus
  `docker-compose.prod.yaml` and `.env.prod.sample` to `~/hexa-prod/` on the Pi,
  `docker load`, and `docker compose up -d`. The service comes up **cold**: the
  hardware component sits at `inactive` and the servo-rail relay stays open —
  container start does **not** energise the robot.
- `./hexa prod engage` (on the Pi, or via `ssh`) — transitions the hardware
  component to `active` (relay click), then spawns `joint_state_broadcaster` and
  `joint_group_position_controller`. After this the robot is drivable.
- `./hexa prod disengage` — unloads the controllers and drops the hardware back
  to `inactive`. Relay opens; the robot goes limp.
- `./hexa prod {start|stop|restart|status|logs|shell|teleop}` — routine
  container ops against the local `hexa-prod` service.

The cold-start gate is implemented by passing the
`hardware_components_initial_state` parameter to `controller_manager` from
`robot.launch.py` when `engage_on_start:=false`, which is the only non-default
setting in `prod.launch.py`. No new C++ in `hexa_hardware` — the relay still
toggles in `on_activate` / `on_deactivate`, and the lifecycle state is held back
externally.

## Troubleshooting

- **`cannot open display`** — `xhost +local:docker` on the host. The wrapper
  attempts this but silently ignores failures.
- **Files in `build/` / `install/` owned by root** — your host UID/GID didn't
  match what was baked into the image. Rebuild from scratch with
  `./hexa dev --clean`, which forwards `UID`/`GID`/`INPUT_GID` into the build.
  The issue only appears if you call `docker compose` directly without those env
  vars set.
- **`ros2 topic list` empty across containers** — check `ROS_DOMAIN_ID` is the
  same in both, and that no host firewall is dropping DDS multicast on loopback.
- **Gazebo Harmonic complains about OpenGL** — the host's GPU userspace isn't
  reachable inside the container. For most workstation GPUs this works out of
  the box; for NVIDIA, install `nvidia-container-toolkit` and add
  `runtime: nvidia` to the compose service.
