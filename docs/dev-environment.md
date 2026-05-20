# Dev environment

The hexapod stack runs on **ROS2 Jazzy Jalisco** (Ubuntu 24.04) in a Docker container. 

The same image will later carry USB passthrough for the real Servo 2040, so
this is complete scaffolding and not only a dev solution.

**caveat** The docker-compose it set up to work on my personal arch linux conf and may require slight modifications to run on other systems.

## Layout

```
Dockerfile               # image definition: jazzy-desktop + ros_gz + ros2_control + dev user
docker/entrypoint.sh     # sources /opt/ros/jazzy/setup.bash, then install/setup.bash if built
docker-compose.yml       # mounts workspace, X11 socket, host network for DDS
hexa                     # top-level host dispatcher (--dev, --tmux, kill)
pod                      # in-container workspace CLI (build / sim / teleop)
scripts/dev.sh           # ensure single long-lived dev container, then docker exec into it
scripts/kill.sh          # stop and remove the dev container
scripts/tmux.sh          # tmux session with sim + teleop sharing one container
.dockerignore            # keeps build/, install/, .git/ out of the build context
```

## Prerequisites

- Docker (`pacman -S docker docker-compose` on Arch; the user must be in the
  `docker` group).
- An X server reachable as `$DISPLAY`. On Omarchy (Hyprland/Wayland), this is
  Xwayland and is running by default. `echo $DISPLAY` should print something
  like `:0`.

No native ROS2 install needed.

## First-time setup

```
./hexa --dev
```

Builds the `hexa-dev` image (takes a few minutes the first time), creates a
long-lived `hexa-dev` container, and drops you at a shell inside `/workspace`
(the repo, bind-mounted). ROS2 is already sourced for you. Subsequent
`./hexa --dev` invocations `docker exec` into the same container instead of
spawning a new one.

To tear it down: `./hexa kill`. To rebuild from scratch (after Dockerfile
edits): `./hexa --dev --clean`, which kills the container, rebuilds the
image, runs `pod build`, then drops you into a shell.

## Daily loop

Inside the container, use the `pod` CLI:

```
pod build                       # colcon build --symlink-install
pod sim                         # ros2 launch hexa_bringup sim.launch.py
pod teleop                      # ros2 launch hexa_teleop teleop.launch.py
```

Extra args are forwarded, e.g. `pod build --packages-select hexa_kinematics`.
`install/setup.bash` is already sourced in new shells.

For the common sim + teleop pair, `./hexa --tmux` opens a tmux session with
both panes attached to the same dev container (`vert` stacks the split).

To build and run the full test suite in one shot:

```
./hexa --dev --test             # runs `pod build`, then colcon test
```

Outside the container, on the host, the same workspace files are visible —
edit with whatever editor you like; nothing in the container is privileged
to write outside `/workspace`.

## GUI smoke check

These should each pop a window on your desktop:

```
./hexa --dev rviz2
./hexa --dev gz sim shapes.sdf
```

If you get `cannot open display`, run `xhost +local:docker` once on the host
and try again. The wrapper does this for you, but a fresh login may reset
the rule.

## DDS / networking

The container runs with `network_mode: host`, so any ROS2 topic published
inside the container is visible to anything else on the same `ROS_DOMAIN_ID`
(default `42` — overridable via the env var of the same name). This makes
it trivial to run a teleop node on the host (if you have ROS2 installed
natively) and have it drive nodes in the container, or vice-versa.

If you ever need to isolate two containers on the same host, give them
different `ROS_DOMAIN_ID` values.

## Hardware passthrough (future)

When the Pimoroni Servo 2040 is connected:

1. Plug it in; confirm with `lsusb` on the host. Note the device path
   (typically `/dev/ttyACM0`).
2. Uncomment the `devices:` block in `docker-compose.yml`.
3. Rebuild from scratch: `./hexa --dev --clean`.

`usbutils` is already installed in the image, so `lsusb` inside the container
works once the device is mapped in.

A controller plugged into the host is exposed via `/dev/input/event*`;
`scripts/dev.sh` forwards the host's `input` group GID so `joy_node` inside
the container can read it without root.

## Cross-build for the Raspberry Pi 3

Not in scope for this scaffolding. The eventual story: a separate
`Dockerfile.arm64` (or buildx multi-arch) producing the same workspace built
for `linux/arm64`. The current image is x86_64 only and intended for
development on a workstation.

## Troubleshooting

- **`cannot open display`** — `xhost +local:docker` on the host. The wrapper
  attempts this but silently ignores failures.
- **Files in `build/` / `install/` owned by root** — your host UID/GID didn't
  match what was baked into the image. Rebuild from scratch with
  `./hexa --dev --clean`, which forwards `UID`/`GID`/`INPUT_GID` into the
  build. The issue only appears if you call `docker compose` directly
  without those env vars set.
- **`ros2 topic list` empty across containers** — check `ROS_DOMAIN_ID` is the
  same in both, and that no host firewall is dropping DDS multicast on
  loopback.
- **Gazebo Harmonic complains about OpenGL** — the host's GPU userspace
  isn't reachable inside the container. For most workstation GPUs this works
  out of the box; for NVIDIA, install `nvidia-container-toolkit` and add
  `runtime: nvidia` to the compose service.
