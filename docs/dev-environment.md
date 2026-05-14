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
scripts/dev.sh           # xhost + docker compose run
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
./scripts/dev.sh
```

Builds `hexapod-dev:jazzy` (takes a few minutes the first time), then drops
you at a shell inside `/workspace` (the repo, bind-mounted). ROS2 is already
sourced for you.

## Daily loop

Inside the container:

```
colcon build --symlink-install
source install/setup.bash       # already done for you in new shells
ros2 launch hexa_bringup sim.launch.py
```

Outside the container, on the host, the same workspace files are visible —
edit with whatever editor you like; nothing in the container is privileged
to write outside `/workspace`.

## GUI smoke check

These should each pop a window on your desktop:

```
./scripts/dev.sh rviz2
./scripts/dev.sh gz sim shapes.sdf
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
3. Rebuild: `docker compose build`.

`usbutils` is already installed in the image, so `lsusb` inside the container
works once the device is mapped in.

## Cross-build for the Raspberry Pi 3

Not in scope for this scaffolding. The eventual story: a separate
`Dockerfile.arm64` (or buildx multi-arch) producing the same workspace built
for `linux/arm64`. The current image is x86_64 only and intended for
development on a workstation.

## Troubleshooting

- **`cannot open display`** — `xhost +local:docker` on the host. The wrapper
  attempts this but silently ignores failures.
- **Files in `build/` / `install/` owned by root** — your host UID/GID didn't
  match what was baked into the image. Rebuild while passing the right IDs:
  `UID=$(id -u) GID=$(id -g) docker compose build`. The `scripts/dev.sh`
  wrapper does this for you; the issue only appears if you call
  `docker compose` directly.
- **`ros2 topic list` empty across containers** — check `ROS_DOMAIN_ID` is the
  same in both, and that no host firewall is dropping DDS multicast on
  loopback.
- **Gazebo Harmonic complains about OpenGL** — the host's GPU userspace
  isn't reachable inside the container. For most workstation GPUs this works
  out of the box; for NVIDIA, install `nvidia-container-toolkit` and add
  `runtime: nvidia` to the compose service.
