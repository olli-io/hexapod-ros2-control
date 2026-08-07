# hexapod-ros2-control

ROS 2 control stack for a 6-leg / 18-DOF hexapod robot.

Part of a multi-repo stack:
- [olli-io/hexapod-servo2040-driver](https://github.com/olli-io/hexapod-servo2040-driver) — Pimoroni Servo 2040 firmware.

## Hardware

- ROS2 : Raspberry Pi 4 or 5
- Pico firmware: Raspberry Pi Pico 2 W
- Pimoroni servo2040
- (Optional) 256×64 SH1122 OLED via SPI for eye animations.

## Quickstart (Sim)

Requires Docker + docker-compose installed and an X server on `$DISPLAY`. No native ROS 2 install needed.

```
git clone git@github.com:olli-io/hexapod-ros2-control.git
cd hexapod-ros2-control
./hexa sim up
```
Stream logs with `./hexa sim logs -f`, stop it with `./hexa sim down`. Drive the sim with an Xbox-style controller (wired or Bluetooth), or open
`{local-pc-ip}:8080` for web teleop. Face animations can be observed with `./hexa sim face`.
Full sim-container details: [`docs/sim-environment.md`](docs/sim-environment.md).

> [!NOTE]
> Only tested on Arch Linux. Other Linux distros should be straightforward with docker; macOS (apple/container) and Windows (WSL) untested.

## Quickstart (Robot)

Install a released image straight onto the Pi — no workstation, no cross-build:

```
curl -fsSL https://raw.githubusercontent.com/olli-io/hexapod-ros2-control/main/install.sh | bash
```

It checks every dependency first (64-bit OS, Docker + compose v2, disk, RAM,
wiring), then downloads the latest release's ARM64 image into `~/hexa-robot/`
and seeds `.env` with this Pi's own group IDs and device names. It does not
start anything — bringing the stack up energizes the servos, so that stays a
deliberate `./hexa robot up`. Pass `--check-only` to run just the checks,
`--tag <tag>` for a specific release, `--start` to bring it up when done.

To build and ship from a workstation instead, all host commands go through the
`hexa` dispatcher in the repo root; nothing is built on the host.

- ```./hexa deploy build``` — cross-build the production image for the Pi (arm64).
- ```./hexa deploy push <hostname@host_ip>``` — ship that image to the robot via ssh.

Full robot-container details:[`docs/robot-environment.md`](docs/robot-environment.md)

> [!NOTE]
> Recommended rpi specs: rPi 5, 2GB ram (minimum), raspberry OS lite.
> The current codebase is configured and tested on the rPi 5, other rpi versions may require some additional setup and configuration. 

## Configuration

Every tunable parameter lives in YAML under `config/` — never hard-coded in
nodes. Edit the YAML, rebuild (`./hexa sim build`), relaunch. The full file-by-file
rundown is in [`docs/configuration.md`](docs/configuration.md).

## Architecture

Design principles, the package list, and the dependency direction are documented
in [`docs/architecture.md`](docs/architecture.md).
