# hexapod-ros2-control

ROS 2 control stack for a 6-leg / 18-DOF hexapod robot.

Companion repo: [hexapod-servo2040-driver](https://github.com/olli-io/hexapod-servo2040-driver) — Pimoroni Servo 2040 firmware.

## Hardware

- Raspberry Pi 5 (2 GB minimum), Raspberry Pi OS Lite — runs ROS 2.
- Raspberry Pi Pico 2 W — runs the locomotion firmware.
- Pimoroni Servo 2040 — drives the 18 servos.
- Optional: 256×64 SH1122 OLED on SPI for the eyes.

## Run the sim

Needs Docker, docker-compose, and an X server on `$DISPLAY`. No native ROS 2.

```
git clone git@github.com:olli-io/hexapod-ros2-control.git
cd hexapod-ros2-control
./hexa sim up
```

- `./hexa sim logs -f` — stream logs.
- `./hexa sim down` — stop the stack.
- `./hexa sim face` — show the face animations.

Drive with an Xbox-style controller, or open `{host-ip}:8080` for web teleop.

Tested on Arch Linux only. Other Linux distros need Docker and nothing more; macOS and Windows are untested.

## Run the robot

Install a released image on the Pi:

```
curl -fsSL https://raw.githubusercontent.com/olli-io/hexapod-ros2-control/main/install.sh | bash
```

The script checks the dependencies, then downloads the latest ARM64 image into
`~/hexa-robot/`. It does not start the stack, because that energizes the servos.
Start it with `./hexa robot up`.

Options: `--check-only`, `--tag release-x.x.x`, `--start`.

To ship from a workstation instead:

- `./hexa deploy build` — cross-build the arm64 image.
- `./hexa deploy push <user@host>` — copy the image to the robot.

## Documentation

- [`docs/sim-environment.md`](docs/sim-environment.md) — sim container.
- [`docs/robot-environment.md`](docs/robot-environment.md) — robot container.
- [`docs/configuration.md`](docs/configuration.md) — every tunable lives in YAML under `config/`. Edit, `./hexa sim build`, relaunch.
- [`docs/architecture.md`](docs/architecture.md) — design principles, packages, dependency direction.
