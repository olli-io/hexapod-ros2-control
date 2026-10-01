# hexapod-ros2-control

ROS 2 control stack for a 6-leg / 18-DOF hexapod robot.

Companion repo: [hexapod-servo2040-driver](https://github.com/olli-io/hexapod-servo2040-driver) — Pimoroni Servo 2040 firmware.

## Hardware

- Raspberry Pi 5 (2 GB minimum), Raspberry Pi OS Lite — runs ROS 2.
- Raspberry Pi Pico 2 W — runs the locomotion firmware.
- Pimoroni Servo 2040 — drives the 18 servos.
- Optional: 256×64 SH1122 OLED on SPI for the eyes.

## Deploy the robot

Install a released image on the Pi:

```
curl -fsSL https://raw.githubusercontent.com/olli-io/hexapod-ros2-control/main/install.sh | bash
```

The script checks the dependencies, then downloads the latest ARM64 image into
`~/hexa-robot/`. It does not start the stack.

Options: `--check-only`, `--tag release-x.x.x`, `--start`.

To start the robot on each power-on, run this once in `~/hexa-robot/`:

```
./hexa robot install-service
```

## Configure the robot

Edit the files in `~/hexa-robot/` on the Pi, then run `./hexa robot restart`.
No rebuild.

> [!IMPORTANT]
> **Calibrate the servos before you walk the robot.** Each robot needs its
> own values. Set them in `servo_calibration.yaml`: for each pin, the pulse
> (µs) at +45° and −45°. Updates keep this file.

- **`servo_calibration.yaml`** — servo calibration for this robot.
- **`tuning.yaml`** — gait, posture and teleop tuning. Each update replaces
  it and saves your copy as `tuning.yaml.bak`. To keep a change, copy it to
  `src/hexa_description/config/tuning.yaml`.

## Run the robot

Starting the robot energizes the servos. Two ways:

1. **Reboot the Pi.** Needs `install-service` (see above).
2. **Over ssh.** From `~/hexa-robot/` on the Pi:
   - `./hexa robot up` — start.
   - `./hexa robot down` — stop safely.
   - `./hexa robot restart` — stop, then start. Reads the config again.

## Run the sim

Needs Docker, docker-compose, and an X server on `$DISPLAY`. No native ROS 1.

```
git clone git@github.com:olli-io/hexapod-ros1-control.git
cd hexapod-ros1-control
./hexa sim up
```

- `./hexa sim logs -f` — stream logs.
- `./hexa sim down` — stop the stack.
- `./hexa sim face` — show the face animations.

Drive with an Xbox-style controller, or open `{host-ip}:8079` for web teleop.

## Build and deploy from source

**Clone or Fork the repo and run these on a linux workstation:**
- `./hexa deploy build` — cross-build the arm64 image.
- `./hexa deploy push <user@host>` — copy the image to the robot. It does not start it.
- `./hexa robot -H <user@host> install-service` — once: start the robot on each
  power-on.

**Build from source if you want to:**
- Adjust the geometry to fit a different hexapod (`geometry.yaml`).
- Change the servo wiring or direction (`hardware.yaml`).
- Add or change gestures (`gestures.yaml`).
- Change the code.
- Run the Pico firmware. It compiles all config in, so each config change
  needs `./hexa deploy --pico` and a reflash.

The config files are in `src/hexa_description/config/`.

## Documentation

- [`docs/sim-environment.md`](docs/sim-environment.md) — sim container.
- [`docs/robot-environment.md`](docs/robot-environment.md) — robot container.
- [`docs/configuration.md`](docs/configuration.md) — every tunable lives in YAML under `config/`. Edit, `./hexa sim build`, relaunch.
- [`docs/architecture.md`](docs/architecture.md) — design principles, packages, dependency direction.
