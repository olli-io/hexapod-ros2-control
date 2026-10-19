# Robot environment

This document adds detail to the deploy, configure and run steps in the root
[`README.md`](../README.md). To prepare the Pi, see [`pi-setup.md`](pi-setup.md).
For the workstation side, see [`sim-environment.md`](sim-environment.md).

The robot image is `robot.Dockerfile`, built for `linux/arm64`. It is shipped as
an image tarball to `~/hexa-robot/` and runs as a long-lived container.

## 1. Install

There are two paths. Both make the same `~/hexa-robot/` layout and start
nothing.

- **Release** — `install.sh`, on the Pi. It seeds `.env` with the group IDs and
  device names of this Pi. It installs the host services (§5) and skips each
  one whose hardware or dependency is missing. It asks for `sudo` at the start.
  More flags: `--dir <path>`, `--keep-archive`.
- **Workstation** — `./hexa deploy build` and `./hexa deploy push pi@<host>`.
  This is the development path. The workstation needs an aarch64 binfmt
  handler with a **static** QEMU; see
  [`sim-environment.md`](sim-environment.md#cross-building-for-the-robot).
  Install the host services by hand (§5).

## 2. Configure

`hexa robot restart` reads all config files in `~/hexa-robot/` again.

- **`.env`** — seeded once and never overwritten. The release path fills it in.
  On the workstation path, set these keys:
  - **`SERVO_DEVICE`** — `/dev/ttyAMA0` (Pi 5, default) or `/dev/ttyS0` (Pi 4).
  - **`INPUT_GID`** — `getent group input | cut -d: -f3`.
  - **`SPI_GID`**, **`GPIO_GID`** — the same for the `spi` and `gpio` groups.
    Display and buttons only.
  - **`I2C_GID`** — the same for the `i2c` group. IMU only.
  - **`BUZZER_PWM`** — Pi 4 only:
    `ls -d /sys/bus/platform/devices/*.pwm/pwm`.
  - **`ROS_DOMAIN_ID`** — DDS domain, default `42`.
- **`servo_calibration.yaml`** — seeded once and never overwritten. The repo
  default is next to it as `servo_calibration.yaml.default`.

For hardware that is not fitted:

- **Display** — set `enabled: false` in `hexa_display`'s `config/display.yaml`.
  If you do not, `hexa_display` aborts at startup.
- **Buttons** — `enabled: false` in `hexa_buttons`'s `config/buttons.yaml` is
  optional. Without it, the node stays inert.
- **IMU** — `enabled: false` in `hexa_imu`'s `config/imu.yaml` is optional.
  Without it, the node retries the bus every few seconds and body
  stabilization stays inactive. `/dev/i2c-1` must exist either way, because
  the container maps it.
- **Buzzer** — no change is necessary. Without the overlay, `up` does not mount
  the PWM block and the buzzer stays silent.

The undervoltage thresholds ship disabled. Calibrate them before you rely on
them; see `src/hexa_hardware/README.md`.

## 3. Run

From the workstation, add `-H pi@<host>`, e.g. `./hexa robot -H pi@<host> up`.
The other commands are `status`, `logs` and `shell`.

`up` energizes the servos one leg at a time (`init.sweep_leg_interval_ms` in
`hardware.yaml`, `0` for all at once) and holds the folded pose. **Start** on
the gamepad, or `/gait/initialize`, stands the robot up. **Start** from a stand
folds the robot and cuts the rail. Nothing walks unattended.

Drive with the gamepad, or with web teleop at `http://<pi-address>:8080`. The
webapp asks to claim control from the gamepad; see
`src/hexa_webteleop/README.md`.

## 4. Update

- **Release** — run `install.sh` again. It keeps the values in `.env`, adds new
  keys, keeps `servo_calibration.yaml`, and installs the host services again.
- **Workstation** — build and push again.
- **Config only** — `./hexa deploy sync-config pi@<host>`. It adds missing
  `.env` keys (old file to `.env.bak`), replaces `tuning.yaml`, and sends the
  `systemd/` scripts and unit templates again. `--force` replaces `.env` with
  the sample. Installed units are rendered copies, so run the applicable
  `install-*` command again.

## 5. Host services

`install.sh` installs these. On the workstation path, run them by hand. Each
command needs `sudo`, so run it on a TTY (`ssh -t`). Each one has an
`uninstall-*` that reverses it.

- **`./hexa robot install-service`** — starts the stack on boot. It waits for
  Docker and the device nodes, then runs `up`. On shutdown it runs `down`. Do
  not use only Docker's `restart:`, because it races the device nodes at boot.
  - `systemctl status hexa-robot` — the result is `active (exited)`.
  - `journalctl -u hexa-robot -b` — the boot log.
  - While the unit is enabled, a `down` does not stay after a reboot.
- **`./hexa robot install-tune`** — plays the `boot` and `shutdown` tunes from
  host units, because Docker is not running at those times.
  `./hexa robot play-tune [event]` plays a tune for a test. For tunes and
  events, see `src/hexa_buzzer/README.md`.
- **`./hexa robot install-network`** — Wi-Fi hotspot mode. The install does not
  change the network; only a switch to the hotspot does.

### Hotspot

Web teleop works on any network at the Pi's address. The hotspot is for places
without a network. To toggle it, hold the info button for 3 s, or use
`./hexa robot network-mode {status|toggle|hotspot|station}`.

- **network** — `hexapod`, password `hexahexa`.
- **address** — `http://192.168.4.1/`. Phones open
  the controller as a captive portal.
- **requirements** — NetworkManager (Bookworm or newer) and a WLAN country.
- **one radio** — hotspot mode drops ssh sessions over Wi-Fi. Switch from
  Ethernet, a console or the button.
- **reboot** — always comes back in station mode. A hotspot that does not start
  rolls back to station mode.
- **uplink** — clients are NATed to `eth0` if it is connected.
