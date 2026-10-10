# Robot environment

This document takes a Raspberry Pi 4 or 5 from a blank SD card to a running
robot. For the workstation side, see [`sim-environment.md`](sim-environment.md).

The robot image is `robot.Dockerfile`, built for `linux/arm64`. It is shipped as
an image tarball to `~/hexa-robot/` and runs as a long-lived container.

## Hardware

- Raspberry Pi 4 or 5 with a 16 GB+ microSD card.
- Pimoroni Servo 2040 on the header UART, wired crossed with a common ground:
  Pi GPIO14 (TXD) to Servo 2040 RX, Pi GPIO15 (RXD) to Servo 2040 TX, GND to GND.
- Servo rail PSU behind the Servo 2040's relay.
- Wired Ethernet or Wi-Fi.
- Optional: 256×64 SH1122 OLED face on SPI0.
- Optional: two momentary push buttons. 
- Optional: passive buzzer, + to GPIO12, − to GND. 

GPIO allocation. All numbers are **BCM GPIO**, not header pin positions:

- **GPIO14, GPIO15** — UART0, the Servo 2040 link.
- **GPIO8–GPIO11** — SPI0 for the face. CE0 is driven by the SPI controller
  (`cs_line: -1`).
- **GPIO24, GPIO25** — face DC / RST.
- **GPIO12** — hardware PWM for the buzzer.
- **GPIO5** (pin 29) — info button: battery and web teleop address; hold 3 s
  for hotspot mode.
- **GPIO6** (pin 31) — Bluetooth button: controller status; hold 3 s for a
  pairing scan.
- **GPIO2, GPIO3** — I²C1, kept free for an MPU6500 IMU.

## 1. Prepare the Pi

Flash **Raspberry Pi OS Lite (64-bit)** with `rpi-imager`. In the advanced
options, set the hostname, the user, the SSH key, Wi-Fi and the WLAN country.

Install Docker, then log out and in again:

```
sudo apt update && sudo apt full-upgrade -y && sudo reboot
curl -fsSL https://get.docker.com | sh
sudo apt install -y docker-compose-plugin git
sudo usermod -aG docker $USER
```

Enable the interfaces. Leave out the lines for hardware that is not fitted:

```
# /boot/firmware/config.txt
dtparam=uart0=on                                    # servo UART, Pi 5 only
dtparam=spi=on                                      # display
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4   # buzzer
```

On a Pi 4, the servo UART is enabled with `raspi-config` instead:

```
sudo raspi-config nonint do_serial_hw 0     # enable the hardware UART
sudo raspi-config nonint do_serial_cons 1   # remove the serial console
```

Reboot, then verify:

```
ls -l /dev/ttyAMA0                    # servo UART, Pi 5 (Pi 4: /dev/ttyS0)
pinctrl get 14,15                     # a1 / uart, not "none"
ls -l /dev/spidev0.0 /dev/gpiochip0   # display, buttons
pinctrl get 12                        # a0 / PWM0_CHAN0, buzzer
```

## 2. Install

Use one of the two paths. Both make the same `~/hexa-robot/` layout and start
nothing.

**A. From a release, on the Pi.** No workstation is necessary:

```
curl -fsSL https://raw.githubusercontent.com/olli-io/hexapod-ros2-control/main/install.sh | bash
```

`install.sh` checks the dependencies first. Then it downloads the release image
and its support files, loads the image, and seeds `.env` with the group IDs and
device names of this Pi. It also installs the host services (§6) and skips each
one whose hardware or dependency is missing. It asks for `sudo` at the start.
Flags: `--start`, `--check-only`, `--tag <tag>`,
`--dir <path>`, `--keep-archive`.

**B. From a workstation.** This is the development path. Install the host
services by hand (§6). The workstation needs
an aarch64 binfmt handler with a **static** QEMU; see
[`sim-environment.md`](sim-environment.md#cross-building-for-the-robot).

```
./hexa deploy build
./hexa deploy push pi@<host>
```

## 3. Configure

There are three config files in `~/hexa-robot/`. `hexa robot restart` reads all
of them again.

- **`.env`** — seeded once and never overwritten. Path A fills it in. On path
  B, set these keys:
  - **`SERVO_DEVICE`** — `/dev/ttyAMA0` (Pi 5, default) or `/dev/ttyS0` (Pi 4).
  - **`INPUT_GID`** — `getent group input | cut -d: -f3`.
  - **`SPI_GID`**, **`GPIO_GID`** — the same for the `spi` and `gpio` groups.
    Display and buttons only.
  - **`BUZZER_PWM`** — Pi 4 only:
    `ls -d /sys/bus/platform/devices/*.pwm/pwm`.
  - **`ROS_DOMAIN_ID`** — DDS domain, default `42`.
- **`tuning.yaml`** — replaced on every deploy. A changed copy on the Pi is
  saved as `tuning.yaml.bak` first. Copy the changes that you want to keep into
  `src/hexa_description/config/tuning.yaml`.
- **`servo_calibration.yaml`** — seeded once and never overwritten. The repo
  default is next to it as `servo_calibration.yaml.default`.

For hardware that is not fitted:

- **Display** — set `enabled: false` in `hexa_display`'s `config/display.yaml`.
  If you do not, `hexa_display` aborts at startup.
- **Buttons** — `enabled: false` in `hexa_buttons`'s `config/buttons.yaml` is
  optional. Without it, the node stays inert.
- **Buzzer** — no change is necessary. Without the overlay, `up` does not mount
  the PWM block and the buzzer stays silent.

The undervoltage thresholds ship disabled. Calibrate them before you rely on
them; see `src/hexa_hardware/README.md`.

## 4. Run

```
./hexa robot up
./hexa robot down
./hexa robot -H pi@<host> up     # from the workstation, over ssh
```

The other commands are `restart`, `status`, `logs` and `shell`.

`up` energizes the servos one leg at a time (`init.sweep_leg_interval_ms` in
`hardware.yaml`, `0` for all at once) and holds the folded pose. **Start** on
the gamepad, or `/gait/initialize`, stands the robot up. **Start** from a stand
folds the robot and cuts the rail. Nothing walks unattended.

Drive with the gamepad, or with web teleop at `http://<pi-address>:8080`. The
webapp asks to claim control from the gamepad; see
`src/hexa_webteleop/README.md`.

## 5. Update

- **Path A** — run `install.sh` again. It keeps the values in `.env`, adds new
  keys, keeps `servo_calibration.yaml`, and installs the host services again.
- **Path B** — `./hexa deploy build` and `./hexa deploy push pi@<host>`. It
  restarts a running container on the new image.
- **Config only** — `./hexa deploy sync-config pi@<host>`. It adds missing
  `.env` keys (old file to `.env.bak`), replaces `tuning.yaml`, and sends the
  `systemd/` scripts and unit templates again. `--force` replaces `.env` with
  the sample. Installed units are rendered copies, so run the applicable
  `install-*` command again.

## 6. Host services

`install.sh` installs these. On path B, run them by hand. Each command needs
`sudo`, so run it on a TTY (`ssh -t`). Each one has an `uninstall-*` that
reverses it.

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

## Traps

- **Do not add `dtoverlay=disable-bt`.** The gamepad uses the onboard
  Bluetooth, and this overlay takes its UART.
- **Do not use `/dev/serial0`.** On a Pi 5 it is the debug connector.
- **Do not use a bare `dtoverlay=pwm-2chan`.** It maps GPIO18/19 and the buzzer
  stays silent with no error.
- **`up` prints "no PWM block"** — the buzzer overlay is missing from
  `config.txt`.
- **`No wifi country is set`** — the hotspot does not start without a WLAN
  country.
