# Robot environment

Steps to take a fresh Raspberry Pi 4 or 5 from a blank SD card to a host ready
to receive `./hexa deploy push`, plus the `./hexa deploy` / `./hexa robot`
workflow that builds, ships, and drives it. See
[`sim-environment.md`](sim-environment.md) for the sim/workstation side.

The sim container is x86_64, Gazebo-heavy, and built around a live source
bind-mount — none of that fits the Pi. The robot path is a separate
`robot.Dockerfile` cross-built for `linux/arm64`, shipped to the Pi as a saved
image tarball, and run as a long-lived service.

## Hardware

- Raspberry Pi 4 or 5 with a 16 GB+ microSD card.
- Pimoroni Servo 2040 on the Pi header UART, GPIO14/15 (`/dev/ttyAMA0` on a
  Pi 5, `/dev/ttyS0` on a Pi 4).
- Servo rail PSU behind the Servo 2040's relay.
- Wired Ethernet or Wi-Fi.
- Optional: 256×64 SH1122 OLED face on the Pi SPI bus (spidev0.0 +
  DC/RST/CS on GPIO), rendered directly by `hexa_display`.
- Optional: passive buzzer on GPIO18 for the boot jingle.

Pi GPIO allocation, so nothing added later steals a line. All numbers are
**BCM GPIO**, never header pin positions — the two differ on every Pi, and BCM
is what `pinctrl`, the `gpiochip` character device, and `config.txt` use:

- **GPIO14, GPIO15** — UART0 TXD / RXD, the Servo 2040 link.
- **GPIO8, GPIO9, GPIO10, GPIO11** — SPI0 (CE0, MISO, MOSI, SCLK) for the
  SH1122 face.
- **GPIO24, GPIO25** — the face's DC / RST control lines.
- **GPIO18** — hardware PWM for the buzzer (RP1 PWM0 channel 2, alt function
  `a3`).
- **GPIO2, GPIO3** — I²C1, free for an MPU6500 IMU.

Watch out for **SPI1**, whose CE0 is GPIO18: putting a second SPI device on the
aux bus with hardware chip-select collides with the buzzer. Use SPI0 CE1
(GPIO7) or I²C for extra sensors instead. GPIO18 is also I²S PCM_CLK, so an
I²S audio HAT and the buzzer are mutually exclusive.

## 1. Flash the OS

Use **Raspberry Pi OS Lite (64-bit)** via `rpi-imager`. In advanced options
set hostname, username, enable SSH with your public key, and configure Wi-Fi
if needed.

## 2. Install Docker

```
sudo apt update && sudo apt full-upgrade -y
sudo reboot
curl -fsSL https://get.docker.com | sh
sudo apt install -y docker-compose-plugin git
sudo usermod -aG docker $USER
```
Exit and re-enter the ssh session, then verify that docker runs:

```
docker run --rm hello-world
```

## 2b. Enable the servo UART (required)

The Servo 2040 hangs off the Pi's header UART, not USB. Wire it crossed, with a
common ground:

- **Pi GPIO14 (UART0 TXD)** — Servo 2040 **RX**.
- **Pi GPIO15 (UART0 RXD)** — Servo 2040 **TX**.
- **Pi GND** — Servo 2040 **GND**. Both boards keep their own supply; only the
  ground is shared.

GPIO numbers here are BCM, not header positions — `pinctrl` and the
`gpiochip` interface both speak BCM, and the physical pin a GPIO lands on
differs from its number.

Which kernel device those two lines become depends on the model, so the
enabling step and `SERVO_DEVICE` differ. **Pi 5** — the GPIO14/15 UART is RP1's
`uart0`, off by default; turn it on:

```
# /boot/firmware/config.txt
dtparam=uart0=on
```

After a reboot it appears as **`/dev/ttyAMA0`**. Nothing else contends for it:
the Pi 5's serial console lives on the separate 3-pin debug connector
(`uart10` → `/dev/ttyAMA10`, which is also what `/dev/serial0` points at), so
leave `cmdline.txt` alone.

**Pi 4** — GPIO14/15 is the mini-UART, and the console *does* sit on it. Enable
the port and evict the getty:

```
sudo raspi-config nonint do_serial_hw 0     # enable the hardware UART
sudo raspi-config nonint do_serial_cons 1   # disable the serial login console
sudo reboot
```

By hand that is `enable_uart=1` in `config.txt` plus dropping the
`console=serial0,115200` token from `/boot/firmware/cmdline.txt`. The device is
**`/dev/ttyS0`**; `enable_uart=1` also pins the core clock so the mini-UART's
baud stays stable, and 115200 is well within its range.

Verify after the reboot that the node exists and the pins carry the UART
function:

```
ls -l /dev/ttyAMA0            # Pi 5   (Pi 4: /dev/ttyS0)
pinctrl get 14,15             # expect a1/uart function, not "none"
```

Then set `SERVO_DEVICE` in `~/hexa-robot/.env` to match — `/dev/ttyAMA0` is the
shipped default, so a Pi 4 is the case that needs the edit. Two traps worth
naming:

- **Do not add `dtoverlay=disable-bt`.** The gamepad pairs over the Pi's
  onboard Bluetooth, and that overlay steals the radio's UART to move the PL011
  onto the header.
- **`/dev/serial0` is not model-portable here.** On a Pi 4 it tracks the
  header UART, but on a Pi 5 it is the debug connector — wiring the servos to
  GPIO14/15 and naming `serial0` would silently talk to the wrong port.

## 2c. Enable the display SPI (optional)

Only needed if the SH1122 OLED face is fitted. `hexa_display` drives the
panel directly over spidev + the kernel GPIO character device. Enable
the SPI bus:

```
# /boot/firmware/config.txt
dtparam=spi=on
```

Reboot, then verify the device nodes exist and note the group IDs:

```
ls -l /dev/spidev0.0 /dev/gpiochip0
getent group spi  | cut -d: -f3            # note for SPI_GID
getent group gpio | cut -d: -f3            # note for GPIO_GID
```

The robot compose maps both device nodes into the container and
forwards the Pi's `spi` / `gpio` group GIDs (`SPI_GID` / `GPIO_GID` in
`.env`) so the node can open them. Wiring (SPI0 + control pins) and
the render rate are configured in `hexa_display`'s `config/display.yaml`.

Without the display fitted, set `enabled: false` in
`hexa_display/config/display.yaml` (the bringup gate skips the face
node); otherwise `hexa_display` aborts at startup when it cannot open
the panel.

## 2d. Enable the buzzer PWM (optional)

Only needed if the passive buzzer is fitted. Wire buzzer **+** to GPIO18 (BCM)
and **−** to GND; add a ~100 Ω resistor in series if it is too loud.

The jingle is played by `systemd/boot-tune.sh` — POSIX shell writing the
kernel's sysfs PWM interface, with no Python, gpiozero, or any other package
installed. It runs on the **Pi host**, not in the container: the point is a
chirp seconds after the kernel hands off, long before Docker and the ROS stack
exist.

Enable the PWM block:

```
# /boot/firmware/config.txt
dtoverlay=pwm-2chan
```

On the Pi 5 the PWM lives in the RP1 southbridge: GPIO18 is PWM0 **channel 2**,
alt function `a3`. Reboot, then confirm the sysfs tree and hear it:

```
ls /sys/class/pwm/                  # expect pwmchip0 (SoC, 2ch) + the RP1 chip (4ch)
cat /sys/class/pwm/pwmchip*/npwm    # the 4-channel one is RP1's
cd ~/hexa-robot && ./hexa robot play-tune
```

The chip number moves with the kernel and the overlays in play, so
`boot-tune.sh` discovers it (4 channels = RP1) rather than hardcoding
`pwmchip2`, and re-asserts the pin's alt mode with `pinctrl` in case the
overlay in `config.txt` did not. If discovery picks wrong, pin it with
`TUNE_PWMCHIP=/sys/class/pwm/pwmchipN`.

Tune it without editing the script — the same `NOTE:beats` melody format the
gpiozero recipes use, `REST` for silence:

```
TUNE_MELODY="C5:1 E5:1 G5:1 C6:1 REST:1 G5:1 C6:3" TUNE_TEMPO=0.11 \
    ./hexa robot play-tune
```

`TUNE_GPIO`, `TUNE_CHANNEL`, and `TUNE_PIN_ALT` cover a different buzzer pin.
Every hardware failure — no buzzer, no overlay, busy channel — logs a line and
exits 0, so the tune can never hold up a boot.

## 3. Note hardware IDs

On the Pi:

```
ls -l /dev/ttyAMA0                          # the servo UART from step 2b (Pi 4: ttyS0)
getent group input | cut -d: -f3            # note the input GID (example: 994)
```

## 4. First deploy from the workstation

`./hexa deploy build` cross-compiles `linux/arm64` under QEMU, so the
workstation kernel needs an aarch64 binfmt_misc handler pointing at a
**static** QEMU interpreter. On Arch this requires manual setup —
installing `qemu-user-static` (extra) ships the static binary but no
binfmt config, while the `qemu-user` package's config in
`/usr/lib/binfmt.d/` points at the *dynamic* interpreter, which fails
inside the build container with `exec /bin/sh: no such file or directory`.
Override it once:

```
sudo install -m 644 /usr/lib/binfmt.d/qemu-aarch64.conf /etc/binfmt.d/qemu-aarch64.conf
sudo sed -i 's|/usr/bin/qemu-aarch64|/usr/bin/qemu-aarch64-static|' /etc/binfmt.d/qemu-aarch64.conf
echo -1 | sudo tee /proc/sys/fs/binfmt_misc/qemu-aarch64
sudo systemctl restart systemd-binfmt
```

Verify the `interpreter` line in `/proc/sys/fs/binfmt_misc/qemu-aarch64`
ends in `-static`. Other distros may register the static handler
automatically on `qemu-user-static` install — check
`/proc/sys/fs/binfmt_misc/qemu-aarch64` before assuming it's broken.
`scripts/deploy.sh` preflights this and refuses to build without a
registered aarch64 handler.

```
./hexa deploy build
./hexa deploy push pi@<host>
```

This ships the image tarball, the compose file, and the launcher
(`hexa` + `scripts/robot.sh`) to `~/hexa-robot/`, loads the image, seeds
`~/hexa-robot/.env` from `.env.robot.sample`, and starts the container **cold**
(relay open, hardware inactive). The shipped launcher is what makes
`./hexa robot <cmd>` work on the Pi.

## 5. Edit `~/hexa-robot/.env` on the Pi

- **`INPUT_GID`** — value from step 3 (typically something like`996`).
- **`ROS_DOMAIN_ID`** — DDS domain, default `42`.
- **`SERVO_DEVICE`** — the servo UART from step 2b. The default `/dev/ttyAMA0`
  is right on a Pi 5; a Pi 4 needs `/dev/ttyS0`.


## 6. Bring up and drive

Run the robot ops on the Pi (the launcher was shipped in step 4). `up` boots the
container cold, waits for `controller_manager`, then energizes (relay on + spawns
the controllers); `down` is the safe-stop (relay off + unload, then compose down).
The gamepad and web teleop are part of the container's launch, so the robot is
drivable as soon as `up` finishes — no separate teleop step:

```
ssh pi@<host> 'cd ~/hexa-robot && ./hexa robot up'
ssh pi@<host> 'cd ~/hexa-robot && ./hexa robot down'
```

Or drive them from the workstation with `-H/--host` — `hexa robot` re-dispatches
the command over ssh in `~/hexa-robot`:

```
./hexa robot -H pi@<host> up
./hexa robot -H pi@<host> down
```

`./hexa robot {restart|status|logs|shell}` are the routine container ops against
the local `hexa-robot` service (also `-H`-dispatchable from the workstation). `up`
is the one attended action that energizes; everything else is safe.

Because energizing is a CLI step in `up` — never the container's CMD — a
`restart: unless-stopped` auto-restart (crash / power blip) brings the stack back
**cold** (relay open), so the servos never flail unattended. The cold-start gate
is implemented by passing `hardware_components_initial_state` to
`controller_manager` from `robot.launch.py` when `engage_on_start:=false`, the
only non-default setting in `bringup.launch.py`. No new C++ in `hexa_hardware` —
the relay still toggles in `on_activate` / `on_deactivate`, and the lifecycle
state is held back externally.

## 6b. Wi-Fi hotspot for web teleop (optional)

**Teleop already works by connecting to the Pi's local ip**

The web teleop (`hexa_webteleop`) hosts an HTTP + WebSocket server on
port 8080 inside the container. With `network_mode: host` the server is
reachable on any of the Pi's network interfaces. To let phones connect
without an existing Wi-Fi network, configure the Pi as a standalone AP:

Install hostapd and dnsmasq:

```
sudo apt install -y hostapd dnsmasq
sudo systemctl stop hostapd dnsmasq
```

Configure a static IP on the wireless interface. Add to
`/etc/dhcpcd.conf` (or the NetworkManager equivalent on Pi OS Bookworm+):

```
interface wlan0
    static ip_address=192.168.50.1/24
    nohook wpa_supplicant
```

Configure dnsmasq (`/etc/dnsmasq.conf`):

```
interface=wlan0
dhcp-range=192.168.50.10,192.168.50.50,255.255.255.0,24h
```

Configure hostapd (`/etc/hostapd/hostapd.conf`):

```
interface=wlan0
driver=nl80211
ssid=Hexapod
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=hexapod123
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
```

Point hostapd at the config and enable both services:

```
sudo sed -i 's|^#DAEMON_CONF=""|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd
sudo systemctl enable hostapd dnsmasq
sudo systemctl start hostapd dnsmasq
```

After the Pi reboots, phones can join the **Hexapod** Wi-Fi network
(password `hexapod123`) and navigate to `http://192.168.50.1:8080` to
open the webapp. The container's host-network WS server is reachable
on the AP interface directly — no port mapping or bridge needed.

The webapp coexists with the gamepad: the gamepad owns `/cmd_vel` by
default, and the webapp prompts to claim control when it connects. See
`src/hexa_webteleop/README.md` for the arbitration protocol.

## 6c. Start on boot (optional)

Out of the box the Pi comes up only *partway*: Docker's `restart:
unless-stopped` restarts the container (which boots cold — hardware inactive,
no controllers), and a human still has to run `hexa robot up` to make the robot
drivable. Install the systemd unit to close that gap:

```
cd ~/hexa-robot && ./hexa robot install-service
sudo systemctl start hexa-robot     # or just reboot
```

`install-service` renders `~/hexa-robot/systemd/hexa-robot.service` (shipped by
`hexa deploy push`) for the current user and install directory, writes it to
`/etc/systemd/system/`, and enables it. It needs `sudo`, so run it on a TTY
(`ssh -t` if driving it remotely).

What the unit does on boot:

- **`ExecStart`** — `hexa robot boot`: waits for the Docker daemon, waits for
  the device nodes compose maps (`SERVO_DEVICE`, plus `SPI_DEVICE` / `GPIO_CHIP`
  when `.env` names them — a node can lag the unit at boot), then runs the
  same `up` an operator would: `compose up -d`, wait for `controller_manager`,
  activate `HexaSystem`, spawn both controllers.
- **`ExecStop`** — `hexa robot down`: the safe-stop (relay off + controllers
  unloaded) before the container is removed, so a `systemctl stop`, reboot, or
  shutdown de-energizes cleanly instead of yanking power.

**The robot still boots limp.** Activating the hardware component does not close
the servo relay — `hexa_hardware` only drives `SET RELAY` once `hexa_locomotion`
publishes `true` on `/hardware/relay_cmd`, and the supervisor only asks for that
once the robot has stood. So after boot the hexapod sits folded and unpowered;
pressing **Start** on the gamepad (or publishing `/gait/initialize`) is what
energizes it. Nothing moves unattended.

Inspecting and undoing:

- **`systemctl status hexa-robot`** — expect `active (exited)`, the unit being
  `Type=oneshot` with `RemainAfterExit=yes`.
- **`journalctl -u hexa-robot -b`** — the boot run's pre-flight and energize log.
- **`sudo systemctl disable hexa-robot`** — stop starting on boot. Note that
  while the unit is enabled, a `hexa robot down` no longer survives a reboot by
  design; disable the unit if you want the robot to stay down.
- **`./hexa robot uninstall-service`** — disable and delete the unit entirely.

## 6d. Boot jingle on boot (optional)

With the buzzer wired and the PWM overlay in place (step 2c), enable the
jingle:

```
cd ~/hexa-robot && ./hexa robot install-tune
```

- **`hexa-boot-tune.service`** — a `Type=oneshot` unit, `WantedBy=multi-user.target`,
  running `systemd/boot-tune.sh` as root (exporting a sysfs PWM channel needs
  it). Nothing `Requires=` it, so it is a pure side effect.
- **Separate from `install-service`** on purpose — the buzzer is optional
  hardware, and enabling the ROS stack's boot unit should not silently start
  making noise.
- **`./hexa robot uninstall-tune`** — disable and delete it; the robot boots
  silent again.
- **`journalctl -u hexa-boot-tune -b`** — why it stayed quiet, if it did.

`hexa robot up` / `down` keep working unchanged with the unit installed.

## 7. Re-deploy

```
./hexa deploy build
./hexa deploy push pi@<host>
```

The container restarts cold after each redeploy — re-run `hexa robot up`.
