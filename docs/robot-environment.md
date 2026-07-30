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
- Optional: passive buzzer on GPIO12 for the boot, shutdown, stack-up,
  over-current, and undervoltage tunes.
- Optional: two momentary push buttons on GPIO5 / GPIO6 for the front-panel
  info screens, read by `hexa_buttons`.

Pi GPIO allocation, so nothing added later steals a line. All numbers are
**BCM GPIO**, never header pin positions — the two differ on every Pi, and BCM
is what `pinctrl`, the `gpiochip` character device, and `config.txt` use:

- **GPIO14, GPIO15** — UART0 TXD / RXD, the Servo 2040 link.
- **GPIO8, GPIO9, GPIO10, GPIO11** — SPI0 (CE0, MISO, MOSI, SCLK) for the
  SH1122 face. CE0 is driven by the SPI controller, not by `hexa_display`
  (`cs_line: -1`): the Pi 5's spi0 node claims GPIO8 as `spi0 CS0`, so
  userspace cannot request the line, and its RP1 controller rejects
  `SPI_NO_CS` (`unsupported mode bits 40` in `dmesg`) — the two together rule
  out a manually driven CS on a Pi 5.
- **GPIO24, GPIO25** — the face's DC / RST control lines.
- **GPIO12** — hardware PWM for the buzzer (RP1 PWM0 channel 0, alt function
  `a0`).
- **GPIO5, GPIO6** — the front-panel buttons (`hexa_buttons`): info and
  Bluetooth. Each switch goes between its line and ground; the SoC's internal
  pull-up holds it high, so no external resistor is needed. Deliberately clear
  of I²C1 below, so an IMU can still be added.
- **GPIO2, GPIO3** — I²C1, free for an MPU6500 IMU.

The buzzer is deliberately **not** on GPIO18, the pin most PWM examples reach
for: that line is I²S PCM_CLK and SPI1 CE0, so it would rule out an audio HAT
and collide with a second SPI device on the aux bus. GPIO12 costs nothing.

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

## 2d. Front-panel buttons (optional)

Two momentary push buttons, each between its BCM line and ground:

- **GPIO5** (physical pin 29, ground on 30) — battery percentage/voltage plus
  the address for the web teleop UI.
- **GPIO6** (physical pin 31, ground on 34) — connected-controller status; hold
  3 s to request a Bluetooth pairing scan.

No `config.txt` change and no extra compose entry: `hexa_buttons` reads the same
`/dev/gpiochip0` the face already uses, so the device mapping and the `gpio`
group from section 2c cover it. Wiring polarity, the line numbers and the
timing are in `hexa_buttons`'s `config/buttons.yaml`.

The node reads the lines with **gpiozero** on an explicitly pinned **lgpio** pin
factory — edge-driven, so nothing is polled between presses. `python3-gpiozero`
and `python3-lgpio` are installed by `robot.Dockerfile`'s runtime stage, which
also bakes in `GPIOZERO_PIN_FACTORY=lgpio`: gpiozero's default factory search
ends in `native`, which maps BCM283x registers directly and on a Pi 5's RP1
reads garbage rather than failing, so it is never left to guess. gpiozero 2.0 or
newer is required for Pi 5 board data — Ubuntu 24.04 ships 2.0.1.

Both screens land on the face's panel, so they need the display fitted to be
visible. Without the buttons fitted, set `enabled: false` in
`hexa_buttons/config/buttons.yaml`; leaving it on with nothing wired is
harmless — the node logs that it could not claim the lines and stays inert.

## 2d. Enable the buzzer PWM (optional)

Only needed if the passive buzzer is fitted. Wire buzzer **+** to GPIO12 (BCM)
and **−** to GND; add a ~100 Ω resistor in series if it is too loud.

Tunes are played by `systemd/buzzer.sh` — POSIX shell writing the kernel's
sysfs PWM interface, with no Python, gpiozero, or any other package installed
on the host. (The *container* does carry gpiozero, for `hexa_buttons`; that is
a separate world, and the buzzer script deliberately depends on nothing.)
It runs on the **Pi host**, never in the container, for two reasons: the boot
tune has to chirp seconds after the kernel hands off, long before Docker and
the ROS stack exist; and the container cannot reach the PWM at all — Docker
mounts `/sys` read-only, and every `/sys/class/pwm/pwmchipN` is a symlink into
`/sys/devices`, so an export from inside fails with `EROFS` however the class
directory is bound.

The robot has five tunes, differing in contour so they are told apart through a
closing door:

- **`boot`** — rising two notes. The Pi has power and the kernel is up. Played
  by `hexa-boot-tune.service`.
- **`up`** — rising triad. The ROS stack is live and the servo link is open
  (`hexa_hardware` activated). Requested by the container.
- **`shutdown`** — falling, the mirror of `boot`. The last thing before power
  is cut: when it stops, the switch is safe. Played by
  `hexa-shutdown-tune.service`.
- **`fault`** — two-tone klaxon, repeated, ~2 s. The Servo 2040 latched an
  over-current trip and dropped the rail. Requested by the container.
- **`undervolt`** — one flat sustained tone, ~1.8 s. Rung 1 of the undervoltage
  ladder: the pack is low but the robot is **still drivable**, so walk it back
  and charge it. The opposite contour to `fault` because it calls for the
  opposite response. Requested by the container, once per power cycle.

The three container-borne tunes take a detour, since the container cannot drive
the buzzer itself: `hexa_hardware` writes the tune name into
`/workspace/log/buzzer` (the `buzzer.spool` path in `hardware.yaml`), which
compose bind-mounts from `~/hexa-robot/log`, and the host's
`hexa-tune-spool.path` unit sees the write and runs `buzzer.sh --spool` with
it. Blank `buzzer.spool` to stop the requests at the source.

Enable the PWM block:

```
# /boot/firmware/config.txt
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

Bare `dtoverlay=pwm-2chan` would map GPIO18/19 instead, so the pins are named
explicitly; `func=4` is ALT0, which `pinctrl` calls `a0`.

On the Pi 5 the PWM lives in the RP1 southbridge: GPIO12 is PWM0 **channel 0**,
alt function `a0`. Reboot, then confirm the sysfs tree and hear it:

```
ls /sys/class/pwm/                  # expect pwmchip0 (SoC, 2ch) + the RP1 chip (4ch)
cat /sys/class/pwm/pwmchip*/npwm    # the 4-channel one is RP1's
cd ~/hexa-robot && ./hexa robot play-tune
```

`play-tune` takes a tune name, so each one can be heard without provoking it —
`./hexa robot play-tune fault` is the only sane way to audition that one.

The chip number moves with the kernel and the overlays in play, so
`buzzer.sh` discovers it (4 channels = RP1) rather than hardcoding
`pwmchip2`, and re-asserts the pin's alt mode with `pinctrl` in case the
overlay in `config.txt` did not. If discovery picks wrong, pin it with
`TUNE_PWMCHIP=/sys/class/pwm/pwmchipN`.

Tune it without editing the script — the same `NOTE:beats` melody format the
gpiozero recipes use, `REST` for silence. `TUNE_MELODY` overrides whichever
tune was named:

```
TUNE_MELODY="C5:1 E5:1 G5:1 C6:1 REST:1 G5:1 C6:3" TUNE_TEMPO=0.11 \
    ./hexa robot play-tune
```

`TUNE_GPIO`, `TUNE_CHANNEL`, and `TUNE_PIN_ALT` cover a different buzzer pin.
Every hardware failure — no buzzer, no overlay, busy channel — logs a line and
exits 0, so a tune can never hold up a boot, a shutdown, or the fault path that
asked for it.

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

Two files on the Pi are config rather than image content, and they update
differently:

- **`~/hexa-robot/.env`** — seed-once from `.env.robot.sample`. It holds
  host-specific GIDs and device names the repo cannot know, so a redeploy never
  touches it once it exists. `hexa deploy sync-config` (below) is how a key
  added to the sample later still reaches an already-provisioned Pi.
- **`~/hexa-robot/tuning.yaml`** — refreshed from the repo on **every** deploy.
  The compose bind-mount puts this file over the image's baked copy, so a
  seed-once overlay would shadow every tuning change you ever deploy and pin
  the schema the Pi was first provisioned with. If the on-Pi file differs from
  the repo's, deploy saves it as `tuning.yaml.bak` and says so, then overwrites.
  Tune on the Pi freely between deploys (`hexa robot restart` re-reads it) —
  just fold anything worth keeping back into
  `src/hexa_description/config/tuning.yaml`.

### 4a. Refreshing config without a deploy

```
./hexa deploy sync-config pi@<host>
```

Config-only: no image, nothing restarted. Use it when the repo's defaults moved
but the image did not — a new key in `.env.robot.sample`, a tuning change, or an
edit to a host-side script like `systemd/buzzer.sh`, which lives outside the
image entirely.

- **`.env`** — appends the keys the Pi is **missing**, with their comments;
  values already set are kept (`INPUT_GID`, `SERVO_DEVICE` and friends are facts
  about this Pi). Old file → `.env.bak`. Keys the Pi has and the sample lacks are
  reported, not removed.
- **`--force`** — overwrite `.env` from the sample instead, host-specific values
  included. For re-provisioning. Still backs up.
- **`tuning.yaml`** — refreshed from the repo, on-Pi edit → `tuning.yaml.bak`.
- **`systemd/`** — re-ships `buzzer.sh`, `network-mode.sh` and the unit
  templates. Shipped, never installed. The scripts go live at once; installed
  units are rendered copies, so the command names the `hexa robot install-*` to
  re-run.

Image, compose file, and launcher stay `push`'s business — shipping them here
would leave code with no matching image. `.env` changes need a container
recreate (`hexa robot -H <host> restart`), which also re-reads `tuning.yaml`.

## 5. Edit `~/hexa-robot/.env` on the Pi

- **`INPUT_GID`** — value from step 3 (typically something like`996`).
- **`ROS_DOMAIN_ID`** — DDS domain, default `42`.
- **`SERVO_DEVICE`** — the servo UART from step 2b. The default `/dev/ttyAMA0`
  is right on a Pi 5; a Pi 4 needs `/dev/ttyS0`.


## 6. Bring up and drive

Run the robot ops on the Pi (the launcher was shipped in step 4). The container
energizes itself on launch — `robot.launch.py` brings `HexaSystem` active and
spawns both controllers — so `up` just does `compose up -d` and waits for
`controller_manager` to report ready; `down` is the safe-stop (relay off + unload,
then compose down). The servo rail closes a moment later and the robot settles
into its folded pose leg by leg (see the energize-sweep note in §6c). The gamepad
and web teleop are part of the container's launch, so the robot is drivable as
soon as `up` finishes — no separate teleop step:

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
the local `hexa-robot` service (also `-H`-dispatchable from the workstation).

Energizing spawns the controllers and activates `HexaSystem`. Activating the
component does not itself touch the relay: it closes when `hexa_locomotion`
publishes `true` on `/hardware/relay_cmd`, which the supervisor asks for once
teleop is publishing — normally within a second of launch, while the engine is
still `folded`. The robot then takes up its **folded pose under power** and stops
there; standing takes a gamepad **Start** (or `/gait/initialize`). So a
`restart: unless-stopped` auto-restart (crash / power blip) brings the stack back
**energized but stationary**: the servos never flail unattended.
Energize-on-launch is implemented by passing
`hardware_components_initial_state: {active: [HexaSystem]}` to `controller_manager`
from `robot.launch.py`, which also spawns both controllers (see the energize-sweep
note below).

## 6b. Wi-Fi hotspot for web teleop (optional)

**Teleop already works by connecting to the Pi's local ip.** The web teleop
(`hexa_webteleop`) hosts an HTTP + WebSocket server on port 8080 inside the
container, and with `network_mode: host` that server is reachable on every one
of the Pi's interfaces. Nothing below is needed on a network you already have.

What this adds is a way to reach the robot where there is no such network:
**hold the info button (GPIO5) for three seconds** and the Pi stops joining
Wi-Fi and starts hosting its own. Hold it again to go back. The face wears its
scanning spinners while the switch runs, and the info screen then carries the
credentials.

- **network** — `hexapod`
- **password** — `hexahexa`
- **the robot** — `http://192.168.4.1/`, or any address at all (see the portal below)

### Install

Shipped by every deploy, installed only when you ask — the same opt-in shape as
the buzzer units, and for a stronger reason: this can take the Pi off the
network you are ssh'd in over.

```
ssh -t <host> 'cd ~/hexa-robot && ./hexa robot install-network'
```

That renders the three unit templates into `/etc/systemd/system`, enables them,
writes the captive-DNS drop-in, and seeds the state file. `uninstall-network`
reverses all of it. To switch without a button fitted:

```
./hexa robot network-mode status      # which mode is the radio in
./hexa robot network-mode toggle      # flip it
./hexa robot network-mode hotspot     # or name the target
./hexa robot network-mode station
```

### Requirements

- **NetworkManager managing wlan0** — Pi OS Bookworm or newer. `nmcli` does all
  the work, so there is no `hostapd` and no `dnsmasq` package to install: an AP
  profile with `ipv4.method=shared` makes NetworkManager run its own dnsmasq for
  DHCP and DNS on that interface.
- **A Wi-Fi country set** — `raspi-config`, Localisation Options, WLAN Country.
  An access point will not start without a regulatory domain; the script checks
  for this and reports `No wifi country is set` on the panel rather than failing
  opaquely.

### How the button reaches the host

The ROS stack runs in an unprivileged container — host networking, non-root
user, no D-Bus socket, no `NET_ADMIN` — so it cannot talk to NetworkManager at
all. It uses the same escape hatch as the buzzer: `hexa_buttons` writes a
request into the bind-mounted log volume, and a host `systemd .path` unit runs
`systemd/network-mode.sh` out here.

- **`log/network`** — container to host. One line: an action and a token.
  Watched by `hexa-network-spool.path`, which runs `hexa-network-spool.service`.
- **`log/network.state`** — host to container. `key=value` lines naming the
  mode, the credentials, and how the last request went. Written tmp + rename so
  a 20 Hz poll can never read half a line.
- **`hexa-network-report.service`** — writes that state file at boot, so the
  container knows which mode the Pi came up in without having to ask.

The host is the authority on which mode the radio is in — the AP profile being
active is the whole definition — which is why the button sends `toggle` rather
than naming a target. Failures come back as short tokens the panel turns into
sentences: `Could not start the hotspot`, `No network helper installed`,
`wlan0 is not managed by NM`, and so on.

### The captive portal

Two pieces, no reverse proxy. `hexa_webteleop` already binds `0.0.0.0:8080`, so
nginx or traefik in front of it would be a hop and a daemon for nothing.

- **Wildcard DNS** — `/etc/NetworkManager/dnsmasq-shared.d/hexa-captive.conf`
  holds `address=/#/192.168.4.1`, so every hostname resolves to the robot.
  NetworkManager only starts the dnsmasq that reads it for a `shared`
  connection, so the file is inert in station mode; it is written once at
  install rather than toggled.
- **A port 80 redirect** — an nftables rule in its own `hexa_portal` table sends
  port 80 on the AP interface to 8080, so the address needs no `:8080` on the
  end. This also catches clients that ignore DHCP's DNS and hard-code a
  resolver, which the wildcard alone cannot reach. Added when the hotspot comes
  up, torn down with one `nft delete table` when it goes.

The OS probe URLs are deliberately **not** hijacked, so phones show their usual
"no internet" notice instead of auto-opening a sign-in window. That is the
better trade for this app: iOS's Captive Network Assistant is a cut-down
browser, and the teleop UI is a WebSocket gamepad surface that wants a real one.
Open `http://hexapod/` — or anything else — in Safari or Chrome.

### Consequences worth knowing

- **One radio.** `wlan0` cannot be an access point and a client at the same
  time, so entering hotspot mode drops any ssh session over Wi-Fi, and the
  hotspot has no route to the internet. That islanding is the point, but it also
  means the switch is one-way from a Wi-Fi shell — use ethernet, a console, or
  the button.
- **A reboot always comes back in station mode.** The AP profile is created with
  `autoconnect no` on purpose: a robot that crashed and came back hosting an AP
  would be unreachable from the workstation.
- **A hotspot that fails to start rolls back.** The script records the station
  profile that was up before it touches anything, and puts it back if the AP
  will not activate, rather than leaving the robot with no network at all.
- **Ethernet, if plugged in, is a route.** `ipv4.method=shared` NATs AP clients
  to whatever uplink exists. With `wlan0` as the AP and nothing on `eth0` there
  is no uplink and the hotspot is islanded; plug ethernet in and clients get out
  through it.

The webapp coexists with the gamepad: the gamepad owns `/cmd_vel` by default,
and the webapp prompts to claim control when it connects. See
`src/hexa_webteleop/README.md` for the arbitration protocol.

## 6c. Start on boot (optional)

Out of the box, Docker's `restart: unless-stopped` restarts the container after a
crash or reboot, and the container energizes itself on launch — but that races the
device nodes compose maps (`/dev/ttyAMA0` and friends can lag the daemon at
power-on), so a fresh boot can fail container-create outright. Install the systemd
unit to pre-flight the daemon and those nodes before bringing the stack up, and to
safe-stop cleanly on shutdown:

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
  same `up` an operator would: `compose up -d` and wait for `controller_manager`.
  The container energizes itself on launch (activates `HexaSystem`, spawns both
  controllers).
- **`ExecStop`** — `hexa robot down`: the safe-stop (relay off + controllers
  unloaded) before the container is removed, so a `systemctl stop`, reboot, or
  shutdown de-energizes cleanly instead of yanking power.

**The robot boots into the folded pose, one leg at a time.** Activating the
hardware component does not close the servo relay — `hexa_hardware` drives
`SET RELAY` off `/hardware/relay_cmd`, which the supervisor raises once teleop is
publishing (any engine state but `fault`). The board closes the relay with every
servo **limp** and drives a servo only once the host has commanded it, so
`hexa_hardware` staggers that: the legs come up one at a time in pin order —
`l_rear, r_rear, l_middle, r_middle, l_front, r_front`, i.e. rear → front —
`init.sweep_leg_interval_ms` apart (150 ms by default, so ~0.75 s for the whole
sweep). That keeps the combined inrush as six small steps instead of one spike
big enough to trip the board's over-current protection. Set the interval to `0`
in `hardware.yaml` to energize every leg at once.

After the sweep the hexapod sits folded and powered, and goes no further:
pressing **Start** on the gamepad (or publishing `/gait/initialize`) is what
stands it up. Nothing walks unattended. Folding again (Start from a stand) parks
the feet and drops the rail, so the robot ends up limp — the same state a
`hexa robot down` leaves it in. An over-current trip also drops the rail and
holds it open until Start recovers; the sweep re-runs on that edge too.

Inspecting and undoing:

- **`systemctl status hexa-robot`** — expect `active (exited)`, the unit being
  `Type=oneshot` with `RemainAfterExit=yes`.
- **`journalctl -u hexa-robot -b`** — the boot run's pre-flight and bring-up log.
- **`sudo systemctl disable hexa-robot`** — stop starting on boot. Note that
  while the unit is enabled, a `hexa robot down` no longer survives a reboot by
  design; disable the unit if you want the robot to stay down.
- **`./hexa robot uninstall-service`** — disable and delete the unit entirely.

## 6c-2. Undervoltage ladder

A draining pack is handled in three escalating rungs, decided by the locomotion
supervisor (`shared/motion_core/supervisor.hpp`) off `~/battery_state` and
published on `/hardware/undervoltage`:

- **rung 1 — warn.** The `undervolt` tune sounds once and the status LED goes to
  the fault cadence. Nothing else changes: the robot stays **drivable**, so it
  can be walked back to the bench.
- **rung 2 — fold.** The gait command is zeroed and a fold is queued, as a
  **Start** from a stand would. The rail is cut on the `FOLDING → FOLDED` park
  edge, so the legs go down under power instead of collapsing mid-stance.
- **rung 3 — cutoff.** The rail is cut immediately, whatever the posture, and
  latched open — the pack is too far gone to finish a fold.

Two properties to know before tuning it:

- **The ladder only escalates.** Cutting the rail unloads the pack and the
  voltage rebounds past the threshold that just fired, so a ladder that followed
  it back up would cut, re-arm, sag and cut again.
- **A cutoff is cleared by power-cycling the robot**, and nothing else. Nothing
  is written to disk; the latch lives in the supervisor and in `hexa_hardware`,
  so the cut survives a locomotion restart. `~/reload_config` is refused from
  rung 2 up, since a rebuilt pipeline would start clean and re-arm.

Thresholds live in `battery:` in `hexa_description/config/hardware.yaml` — one
source for the supervisor, the baked firmware config, and `hexa_hardware`. They
ship **disabled** (`0.0` on all three rungs) because the Servo 2040's
voltage-divider scale is uncalibrated. Measure the pack against `~/battery_state`
first, then set all three in descending order (codegen rejects a mis-ordered
ladder). Until then the robot beeps, folds and cuts for over-current only.

## 6d. Buzzer units (optional)

With the buzzer wired and the PWM overlay in place (step 2c), enable the four
buzzer units in one go:

```
cd ~/hexa-robot && ./hexa robot install-tune
```

Each runs `systemd/buzzer.sh` as root — exporting a sysfs PWM channel needs it
— and nothing `Requires=` any of them, so they are pure side effects:

- **`hexa-boot-tune.service`** — `Type=oneshot`, `WantedBy=multi-user.target`.
  Plays `boot`.
- **`hexa-shutdown-tune.service`** — `Type=oneshot`, `DefaultDependencies=no`,
  `WantedBy=final.target`. Plays `shutdown` in the last stage of the shutdown
  transaction, after every normal unit (`hexa-robot.service` included) has
  stopped, so the relay is already open and the container already gone.
- **`hexa-tune-spool.path`** — watches `~/hexa-robot/log/buzzer` with
  `PathModified=` and triggers the service below on each write.
  `PathExists=` would stay satisfied and restart it in a loop.
- **`hexa-tune-spool.service`** — plays whatever tune the container last wrote
  (`up`, `fault`, `undervolt`). Triggered only by the `.path`, never enabled on
  its own. It
  only reads the spool: writing it back would be another modification of the
  watched file, and the trigger would loop.

Notes:

- **Separate from `install-service`** on purpose — the buzzer is optional
  hardware, and enabling the ROS stack's boot unit should not silently start
  making noise.
- **`./hexa robot uninstall-tune`** — disable and delete all four; the robot is
  silent again.
- **`journalctl -u hexa-boot-tune -u hexa-shutdown-tune -u hexa-tune-spool -b`**
  — why one stayed quiet, if it did.

`hexa robot up` / `down` keep working unchanged with the units installed.

## 7. Re-deploy

```
./hexa deploy build
./hexa deploy push pi@<host>
```

The container restarts cold after each redeploy — re-run `hexa robot up`.
