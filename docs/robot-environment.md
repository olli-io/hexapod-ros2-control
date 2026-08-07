# Robot environment

Taking a fresh Raspberry Pi 4 or 5 from a blank SD card to a host ready to
receive `./hexa deploy push`, plus the `./hexa deploy` / `./hexa robot`
workflow. See [`sim-environment.md`](sim-environment.md) for the
sim/workstation side.

The robot image is a separate `robot.Dockerfile`, cross-built for `linux/arm64`
on the workstation, shipped to the Pi as an image tarball, and run as a
long-lived service.

Sections 1–8 are the required path, in the order you do them. Everything from
§9 on is optional or occasional; if you are fitting the display, the buttons or
the buzzer, read §12–§15 before §5 so their `config.txt` lines and `.env` keys
land in the first deploy.

## Hardware

- Raspberry Pi 4 or 5 with a 16 GB+ microSD card.
- Pimoroni Servo 2040 on the Pi header UART, GPIO14/15.
- Servo rail PSU behind the Servo 2040's relay.
- Wired Ethernet or Wi-Fi.
- Optional: 256×64 SH1122 OLED face on SPI0 (§12).
- Optional: two momentary push buttons on GPIO5 / GPIO6 (§13).
- Optional: passive buzzer on GPIO12 (§15).

Pi GPIO allocation, so nothing added later steals a line. All numbers are
**BCM GPIO**, never header pin positions — BCM is what `pinctrl`, the
`gpiochip` character device, and `config.txt` use:

- **GPIO14, GPIO15** — UART0 TXD / RXD, the Servo 2040 link.
- **GPIO8, GPIO9, GPIO10, GPIO11** — SPI0 (CE0, MISO, MOSI, SCLK) for the face.
  CE0 is driven by the SPI controller, not by `hexa_display` (`cs_line: -1`).
- **GPIO24, GPIO25** — the face's DC / RST control lines.
- **GPIO12** — hardware PWM for the buzzer.
- **GPIO5, GPIO6** — the front-panel buttons: info and Bluetooth. Each switch
  goes between its line and ground; the internal pull-up holds it high, so no
  external resistor is needed.
- **GPIO2, GPIO3** — I²C1, left free for an MPU6500 IMU.

## 1. Flash the OS

Use **Raspberry Pi OS Lite (64-bit)** via `rpi-imager`. In advanced options set
hostname, username, enable SSH with your public key, and configure Wi-Fi if
needed.

## 2. Install Docker

```
sudo apt update && sudo apt full-upgrade -y
sudo reboot
curl -fsSL https://get.docker.com | sh
sudo apt install -y docker-compose-plugin git
sudo usermod -aG docker $USER
```

Exit and re-enter the ssh session, then verify:

```
docker run --rm hello-world
```

## 3. Enable the servo UART

Wire the Servo 2040 to the header UART, crossed, with a common ground:

- **Pi GPIO14 (UART0 TXD)** — Servo 2040 **RX**.
- **Pi GPIO15 (UART0 RXD)** — Servo 2040 **TX**.
- **Pi GND** — Servo 2040 **GND**. Both boards keep their own supply.

**Pi 5** — the GPIO14/15 UART is off by default:

```
# /boot/firmware/config.txt
dtparam=uart0=on
```

After a reboot it is **`/dev/ttyAMA0`**. Leave `cmdline.txt` alone; the serial
console is on the separate debug connector.

**Pi 4** — GPIO14/15 is the mini-UART and the console sits on it. Enable the
port and evict the getty:

```
sudo raspi-config nonint do_serial_hw 0     # enable the hardware UART
sudo raspi-config nonint do_serial_cons 1   # disable the serial login console
sudo reboot
```

The device is **`/dev/ttyS0`**. Verify after the reboot:

```
ls -l /dev/ttyAMA0            # Pi 5   (Pi 4: /dev/ttyS0)
pinctrl get 14,15             # expect a1/uart function, not "none"
```

Two traps:

- **Do not add `dtoverlay=disable-bt`.** The gamepad pairs over the onboard
  Bluetooth, and that overlay steals the radio's UART.
- **Do not use `/dev/serial0`.** On a Pi 5 it is the debug connector, not the
  header UART.

## 4. Note hardware IDs

```
ls -l /dev/ttyAMA0                          # servo UART from §3 (Pi 4: ttyS0)
getent group input | cut -d: -f3            # INPUT_GID (example: 994)
```

## 5. First install

Two ways in. Either gets you the same `~/hexa-robot/` layout.

**From a release, on the Pi** — no workstation and no cross-build:

```
curl -fsSL https://raw.githubusercontent.com/olli-io/hexapod-ros2-control/main/install.sh | bash
```

`install.sh` checks the dependencies before it downloads anything (64-bit
userland, Docker + compose v2, free space, RAM, and the UART / SPI / PWM this
section's wiring should have produced), then pulls the release's ARM64 image
tarball plus the matching compose, launcher, systemd templates, `tuning.yaml`
and buzzer player, loads the image, and seeds `.env` with **this** Pi's group
IDs and device names — so §4 and §6 are already done for you. It starts
nothing unless you pass `--start`. Useful flags: `--check-only` (checks, then
stop), `--tag <tag>` (a specific release), `--dir <path>`, `--keep-archive`.
Re-running it later is the upgrade path: it keeps your `.env` values and only
appends keys the release added, exactly as `sync-config` does (§9).

Those releases are produced by `.github/workflows/release.yml`: pushing a
`release-x.x.x` tag reachable from `main` builds this image on a native arm64
runner and attaches it as the single `hexa-robot_arm64_<tag>.tar.gz` asset that
`install.sh` looks for. The support files come from the same tag's source
archive, so the tag has to be a real one on the released commit. §5a below is
the alternative path, not the only one.

**From a workstation** — the development path, and the rest of this section:

## 5a. First deploy from the workstation

`./hexa deploy build` cross-compiles `linux/arm64` under QEMU, so the
workstation needs an aarch64 binfmt_misc handler pointing at a **static** QEMU
interpreter. On Arch, set it up once:

```
sudo install -m 644 /usr/lib/binfmt.d/qemu-aarch64.conf /etc/binfmt.d/qemu-aarch64.conf
sudo sed -i 's|/usr/bin/qemu-aarch64|/usr/bin/qemu-aarch64-static|' /etc/binfmt.d/qemu-aarch64.conf
echo -1 | sudo tee /proc/sys/fs/binfmt_misc/qemu-aarch64
sudo systemctl restart systemd-binfmt
```

Verify the `interpreter` line in `/proc/sys/fs/binfmt_misc/qemu-aarch64` ends in
`-static`. Other distros may register the static handler automatically —
check before assuming it's broken. `scripts/deploy.sh` refuses to build without
a registered aarch64 handler.

```
./hexa deploy build
./hexa deploy push pi@<host>
```

This ships the image tarball, the compose file, and the launcher (`hexa` +
`scripts/robot.sh`) to `~/hexa-robot/`, loads the image, seeds
`~/hexa-robot/.env` from `.env.robot.sample`, and starts the container **cold**
(relay open, hardware inactive).

Two files on the Pi are config rather than image content:

- **`~/hexa-robot/.env`** — seeded once, never touched by a redeploy. It holds
  host-specific GIDs and device names. Use `hexa deploy sync-config` (§9) to
  pick up keys added to the sample later.
- **`~/hexa-robot/tuning.yaml`** — refreshed from the repo on **every** deploy
  (the bind-mount shadows the image's baked copy). If the on-Pi file differs,
  deploy saves it as `tuning.yaml.bak` and says so, then overwrites. Tune on the
  Pi freely between deploys (`hexa robot restart` re-reads it) — just fold
  anything worth keeping back into `src/hexa_description/config/tuning.yaml`.

## 6. Edit `~/hexa-robot/.env` on the Pi

- **`SERVO_DEVICE`** — the servo UART from §3. Default `/dev/ttyAMA0` is right
  on a Pi 5; a Pi 4 needs `/dev/ttyS0`.
- **`INPUT_GID`** — value from §4 (typically something like `996`).
- **`ROS_DOMAIN_ID`** — DDS domain, default `42`.
- **`SPI_GID`**, **`GPIO_GID`** — from §12, if the display or buttons are fitted.

## 7. Bring up and drive

The container energizes itself on launch, and the gamepad and web teleop are
part of that launch, so the robot is drivable as soon as `up` finishes:

```
ssh pi@<host> 'cd ~/hexa-robot && ./hexa robot up'
ssh pi@<host> 'cd ~/hexa-robot && ./hexa robot down'
```

Or drive it from the workstation with `-H/--host`, which re-dispatches over ssh:

```
./hexa robot -H pi@<host> up
./hexa robot -H pi@<host> down
```

`./hexa robot {restart|status|logs|shell}` are the routine container ops, also
`-H`-dispatchable.

**What `up` leaves you with:** the servo rail closes a moment later and the
robot takes up its **folded pose under power**, one leg at a time (rear → front,
`init.sweep_leg_interval_ms` apart — 150 ms by default — to keep inrush off the
board's over-current trip; set it to `0` in `hardware.yaml` to energize all at
once). It goes no further: pressing **Start** on the gamepad (or publishing
`/gait/initialize`) is what stands it up. Nothing walks unattended, and an
auto-restart after a crash comes back energized but stationary.

Folding again (Start from a stand) parks the feet and drops the rail, leaving
the robot limp — the same state `hexa robot down` leaves it in. An over-current
trip also drops the rail and holds it open until Start recovers.

## 8. Re-deploy

```
./hexa deploy build
./hexa deploy push pi@<host>
```

The container restarts cold after each redeploy — re-run `hexa robot up`.

## 9. Refreshing config without a deploy

```
./hexa deploy sync-config pi@<host>
```

Config-only: no image, nothing restarted. Use it when the repo's defaults moved
but the image did not.

- **`.env`** — appends only the keys the Pi is **missing**, with their comments;
  existing values are kept. Old file → `.env.bak`. Keys the Pi has and the
  sample lacks are reported, not removed.
- **`--force`** — overwrite `.env` from the sample instead, host-specific values
  included. For re-provisioning. Still backs up.
- **`tuning.yaml`** — refreshed from the repo, on-Pi edit → `tuning.yaml.bak`.
- **`systemd/`** — re-ships `network-mode.sh` and the unit templates, plus
  `hexa_buzzer/` (the tune player the boot and shutdown units run). Scripts go
  live at once; installed units are rendered copies, so re-run the matching
  `hexa robot install-*`.

`.env` changes need a container recreate (`hexa robot -H <host> restart`), which
also re-reads `tuning.yaml`.

## 10. Undervoltage ladder

A draining pack is handled in three escalating rungs, off `~/battery_state`:

- **rung 1 — warn.** The `undervolt` tune sounds once and the status LED goes to
  the fault cadence. The robot stays **drivable**, so it can be walked back to
  the bench.
- **rung 2 — fold.** The gait command is zeroed and a fold is queued; the rail
  is cut once the legs are parked.
- **rung 3 — cutoff.** The rail is cut immediately, whatever the posture, and
  latched open.

Two properties to know before tuning it:

- **The ladder only escalates.** Cutting the rail unloads the pack and the
  voltage rebounds, so it never de-escalates on its own.
- **A cutoff is cleared by power-cycling the robot**, and nothing else — it
  survives a locomotion restart, and `~/reload_config` is refused from rung 2 up.

Thresholds live in `battery:` in `hexa_description/config/hardware.yaml`. They
ship **disabled** (`0.0` on all three rungs) because the Servo 2040's voltage
divider is uncalibrated. Measure the pack against `~/battery_state` first, then
set all three in descending order (codegen rejects a mis-ordered ladder). Until
then the robot beeps, folds and cuts for over-current only.

## 11. Start on boot (optional)

Docker's `restart: unless-stopped` alone races the device nodes compose maps at
power-on, so a fresh boot can fail container-create outright. Install the
systemd unit, which pre-flights the Docker daemon and those nodes before
bringing the stack up, and safe-stops on shutdown:

```
cd ~/hexa-robot && ./hexa robot install-service
sudo systemctl start hexa-robot     # or just reboot
```

It needs `sudo`, so run it on a TTY (`ssh -t` if remote).

- **`ExecStart`** — `hexa robot boot`: wait for Docker, wait for `SERVO_DEVICE`
  (plus `SPI_DEVICE` / `GPIO_CHIP` when `.env` names them), then the same `up`
  an operator would run.
- **`ExecStop`** — `hexa robot down`: relay off and controllers unloaded before
  the container is removed.

Inspecting and undoing:

- **`systemctl status hexa-robot`** — expect `active (exited)`; the unit is
  `Type=oneshot` with `RemainAfterExit=yes`.
- **`journalctl -u hexa-robot -b`** — the boot run's pre-flight and bring-up log.
- **`sudo systemctl disable hexa-robot`** — stop starting on boot. While the
  unit is enabled a `hexa robot down` does not survive a reboot.
- **`./hexa robot uninstall-service`** — disable and delete the unit.

## 12. Display SPI (optional hardware)

Only needed if the SH1122 OLED face is fitted.

```
# /boot/firmware/config.txt
dtparam=spi=on
```

Reboot, then note the group IDs for `.env` (§6):

```
ls -l /dev/spidev0.0 /dev/gpiochip0
getent group spi  | cut -d: -f3            # SPI_GID
getent group gpio | cut -d: -f3            # GPIO_GID
```

Compose maps both device nodes into the container and forwards `SPI_GID` /
`GPIO_GID` from `.env`. Wiring and render rate live in `hexa_display`'s
`config/display.yaml`.

Without the display fitted, set `enabled: false` in that file; otherwise
`hexa_display` aborts at startup when it cannot open the panel.

## 13. Front-panel buttons (optional hardware)

Two momentary push buttons, each between its BCM line and ground:

- **GPIO5** (physical pin 29, ground on 30) — battery percentage/voltage plus
  the web teleop address; hold 3 s to toggle Wi-Fi hotspot mode (§14).
- **GPIO6** (physical pin 31, ground on 34) — connected-controller status; hold
  3 s to request a Bluetooth pairing scan.

No `config.txt` change and no extra compose entry: `hexa_buttons` uses the same
`/dev/gpiochip0` and `gpio` group as §12. Line numbers, polarity and timing are
in `hexa_buttons`'s `config/buttons.yaml`.

Both screens land on the face's panel, so the display must be fitted to see
them. Without the buttons fitted, set `enabled: false` in `buttons.yaml`;
leaving it on with nothing wired is harmless — the node logs that it could not
claim the lines and stays inert.

## 14. Wi-Fi hotspot for web teleop (optional)

**Teleop already works by connecting to the Pi's local IP** — `hexa_webteleop`
serves port 8080 on every interface. Nothing here is needed on a network you
already have.

What this adds is a way to reach the robot where there is no such network:
**hold the info button (GPIO5) for three seconds** and the Pi stops joining
Wi-Fi and starts hosting its own. Hold it again to go back. The info screen
carries the credentials.

- **network** — `hexapod`
- **password** — `hexahexa`
- **the robot** — `http://control.hexa/`, or `http://192.168.4.1/`, or any
  address at all: the hotspot resolves every hostname to the robot, and the
  teleop server sends anything that is not one of its own files to the
  controller page.

Usually there is nothing to type. The hotspot advertises the controller's URL
in the DHCP lease (RFC 8910) and leaves every connectivity probe unanswered, so
a phone joining the network decides it has found a captive portal and opens the
controller by itself — the same popup a café's sign-in page arrives in. Pull
down and reconnect if a phone was already joined before this was installed.

### Install

Shipped by every deploy, installed only when you ask — this can take the Pi off
the network you are ssh'd in over.

```
ssh -t <host> 'cd ~/hexa-robot && ./hexa robot install-network'
```

`uninstall-network` reverses all of it. To switch without a button fitted:

```
./hexa robot network-mode status      # which mode is the radio in
./hexa robot network-mode toggle      # flip it
./hexa robot network-mode hotspot     # or name the target
./hexa robot network-mode station
```

### Requirements

- **NetworkManager managing wlan0** — Pi OS Bookworm or newer. No `hostapd` or
  `dnsmasq` package to install.
- **A Wi-Fi country set** — `raspi-config`, Localisation Options, WLAN Country.
  An AP will not start without a regulatory domain; the panel reports
  `No wifi country is set` if it is missing.

### Consequences worth knowing

- **One radio.** `wlan0` cannot be an AP and a client at once, so entering
  hotspot mode drops any ssh session over Wi-Fi. Switch from ethernet, a
  console, or the button.
- **A reboot always comes back in station mode**, on purpose.
- **A hotspot that fails to start rolls back** to the station profile that was
  up before.
- **Ethernet, if plugged in, is a route.** AP clients are NATed to whatever
  uplink exists; with nothing on `eth0` the hotspot is islanded.
- **The popup browser is a cut-down one.** iOS and Android open captive
  portals in a sandboxed web view, not the real browser: it works, but it has
  no address bar, and dismissing it drops the connection to the robot. "Open in
  browser" (or typing `control.hexa`) moves the controller somewhere it can
  stay.
- **`control.hexa` only exists on the hotspot.** It is the AP's own DNS
  answering, so on any other network the robot is its address as usual — which
  is what the info screen shows there.

The webapp coexists with the gamepad: the gamepad owns `/cmd_vel` by default,
and the webapp prompts to claim control when it connects. See
`src/hexa_webteleop/README.md`.

## 15. Buzzer (optional)

Wire buzzer **+** to GPIO12 (BCM) and **−** to GND; add a ~100 Ω resistor in
series if it is too loud. Then:

```
# /boot/firmware/config.txt
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

The parameters are not optional. A bare `dtoverlay=pwm-2chan` maps GPIO18/19
and leaves GPIO12 an input — every write still succeeds and the wire hears
nothing.

Reboot, then confirm the mux and hear it:

```
pinctrl get 12                      # expect a0 / PWM0_CHAN0, not "none"
cd ~/hexa-robot && ./hexa robot play-tune
```

`play-tune` takes an event name, so each can be auditioned without provoking it
(`./hexa robot play-tune fault`). The events, and the tune each one plays:

- **`boot`** — `chirp`, rising two notes. Kernel is up.
- **`up`** — `ready`, a rising triad. ROS stack live, servo link open.
- **`shutdown`** — `sigh`, falling. Last thing before power is cut; when it
  stops, the switch is safe.
- **`fault`** — `klaxon`, two-tone, ~2 s. Servo 2040 over-current trip.
- **`undervolt`** — `groan`, one flat sustained tone, ~1.8 s. Pack low, still
  drivable.

### Who plays what

Three of the five tunes are played by **`hexa_buzzer`, in the container**, off
`/buzzer/play`. It needs no unit — only the PWM mount below.

The other two cannot be: `boot` lands long before Docker exists, and `shutdown`
at `final.target`, after the container is gone. Those two are host systemd
units, running the **same Python player** (`hexa deploy` ships it, and the two
config files, to `~/hexa-robot/hexa_buzzer/`), so both sides resolve an event
through the same tables:

```
cd ~/hexa-robot && ./hexa robot install-tune
```

- **`hexa-boot-tune.service`** — plays `boot` at multi-user.target.
- **`hexa-shutdown-tune.service`** — plays `shutdown` at final.target, after the
  container is gone and the relay is open.

Separate from `install-service` on purpose: enabling the ROS stack's boot unit
should not silently start making noise. `./hexa robot uninstall-tune` disables
and deletes both; `journalctl -u hexa-boot-tune -u hexa-shutdown-tune -b` says
why one stayed quiet. `hexa robot up` / `down` work unchanged with the units
installed. To silence the container's three instead, set `enabled: false` in
`hexa_buzzer/config/buzzer.yaml` — `uninstall-tune` does not touch them.

### The PWM mount

The container reaches the PWM block through a bind mount at `/pwm`, declared in
`docker-compose.buzzer.yaml`. `hexa robot up` adds that overlay **only when the
host has the tree**, and says which it did:

```
>> Buzzer PWM at /sys/bus/platform/devices/1f00098000.pwm/pwm -> /pwm
```

The check is not a formality: a bind mount whose source does not exist stops the
container from starting at all, and plenty of builds have no buzzer fitted. If
you see the "no PWM block" line instead, the overlay is missing from
`config.txt` — `pinctrl get 12` will say so too.

`BUZZER_PWM` in `.env` overrides the source directory. The default is the Pi 5's
RP1 PWM0; on a Pi 4 the block sits at a different platform address, so run
`ls -d /sys/bus/platform/devices/*.pwm/pwm` and set it. Channel 0 is GPIO12 on
both. It is reached by platform address rather than by `pwmchipN` number because
that number is kernel probe order and has moved between releases.

A writable bind of that one directory is all it takes — the container already
runs as root, so there is no udev rule or supplementary group involved. Before
this, the container could not make a sound at all: `/sys` is mounted read-only,
so an export from inside failed with `EROFS`, and a tune had to be requested by
writing a name into a file on the log volume for a host `.path` unit to relay to
a shell script. That spool and its two units are gone.

### Tunes

`hexa_buzzer/config/tunes.yaml` holds the tunes, in RTTTL — the Nokia ringtone
format, `name:defaults:notes`. `b` is beats per minute, `d` and `o` the note
value and octave a note gets when it gives none, and a duration is a fraction of
a whole note (`16b5` is a sixteenth-note B5, `8p` an eighth rest). Which tune an
event plays is the `events:` map in `hexa_buzzer/config/buzzer.yaml`, so giving
`fault` a different voice is one word there, and a new tune needs no code at
all. Audition one without editing anything:

```
cd ~/hexa-robot
sudo PYTHONPATH=. python3 -m hexa_buzzer.player \
    --rtttl "coin:d=4,o=6,b=200:16b5,2e"
```

Any ringtone in the format plays as-is, which is most of why it was chosen.
There is no tempo setting outside the string: each tune carries its own `b=`,
so a pasted one keeps the tempo it was written at. `--pwm-dev` and `--channel`
take a different block or a different buzzer pin (plus matching overlay
parameters).

Every hardware failure logs a line and exits 0, so a tune can never hold up a
boot, a shutdown, or the fault path that asked for it.
