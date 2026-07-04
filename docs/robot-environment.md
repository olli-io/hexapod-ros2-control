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
- Pimoroni Servo 2040 over USB (enumerates as `/dev/ttyACM0`).
- Servo rail PSU behind the Servo 2040's relay.
- Wired Ethernet or Wi-Fi.
- Optional: 256×64 SH1122 OLED face on the Pi SPI bus (spidev0.0 +
  DC/RST/CS on GPIO), rendered directly by `hexa_display`.

## 1. Flash the OS

Use **Raspberry Pi OS Lite (64-bit)** via `rpi-imager`. In advanced options
set hostname, username, enable SSH with your public key, and configure Wi-Fi
if needed.

## 2. Install Docker

```
sudo apt update && sudo apt full-upgrade -y
sudo reboot
curl -fsSL https://get.docker.com | sh
sudo apt install i-y docker-compose-plugin git
sudo usermod -aG docker $USER
```
Exit and re-enter the ssh session, then verify that docker runs:

```
docker run --rm hello-world
```
You may need to run:

```
sudo usermod -aG docker <your_username>
```

## 2b. Enable the display SPI (optional)

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

## 3. Note hardware IDs

Plug in the Servo 2040, then on the Pi:

```
ls /dev/serial/by-id/                       # note the usb-Rasperry-Pi-Pico... path
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
- **`SERVO_DEVICE`** — the `/dev/serial/by-id/usb-Pimoroni_Servo_2040-...`
  path from step 3.


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

## 7. Re-deploy

```
./hexa deploy build
./hexa deploy push pi@<host>
```

The container restarts cold after each redeploy — re-run `hexa robot up`.
