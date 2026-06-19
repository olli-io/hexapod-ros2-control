# Robot environment

Steps to take a fresh Raspberry Pi 4 or 5 from a blank SD card to a host ready
to receive `./hexa --prod deploy`. See [`dev-environment.md`](dev-environment.md)
for the workstation side.

## Hardware

- Raspberry Pi 4 or 5 with a 16 GB+ microSD card.
- Pimoroni Servo 2040 over USB (enumerates as `/dev/ttyACM0`).
- Servo rail PSU behind the Servo 2040's relay.
- Wired Ethernet or Wi-Fi.
- Optional: ESP32 OLED face on the Pi UART header (GPIO14/15,
  firmware in the `hexapod-esp32-display` repo).

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

## 2b. Enable the display UART (optional)

Only needed if the ESP32 face is fitted. Free the PL011 UART from
Bluetooth so `/dev/serial0` points at the header pins:

```
# /boot/firmware/config.txt
enable_uart=1
dtoverlay=disable-bt
```

Reboot, then verify `/dev/serial0` resolves to `ttyAMA0`:

```
ls -l /dev/serial0
```

The prod compose maps the resolved device (`DISPLAY_DEVICE`, default
`/dev/ttyAMA0`) into the container as `/dev/serial0`. Without the
display fitted the node simply keeps retrying in the background.

## 3. Note hardware IDs

Plug in the Servo 2040, then on the Pi:

```
ls /dev/serial/by-id/                       # note the usb-Rasperry-Pi-Pico... path
getent group input | cut -d: -f3            # note the input GID (example: 994)
```

## 4. First deploy from the workstation

`./hexa --prod build` cross-compiles `linux/arm64` under QEMU, so the
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
`scripts/prod.sh` preflights this and refuses to build without a
registered aarch64 handler.

```
./hexa --prod build
./hexa --prod deploy pi@<host>
```

This ships the image tarball, loads it, seeds `~/hexa-prod/.env` from
`.env.prod.sample`, and starts the container **cold** (relay open, hardware
inactive).

## 5. Edit `~/hexa-prod/.env` on the Pi

- **`INPUT_GID`** — value from step 3 (typically something like`996`).
- **`ROS_DOMAIN_ID`** — DDS domain, default `42`.
- **`SERVO_DEVICE`** — the `/dev/serial/by-id/usb-Pimoroni_Servo_2040-...`
  path from step 3.

Restart the container:

```
cd ~/hexa-prod
docker compose -f docker-compose.prod.yaml down
docker compose -f docker-compose.prod.yaml up -d
```

## 6. Engage and drive

```
ssh pi@<host> 'cd ~/hexa-prod && ./hexa --prod engage'
ssh pi@<host> 'cd ~/hexa-prod && ./hexa --prod teleop'
./hexa --prod disengage
```

## 6b. Wi-Fi hotspot for web teleop (optional)

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
./hexa --prod build
./hexa --prod deploy pi@<host>
```

The container restarts cold after each redeploy — re-run `engage`.
