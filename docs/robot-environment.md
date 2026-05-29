# Robot environment

Steps to take a fresh Raspberry Pi 4 or 5 from a blank SD card to a host ready
to receive `./hexa --prod deploy`. See [`dev-environment.md`](dev-environment.md)
for the workstation side.

## Hardware

- Raspberry Pi 4 or 5 with a 16 GB+ microSD card.
- Pimoroni Servo 2040 over USB (enumerates as `/dev/ttyACM0`).
- Servo rail PSU behind the Servo 2040's relay.
- Wired Ethernet or Wi-Fi.

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

## 3. Note hardware IDs

Plug in the Servo 2040, then on the Pi:

```
ls /dev/serial/by-id/                       # note the usb-Rasperry-Pi-Pico... path
getent group input | cut -d: -f3            # note the input GID (example: 994)
```

## 4. First deploy from the workstation

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

## 7. Re-deploy

```
./hexa --prod build
./hexa --prod deploy pi@<host>
```

The container restarts cold after each redeploy — re-run `engage`.
