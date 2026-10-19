# Bluetooth controller setup

This document pairs a Bluetooth gamepad with the Pi over SSH. Do this one time
for each controller. For the supported controllers, see
[`src/hexa_teleop/README.md`](../src/hexa_teleop/README.md).

Pair on the host, not in the robot container. BlueZ runs on the host, and the
container reads the controller from `/dev/input`.

## Steps

1. Connect to the Pi with SSH and start the Bluetooth shell:
   ```
   bluetoothctl
   ```
2. Turn on the radio and the pairing agent:
   ```
   power on
   agent on
   default-agent
   ```
3. Put the controller in pairing mode. On an Xbox controller, hold the pair
   button until the logo flashes fast. Put an 8BitDo controller in X-input
   mode first.
4. Start the scan and find the controller's MAC address:
   ```
   scan on
   ```
   Wait for a line such as `[NEW] Device AA:BB:CC:DD:EE:FF Xbox Wireless Controller`.
5. Pair, trust and connect. Use the MAC address from step 4:
   ```
   scan off
   pair AA:BB:CC:DD:EE:FF
   trust AA:BB:CC:DD:EE:FF
   connect AA:BB:CC:DD:EE:FF
   quit
   ```
   `trust` lets the controller connect again by itself after a reboot.

## Check

```
ls -l /dev/input/js*                  # the controller is js0
bluetoothctl info AA:BB:CC:DD:EE:FF   # Connected: yes
```

## Traps

- **The controller does not show in the scan** — it left pairing mode. Put it
  in pairing mode again, then run `scan on` again.
- **`Failed to pair`** — remove the old entry with `remove AA:BB:CC:DD:EE:FF`,
  then do steps 3 to 5 again.
- **The controller connects, then disconnects** — do `trust` again, and make
  sure that the controller has no pairing to a different host nearby.
- **Do not add `dtoverlay=disable-bt`.** See [`pi-setup.md`](pi-setup.md).
