# Pi setup

This document takes a Raspberry Pi 4 or 5 from a blank SD card to a Pi that is
ready for the install. For the install and the robot container, see
[`robot-environment.md`](robot-environment.md). To pair a gamepad, see
[`bluetooth-controller-setup.md`](bluetooth-controller-setup.md).

## Hardware

- Raspberry Pi 4 or 5 with a 32 GB+ microSD card.
- Pimoroni Servo 2040 on the header UART, wired crossed with a common ground:
  Pi GPIO14 (TXD) to Servo 2040 RX, Pi GPIO15 (RXD) to Servo 2040 TX, GND to GND.
- Servo rail PSU behind the Servo 2040's relay.
- Wired Ethernet or Wi-Fi.
- Optional: 256×64 SH1122 OLED face on SPI0.
- Optional: two momentary push buttons/microswitches.
- Optional: passive buzzer, + to GPIO12, − to GND.
- Optional: MPU6500 IMU on I²C1 for body stabilization.

## Steps

1. Flash **Raspberry Pi OS Lite (64-bit)** with Raspberry Pi Imager. In the OS
   settings, set the hostname, user, SSH key, Wi-Fi and Wi-Fi country. The
   regular **Raspberry Pi OS (64-bit)** with the desktop features also works, but Lite
   boots faster. The robot usually has no monitor, and you can run all necessary setup over
   SSH.
2. Update the OS. The Pi reboots:
   ```
   sudo apt update && sudo apt full-upgrade -y && sudo reboot
   ```
3. Install Docker, then log out and in again:
   ```
   curl -fsSL https://get.docker.com | sh
   sudo apt install -y docker-compose-plugin
   sudo usermod -aG docker $USER
   ```
4. Add these lines to `/boot/firmware/config.txt`. The first three are
   necessary, also without an IMU. Add the others for the hardware that is
   fitted:
   ```
   dtparam=uart0=on                                    # servos, Pi 5 only
   dtparam=i2c_arm=on                                  # IMU
   dtparam=i2c_arm_baudrate=400000                     # IMU
   dtparam=spi=on                                      # OLED
   dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4   # buzzer
   ```
   On a Pi 4, enable the servo UART with `raspi-config` instead of
   `dtparam=uart0=on`:
   ```
   sudo raspi-config nonint do_serial_hw 0     # enable the hardware UART
   sudo raspi-config nonint do_serial_cons 1   # remove the serial console
   ```
5. Reboot.

If you want, you can run these checks:

```
ls -l /dev/ttyAMA0                    # servo UART, Pi 5 (Pi 4: /dev/ttyS0)
pinctrl get 14,15                     # a1 / uart, not "none"
ls -l /dev/spidev0.0 /dev/gpiochip0   # display, buttons
ls -l /dev/i2c-1                      # IMU
sudo apt install -y i2c-tools && i2cdetect -y 1   # IMU answers at 68
pinctrl get 12                        # a0 / PWM0_CHAN0, buzzer
```

## Traps

- **Do not add `dtoverlay=disable-bt`.** The gamepad uses the onboard
  Bluetooth, and this overlay takes its UART.
- **Do not use `/dev/serial0`.** On a Pi 5 it is the debug connector.
- **Do not use a bare `dtoverlay=pwm-2chan`.** It maps GPIO18/19 and the buzzer
  stays silent with no error.
- **`up` prints "no PWM block"** — the buzzer overlay is missing from
  `config.txt`.
- **`No wifi country is set`** — the hotspot does not start without a WLAN
  country. Set it in Raspberry Pi Imager or with `raspi-config`.
