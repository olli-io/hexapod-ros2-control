# Part 03 — Servo 2040 slave link (Chica protocol over Pico UART)

**Goal:** drive the Servo 2040 as a PWM slave from the Pico over UART using the
existing Chica protocol — command a servo, toggle the power relay, and read
battery telemetry. Replaces the host↔board link.

**Depends on:** 01. **Blocks:** 05 (needs to command joints).

## Scope

- `src/servo_out.{hpp,cpp}` — fork of `src/hexa_hardware/src/servo2040_protocol.cpp` + `joint_calibration.cpp`, reused **verbatim** except the transport:
  - Keep SET/GET framing: `kCmdSet='S'|0x80`, `kCmdGet='G'|0x80`, data bytes MSB=0, 14-bit value packed as `lo=v&0x7F; hi=(v>>7)&0x7F`, max `0x3FFF`. SET = `[0x80|'S'][start_pin][count]` + count×(lo,hi); GET reply resynced by scanning for a command byte.
  - Keep the run-grouping (consecutive servo pins → one SET frame; default wiring = 6 frames for 18 joints on pins 1–18).
  - Keep `to_pulse_us(theta_rad)` calibration: `center=(us+45+us−45)/2`, `slope=(us+45−us−45)/(π/2)`, clamp to `[min_us,max_us]`; reversed mounts swap endpoints. **Convert doubles to float.**
  - Keep relay control (default pin 24, energize/de-energize) and battery-ADC GET (aux pins 26/27, scales `0.00366` V, `0.00098` A).
  - **Replace `UartTransport` (termios)** with a Pico SDK transport: `uart_init`, `uart_write_blocking`, `uart_is_readable`/`uart_getc`. Pick + document a baud (e.g. 921600) and the TX/RX GPIO. Discard `UsbTransport`, `I2cTransport`, `hardware_factory.cpp`.
- Pin/calibration values come from `hexa_hardware/config/hardware.yaml` (18× {pin, joint_position, us_at_±45, min/max_us}) — hardcode here for now; folded into `config_generated.hpp` in part 04.
- Confirm the Servo 2040's firmware still speaks Chica over hardware UART (not only USB-CDC); adjust baud/wiring to match.

## Done when / verification

- Send a SET frame for one pin at 1500 µs → that servo centers; sweep 1000/2000 µs → it moves symmetrically.
- Energize relay → servo rail powers (measure/observe); de-energize → rail drops.
- Poll a GET → decode plausible battery voltage/current; compare against a multimeter.
- Time a full 6-frame SET burst + one GET at the chosen baud → confirm it fits well inside 20 ms (the tick budget). Log the measured round-trip.
