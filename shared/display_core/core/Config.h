#pragma once

#include <stddef.h>
#include <stdint.h>

namespace cfg {

// ---------------------------------------------------------------------------
// Firmware version
// ---------------------------------------------------------------------------
constexpr uint8_t FW_VER_MAJOR = 0;
constexpr uint8_t FW_VER_MINOR = 1;
constexpr uint8_t FW_VER_PATCH = 0;

// ---------------------------------------------------------------------------
// Pin map — XIAO ESP32-C3 (silkscreen D-label → raw GPIO)
// ---------------------------------------------------------------------------
// SPI to SH1122 OLED (4-wire HW SPI)
//   D8  GPIO8   SCK   (strapping; HIGH at boot. SPI mode 0 idles low after init.)
//   D10 GPIO10  MOSI
//   D3  GPIO5   CS
//   D2  GPIO4   DC
//   D1  GPIO3   RST
//
// UART1 to Raspberry Pi
//   D6  GPIO21  TX
//   D7  GPIO20  RX
//
// Reserved / left free: D0 (GPIO2 strapping), D4/D5 (I2C SDA/SCL),
// D9 (GPIO9 BOOT button).
constexpr int PIN_OLED_SCK  = 8;
constexpr int PIN_OLED_MOSI = 10;
constexpr int PIN_OLED_CS   = 5;
constexpr int PIN_OLED_DC   = 4;
constexpr int PIN_OLED_RST  = 3;

constexpr int PIN_UART_TX   = 21;
constexpr int PIN_UART_RX   = 20;

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
constexpr uint32_t OLED_SPI_HZ = 8'000'000;

// ---------------------------------------------------------------------------
// UART link to Pi
// ---------------------------------------------------------------------------
constexpr uint32_t UART_BAUD     = 921600;
// Driver ring buffers. The main task pumps RX between renders, so the ring
// must cover a full render stall: ~10 ms of SPI flush at ~92 B/ms incoming
// is ~1 KB; 2 KB leaves 2x headroom.
constexpr size_t   RX_RING_SIZE  = 2048;
constexpr size_t   TX_RING_SIZE  = 256;

// Heartbeat: any valid frame counts as link activity, so the Pi must send
// something (PING suffices) at least this often. After this much silence
// the eyes fall back to DEAD until a frame arrives again.
constexpr uint32_t LINK_TIMEOUT_MS = 3000;

// ---------------------------------------------------------------------------
// Render loop
// ---------------------------------------------------------------------------
constexpr uint32_t MIN_RENDER_INTERVAL_MS = 16;  // ~60 Hz cap

}  // namespace cfg
