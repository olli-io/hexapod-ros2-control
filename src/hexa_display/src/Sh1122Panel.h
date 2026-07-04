#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "u8g2.h"

namespace face {

struct PanelConfig {
    std::string spi_device   = "/dev/spidev0.0";
    uint32_t    spi_speed_hz = 8'000'000;
    std::string gpio_chip    = "/dev/gpiochip0";
    int         dc_line  = -1;   // GPIO offset, -1 if unused
    int         rst_line = -1;
    int         cs_line  = -1;
    bool        headless = false;  // skip all SPI/GPIO I/O
};

// Self-contained SH1122 (256x64) panel driver for Linux. Wraps u8g2's built-in
// sh1122 driver and supplies its transport directly: pixel bytes over spidev,
// DC/RST/CS over the kernel GPIO character device (uABI v2 ioctls, no libgpiod).
// Per-instance state is carried through u8g2's user_ptr, so there are no
// file-scope globals and the class is reentrant/testable.
//
// present() only pushes the framebuffer when it changed since the last flush —
// a static face costs no SPI traffic. In headless mode the full pipeline runs
// (buffer, dirty check, flush accounting) but no device is touched.
class Sh1122Panel {
public:
    Sh1122Panel() = default;
    ~Sh1122Panel();

    Sh1122Panel(const Sh1122Panel&) = delete;
    Sh1122Panel& operator=(const Sh1122Panel&) = delete;

    // Opens the bus/GPIO, un-wedges + initialises the panel. Returns false on
    // I/O failure (always succeeds in headless mode).
    bool begin(const PanelConfig& cfg);

    // Renderers draw through the u8g2 C API on this handle.
    u8g2_t* u8g2() { return &_u8g2; }

    void clearBuffer() { u8g2_ClearBuffer(&_u8g2); }

    // Flush the buffer over SPI only if it differs from the last flush.
    // Returns true if a flush actually happened.
    bool present();

    // Blank the panel and enter power-save.
    void sleep();

    // Total flushes performed (changed frames). Useful for tests / diagnostics.
    uint64_t flushCount() const { return _flushes; }

private:
    static uint8_t byteCb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);
    static uint8_t gpioCb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);

    void   spiWrite(const uint8_t* data, size_t len);
    void   gpioWrite(int bit, int value);  // bit = index into the line request; -1 = no-op
    void   resetPulse();
    size_t bufferBytes() const;

    u8g2_t _u8g2{};
    bool   _headless = false;
    bool   _up       = false;

    int _spi_fd  = -1;
    int _gpio_fd = -1;  // single line-request fd for all output lines
    int _dc_bit = -1, _rst_bit = -1, _cs_bit = -1;  // line indices within the request

    std::vector<uint8_t> _last;  // last flushed framebuffer (for dirty check)
    uint64_t             _flushes = 0;
};

}  // namespace face
