// SH1122 (256x64) OLED panel driver for the Pico 2 W (part 11).
//
// The firmware twin of src/hexa_display/src/Sh1122Panel — same public API, same
// u8g2 sh1122 driver, same dirty-flush contract, so the eyes rasterize
// bit-identically to the sim/ROS face. Only the transport differs: the Linux
// spidev + GPIO character device become Pico SDK hardware SPI + gpio_put. The
// eye render loop that drives this runs on core1 (see face.cpp).
//
// present() pushes only the changed tile-rows over SPI (full-width dirty band,
// the sh1122 u8g2 driver's tested path — its horizontal-window addressing is
// buggy, see u8x8_d_sh1122.c), so a static face costs no bus traffic and a
// blink/gaze move flushes only the eye band.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "hardware/spi.h"

#include "u8g2.h"

namespace face {

struct PanelConfigPico {
    spi_inst_t* spi      = spi0;
    std::uint32_t spi_hz = 8'000'000;
    unsigned sck  = 18;   // SPI SCK  (GPIO_FUNC_SPI)
    unsigned mosi = 19;   // SPI MOSI (GPIO_FUNC_SPI)
    int cs  = 17;         // manual chip select; -1 to disable (CS tied low)
    int dc  = 20;         // data/command
    int rst = 21;         // reset; -1 to disable
    bool headless = false;  // skip all SPI/GPIO I/O (bring-up / no panel fitted)
};

class Sh1122PanelPico {
public:
    Sh1122PanelPico() = default;
    ~Sh1122PanelPico();

    Sh1122PanelPico(const Sh1122PanelPico&) = delete;
    Sh1122PanelPico& operator=(const Sh1122PanelPico&) = delete;

    // Bring up SPI + the control GPIOs, un-wedge and init the panel. Returns
    // false only on a bad config (always succeeds in headless mode).
    bool begin(const PanelConfigPico& cfg);

    // Renderers draw through the u8g2 C API on this handle.
    u8g2_t* u8g2() { return &_u8g2; }

    void clearBuffer() { u8g2_ClearBuffer(&_u8g2); }

    // Flush the changed tile-rows over SPI (nothing if unchanged since the last
    // present). Returns true if a flush actually happened.
    bool present();

    // Blank the panel and enter power-save.
    void sleep();

    // Total flushes performed (changed frames). Useful for diagnostics.
    std::uint64_t flushCount() const { return _flushes; }

private:
    static std::uint8_t byteCb(u8x8_t* u8x8, std::uint8_t msg, std::uint8_t arg_int,
                               void* arg_ptr);
    static std::uint8_t gpioCb(u8x8_t* u8x8, std::uint8_t msg, std::uint8_t arg_int,
                               void* arg_ptr);

    void   spiWrite(const std::uint8_t* data, std::size_t len);
    void   gpioWrite(int pin, int value);  // pin < 0 = no-op
    void   resetPulse();
    std::size_t bufferBytes() const;

    u8g2_t _u8g2{};
    PanelConfigPico _cfg{};
    bool   _headless = false;
    bool   _up       = false;

    std::vector<std::uint8_t> _last;  // last flushed framebuffer (dirty check)
    std::uint64_t             _flushes = 0;
};

}  // namespace face
