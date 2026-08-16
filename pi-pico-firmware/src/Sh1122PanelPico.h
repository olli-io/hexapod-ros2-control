// SH1122 (256x64) OLED panel driver for the Pico 2 W (part 11).
//
// The firmware twin of src/hexa_display/src/Sh1122Panel — same u8g2 driver and
// dirty-flush contract, so the eyes rasterize bit-identically to the sim/ROS
// face. Only the transport differs: Linux spidev + GPIO chardev become Pico SDK
// hardware SPI + gpio_put. Driven by the core1 render loop (face.cpp).
//
// present() pushes only the changed tile-rows, full width — the sh1122 driver's
// horizontal-window addressing is buggy (u8x8_d_sh1122.c), so the tile-row band
// is its tested path.
//
// The dirty shadow is a fixed array, not a vector: core1 must never touch the
// heap, because core0 allocates on essentially every control tick and whether
// newlib's malloc is mutex-guarded here is a build-config detail.
#pragma once

#include <cstddef>
#include <cstdint>

#include "hardware/spi.h"

#include "u8g2.h"

namespace face {

struct PanelConfigPico {
    spi_inst_t* spi      = spi0;
    std::uint32_t spi_hz = 25'000'000;
    unsigned sck  = 18;   // SPI SCK  (GPIO_FUNC_SPI)
    unsigned mosi = 19;   // SPI MOSI (GPIO_FUNC_SPI)
    int cs  = 17;         // manual chip select; -1 to disable (CS tied low)
    int dc  = 20;         // data/command
    int rst = 21;         // reset; -1 to disable
    bool headless = false;  // skip all SPI/GPIO I/O (bring-up / no panel fitted)
};

class Sh1122PanelPico {
public:
    // u8g2_Setup_sh1122_256x64_f's full buffer: 256*64 pixels, 1 bit each.
    static constexpr std::size_t kBufBytes = 256 * 64 / 8;  // 2048

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

    std::uint64_t flushCount() const { return _flushes; }

    // At 25 MHz expect ~2300-2600 us for the eye band, ~3000-3500 us full frame.
    std::uint64_t lastFlushUs() const { return _last_flush_us; }

    // What spi_init() negotiated — the SDK rounds DOWN to
    // clk_peri / (prescale * postdiv), so a 30 MHz request becomes 25 MHz.
    std::uint32_t actualSpiHz() const { return _actual_spi_hz; }

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

    std::uint8_t  _last[kBufBytes]{};  // last flushed framebuffer (dirty check)
    bool          _last_valid = false;
    std::uint64_t _flushes = 0;
    std::uint64_t _last_flush_us = 0;
    std::uint32_t _actual_spi_hz = 0;
};

}  // namespace face
