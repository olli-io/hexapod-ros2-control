// SH1122 panel driver — Pico SDK transport (part 11). See Sh1122PanelPico.h.
//
// Forked from src/hexa_display/src/Sh1122Panel.cpp: the u8g2 setup, the
// per-instance user_ptr wiring, and the dirty-flush accounting are reused; only
// the transport callbacks + begin() are re-pointed at the Pico SDK (spidev ->
// spi_write_blocking, GPIO chardev -> gpio_put, nanosleep -> busy_wait/sleep).
// present() additionally shrinks the whole-buffer SendBuffer to a dirty tile-row
// UpdateDisplayArea (full width — the sh1122 driver's tested addressing path).

#include "Sh1122PanelPico.h"

#include <cstring>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"

namespace face {
namespace {

Sh1122PanelPico* self(u8x8_t* u8x8) {
    return static_cast<Sh1122PanelPico*>(u8x8_GetUserPtr(u8x8));
}

}  // namespace

Sh1122PanelPico::~Sh1122PanelPico() {
    if (_up && !_headless) sleep();
}

std::size_t Sh1122PanelPico::bufferBytes() const {
    return static_cast<std::size_t>(u8g2_GetBufferTileWidth(const_cast<u8g2_t*>(&_u8g2))) *
           u8g2_GetBufferTileHeight(const_cast<u8g2_t*>(&_u8g2)) * 8u;
}

// --- GPIO / SPI transport ---------------------------------------------------

void Sh1122PanelPico::gpioWrite(int pin, int value) {
    if (pin < 0 || _headless) return;
    gpio_put(static_cast<uint>(pin), value != 0);
}

void Sh1122PanelPico::resetPulse() {
    if (_cfg.rst < 0) return;
    gpioWrite(_cfg.rst, 0);
    sleep_ms(10);
    gpioWrite(_cfg.rst, 1);
    sleep_ms(50);
}

void Sh1122PanelPico::spiWrite(const std::uint8_t* data, std::size_t len) {
    if (len == 0 || _headless) return;
    spi_write_blocking(_cfg.spi, data, len);
}

// --- u8g2 transport callbacks (per-instance via user_ptr) -------------------

std::uint8_t Sh1122PanelPico::byteCb(u8x8_t* u8x8, std::uint8_t msg,
                                     std::uint8_t arg_int, void* arg_ptr) {
    Sh1122PanelPico* p = self(u8x8);
    switch (msg) {
        case U8X8_MSG_BYTE_INIT:
            break;  // bus opened eagerly in begin()
        case U8X8_MSG_BYTE_SEND:
            p->spiWrite(static_cast<const std::uint8_t*>(arg_ptr), arg_int);
            break;
        case U8X8_MSG_BYTE_SET_DC:
            p->gpioWrite(p->_cfg.dc, arg_int);
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            p->gpioWrite(p->_cfg.cs, u8x8->display_info->chip_enable_level);
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            p->gpioWrite(p->_cfg.cs, u8x8->display_info->chip_disable_level);
            break;
        default:
            return 0;
    }
    return 1;
}

std::uint8_t Sh1122PanelPico::gpioCb(u8x8_t* u8x8, std::uint8_t msg,
                                     std::uint8_t arg_int, void* /*arg_ptr*/) {
    Sh1122PanelPico* p = self(u8x8);
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;  // lines configured eagerly in begin()
        case U8X8_MSG_GPIO_CS:
            p->gpioWrite(p->_cfg.cs, arg_int);
            break;
        case U8X8_MSG_GPIO_DC:
            p->gpioWrite(p->_cfg.dc, arg_int);
            break;
        case U8X8_MSG_GPIO_RESET:
            p->gpioWrite(p->_cfg.rst, arg_int);
            break;
        case U8X8_MSG_DELAY_MILLI:
            sleep_ms(arg_int);
            break;
        case U8X8_MSG_DELAY_10MICRO:
            sleep_us(10);
            break;
        case U8X8_MSG_DELAY_100NANO:
        case U8X8_MSG_DELAY_NANO:
            busy_wait_us(1);  // conservative; only during init sequences
            break;
        default:
            return 0;
    }
    return 1;
}

// --- lifecycle --------------------------------------------------------------

bool Sh1122PanelPico::begin(const PanelConfigPico& cfg) {
    _cfg = cfg;
    _headless = cfg.headless;

    u8g2_Setup_sh1122_256x64_f(&_u8g2, U8G2_R0, &Sh1122PanelPico::byteCb,
                               &Sh1122PanelPico::gpioCb);
    u8x8_SetUserPtr(u8g2_GetU8x8(&_u8g2), this);

    if (_headless) {
        u8g2_ClearBuffer(&_u8g2);
        _up = true;
        return true;
    }

    // SPI bus: mode 0, 8-bit, MSB first. CS is driven manually as a GPIO.
    spi_init(_cfg.spi, _cfg.spi_hz);
    spi_set_format(_cfg.spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(_cfg.sck, GPIO_FUNC_SPI);
    gpio_set_function(_cfg.mosi, GPIO_FUNC_SPI);

    // Control lines: DC / RST / CS as outputs, idle high so the SH1122 never
    // latches a half-received command during bring-up.
    for (int pin : {_cfg.dc, _cfg.rst, _cfg.cs}) {
        if (pin < 0) continue;
        gpio_init(static_cast<uint>(pin));
        gpio_set_dir(static_cast<uint>(pin), GPIO_OUT);
        gpio_put(static_cast<uint>(pin), 1);
    }

    resetPulse();

    u8g2_InitDisplay(&_u8g2);
    u8g2_SetPowerSave(&_u8g2, 0);
    u8g2_ClearBuffer(&_u8g2);
    _up = true;
    return true;
}

bool Sh1122PanelPico::present() {
    const std::uint8_t* buf = u8g2_GetBufferPtr(&_u8g2);
    const std::size_t n = bufferBytes();

    // First flush (or a buffer-size change): push the whole frame.
    if (_last.size() != n) {
        _last.assign(buf, buf + n);
        ++_flushes;
        if (!_headless) u8g2_SendBuffer(&_u8g2);
        return true;
    }

    // Dirty range in bytes → full-width tile-row band. The u8g2 buffer is laid
    // out tile-row major (tile_width tiles across × 8 bytes each), so one tile
    // row is tile_width*8 bytes and covers 8 pixel rows.
    std::size_t first = 0;
    while (first < n && buf[first] == _last[first]) ++first;
    if (first == n) return false;  // unchanged — no flush
    std::size_t last = n;
    while (last > first && buf[last - 1] == _last[last - 1]) --last;

    const unsigned tiles_w = u8g2_GetBufferTileWidth(&_u8g2);
    const std::size_t bytes_per_tile_row = static_cast<std::size_t>(tiles_w) * 8u;
    const std::uint8_t ty = static_cast<std::uint8_t>(first / bytes_per_tile_row);
    const std::uint8_t ty_end =
        static_cast<std::uint8_t>((last - 1) / bytes_per_tile_row);
    const std::uint8_t th = static_cast<std::uint8_t>(ty_end - ty + 1);

    _last.assign(buf, buf + n);
    ++_flushes;
    if (!_headless)
        u8g2_UpdateDisplayArea(&_u8g2, 0, ty, static_cast<std::uint8_t>(tiles_w), th);
    return true;
}

void Sh1122PanelPico::sleep() {
    u8g2_ClearBuffer(&_u8g2);
    if (!_headless) {
        u8g2_SendBuffer(&_u8g2);
        u8g2_SetPowerSave(&_u8g2, 1);
    }
    _last.clear();
}

}  // namespace face
