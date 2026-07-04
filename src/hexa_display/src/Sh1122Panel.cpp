#include "Sh1122Panel.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace face {
namespace {

constexpr char kConsumer[] = "hexapod_face";

void delayNs(long ns) {
    timespec ts{ns / 1000000000L, ns % 1000000000L};
    nanosleep(&ts, nullptr);
}

Sh1122Panel* self(u8x8_t* u8x8) {
    return static_cast<Sh1122Panel*>(u8x8_GetUserPtr(u8x8));
}

}  // namespace

Sh1122Panel::~Sh1122Panel() {
    if (_up && !_headless) sleep();
    if (_spi_fd  >= 0) close(_spi_fd);
    if (_gpio_fd >= 0) close(_gpio_fd);
}

size_t Sh1122Panel::bufferBytes() const {
    return static_cast<size_t>(u8g2_GetBufferTileWidth(const_cast<u8g2_t*>(&_u8g2))) *
           u8g2_GetBufferTileHeight(const_cast<u8g2_t*>(&_u8g2)) * 8u;
}

// --- GPIO: kernel character-device uABI v2 (no libgpiod) --------------------

void Sh1122Panel::gpioWrite(int bit, int value) {
    if (bit < 0 || _gpio_fd < 0) return;
    gpio_v2_line_values v{};
    v.mask = 1ULL << bit;
    v.bits = value ? (1ULL << bit) : 0ULL;
    if (ioctl(_gpio_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) < 0)
        fprintf(stderr, "[Sh1122Panel] set gpio bit %d: %s\n", bit, strerror(errno));
}

void Sh1122Panel::resetPulse() {
    if (_rst_bit < 0) return;
    gpioWrite(_rst_bit, 0);
    delayNs(10'000'000L);  // 10 ms
    gpioWrite(_rst_bit, 1);
    delayNs(50'000'000L);  // 50 ms
}

// --- u8g2 transport callbacks (per-instance via user_ptr) -------------------

uint8_t Sh1122Panel::byteCb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr) {
    Sh1122Panel* p = self(u8x8);
    switch (msg) {
        case U8X8_MSG_BYTE_INIT:
            break;  // bus opened eagerly in begin()
        case U8X8_MSG_BYTE_SEND:
            p->spiWrite(static_cast<const uint8_t*>(arg_ptr), arg_int);
            break;
        case U8X8_MSG_BYTE_SET_DC:
            p->gpioWrite(p->_dc_bit, arg_int);
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            p->gpioWrite(p->_cs_bit, u8x8->display_info->chip_enable_level);
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            p->gpioWrite(p->_cs_bit, u8x8->display_info->chip_disable_level);
            break;
        default:
            return 0;
    }
    return 1;
}

uint8_t Sh1122Panel::gpioCb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* /*arg_ptr*/) {
    Sh1122Panel* p = self(u8x8);
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;  // lines requested eagerly in begin()
        case U8X8_MSG_GPIO_CS:
            p->gpioWrite(p->_cs_bit, arg_int);
            break;
        case U8X8_MSG_GPIO_DC:
            p->gpioWrite(p->_dc_bit, arg_int);
            break;
        case U8X8_MSG_GPIO_RESET:
            p->gpioWrite(p->_rst_bit, arg_int);
            break;
        case U8X8_MSG_DELAY_MILLI:
            delayNs(static_cast<long>(arg_int > 0 ? arg_int : 1) * 1'000'000L);
            break;
        case U8X8_MSG_DELAY_10MICRO:
            delayNs(10'000L);
            break;
        case U8X8_MSG_DELAY_100NANO:
        case U8X8_MSG_DELAY_NANO:
            delayNs(1'000L);
            break;
        default:
            return 0;
    }
    return 1;
}

void Sh1122Panel::spiWrite(const uint8_t* data, size_t len) {
    if (len == 0 || _spi_fd < 0) return;
    while (len > 0) {
        ssize_t n = write(_spi_fd, data, len);
        if (n <= 0) {
            fprintf(stderr, "[Sh1122Panel] spidev write: %s\n", strerror(errno));
            return;
        }
        data += n;
        len -= static_cast<size_t>(n);
    }
}

// --- lifecycle --------------------------------------------------------------

bool Sh1122Panel::begin(const PanelConfig& cfg) {
    _headless = cfg.headless;

    u8g2_Setup_sh1122_256x64_f(&_u8g2, U8G2_R0, &Sh1122Panel::byteCb,
                               &Sh1122Panel::gpioCb);
    u8x8_SetUserPtr(u8g2_GetU8x8(&_u8g2), this);

    if (_headless) {
        u8g2_ClearBuffer(&_u8g2);
        _up = true;
        return true;
    }

    // SPI bus: manual CS (SPI_NO_CS), mode 0, MSB first, 8-bit words.
    _spi_fd = open(cfg.spi_device.c_str(), O_RDWR);
    if (_spi_fd < 0) {
        fprintf(stderr, "[Sh1122Panel] open %s: %s\n",
                cfg.spi_device.c_str(), strerror(errno));
        return false;
    }
    uint8_t mode = SPI_MODE_0 | SPI_NO_CS;
    uint8_t bits = 8;
    uint32_t speed = cfg.spi_speed_hz;
    if (ioctl(_spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        fprintf(stderr, "[Sh1122Panel] spidev config: %s\n", strerror(errno));
        return false;
    }

    // GPIO: request DC / RST / CS as outputs in one line request. Idle high so
    // the SH1122 never latches a half-received command during bus bring-up.
    int chip_fd = open(cfg.gpio_chip.c_str(), O_RDONLY);
    if (chip_fd < 0) {
        fprintf(stderr, "[Sh1122Panel] open %s: %s\n",
                cfg.gpio_chip.c_str(), strerror(errno));
        return false;
    }
    gpio_v2_line_request req{};
    int n = 0;
    if (cfg.dc_line  >= 0) { req.offsets[n] = static_cast<uint32_t>(cfg.dc_line);  _dc_bit  = n++; }
    if (cfg.rst_line >= 0) { req.offsets[n] = static_cast<uint32_t>(cfg.rst_line); _rst_bit = n++; }
    if (cfg.cs_line  >= 0) { req.offsets[n] = static_cast<uint32_t>(cfg.cs_line);  _cs_bit  = n++; }
    req.num_lines = static_cast<uint32_t>(n);
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    std::strncpy(req.consumer, kConsumer, sizeof(req.consumer) - 1);
    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
        fprintf(stderr, "[Sh1122Panel] request gpio lines: %s\n", strerror(errno));
        close(chip_fd);
        return false;
    }
    close(chip_fd);
    _gpio_fd = req.fd;

    // Drive all requested lines high, then un-wedge the panel before u8g2's own
    // init sequence (matches the firmware's cold-boot reset pulse).
    if (n > 0) {
        gpio_v2_line_values v{};
        v.mask = (n >= 64) ? ~0ULL : ((1ULL << n) - 1);
        v.bits = v.mask;
        ioctl(_gpio_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
    }
    resetPulse();

    u8g2_InitDisplay(&_u8g2);
    u8g2_SetPowerSave(&_u8g2, 0);
    u8g2_ClearBuffer(&_u8g2);
    _up = true;
    return true;
}

bool Sh1122Panel::present() {
    const uint8_t* buf = u8g2_GetBufferPtr(&_u8g2);
    const size_t n = bufferBytes();

    if (_last.size() == n && std::memcmp(buf, _last.data(), n) == 0)
        return false;  // unchanged — no flush

    _last.assign(buf, buf + n);
    ++_flushes;
    if (!_headless) u8g2_SendBuffer(&_u8g2);
    return true;
}

void Sh1122Panel::sleep() {
    u8g2_ClearBuffer(&_u8g2);
    if (!_headless) {
        u8g2_SendBuffer(&_u8g2);
        u8g2_SetPowerSave(&_u8g2, 1);
    }
    _last.clear();
}

}  // namespace face
