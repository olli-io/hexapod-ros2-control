#include "sh1122.hpp"

#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "eyes.hpp"

namespace {

void sleep_s(double s) {
    std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

[[noreturn]] void fail(const std::string& what) {
    throw std::runtime_error(what + ": " + std::strerror(errno));
}

// Request one GPIO line as an output, starting low. Returns the line fd.
int request_output(int chip_fd, int offset, const char* consumer) {
    gpio_v2_line_request req;
    std::memset(&req, 0, sizeof req);
    req.offsets[0] = offset;
    req.num_lines = 1;
    std::strncpy(req.consumer, consumer, sizeof req.consumer - 1);
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0)
        fail("request GPIO line " + std::to_string(offset));
    return req.fd;
}

constexpr size_t kChunk = 4096;   // spidev's default transfer limit

}  // namespace

void Sh1122::set_line(int fd, int value) {
    gpio_v2_line_values v;
    std::memset(&v, 0, sizeof v);
    v.mask = 1;
    v.bits = value ? 1 : 0;
    if (ioctl(fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) < 0) fail("set GPIO line");
}

void Sh1122::write_all(const uint8_t* p, size_t n) {
    while (n > 0) {
        size_t len = std::min(n, kChunk);
        ssize_t w = write(spi_fd_, p, len);
        if (w < 0) fail("SPI write");
        p += w;
        n -= size_t(w);
    }
}

void Sh1122::command(const std::vector<uint8_t>& bytes) {
    set_line(dc_fd_, 0);   // command mode
    write_all(bytes.data(), bytes.size());
}

void Sh1122::data(const uint8_t* p, size_t n) {
    set_line(dc_fd_, 1);   // data mode
    write_all(p, n);
}

Sh1122::Sh1122(const Sh1122Config& cfg) {
    std::string path = "/dev/spidev" + std::to_string(cfg.port) + "." +
                       std::to_string(cfg.device);
    spi_fd_ = open(path.c_str(), O_RDWR);
    if (spi_fd_ < 0) fail("open " + path);
    uint8_t mode = SPI_MODE_0, bits = 8;
    uint32_t speed = cfg.speed_hz;
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) fail("set SPI mode");
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) fail("set SPI bits");
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) fail("set SPI speed");

    int chip = open(cfg.gpiochip.c_str(), O_RDWR);
    if (chip < 0) fail("open " + cfg.gpiochip);
    dc_fd_ = request_output(chip, cfg.dc, "eyes-dc");
    rst_fd_ = request_output(chip, cfg.rst, "eyes-rst");
    close(chip);

    set_line(rst_fd_, 0);   // reset pulse
    set_line(rst_fd_, 1);

    sleep_s(0.1);
    command({0xAE});
    command({0xB0, 0x00});
    command({0x00});
    command({0x10});
    command({0xD5, 0x50});
    command({0xD9, 0x22});
    command({0x40});
    command({0x81, uint8_t(cfg.contrast)});
    command({uint8_t(cfg.hflip ? 0xA1 : 0xA0)});
    command({uint8_t(cfg.vflip ? 0xC0 : 0xC8)});
    command({0xA4});
    command({0xA6});
    command({0xA8, 0x3F});
    command({0xAD, 0x80});
    command({0xD3, 0x00});
    command({0xDB, 0x30});
    command({0xDC, 0x30});
    command({0x33});
    sleep_s(0.1);
    command({0xAF});
}

Sh1122::~Sh1122() {
    if (spi_fd_ >= 0) close(spi_fd_);
    if (dc_fd_ >= 0) close(dc_fd_);
    if (rst_fd_ >= 0) close(rst_fd_);
}

void Sh1122::show(const Image& img) {
    constexpr size_t kRow = kW / 2;
    std::vector<uint8_t> packed(kRow * kH);
    for (int row = 0; row < kH; ++row) {
        const uint8_t* line = &img.px[size_t(row) * kW];
        uint8_t* out = &packed[size_t(row) * kRow];
        for (size_t i = 0; i < kRow; ++i)
            out[i] = uint8_t((line[2 * i] & 0xF0) | (line[2 * i + 1] >> 4));
    }
    for (int row = 0; row < kH; ++row) {
        const uint8_t* out = &packed[size_t(row) * kRow];
        if (have_sent_ && std::memcmp(out, &sent_[size_t(row) * kRow], kRow) == 0) continue;
        command({0xB0, uint8_t(row), 0x00, 0x10});
        data(out, kRow);
    }
    sent_ = std::move(packed);
    have_sent_ = true;
}

void Sh1122::off() {
    show(Image(kW, kH, 0));
    command({0xAE});
}
