// SH1122 256x64 OLED over Linux spidev, with D/C and reset on GPIO lines
// driven through the GPIO character device. No libraries needed.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "image.hpp"

struct Sh1122Config {
    int port = 0;                       // SPI bus, /dev/spidev<port>.<device>
    int device = 0;                     // SPI chip select
    uint32_t speed_hz = 8000000;
    int dc = 24;                        // line offset for D/C on the GPIO chip (BCM number)
    int rst = 25;                       // line offset for reset
    std::string gpiochip = "/dev/gpiochip0";
    int contrast = 0x7F;
    bool hflip = false;
    bool vflip = true;
};

class Sh1122 {
public:
    explicit Sh1122(const Sh1122Config& cfg);   // opens, resets and initialises; throws std::runtime_error
    ~Sh1122();
    Sh1122(const Sh1122&) = delete;
    Sh1122& operator=(const Sh1122&) = delete;

    // 256x64 greyscale. Only rows that changed since the last call are sent.
    void show(const Image& img);
    void off();                    // blank the screen and switch the panel off

private:
    void command(const std::vector<uint8_t>& bytes);
    void data(const uint8_t* p, size_t n);
    void write_all(const uint8_t* p, size_t n);
    void set_line(int fd, int value);

    int spi_fd_ = -1;
    int dc_fd_ = -1;
    int rst_fd_ = -1;
    std::vector<uint8_t> sent_;   // packed rows already on the panel
    bool have_sent_ = false;
};
