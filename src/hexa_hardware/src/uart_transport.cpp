#include "hexa_hardware/uart_transport.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace hexa_hardware {

namespace {

speed_t to_speed(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
  }
}

}  // namespace

UartTransport::UartTransport(std::string device, int baud)
    : device_(std::move(device)), baud_(baud) {}

UartTransport::~UartTransport() {
  close();
}

void UartTransport::open() {
  close();
  const int fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    throw std::runtime_error("hexa_hardware: open(" + device_ + ") failed: " +
                             std::strerror(errno));
  }
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    throw std::runtime_error("hexa_hardware: tcgetattr failed");
  }
  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  // 0.1 s read granularity; we drive timing with poll().
  const speed_t s = to_speed(baud_);
  cfsetispeed(&tty, s);
  cfsetospeed(&tty, s);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    throw std::runtime_error("hexa_hardware: tcsetattr failed");
  }
  tcflush(fd, TCIOFLUSH);
  fd_ = fd;
}

void UartTransport::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void UartTransport::write(std::span<const std::uint8_t> data) {
  if (fd_ < 0) {
    throw std::runtime_error("hexa_hardware: write on closed UART");
  }
  std::size_t written = 0;
  while (written < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
    if (n < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error("hexa_hardware: write failed: " +
                               std::string(std::strerror(errno)));
    }
    written += static_cast<std::size_t>(n);
  }
}

std::size_t UartTransport::read(std::span<std::uint8_t> buf, int timeout_ms) {
  if (fd_ < 0) return 0;
  std::size_t got = 0;
  while (got < buf.size()) {
    pollfd pfd{fd_, POLLIN, 0};
    const int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr <= 0) break;  // timeout or error
    const ssize_t r = ::read(fd_, buf.data() + got, buf.size() - got);
    if (r <= 0) {
      if (r < 0 && errno == EINTR) continue;
      break;
    }
    got += static_cast<std::size_t>(r);
  }
  return got;
}

}  // namespace hexa_hardware
