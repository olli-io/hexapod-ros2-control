#include "hexa_hardware/servo_bus.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

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

void encode_set(std::uint8_t start, std::span<const std::uint16_t> values,
                std::vector<std::uint8_t>& out) {
  out.clear();
  out.reserve(3 + values.size() * 2);
  out.push_back(kCmdSet);
  out.push_back(start & 0x7F);
  out.push_back(static_cast<std::uint8_t>(values.size()) & 0x7F);
  for (std::uint16_t v : values) {
    std::uint16_t c = v > kValueMax ? kValueMax : v;
    out.push_back(static_cast<std::uint8_t>(c & 0x7F));
    out.push_back(static_cast<std::uint8_t>((c >> 7) & 0x7F));
  }
}

void encode_get(std::uint8_t start, std::uint8_t count,
                std::vector<std::uint8_t>& out) {
  out.clear();
  out.reserve(3);
  out.push_back(kCmdGet);
  out.push_back(start & 0x7F);
  out.push_back(count & 0x7F);
}

bool decode_get_payload(std::span<const std::uint8_t> payload,
                        std::uint8_t& start,
                        std::vector<std::uint16_t>& values) {
  if (payload.size() < 2) {
    return false;
  }
  start = payload[0] & 0x7F;
  const std::uint8_t count = payload[1] & 0x7F;
  if (payload.size() < static_cast<std::size_t>(2 + count * 2)) {
    return false;
  }
  values.clear();
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint8_t lo = payload[2 + i * 2] & 0x7F;
    const std::uint8_t hi = payload[3 + i * 2] & 0x7F;
    values.push_back(static_cast<std::uint16_t>(lo) |
                     (static_cast<std::uint16_t>(hi) << 7));
  }
  return true;
}

int open_serial(const std::string& device, int baud) {
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    throw std::runtime_error("hexa_hardware: open(" + device + ") failed: " +
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
  const speed_t s = to_speed(baud);
  cfsetispeed(&tty, s);
  cfsetospeed(&tty, s);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    throw std::runtime_error("hexa_hardware: tcsetattr failed");
  }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

ServoBus::~ServoBus() {
  close();
}

void ServoBus::open(const std::string& device, int baud) {
  close();
  fd_ = open_serial(device, baud);
}

void ServoBus::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void ServoBus::write_frame(std::span<const std::uint8_t> frame) {
  if (fd_ < 0) {
    throw std::runtime_error("hexa_hardware: write on closed bus");
  }
  std::size_t written = 0;
  while (written < frame.size()) {
    const ssize_t n = ::write(fd_, frame.data() + written, frame.size() - written);
    if (n < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error("hexa_hardware: write failed: " +
                               std::string(std::strerror(errno)));
    }
    written += static_cast<std::size_t>(n);
  }
}

void ServoBus::send_set(std::uint8_t start, std::span<const std::uint16_t> values) {
  encode_set(start, values, encode_buf_);
  write_frame(encode_buf_);
}

void ServoBus::send_digital(std::uint8_t pin, bool value) {
  const std::uint16_t v = value ? 1u : 0u;
  send_set(pin, std::span<const std::uint16_t>(&v, 1));
}

std::size_t ServoBus::read_with_timeout(std::uint8_t* buf, std::size_t n, int timeout_ms) {
  std::size_t got = 0;
  while (got < n) {
    pollfd pfd{fd_, POLLIN, 0};
    const int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr <= 0) break;  // timeout or error
    const ssize_t r = ::read(fd_, buf + got, n - got);
    if (r <= 0) {
      if (r < 0 && errno == EINTR) continue;
      break;
    }
    got += static_cast<std::size_t>(r);
  }
  return got;
}

bool ServoBus::request_get(std::uint8_t start, std::uint8_t count,
                           std::vector<std::uint16_t>& values_out,
                           int timeout_ms) {
  if (fd_ < 0) return false;

  encode_get(start, count, encode_buf_);
  write_frame(encode_buf_);

  // Resync: drop bytes until we see a command byte (MSB set). Discard any
  // command that isn't G (e.g. a stray S echo).
  std::uint8_t b = 0;
  while (true) {
    if (read_with_timeout(&b, 1, timeout_ms) != 1) return false;
    if ((b & 0x80) == 0) continue;
    if (b == kCmdGet) break;
  }
  // Read payload: [start][count][2*count value bytes].
  const std::size_t payload_len = 2 + static_cast<std::size_t>(count) * 2;
  std::vector<std::uint8_t> payload(payload_len);
  if (read_with_timeout(payload.data(), payload_len, timeout_ms) != payload_len) {
    return false;
  }
  std::uint8_t reply_start = 0;
  if (!decode_get_payload(payload, reply_start, values_out)) return false;
  return reply_start == start && values_out.size() == count;
}

}  // namespace hexa_hardware
