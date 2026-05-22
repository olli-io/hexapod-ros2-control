// Chica binary protocol over UART (Servo 2040 etc.).
//
// Framing rule (used for resync): command bytes have MSB=1, data bytes
// have MSB=0. A 14-bit value `v` packs into two data bytes as
// `lo = v & 0x7F; hi = (v >> 7) & 0x7F`.

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace hexa_hardware {

constexpr std::uint8_t kCmdSet = 'S' | 0x80;
constexpr std::uint8_t kCmdGet = 'G' | 0x80;
constexpr std::uint16_t kValueMax = 0x3FFF;  // 14-bit
constexpr std::size_t kMaxBatch = 64;        // 128 pin space; one frame must fit comfortably

// Encode a SET frame into `out` (cleared first). Values are clamped to 14 bits.
void encode_set(std::uint8_t start, std::span<const std::uint16_t> values,
                std::vector<std::uint8_t>& out);

// Encode a GET request frame (3 bytes) into `out`.
void encode_get(std::uint8_t start, std::uint8_t count,
                std::vector<std::uint8_t>& out);

// Decode a GET reply payload. `payload` must start *after* the GET command
// byte (i.e. start with [start_idx][count][val_lo][val_hi]...). Returns true
// on success and fills `start` / `values`. Returns false if the buffer is
// too short or count doesn't match.
bool decode_get_payload(std::span<const std::uint8_t> payload,
                        std::uint8_t& start,
                        std::vector<std::uint16_t>& values);

// Open a serial device in raw, 8N1, blocking-with-VTIME=1 mode. Returns the
// fd or throws std::runtime_error. `baud` may be ignored by USB-CDC bridges
// but is honoured for true UARTs.
int open_serial(const std::string& device, int baud);

// Owns the serial fd, encodes/decodes frames, drives the half-duplex link.
// Not thread-safe — the SystemInterface uses it from the controller-manager
// thread only.
class ServoBus {
 public:
  ServoBus() = default;
  ~ServoBus();

  ServoBus(const ServoBus&) = delete;
  ServoBus& operator=(const ServoBus&) = delete;

  void open(const std::string& device, int baud);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Blocking write of an already-encoded frame. Throws on I/O error.
  void write_frame(std::span<const std::uint8_t> frame);

  // Convenience: encode-and-send a SET.
  void send_set(std::uint8_t start, std::span<const std::uint16_t> values);

  // Convenience: encode-and-send a SET of one digital pin (0/1).
  void send_digital(std::uint8_t pin, bool value);

  // Issue a GET, then read the reply within `timeout_ms`. Discards bytes
  // until a command byte (MSB set) is seen, allowing resync from a partial
  // frame on the wire. Returns true on success.
  bool request_get(std::uint8_t start, std::uint8_t count,
                   std::vector<std::uint16_t>& values_out,
                   int timeout_ms = 50);

 private:
  // Read at most `n` bytes with the given total timeout. Returns bytes read.
  std::size_t read_with_timeout(std::uint8_t* buf, std::size_t n, int timeout_ms);

  int fd_ = -1;
  // Reusable encode buffer to avoid per-call allocation on the hot path.
  std::vector<std::uint8_t> encode_buf_;
};

}  // namespace hexa_hardware
