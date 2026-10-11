#include "stepper_hardware/serial_link.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

namespace stepper_hardware
{

namespace
{

speed_t baud_constant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return 0;
  }
}

std::uint64_t now_ns()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

SerialLink::~SerialLink() { close(); }

bool SerialLink::open(const std::string & port, int baud, std::string & error)
{
  close();

  const speed_t sp = baud_constant(baud);
  if (sp == 0) {
    error = "baud " + std::to_string(baud) + " ไม่รองรับ";
    return false;
  }

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error = "เปิด " + port + " ไม่ได้: " + std::strerror(errno);
    return false;
  }

  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    error = std::string("tcgetattr ล้มเหลว: ") + std::strerror(errno);
    close();
    return false;
  }
  ::cfmakeraw(&tio);
  ::cfsetispeed(&tio, sp);
  ::cfsetospeed(&tio, sp);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
  tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
  tio.c_cflag &= ~static_cast<tcflag_t>(PARENB);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error = std::string("tcsetattr ล้มเหลว: ") + std::strerror(errno);
    close();
    return false;
  }
  ::tcflush(fd_, TCIOFLUSH);

  rx_.clear();
  rx_.reserve(1024);
  data_ = Snapshot{};
  seq_.store(0, std::memory_order_relaxed);
  dropped_.store(0, std::memory_order_relaxed);

  running_.store(true, std::memory_order_release);
  thread_ = std::thread(&SerialLink::reader, this);
  return true;
}

void SerialLink::close()
{
  running_.store(false, std::memory_order_release);
  if (thread_.joinable()) {
    thread_.join();
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialLink::send(const std::string & line)
{
  if (fd_ < 0) {
    return false;
  }
  const ssize_t n = ::write(fd_, line.data(), line.size());
  if (n < 0 || static_cast<std::size_t>(n) != line.size()) {
    // EAGAIN = บัฟเฟอร์ของ tty เต็ม แปลว่า baud ต่ำไปสำหรับ update_rate ที่ตั้งไว้
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

void SerialLink::publish(const Snapshot & snap)
{
  const unsigned s = seq_.load(std::memory_order_relaxed);
  seq_.store(s + 1, std::memory_order_release);          // คี่ = กำลังเขียน
  std::atomic_thread_fence(std::memory_order_release);
  data_ = snap;
  std::atomic_thread_fence(std::memory_order_release);
  seq_.store(s + 2, std::memory_order_release);          // คู่ = อ่านได้
}

Snapshot SerialLink::latest() const
{
  // ลองไม่กี่รอบพอ ถ้าชนกับ thread เขียนจริง ๆ ก็คืน valid=false
  // ให้ผู้เรียกใช้ค่ารอบก่อนแทน ดีกว่าวนรอในลูป real-time
  for (int attempt = 0; attempt < 4; ++attempt) {
    const unsigned s1 = seq_.load(std::memory_order_acquire);
    if (s1 & 1u) {
      continue;
    }
    Snapshot out = data_;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (seq_.load(std::memory_order_relaxed) == s1) {
      return out;
    }
  }
  return Snapshot{};
}

std::string SerialLink::take_log()
{
  std::lock_guard<std::mutex> lock(log_mutex_);
  std::string out;
  out.swap(log_);
  return out;
}

void SerialLink::reader()
{
  char chunk[512];
  while (running_.load(std::memory_order_acquire)) {
    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, 100);
    if (rc <= 0) {
      continue;                       // หมดเวลารอ วนไปเช็ก running_ ใหม่
    }
    const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n <= 0) {
      continue;
    }
    rx_.append(chunk, static_cast<std::size_t>(n));

    std::size_t start = 0;
    for (;;) {
      const std::size_t nl = rx_.find('\n', start);
      if (nl == std::string::npos) {
        break;
      }
      handle_line(rx_.substr(start, nl - start));
      start = nl + 1;
    }
    rx_.erase(0, start);
    if (rx_.size() > 4096) {
      rx_.clear();                    // กันบัฟเฟอร์บวมถ้าเจอขยะไม่มี '\n'
    }
  }
}

void SerialLink::handle_line(const std::string & line)
{
  if (line.empty()) {
    return;
  }
  if (line[0] == 'S' && line.size() > 2 && line[1] == ' ') {
    unsigned long ms = 0;
    unsigned sys = 0;
    int p[kAxisCount]{};
    int v[kAxisCount]{};
    unsigned f[kAxisCount]{};
    const int got = std::sscanf(
      line.c_str(), "S %lu %u %d %d %u %d %d %u %d %d %u %d %d %u",
      &ms, &sys,
      &p[0], &v[0], &f[0], &p[1], &v[1], &f[1],
      &p[2], &v[2], &f[2], &p[3], &v[3], &f[3]);
    if (got != 14) {
      return;
    }
    Snapshot snap;
    for (std::size_t i = 0; i < kAxisCount; ++i) {
      snap.pos[i] = p[i];
      snap.vel[i] = v[i];
      snap.flags[i] = static_cast<std::uint8_t>(f[i]);
    }
    snap.sys = static_cast<std::uint8_t>(sys);
    snap.board_ms = ms;
    snap.host_ns = now_ns();
    snap.valid = true;
    publish(snap);
    return;
  }

  if (line[0] == '#' || line.compare(0, 3, "ERR") == 0) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (log_.size() < 4096) {
      log_ += line;
      log_ += '\n';
    }
  }
}

}  // namespace stepper_hardware
