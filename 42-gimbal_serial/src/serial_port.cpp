#include "gimbal_serial/serial_port.hpp"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>

namespace gimbal_serial
{
namespace
{

struct BaudEntry
{
  int baud;
  speed_t speed;
};

const BaudEntry kBauds[] = {
  {9600, B9600}, {19200, B19200}, {38400, B38400}, {57600, B57600}, {115200, B115200},
  {230400, B230400}, {460800, B460800}, {500000, B500000}, {576000, B576000},
  {921600, B921600}, {1000000, B1000000}, {1152000, B1152000}, {1500000, B1500000},
  {2000000, B2000000}, {2500000, B2500000}, {3000000, B3000000}, {3500000, B3500000},
  {4000000, B4000000},
};

bool lookupSpeed(int baud, speed_t & out)
{
  for (const auto & e : kBauds) {
    if (e.baud == baud) {out = e.speed; return true;}
  }
  return false;
}

std::string errnoText(const char * what)
{
  return std::string(what) + ": " + std::strerror(errno);
}

}  // namespace

bool SerialPort::baudSupported(int baud)
{
  speed_t s;
  return lookupSpeed(baud, s);
}

bool SerialPort::open(const std::string & path, int baud, std::string & err)
{
  close();
  last_errno_ = 0;
  speed_t speed;
  if (!lookupSpeed(baud, speed)) {
    err = "unsupported baudrate " + std::to_string(baud);
    return false;
  }

  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    last_errno_ = errno;
    err = errnoText(("open " + path).c_str());
    return false;
  }
  // 第 2 节：先 TIOCEXCL，再 flock。任何一步失败直接报错，不静默重试。
  if (ioctl(fd, TIOCEXCL) != 0) {
    last_errno_ = errno;
    err = errnoText(("ioctl(TIOCEXCL) " + path).c_str());
    ::close(fd);
    return false;
  }
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
    last_errno_ = errno;
    err = errnoText(("flock(LOCK_EX|LOCK_NB) " + path + "，可能已被其他进程占用").c_str());
    ::close(fd);
    return false;
  }

  termios tio;
  if (tcgetattr(fd, &tio) != 0) {
    last_errno_ = errno;
    err = errnoText(("tcgetattr " + path).c_str());
    ioctl(fd, TIOCNXCL);  // 已持有 flock，独占标志是我们设的，关之前清掉
    ::close(fd);
    return false;
  }
  cfmakeraw(&tio);                    // 关闭回显、行缓冲、软件流控，8 位
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);  // 8N1，无流控
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
    last_errno_ = errno;
    err = errnoText("cfsetspeed");
    ioctl(fd, TIOCNXCL);  // 已持有 flock，独占标志是我们设的，关之前清掉
    ::close(fd);
    return false;
  }
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    last_errno_ = errno;
    err = errnoText(("tcsetattr " + path).c_str());
    ioctl(fd, TIOCNXCL);  // 已持有 flock，独占标志是我们设的，关之前清掉
    ::close(fd);
    return false;
  }
  tcflush(fd, TCIOFLUSH);

  fd_ = fd;
  path_ = path;
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    // 先清 TIOCEXCL 再 close：pty 上只要 master 还开着，slave 的 tty 对象就不释放，
    // TTY_EXCLUSIVE 会一直留着，非 root 重开会得到 EBUSY（root 绕过 TIOCEXCL，看不出来）。
    ioctl(fd_, TIOCNXCL);
    ::close(fd_);  // 同时释放 flock
    fd_ = -1;
  }
}

ssize_t SerialPort::readSome(std::uint8_t * buf, std::size_t cap, int timeout_ms)
{
  if (fd_ < 0) {return -1;}
  pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;
  const int pr = poll(&pfd, 1, timeout_ms);
  if (pr < 0) {return errno == EINTR ? 0 : -1;}
  if (pr == 0) {return 0;}
  if (pfd.revents & POLLIN) {
    const ssize_t n = ::read(fd_, buf, cap);
    if (n > 0) {return n;}
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {return 0;}
    return -1;  // 0（对端关闭）或其他错误
  }
  return -1;  // POLLHUP、POLLERR、POLLNVAL
}

bool SerialPort::writeAll(const std::uint8_t * data, std::size_t size)
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd_ < 0) {return false;}
  std::size_t off = 0;
  int stalls = 0;
  while (off < size) {
    const ssize_t n = ::write(fd_, data + off, size - off);
    if (n > 0) {
      off += static_cast<std::size_t>(n);
      stalls = 0;
      continue;
    }
    if (n < 0 && errno == EINTR) {continue;}
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd pfd;
      pfd.fd = fd_;
      pfd.events = POLLOUT;
      pfd.revents = 0;
      if (poll(&pfd, 1, 100) <= 0 && ++stalls >= 10) {return false;}  // 最多等约 1 s
      continue;
    }
    return false;
  }
  return true;
}

PortChoice selectPort(
  const std::string & param, const std::string & env, bool gimbal_exists,
  const std::vector<std::string> & candidates)
{
  if (!param.empty() && param != "auto") {return {param, "param"};}
  if (!env.empty()) {return {env, "RM_SIM_SERIAL"};}
  if (gimbal_exists) {return {"/dev/gimbal", "udev"};}
  if (!candidates.empty()) {return {candidates.front(), "scan"};}
  return {"", "none"};
}

}  // namespace gimbal_serial
