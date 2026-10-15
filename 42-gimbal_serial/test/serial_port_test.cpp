// SerialPort 在 pty 上的行为：独占打开、读写、设备消失。不需要真实串口。
#include <gtest/gtest.h>

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include <string>
#include <vector>

#include "gimbal_serial/serial_port.hpp"

using gimbal_serial::SerialPort;

namespace
{

struct Pty
{
  int master = -1;
  std::string slave;
  Pty()
  {
    master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) {
      ADD_FAILURE() << "cannot create pty";
      return;
    }
    fcntl(master, F_SETFL, fcntl(master, F_GETFL) | O_NONBLOCK);
    const char * name = ptsname(master);
    if (name) {slave = name;}
  }
  ~Pty()
  {
    if (master >= 0) {::close(master);}
  }
};

}  // namespace

TEST(SerialPort, OpenReadWrite)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  SerialPort sp;
  std::string err;
  ASSERT_TRUE(sp.open(pty.slave, 921600, err)) << err;
  EXPECT_TRUE(sp.isOpen());

  // raw 模式：0x0D、0x0A 不能被改写，也不回显
  const std::vector<std::uint8_t> tx = {0xAA, 0xFF, 0x0D, 0x0A, 0x03, 0x11, 0x13, 0x00, 0x80};
  ASSERT_EQ(write(pty.master, tx.data(), tx.size()), static_cast<ssize_t>(tx.size()));
  std::uint8_t buf[64];
  std::vector<std::uint8_t> rx;
  for (int i = 0; i < 20 && rx.size() < tx.size(); ++i) {
    const ssize_t n = sp.readSome(buf, sizeof buf, 50);
    ASSERT_GE(n, 0);
    rx.insert(rx.end(), buf, buf + n);
  }
  EXPECT_EQ(rx, tx);

  // 超时返回 0
  EXPECT_EQ(sp.readSome(buf, sizeof buf, 20), 0);

  // 写：字节原样到达 master
  const std::vector<std::uint8_t> out = {0xAA, 0xEE, 0x30, 0x08, 0x0A, 0x0D, 0x13};
  ASSERT_TRUE(sp.writeAll(out.data(), out.size()));
  std::vector<std::uint8_t> got;
  for (int i = 0; i < 20 && got.size() < out.size(); ++i) {
    const ssize_t n = read(pty.master, buf, sizeof buf);
    if (n > 0) {got.insert(got.end(), buf, buf + n);} else {usleep(10000);}
  }
  EXPECT_EQ(got, out);
}

TEST(SerialPort, SecondOpenFails)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  SerialPort a, b;
  std::string err;
  ASSERT_TRUE(a.open(pty.slave, 921600, err)) << err;
  // 独占：TIOCEXCL 或 flock 至少有一个会拒绝第二次打开（root 可绕过 TIOCEXCL，但绕不过 flock）
  EXPECT_FALSE(b.open(pty.slave, 921600, err));
  EXPECT_FALSE(err.empty());
  EXPECT_FALSE(b.isOpen());
  // TIOCEXCL 对 root 不生效：root 下上面的 FALSE 只来自 flock，独占标志本身没有被验证。
  if (geteuid() == 0) {
    GTEST_LOG_(WARNING) << "root 下未覆盖独占（TIOCEXCL 被 root 绕过）；下面用 TIOCGEXCL 直接检查标志位，"
      "非 root 的 EBUSY 行为要用非 root 用户再跑一遍这个测试";
    RecordProperty("root_exclusive_not_covered", "true");
  } else {
    const int raw = ::open(pty.slave.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    const int open_errno = errno;
    EXPECT_LT(raw, 0);
    EXPECT_EQ(open_errno, EBUSY);  // 独占期间，非 root 的裸 open 被内核拒绝
    if (raw >= 0) {::close(raw);}
  }
  a.close();
  // close() 必须清掉 TIOCEXCL：pty 上 master 还开着时 slave 的 tty 对象不释放，标志会一直留着，
  // 非 root 再打开会得到 EBUSY。root 能打开，直接读标志位（非 root 打不开就说明标志还在）。
  {
    const int raw = ::open(pty.slave.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    ASSERT_GE(raw, 0) << "close() 之后仍然打不开：" << strerror(errno);
    int excl = -1;
    ASSERT_EQ(ioctl(raw, TIOCGEXCL, &excl), 0);
    EXPECT_EQ(excl, 0) << "close() 没有清 TIOCEXCL";
    ::close(raw);
  }
  // 关闭后可以再次打开
  EXPECT_TRUE(b.open(pty.slave, 921600, err)) << err;
}

TEST(SerialPort, OpenErrors)
{
  SerialPort sp;
  std::string err;
  EXPECT_FALSE(sp.open("/dev/definitely_not_here", 921600, err));
  EXPECT_NE(err.find("open"), std::string::npos);
  Pty pty;
  EXPECT_FALSE(sp.open(pty.slave, 12345, err));  // 不支持的波特率
  EXPECT_FALSE(SerialPort::baudSupported(12345));
  EXPECT_TRUE(SerialPort::baudSupported(921600));
}

TEST(SerialPort, PeerCloseIsFatal)
{
  SerialPort sp;
  std::string err;
  {
    Pty pty;
    ASSERT_TRUE(sp.open(pty.slave, 921600, err)) << err;
  }  // master 关闭
  std::uint8_t buf[16];
  ssize_t n = 0;
  for (int i = 0; i < 5 && n == 0; ++i) {n = sp.readSome(buf, sizeof buf, 50);}
  EXPECT_LT(n, 0);
}
