// SerialPort 在 pty 上的行为：独占打开、读写、设备消失。不需要真实串口。
#include <gtest/gtest.h>

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include <string>
#include <vector>

#include "gimbal_serial/serial_port.hpp"

using gimbal_serial::SerialPort;
using gimbal_serial::UartCounterResult;
using gimbal_serial::UartCounterStatus;

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

  bool seedDirtyTermios()
  {
    const int fd = ::open(slave.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {return false;}
    termios tio{};
    bool ok = tcgetattr(fd, &tio) == 0;
    if (ok) {
      tio.c_iflag |= IGNPAR | PARMRK | ISTRIP;
      tio.c_iflag &= ~INPCK;
      ok = tcsetattr(fd, TCSANOW, &tio) == 0;
    }
    ::close(fd);
    return ok;
  }
};

}  // namespace

TEST(SerialPort, OpenReadWrite)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  ASSERT_TRUE(pty.seedDirtyTermios());  // 上次打开遗留的输入标志不能污染本次 raw 配置
  SerialPort sp;
  std::string err;
  ASSERT_TRUE(sp.open(pty.slave, 921600, err)) << err;
  EXPECT_TRUE(sp.isOpen());

  // PTY 不实现硬件错误计数；查询不能改变 raw 字节流或妨碍后续收发。
  const auto counters = sp.readErrorCounters();
  EXPECT_EQ(counters.status, UartCounterStatus::Unsupported);
  EXPECT_EQ(counters.error_number, ENOTTY);

  // raw 模式：0x0D、0x0A 不能被改写，也不回显
  const std::vector<std::uint8_t> tx = {
    0xAA, 0xFF, 0x0D, 0x0A, 0x03, 0x11, 0x13, 0x00, 0x80,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xEC};
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
  const std::vector<std::uint8_t> out = {
    0xAA, 0xEE, 0x30, 0x08, 0x0A, 0x0D, 0x13, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xEC};
  ASSERT_TRUE(sp.writeAll(out.data(), out.size()));
  std::vector<std::uint8_t> got;
  for (int i = 0; i < 20 && got.size() < out.size(); ++i) {
    const ssize_t n = read(pty.master, buf, sizeof buf);
    if (n > 0) {got.insert(got.end(), buf, buf + n);} else {usleep(10000);}
  }
  EXPECT_EQ(got, out);
}

TEST(SerialPort, ErrorCountersRequireOpenPort)
{
  SerialPort sp;
  const auto before_open = sp.readErrorCounters();
  EXPECT_EQ(before_open.status, UartCounterStatus::Error);
  EXPECT_EQ(before_open.error_number, EBADF);

  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  std::string err;
  ASSERT_TRUE(sp.open(pty.slave, 921600, err)) << err;
  sp.close();
  const auto after_close = sp.readErrorCounters();
  EXPECT_EQ(after_close.status, UartCounterStatus::Error);
  EXPECT_EQ(after_close.error_number, EBADF);
}

TEST(SerialPort, InjectedErrorCounterReaderGetsOpenDescriptor)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  ASSERT_TRUE(pty.seedDirtyTermios());
  int calls = 0;
  SerialPort sp([&calls](int fd) {
    ++calls;
    EXPECT_GE(fcntl(fd, F_GETFD), 0);
    termios tio{};
    const int termios_result = tcgetattr(fd, &tio);
    EXPECT_EQ(termios_result, 0);
    if (termios_result == 0) {
      // 接收错误检测开启，但仍为 8N1；正常 FF 字节不会受 PARMRK 转义影响。
      EXPECT_NE(tio.c_iflag & INPCK, 0u);
      EXPECT_EQ(tio.c_iflag & (PARMRK | IGNPAR | ISTRIP), 0u);
      EXPECT_EQ(tio.c_cflag & PARENB, 0u);
    }
    return UartCounterResult{UartCounterStatus::Ok, {11, 22, 33, 44, 55}, 0};
  });
  EXPECT_EQ(sp.readErrorCounters().error_number, EBADF);
  EXPECT_EQ(calls, 0);

  std::string err;
  ASSERT_TRUE(sp.open(pty.slave, 921600, err)) << err;
  const auto result = sp.readErrorCounters();
  EXPECT_EQ(result.status, UartCounterStatus::Ok);
  EXPECT_EQ(result.error_number, 0);
  EXPECT_EQ(result.counters.frame, 11u);
  EXPECT_EQ(result.counters.parity, 22u);
  EXPECT_EQ(result.counters.overrun, 33u);
  EXPECT_EQ(result.counters.brk, 44u);
  EXPECT_EQ(result.counters.buf_overrun, 55u);
  EXPECT_EQ(calls, 1);

  sp.close();
  EXPECT_EQ(sp.readErrorCounters().error_number, EBADF);
  EXPECT_EQ(calls, 1);
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

// 端口身份解析：临时目录模拟 /dev/serial/by-id，验证反查、同芯片去重与优雅降级。
// glob 按名字排序：usb-FTDI < usb-QinHeng < usb-Silicon_Labs，同类芯片列表应为 [CH340, CP2102]。
TEST(SerialPort, PortIdentityByIdReverseLookup)
{
  char dir_tpl[] = "/tmp/gimbal_byid_test_XXXXXX";
  const char * dir = mkdtemp(dir_tpl);
  ASSERT_NE(dir, nullptr);
  const std::string by_id = std::string(dir) + "/by-id";
  ASSERT_EQ(mkdir(by_id.c_str(), 0755), 0);
  const std::string cp210_target = std::string(dir) + "/dev_cp210";
  const std::string ch340_target = std::string(dir) + "/dev_ch340";
  const std::string ftdi_target = std::string(dir) + "/dev_ftdi";
  for (const auto & t : {cp210_target, ch340_target, ftdi_target}) {
    const int fd = ::open(t.c_str(), O_CREAT | O_RDWR, 0644);
    ASSERT_GE(fd, 0) << t;
    ::close(fd);
  }
  const std::string cp210_entry =
    by_id + "/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
  const std::string cp210_entry2 = by_id + "/usb-Silicon_Labs_CP2104_OTHER_0002-if00-port0";
  const std::string ch340_entry = by_id + "/usb-QinHeng_Electronics_CH340_serial_converter_55BB-if00-port0";
  const std::string ftdi_entry = by_id + "/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0";
  ASSERT_EQ(symlink(cp210_target.c_str(), cp210_entry.c_str()), 0);
  ASSERT_EQ(symlink(cp210_target.c_str(), cp210_entry2.c_str()), 0);  // 同一设备的第二个接口条目
  ASSERT_EQ(symlink(ch340_target.c_str(), ch340_entry.c_str()), 0);
  ASSERT_EQ(symlink(ftdi_target.c_str(), ftdi_entry.c_str()), 0);

  const auto id_cp = gimbal_serial::inspectPortIdentity(cp210_target, by_id);
  EXPECT_FALSE(id_cp.resolved.empty());
  EXPECT_EQ(id_cp.by_id, cp210_entry);
  // 同类芯片按设备去重：CP210x 两条目只算一颗，FTDI 不算
  ASSERT_EQ(id_cp.rule_chip_by_ids.size(), 2u);
  EXPECT_EQ(id_cp.rule_chip_by_ids[0], ch340_entry);
  EXPECT_EQ(id_cp.rule_chip_by_ids[1], cp210_entry);

  // 打开 FTDI 目标：反查命中自己的条目；机器级同芯片统计与打开哪个设备无关
  const auto id_ft = gimbal_serial::inspectPortIdentity(ftdi_target, by_id);
  EXPECT_EQ(id_ft.by_id, ftdi_entry);
  EXPECT_EQ(id_ft.rule_chip_by_ids.size(), 2u);

  using gimbal_serial::isGimbalRuleChipByIdName;
  EXPECT_TRUE(isGimbalRuleChipByIdName("usb-Silicon_Labs_CP2102_x"));
  EXPECT_TRUE(isGimbalRuleChipByIdName("USB-QINHENG_CH340_Y"));
  EXPECT_TRUE(isGimbalRuleChipByIdName("usb-wch_CH9102_z"));
  EXPECT_FALSE(isGimbalRuleChipByIdName("usb-FTDI_FT232R_x"));
  EXPECT_FALSE(isGimbalRuleChipByIdName(""));

  // PTY：没有 by-id 条目，resolved 指向 /dev/pts
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  const auto id_pty = gimbal_serial::inspectPortIdentity(pty.slave, by_id);
  EXPECT_EQ(id_pty.by_id, "");
  EXPECT_EQ(id_pty.resolved.find("/dev/pts/"), 0u);

  // by-id 目录不存在：全空不崩；路径不存在：resolved 为空
  const auto id_none = gimbal_serial::inspectPortIdentity(cp210_target, std::string(dir) + "/no_such");
  EXPECT_EQ(id_none.by_id, "");
  EXPECT_EQ(id_none.rule_chip_by_ids.size(), 0u);
  const auto id_ghost = gimbal_serial::inspectPortIdentity("/dev/definitely_not_here", by_id);
  EXPECT_EQ(id_ghost.resolved, "");

  for (const auto & e : {cp210_entry, cp210_entry2, ch340_entry, ftdi_entry}) {::unlink(e.c_str());}
  for (const auto & t : {cp210_target, ch340_target, ftdi_target}) {::unlink(t.c_str());}
  ::rmdir(by_id.c_str());
  ::rmdir(dir);
}
