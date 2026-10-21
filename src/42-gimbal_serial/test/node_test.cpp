// 节点集成测试：pty 充当串口，BridgeNode 与测试节点在同一进程里通信。
// 覆盖：Imu 与 TF 的发布（stamp 取主机接收时刻）、坏帧丢弃、复位检测下仍然发布、
// cmd_vel_gimbal 转发（每条一帧，不保活）、yaw_sign、imu_axis_sign、独占打开、退出补零指令。
#include <gtest/gtest.h>

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include <rcutils/logging.h>
#include <rcutils/time.h>

#include <bridge_node.hpp>
#include <protocol.hpp>

using namespace std::chrono_literals;
using namespace gimbal_serial;  // NOLINT

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
  ~Pty() {if (master >= 0) {::close(master);}}

  void send(const std::vector<std::uint8_t> & b) {ASSERT_EQ(write(master, b.data(), b.size()), static_cast<ssize_t>(b.size()));}

  // 读到至少 want 字节或超时
  std::vector<std::uint8_t> readAtLeast(std::size_t want, std::chrono::milliseconds timeout)
  {
    std::vector<std::uint8_t> out;
    const auto end = std::chrono::steady_clock::now() + timeout;
    std::uint8_t buf[256];
    while (out.size() < want && std::chrono::steady_clock::now() < end) {
      const ssize_t n = read(master, buf, sizeof buf);
      if (n > 0) {out.insert(out.end(), buf, buf + n);} else {std::this_thread::sleep_for(5ms);}
    }
    return out;
  }
};

std::vector<std::uint8_t> imuFrame(
  std::uint16_t seq, std::uint32_t stamp, std::array<std::int16_t, 3> a,
  std::array<std::int16_t, 3> g, std::array<std::int16_t, 4> q)
{
  std::vector<std::uint8_t> f = {proto::kHead, proto::kAddrMcuToPc, proto::kFuncImu, proto::kImuLen};
  auto u16 = [&f](std::uint16_t v) {f.push_back(v & 0xFF); f.push_back(v >> 8);};
  u16(seq);
  u16(stamp & 0xFFFF);
  u16(stamp >> 16);
  for (auto v : a) {u16(static_cast<std::uint16_t>(v));}
  for (auto v : g) {u16(static_cast<std::uint16_t>(v));}
  for (auto v : q) {u16(static_cast<std::uint16_t>(v));}
  std::uint8_t s, ss;
  proto::calcVerify(f.data(), proto::kImuLen, s, ss);
  f.push_back(s);
  f.push_back(ss);
  return f;
}

class Probe
{
public:
  // imu_topic 默认取节点的默认参数值：让集成测试直接验证赛题要求的话题名。
  explicit Probe(const std::string & name, const std::string & imu_topic = "/red_standard_robot1/gimbal")
  {
    node = std::make_shared<rclcpp::Node>(name);
    imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic,
      rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        std::lock_guard<std::mutex> l(mu);
        imus.push_back(*m);
        recv_times.push_back(node->now());
      });
    tf_sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 100, [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
        std::lock_guard<std::mutex> l(mu);
        for (const auto & t : m->transforms) {tfs.push_back(t);}
      });
    cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>(
      "/offboardlink/cmd_vel_gimbal", rclcpp::QoS(10).reliable());
  }
  std::size_t imuCount() {std::lock_guard<std::mutex> l(mu); return imus.size();}
  bool waitImus(std::size_t n, std::chrono::milliseconds timeout)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      if (imuCount() >= n) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  std::mutex mu;
  std::vector<sensor_msgs::msg::Imu> imus;
  std::vector<rclcpp::Time> recv_times;
  std::vector<geometry_msgs::msg::TransformStamped> tfs;
};

// 在后台线程里 spin 一批节点，析构时停止。
class Spinner
{
public:
  Spinner()
  : exec_(rclcpp::ExecutorOptions(), 2)
  {
  }
  void add(rclcpp::Node::SharedPtr n) {exec_.add_node(n);}
  void start() {th_ = std::thread([this] {exec_.spin();});}
  ~Spinner()
  {
    exec_.cancel();
    if (th_.joinable()) {th_.join();}
  }

private:
  rclcpp::executors::MultiThreadedExecutor exec_;
  std::thread th_;
};

rclcpp::NodeOptions bridgeOptions(const std::string & port, const std::vector<rclcpp::Parameter> & extra)
{
  std::vector<rclcpp::Parameter> p = {
    rclcpp::Parameter("port", port), rclcpp::Parameter("imu_wait_warn_s", 60.0)};
  p.insert(p.end(), extra.begin(), extra.end());
  return rclcpp::NodeOptions().parameter_overrides(p);
}

// 等待 cmd 订阅与发布端配上（同进程内几乎立即完成）
bool waitMatched(Probe & pr)
{
  for (int i = 0; i < 400; ++i) {
    if (pr.cmd_pub->get_subscription_count() > 0) {return true;}
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

}  // namespace

TEST(BridgeNode, PublishesImuAndTf)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  auto bridge = std::make_shared<BridgeNode>(bridgeOptions(pty.slave, {}));
  Probe probe("probe_imu");
  Spinner sp;
  sp.add(bridge);
  sp.add(probe.node);
  sp.start();

  // 前面垃圾字节，第 3 帧校验错，第 5 帧 q 全零（应用层丢弃），其余 8 帧有效；
  // 第 6 帧起 seq、stamp 复位，仍然要发布。
  std::vector<std::uint8_t> stream = {0x55, 0x00, 0xAA, 0x12, 0x99};
  const std::array<std::int16_t, 4> level = {32760, 0, 0, 0};
  int valid = 0;
  for (int i = 0; i < 10; ++i) {
    std::uint16_t seq = i < 5 ? i : 5000 + (i - 5);           // 中间 seq 突变
    std::uint32_t stamp = i < 5 ? 1000000 + i * 2000 : 500 + (i - 5) * 2000;  // stamp 倒退
    auto f = imuFrame(seq, stamp, {-271, 0, 1338}, {-559, 200, 2760}, level);
    if (i == 2) {f[10] ^= 0x01; --valid;}  // 数据位翻转 -> 校验错
    if (i == 4) {f = imuFrame(seq, stamp, {0, 0, 1365}, {0, 0, 0}, {0, 0, 0, 0});--valid;}
    stream.insert(stream.end(), f.begin(), f.end());
    ++valid;
  }
  ASSERT_EQ(valid, 8);
  pty.send(stream);
  ASSERT_TRUE(probe.waitImus(8, 5s));
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(probe.imuCount(), 8u);

  std::lock_guard<std::mutex> l(probe.mu);
  const auto & m = probe.imus.front();
  EXPECT_EQ(m.header.frame_id, "gimbal_link");
  EXPECT_NEAR(m.linear_acceleration.x, -271 * proto::kAccelScale, 1e-9);  // 比力，不取负
  EXPECT_NEAR(m.linear_acceleration.z, 1338 * proto::kAccelScale, 1e-9);
  EXPECT_NEAR(m.angular_velocity.x, -559 * proto::kGyroScale, 1e-9);      // 陀螺 x、y 不取负
  EXPECT_NEAR(m.angular_velocity.y, 200 * proto::kGyroScale, 1e-9);
  EXPECT_NEAR(m.angular_velocity.z, 2760 * proto::kGyroScale, 1e-9);
  EXPECT_NEAR(m.orientation.w, 1.0, 1e-12);
  EXPECT_EQ(m.orientation_covariance[8], 1.0e-2);
  EXPECT_EQ(m.angular_velocity_covariance[0], 1.0e-5);
  EXPECT_EQ(m.linear_acceleration_covariance[4], 1.0e-2);

  // header.stamp 是主机接收时刻：不早于测试开始，不晚于收到时刻（同一时钟）
  for (std::size_t i = 0; i < probe.imus.size(); ++i) {
    const rclcpp::Time st(probe.imus[i].header.stamp, RCL_ROS_TIME);
    EXPECT_LE(st.nanoseconds(), probe.recv_times[i].nanoseconds());
    EXPECT_GT(st.nanoseconds(), probe.recv_times[i].nanoseconds() - 2000000000LL);
    if (i > 0) {
      const rclcpp::Time prev(probe.imus[i - 1].header.stamp, RCL_ROS_TIME);
      EXPECT_GE(st.nanoseconds(), prev.nanoseconds());
    }
  }

  // TF：父 world，子 gimbal_link，平移 0，与 Imu 同一 stamp 同一旋转
  ASSERT_EQ(probe.tfs.size(), 8u);
  for (std::size_t i = 0; i < probe.tfs.size(); ++i) {
    const auto & t = probe.tfs[i];
    EXPECT_EQ(t.header.frame_id, "world");
    EXPECT_EQ(t.child_frame_id, "gimbal_link");
    EXPECT_EQ(t.transform.translation.x, 0.0);
    EXPECT_EQ(t.transform.translation.y, 0.0);
    EXPECT_EQ(t.transform.translation.z, 0.0);
    EXPECT_EQ(rclcpp::Time(t.header.stamp).nanoseconds(),
      rclcpp::Time(probe.imus[i].header.stamp).nanoseconds());
    EXPECT_EQ(t.transform.rotation.w, probe.imus[i].orientation.w);
  }
}

// 持续模拟新鲜IMU；析构自动停止，不依赖真实设备。
struct FreshImuFeed
{
  Pty & pty;
  std::atomic<bool> running{true};
  std::thread thread;
  explicit FreshImuFeed(Pty & p) : pty(p), thread([this]() {
    std::uint16_t seq = 0;
    std::uint32_t stamp = 0;
    while (running) {
      const auto frame = imuFrame(seq++, stamp, {0,0,1365}, {0,0,0}, {32760,0,0,0});
      ::write(pty.master, frame.data(), frame.size());
      stamp += 20000;
      std::this_thread::sleep_for(20ms);
    }
  }) {}
  ~FreshImuFeed() {running = false; thread.join();}
};

TEST(BridgeNode, NoCommandBeforeFirstImuOrOnUnusedShutdown)
{
  Pty pty;
  auto bridge = std::make_shared<BridgeNode>(bridgeOptions(pty.slave, {
    rclcpp::Parameter("subscribe_cmd_vel", true)}));
  Probe probe("probe_no_imu");
  {
    Spinner sp; sp.add(bridge); sp.add(probe.node); sp.start();
    ASSERT_TRUE(waitMatched(probe));
    geometry_msgs::msg::Twist command; command.angular.z = 1.0;
    probe.cmd_pub->publish(command);
    EXPECT_TRUE(pty.readAtLeast(1, 200ms).empty());
  }
  bridge.reset();
  EXPECT_TRUE(pty.readAtLeast(1, 200ms).empty());
}

TEST(BridgeNode, CmdVelForwardedOneFramePerMessageNoKeepalive)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  auto bridge = std::make_shared<BridgeNode>(bridgeOptions(pty.slave, {
    rclcpp::Parameter("subscribe_cmd_vel", true),
    rclcpp::Parameter("yaw_sign", -1)}));
  Probe probe("probe_cmd");
  FreshImuFeed feed(pty);
  {
    Spinner sp;
    sp.add(bridge);
    sp.add(probe.node);
    sp.start();
    ASSERT_TRUE(waitMatched(probe));

    geometry_msgs::msg::Twist t;
    t.angular.z = 3.0;    // yaw
    t.angular.y = -0.125; // pitch
    // 首帧后0.6秒内仍静默，确保已经接收过IMU再开始计等待窗口。
    ASSERT_TRUE(probe.waitImus(1, 3s));
    EXPECT_TRUE(pty.readAtLeast(1, 50ms).empty());
    std::this_thread::sleep_for(650ms);
    probe.cmd_pub->publish(t);
    // yaw_sign = -1：线上 yaw_omega = -3.0；angular.y 对应 pitch
    auto got = pty.readAtLeast(proto::kCmdFrameLen, 3s);
    const auto want = proto::encodeCmd(-3.0f, -0.125f);
    ASSERT_EQ(got.size(), proto::kCmdFrameLen);
    EXPECT_EQ(got, std::vector<std::uint8_t>(want.begin(), want.end()));

    // 不做保活：500 ms 内没有任何多余字节
    EXPECT_TRUE(pty.readAtLeast(1, 500ms).empty());

    // 再来一条 -> 再一帧；非有限值直接丢弃
    t.angular.z = std::nan("");
    probe.cmd_pub->publish(t);
    t.angular.z = 0.0;
    t.angular.y = 0.0;
    probe.cmd_pub->publish(t);
    got = pty.readAtLeast(proto::kCmdFrameLen, 3s);
    ASSERT_EQ(got.size(), proto::kCmdFrameLen);
    const auto zero = proto::encodeCmd(-0.0f, 0.0f);  // yaw_sign * 0.0 = -0.0，仍是有限值
    EXPECT_EQ(got, std::vector<std::uint8_t>(zero.begin(), zero.end()));
    EXPECT_TRUE(pty.readAtLeast(1, 300ms).empty());
  }
  // 退出时补 3 帧零指令（第 3.5 节）
  bridge.reset();
  const auto tail = pty.readAtLeast(3 * proto::kCmdFrameLen, 2s);
  ASSERT_EQ(tail.size(), 3 * proto::kCmdFrameLen);
  const auto z = proto::encodeCmd(0.0f, 0.0f);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(
      std::vector<std::uint8_t>(tail.begin() + i * 14, tail.begin() + (i + 1) * 14),
      std::vector<std::uint8_t>(z.begin(), z.end()));
  }
}

TEST(BridgeNode, ImuTopicParameterOverridesDefault)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  auto bridge = std::make_shared<BridgeNode>(bridgeOptions(pty.slave, {
    rclcpp::Parameter("imu_topic", std::string("/custom/imu_topic"))}));
  Probe probe("probe_custom_imu_topic", "/custom/imu_topic");
  Spinner sp;
  sp.add(bridge);
  sp.add(probe.node);
  sp.start();
  pty.send(imuFrame(0, 0, {0, 0, 1365}, {0, 0, 0}, {32760, 0, 0, 0}));
  EXPECT_TRUE(probe.waitImus(1, 5s));
  EXPECT_EQ(probe.imuCount(), 1u);
}

TEST(BridgeNode, AxisSignOnlyAffectsAccelAndGyro)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  auto bridge = std::make_shared<BridgeNode>(bridgeOptions(pty.slave, {
    rclcpp::Parameter("imu_axis_sign", std::vector<int64_t>{-1, 1, -1})}));
  Probe probe("probe_sign");
  Spinner sp;
  sp.add(bridge);
  sp.add(probe.node);
  sp.start();
  const std::array<std::int16_t, 4> q = {32596, 0, 3271, 0};  // 低头约 0.2 rad
  pty.send(imuFrame(1, 1000, {-271, 5, 1338}, {-559, 200, 2760}, q));
  ASSERT_TRUE(probe.waitImus(1, 5s));
  std::lock_guard<std::mutex> l(probe.mu);
  const auto & m = probe.imus.front();
  EXPECT_NEAR(m.linear_acceleration.x, 271 * proto::kAccelScale, 1e-9);
  EXPECT_NEAR(m.linear_acceleration.y, 5 * proto::kAccelScale, 1e-9);
  EXPECT_NEAR(m.linear_acceleration.z, -1338 * proto::kAccelScale, 1e-9);
  EXPECT_NEAR(m.angular_velocity.x, 559 * proto::kGyroScale, 1e-9);
  EXPECT_NEAR(m.angular_velocity.z, -2760 * proto::kGyroScale, 1e-9);
  // q 不受影响
  EXPECT_GT(m.orientation.y, 0.09);
  EXPECT_GT(m.orientation.w, 0.99);
}

TEST(BridgeNode, ExclusiveOpenAndBadParams)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  auto first = std::make_shared<BridgeNode>(bridgeOptions(pty.slave, {}));
  // 同一设备第二次打开：直接报错，不静默重试
  EXPECT_THROW(std::make_shared<BridgeNode>(bridgeOptions(pty.slave, {})), std::runtime_error);
  first.reset();

  Pty pty2;
  EXPECT_THROW(
    std::make_shared<BridgeNode>(bridgeOptions(pty2.slave, {
      rclcpp::Parameter("imu_axis_sign", std::vector<int64_t>{1, 2, 1})})),
    std::runtime_error);
  EXPECT_THROW(
    std::make_shared<BridgeNode>(bridgeOptions("/dev/does_not_exist", {})), std::runtime_error);
  // 波特率不支持
  EXPECT_THROW(
    std::make_shared<BridgeNode>(bridgeOptions(pty2.slave, {rclcpp::Parameter("baudrate", 12345)})),
    std::runtime_error);
}

TEST(BridgeNode, PortAutoUsesRmSimSerial)
{
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  setenv("RM_SIM_SERIAL", pty.slave.c_str(), 1);
  // port 参数为 auto（默认）：使用环境变量
  auto bridge = std::make_shared<BridgeNode>(
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("imu_wait_warn_s", 60.0)}));
  Probe probe("probe_env");
  Spinner sp;
  sp.add(bridge);
  sp.add(probe.node);
  sp.start();
  pty.send(imuFrame(0, 0, {0, 0, 1365}, {0, 0, 0}, {32760, 0, 0, 0}));
  EXPECT_TRUE(probe.waitImus(1, 5s));
  unsetenv("RM_SIM_SERIAL");
}

// 无 0x01 告警按“零字节 vs 有字节无帧”分支。捕获 rcutils 日志逐条核对。
namespace
{
std::mutex g_log_mu;
std::vector<std::string> * g_logs = nullptr;
struct CapturedLog
{
  int severity;
  std::string message;
};
std::vector<CapturedLog> * g_log_entries = nullptr;

void capturingLogHandler(
  const rcutils_log_location_t * /*location*/, int severity, const char * /*name*/,
  rcutils_time_point_value_t /*timestamp*/, const char * format, va_list * args)
{
  char buf[2048];
  if (vsnprintf(buf, sizeof buf, format, *args) < 0) {return;}
  std::lock_guard<std::mutex> l(g_log_mu);
  if (g_logs) {g_logs->emplace_back(buf);}
  if (g_log_entries) {g_log_entries->push_back({severity, buf});}
}

bool logsContain(const std::vector<std::string> & logs, const char * needle)
{
  for (const auto & s : logs) {
    if (s.find(needle) != std::string::npos) {return true;}
  }
  return false;
}

// 日志由 ROS 定时器和串口读线程共同写入；只在持锁快照中检查，析构恢复原 handler。
class LogCapture
{
public:
  LogCapture() : original_(rcutils_logging_get_output_handler())
  {
    {
      std::lock_guard<std::mutex> l(g_log_mu);
      g_log_entries = &entries_;
    }
    rcutils_logging_set_output_handler(capturingLogHandler);
  }
  ~LogCapture()
  {
    rcutils_logging_set_output_handler(original_);
    std::lock_guard<std::mutex> l(g_log_mu);
    g_log_entries = nullptr;
  }

  std::vector<std::string> matching(int severity, const char * needle)
  {
    std::lock_guard<std::mutex> l(g_log_mu);
    std::vector<std::string> messages;
    for (const auto & entry : entries_) {
      if (entry.severity == severity && entry.message.find(needle) != std::string::npos) {
        messages.push_back(entry.message);
      }
    }
    return messages;
  }

  bool waitMatching(int severity, const char * needle, std::size_t count)
  {
    const auto end = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < end) {
      if (matching(severity, needle).size() >= count) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

private:
  rcutils_logging_output_handler_t original_;
  std::vector<CapturedLog> entries_;
};

// 通过公开的 reader 注入驱动查询结果，不触碰真实 UART，也不绕过节点后台轮询。
class UartCounterSource
{
public:
  explicit UartCounterSource(UartCounterResult result) : result_(result) {}
  UartCounterReader reader()
  {
    return [this](int /*fd*/) {
      std::lock_guard<std::mutex> l(mu_);
      ++calls_;
      return result_;
    };
  }
  void set(UartCounterResult result)
  {
    std::lock_guard<std::mutex> l(mu_);
    result_ = result;
  }
  std::size_t calls()
  {
    std::lock_guard<std::mutex> l(mu_);
    return calls_;
  }

private:
  std::mutex mu_;
  UartCounterResult result_;
  std::size_t calls_ = 0;
};

bool waitCounterCalls(UartCounterSource & source, std::size_t calls)
{
  const auto end = std::chrono::steady_clock::now() + 4s;
  while (std::chrono::steady_clock::now() < end) {
    if (source.calls() >= calls) {return true;}
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

void expectUartDeltas(const std::string & message, const UartErrorCounters & expected)
{
  // 完整数值与单词边界：overrun 不能误匹配 buf_overrun，4 不能误匹配 400。
  const auto field = [&message](const char * name, std::uint32_t value) {
    const std::regex pattern("\\b" + std::string(name) + "=" + std::to_string(value) + "\\b");
    EXPECT_TRUE(std::regex_search(message, pattern)) << message;
  };
  field("frame", expected.frame);
  field("parity", expected.parity);
  field("overrun", expected.overrun);
  field("brk", expected.brk);
  field("buf_overrun", expected.buf_overrun);
}
}  // namespace

TEST(BridgeNode, NoImuWarningBranches)
{
  std::vector<std::string> logs;
  {
    std::lock_guard<std::mutex> l(g_log_mu);
    g_logs = &logs;
  }
  const auto original_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(capturingLogHandler);

  // 分支一：干净 PTY、零字节 -> 告警含“累计 0 字节”与适配器排查提示
  {
    Pty pty;
    ASSERT_FALSE(pty.slave.empty());
    auto bridge = std::make_shared<BridgeNode>(
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("port", pty.slave),
          rclcpp::Parameter("imu_wait_warn_s", 0.3)}));
    Spinner sp;
    sp.add(bridge);
    sp.start();
    std::this_thread::sleep_for(1500ms);  // 诊断定时器 1 s 周期，首拍即触发
    bridge.reset();
  }
  const bool zero_byte_branch = logsContain(logs, "累计 0 字节");
  EXPECT_TRUE(logsContain(logs, "ls -l /dev/serial/by-id/"));
  logs.clear();

  // 分支二：构造后写噪声字节（open 会 tcflush，必须后写）-> 告警走“已收到 N 字节”
  {
    Pty pty;
    ASSERT_FALSE(pty.slave.empty());
    auto bridge = std::make_shared<BridgeNode>(
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("port", pty.slave),
          rclcpp::Parameter("imu_wait_warn_s", 0.3)}));
    Spinner sp;
    sp.add(bridge);
    sp.start();
    const std::vector<std::uint8_t> noise(200, 0x55);
    ASSERT_EQ(
      write(pty.master, noise.data(), noise.size()), static_cast<ssize_t>(noise.size()));
    std::this_thread::sleep_for(1500ms);
    bridge.reset();
  }
  const bool bytes_branch = logsContain(logs, "已收到") && logsContain(logs, "字节");
  EXPECT_FALSE(logsContain(logs, "累计 0 字节"));

  rcutils_logging_set_output_handler(original_handler);
  {
    std::lock_guard<std::mutex> l(g_log_mu);
    g_logs = nullptr;
  }
  EXPECT_TRUE(zero_byte_branch);
  EXPECT_TRUE(bytes_branch);
}

TEST(BridgeNode, UartCountersIgnoreHistoricalErrorsAndHandleWraparound)
{
  LogCapture logs;
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  const auto near_wrap = std::numeric_limits<std::uint32_t>::max() - 2;
  UartCounterSource source({
    UartCounterStatus::Ok, {near_wrap, near_wrap, near_wrap, near_wrap, near_wrap}, 0});
  auto bridge = std::make_shared<BridgeNode>(
    bridgeOptions(pty.slave, {}), source.reader());
  ASSERT_EQ(source.calls(), 1u);  // 启动即读取驱动已有计数作为基线
  ASSERT_TRUE(waitCounterCalls(source, 2));
  EXPECT_TRUE(logs.matching(RCUTILS_LOG_SEVERITY_WARN, "UART 错误增量").empty());

  // 模拟驱动 uint32 计数回绕；五种错误分别增加 4/5/6/7/8，不能报负值或巨量。
  source.set({UartCounterStatus::Ok, {1, 2, 3, 4, 5}, 0});
  ASSERT_TRUE(waitCounterCalls(source, 3));
  ASSERT_TRUE(logs.waitMatching(RCUTILS_LOG_SEVERITY_WARN, "UART 错误增量", 1));
  auto warnings = logs.matching(RCUTILS_LOG_SEVERITY_WARN, "UART 错误增量");
  ASSERT_EQ(warnings.size(), 1u);
  expectUartDeltas(warnings.front(), {4, 5, 6, 7, 8});

  // 同一计数下一拍不重复报警，证明每次成功查询都推进基线。
  ASSERT_TRUE(waitCounterCalls(source, 4));
  bridge.reset();  // 等读线程完成该次检查，避免日志断言与后台回调竞争
  EXPECT_EQ(logs.matching(RCUTILS_LOG_SEVERITY_WARN, "UART 错误增量").size(), 1u);
}

TEST(BridgeNode, UnsupportedUartCountersAreReportedOnceAndDoNotStopImu)
{
  LogCapture logs;
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  UartCounterSource source({UartCounterStatus::Unsupported, {}, ENOTTY});
  auto bridge = std::make_shared<BridgeNode>(
    bridgeOptions(pty.slave, {}), source.reader());
  Probe probe("probe_unsupported_uart_counters");
  FreshImuFeed feed(pty);
  {
    Spinner spinner;
    spinner.add(bridge);
    spinner.add(probe.node);
    spinner.start();
    // 持续超过诊断周期，计数器停查后仍应原样发布有效 IMU。
    ASSERT_TRUE(probe.waitImus(70, 4s));
  }
  EXPECT_EQ(source.calls(), 1u);
  const auto info = logs.matching(RCUTILS_LOG_SEVERITY_INFO, "UART");
  ASSERT_EQ(info.size(), 1u);
  EXPECT_NE(info.front().find("不支持"), std::string::npos);
  EXPECT_TRUE(logs.matching(RCUTILS_LOG_SEVERITY_WARN, "UART").empty());
}

TEST(BridgeNode, UartCounterQueryErrorsRecoverWithoutFalseHistoricalDelta)
{
  LogCapture logs;
  Pty pty;
  ASSERT_FALSE(pty.slave.empty());
  UartCounterSource source({UartCounterStatus::Error, {}, EIO});
  auto bridge = std::make_shared<BridgeNode>(
    bridgeOptions(pty.slave, {}), source.reader());
  ASSERT_EQ(source.calls(), 1u);
  EXPECT_EQ(logs.matching(RCUTILS_LOG_SEVERITY_WARN, "查询失败").size(), 1u);

  // 连续两次失败仍只告警一次，且失败不能导致放弃后续轮询。
  ASSERT_TRUE(waitCounterCalls(source, 2));

  // 首次成功恢复只能建基线，不能将设备开机以来的旧错误报成本次新增。
  source.set({UartCounterStatus::Ok, {1000000, 2000000, 3000000, 4000000, 5000000}, 0});
  ASSERT_TRUE(waitCounterCalls(source, 3));
  EXPECT_EQ(logs.matching(RCUTILS_LOG_SEVERITY_WARN, "查询失败").size(), 1u);
  EXPECT_TRUE(logs.matching(RCUTILS_LOG_SEVERITY_WARN, "UART 错误增量").empty());

  // 成功恢复后再次出现失败应再次提示，但保留最后一次成功的计数基线。
  source.set({UartCounterStatus::Error, {}, EIO});
  ASSERT_TRUE(waitCounterCalls(source, 4));
  ASSERT_TRUE(logs.waitMatching(RCUTILS_LOG_SEVERITY_WARN, "查询失败", 2));

  // 失败不能清空已有基线；再次恢复后仅报距最后一次成功查询的真实增量。
  source.set({UartCounterStatus::Ok, {1000001, 2000002, 3000003, 4000004, 5000005}, 0});
  ASSERT_TRUE(waitCounterCalls(source, 5));
  ASSERT_TRUE(logs.waitMatching(RCUTILS_LOG_SEVERITY_WARN, "UART 错误增量", 1));
  bridge.reset();
  const auto warnings = logs.matching(RCUTILS_LOG_SEVERITY_WARN, "UART 错误增量");
  ASSERT_EQ(warnings.size(), 1u);
  expectUartDeltas(warnings.front(), {1, 2, 3, 4, 5});
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
