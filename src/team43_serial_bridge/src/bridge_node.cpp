#include "bridge_node.hpp"

#include <glob.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include "imu_convert.hpp"

using namespace std::chrono_literals;

namespace gimbal_serial
{
namespace
{

// 通信契约第 2 节固定为 921600 bps、8N1、无流控。
constexpr int kBaudrate = 921600;

std::int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 在 /dev 下按名字排序枚举 ttyUSB*、ttyACM*。
std::vector<std::string> scanSerialCandidates()
{
  std::vector<std::string> out;
  for (const char * pattern : {"/dev/ttyUSB*", "/dev/ttyACM*"}) {
    glob_t g{};
    if (glob(pattern, 0, nullptr, &g) == 0) {
      for (std::size_t i = 0; i < g.gl_pathc; ++i) {out.emplace_back(g.gl_pathv[i]);}
    }
    globfree(&g);
  }
  return out;
}

rcl_interfaces::msg::ParameterDescriptor dynamicDescriptor(const std::string & text)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  d.dynamic_typing = true;  // yaml 里写 1 还是 1.0 都接受
  return d;
}

}  // namespace

BridgeNode::BridgeNode(const rclcpp::NodeOptions & options, UartCounterReader counter_reader)
: rclcpp::Node("robo_bridge", options), serial_(std::move(counter_reader))
{
  // 本队实机固定 /dev/ttyUSB0；仿真/多适配器场景显式传 port:=auto 走枚举。
  const std::string port_param = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  const bool sub_cmd = declare_parameter<bool>("subscribe_cmd_vel", false);
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/red_standard_robot1/gimbal");
  const std::string cmd_topic =
    declare_parameter<std::string>("cmd_vel_topic", "/offboardlink/cmd_vel_gimbal");
  imu_wait_warn_s_ = declare_parameter<double>("imu_wait_warn_s", 15.0);

  axis_sign_ = readAxisSign();
  yaw_sign_ = readSign("yaw_sign");
  pitch_sign_ = readSign("pitch_sign");
  {
    declare_parameter(
      "omega_max", rclcpp::ParameterValue(2.0),
      dynamicDescriptor("转发前对两轴角速度限幅（rad/s），须在 (0, 2] 内，与 Python 链一致"));
    const rclcpp::Parameter p = get_parameter("omega_max");
    double v = 2.0;
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      v = static_cast<double>(p.as_int());
    } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      v = p.as_double();
    }
    if (!std::isfinite(v) || v <= 0.0 || v > proto::kOmegaHardMax) {
      throw std::runtime_error("omega_max 必须在 (0, 2] rad/s 内（赛题硬上限 2）");
    }
    omega_max_ = v;
  }

  // 串口：显式参数 > RM_SIM_SERIAL > /dev/gimbal > 枚举。
  const char * env = std::getenv("RM_SIM_SERIAL");
  const std::string env_port = env ? env : "";
  const PortChoice choice = selectPort(
    port_param, env_port, access("/dev/gimbal", F_OK) == 0, scanSerialCandidates());
  if (choice.path.empty()) {
    throw std::runtime_error(
            "未找到串口：port 参数为 auto，RM_SIM_SERIAL 未设置，/dev/gimbal 不存在，"
            "也没有 /dev/ttyUSB*、/dev/ttyACM*");
  }
  if (choice.source == "param" && !env_port.empty()) {
    RCLCPP_WARN(
      get_logger(), "port 参数显式设为 %s，环境变量 RM_SIM_SERIAL=%s 被忽略（port 设为 auto 才会使用它）",
      choice.path.c_str(), env_port.c_str());
  }
  std::string err;
  if (!serial_.open(choice.path, kBaudrate, err)) {
    throw SerialOpenError(
            "打开串口失败：" + err, SerialPort::isDeviceAbsentErrno(serial_.lastErrno()));
  }
  // 打开后立即建立本次会话基线，不把驱动保存的历史错误当作新错误。
  checkUartErrors();
  // 身份只用于日志：/dev/gimbal 指向错误适配器时，打开成功不会有任何异常。
  const PortIdentity identity = inspectPortIdentity(choice.path);
  const std::string identity_note = identity.by_id.empty()
    ? "（无 by-id 条目，可能是 PTY）"
    : "（by-id: " + identity.by_id + "）";
  RCLCPP_INFO(
    get_logger(), "串口 %s（来源 %s）-> %s%s，%d 8N1，已独占。imu_axis_sign=(%g, %g, %g)",
    choice.path.c_str(), choice.source.c_str(),
    identity.resolved.empty() ? choice.path.c_str() : identity.resolved.c_str(),
    identity_note.c_str(), kBaudrate, axis_sign_[0], axis_sign_[1], axis_sign_[2]);
  if (identity.rule_chip_by_ids.size() >= 2) {
    std::string listing;
    for (const auto & name : identity.rule_chip_by_ids) {listing += " " + name;}
    RCLCPP_WARN(
      get_logger(),
      "检测到 %zu 颗 CP210x/CH340 类适配器：%s。99-gimbal.rules 按芯片匹配，多颗同型号时 "
      "/dev/gimbal 指向后插的那颗；若本次打开的不是云台，请用 port:=/dev/serial/by-id/<名字> 显式指定。",
      identity.rule_chip_by_ids.size(), listing.c_str());
  }

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile());
  reset_pub_ = create_publisher<std_msgs::msg::UInt64>(
    "/task1/gimbal_reset_epoch", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::UInt64 epoch;
  epoch_base_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
  epoch.data = epoch_base_;
  reset_pub_->publish(epoch);
  if (publish_tf_) {tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);}
  RCLCPP_INFO(
    get_logger(), "IMU 将发布到 %s（QoS reliable, keep_last 200），TF world -> gimbal_link%s",
    imu_topic_.c_str(), publish_tf_ ? "" : "（publish_tf=false，不发布）");

  if (sub_cmd) {
    // 与 best_effort、reliable 两类发布者都能匹配。
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&BridgeNode::onCmdVel, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "订阅 %s：angular.z -> yaw_omega，angular.y -> pitch_omega（关节角速度，rad/s），"
      "每条消息转发一帧 0x30，不做保活。yaw_sign=%g pitch_sign=%g",
      cmd_topic.c_str(), yaw_sign_, pitch_sign_);
  }

  start_ = std::chrono::steady_clock::now();
  diag_timer_ = create_wall_timer(1s, std::bind(&BridgeNode::onDiagTimer, this));
  RCLCPP_INFO(
    get_logger(),
    "等待 IMU。固件上电后要等到温和标定完成才发第一帧 0x01（约 N 秒，见 PROTOCOL.md 5.2 节）。");

  running_ = true;
  reader_ = std::thread(&BridgeNode::readLoop, this);
}

BridgeNode::~BridgeNode()
{
  running_ = false;
  if (reader_.joinable()) {reader_.join();}
  const auto shutdown_ns = steadyNowNs();
  if (cmd_sub_ && serial_.isOpen() && n_commands_.load() > 0 &&
    shutdown_ns >= command_ready_ns_.load()) {
    // 第 3.5 节：退出时补几帧零指令。退出可能晚于最后反馈100ms，
    // 反馈过期只能阻止运动指令，不能阻止已经用过的桥补发零速。
    const auto zero = proto::encodeCmd(0.0f, 0.0f);
    for (int i = 0; i < 3; ++i) {serial_.writeAll(zero.data(), zero.size());}
  }
  serial_.close();
}

proto::AxisSign BridgeNode::readAxisSign()
{
  declare_parameter(
    "imu_axis_sign", rclcpp::ParameterValue(std::vector<int64_t>{1, 1, 1}),
    dynamicDescriptor("只作用于 accel 和 gyro，不作用于 q；本套件固定为 (1, 1, 1)"));
  const rclcpp::Parameter p = get_parameter("imu_axis_sign");
  std::vector<double> v;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    for (auto x : p.as_integer_array()) {v.push_back(static_cast<double>(x));}
  } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    v = p.as_double_array();
  } else {
    throw std::runtime_error("imu_axis_sign 必须是长度为 3 的数组，元素为 1 或 -1");
  }
  if (v.size() != 3) {throw std::runtime_error("imu_axis_sign 必须恰好有 3 个元素");}
  for (double x : v) {
    if (x != 1.0 && x != -1.0) {throw std::runtime_error("imu_axis_sign 的元素必须是 1 或 -1");}
  }
  return {{v[0], v[1], v[2]}};
}

double BridgeNode::readSign(const std::string & name)
{
  declare_parameter(
    name, rclcpp::ParameterValue(1),
    dynamicDescriptor("PC 侧发送前乘的符号（第 8 节），只对 cmd_vel_gimbal 转发生效，1 或 -1"));
  const rclcpp::Parameter p = get_parameter(name);
  double v = 0.0;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    v = static_cast<double>(p.as_int());
  } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    v = p.as_double();
  }
  if (v != 1.0 && v != -1.0) {throw std::runtime_error(name + " 必须是 1 或 -1");}
  return v;
}

void BridgeNode::readLoop()
{
  std::uint8_t buf[4096];
  auto next_uart_check = std::chrono::steady_clock::now() + 1s;
  while (running_ && rclcpp::ok()) {
    const ssize_t n = serial_.readSome(buf, sizeof buf, 100);
    if (n < 0) {
      RCLCPP_ERROR(get_logger(), "串口读取失败（设备被拔出或对端关闭），bridge 退出");
      rclcpp::shutdown();
      return;
    }
    if (n > 0) {
      last_rx_ns_ = steadyNowNs();
      n_rx_bytes_.fetch_add(static_cast<std::uint64_t>(n));
    }
    for (ssize_t i = 0; i < n; ++i) {
      parser_.feed(buf[i], [this](const std::uint8_t * f, std::size_t len) {onFrame(f, len);});
    }
    if (n > 0) {
      // 解析器只由本读线程访问；跨线程诊断只读取原子计数快照。
      const auto & stats = parser_.stats();
      n_bad_checksum_ = stats.bad_checksum;
      n_bad_app_ = stats.bad_app;
      n_headers_seen_ = stats.headers_seen;
      n_noise_bytes_ = stats.noise_bytes;
      n_bad_addr_ = stats.bad_addr;
      n_bad_func_ = stats.bad_func;
      n_bad_len_ = stats.bad_len;
    }
    const auto check_time = std::chrono::steady_clock::now();
    if (check_time >= next_uart_check) {
      checkUartErrors();
      next_uart_check = check_time + 1s;
    }
  }
}

void BridgeNode::onFrame(const std::uint8_t * frame, std::size_t /*n*/)
{
  // 第 7 节：帧通过校验之后立刻取主机接收时刻，Imu 与 TF 用同一个值，不加偏移。
  const builtin_interfaces::msg::Time stamp = now();

  proto::ImuSample s;
  if (!proto::decodeImu(frame, s)) {return;}  // 解析器已检查过，这里只是防御

  std::lock_guard<std::mutex> guard(command_gate_mutex_);
  const proto::ImuStep step = tracker_.update(s.seq, s.stamp_us);
  const auto receive_ns = steadyNowNs();
  if (step.first || step.reset) {
    command_ready_ns_ = receive_ns + 600000000;  // 首帧/复位后保留0.6秒有效指令静默
    n_commands_ = 0;
  }
  last_imu_ns_ = receive_ns;
  if (step.first) {
    RCLCPP_INFO(get_logger(), "收到第一帧 0x01：IMU 已就绪，开始发布 %s 和 TF", imu_topic_.c_str());
  } else if (step.reset) {
    // 第 6.3 节：重设 prev，重置滤波器（bridge 自身没有滤波器，下游用 header.stamp 差的节点
    // 需要自己处理），这一帧不计丢帧、不求 dt，但照常发布。
    ++n_resets_;
    std_msgs::msg::UInt64 epoch;
    epoch.data = epoch_base_ + n_resets_.load();
    reset_pub_->publish(epoch);
    RCLCPP_WARN(get_logger(), "检测到 MCU 复位（seq=%u stamp_us=%u），此帧不计丢帧", s.seq, s.stamp_us);
  } else {
    if (step.dropped > 0) {n_dropped_ += step.dropped;}
    if (step.gap) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "IMU 断流 %.1f ms（dt > 100 ms）", step.dt_us / 1000.0);
    }
  }
  ++n_imu_;

  imu_pub_->publish(makeImuMsg(s, axis_sign_, stamp));
  if (tf_br_) {tf_br_->sendTransform(makeTf(s, stamp));}
}

void BridgeNode::onCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> guard(command_gate_mutex_);
  const auto command_ns = steadyNowNs();
  if (n_imu_.load() == 0 || command_ns < command_ready_ns_.load() ||
    command_ns - last_imu_ns_.load() > 100000000) {return;}
  const double y_raw = yaw_sign_ * msg->angular.z;
  const double p_raw = pitch_sign_ * msg->angular.y;
  if (!std::isfinite(y_raw) || !std::isfinite(p_raw)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "cmd_vel_gimbal 含非有限值，已丢弃");
    return;
  }
  // 转发前限幅：该话题一旦出现第二个发布者（teleop/调试工具），未限幅值会直达 MCU。
  const double y = std::clamp(y_raw, -omega_max_, omega_max_);
  const double p = std::clamp(p_raw, -omega_max_, omega_max_);
  const auto f = proto::encodeCmd(static_cast<float>(y), static_cast<float>(p));
  if (!std::isfinite(proto::readF32(&f[4])) || !std::isfinite(proto::readF32(&f[8]))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "cmd_vel_gimbal 超出 float 范围，已丢弃");
    return;
  }
  if (!serial_.writeAll(f.data(), f.size())) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "串口写入失败");
  } else {
    ++n_commands_;
  }
}

void BridgeNode::onDiagTimer()
{
  const std::uint64_t cur[7] = {
    n_bad_checksum_.load(), n_bad_app_.load(), n_resets_.load(), n_dropped_.load(),
    n_bad_addr_.load(), n_bad_func_.load(), n_bad_len_.load()};
  bool changed = false;
  for (int i = 0; i < 7; ++i) {changed = changed || cur[i] != last_reported_[i];}
  if (changed) {
    RCLCPP_WARN(
      get_logger(), "累计：校验错 %lu，应用层丢弃（NaN/inf/q 范数）%lu，MCU 复位 %lu，丢帧 %lu，"
      "地址拒绝 %lu，功能拒绝 %lu，长度拒绝 %lu（是解析候选次数，不是完整坏帧数）",
      static_cast<unsigned long>(cur[0]), static_cast<unsigned long>(cur[1]),
      static_cast<unsigned long>(cur[2]), static_cast<unsigned long>(cur[3]),
      static_cast<unsigned long>(cur[4]), static_cast<unsigned long>(cur[5]),
      static_cast<unsigned long>(cur[6]));
    for (int i = 0; i < 7; ++i) {last_reported_[i] = cur[i];}
  }
  // 只提示，不判故障，也不退出（5.2 节：等待时间必须明显大于上电标定时间 N）。
  if (n_imu_.load() == 0 && !warned_no_imu_) {
    const double waited =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    if (waited > imu_wait_warn_s_) {
      warned_no_imu_ = true;
      const auto rx = n_rx_bytes_.load();
      if (rx == 0) {
        RCLCPP_WARN(
          get_logger(), "已等待 %.0f 秒仍没有合格 0x01；累计 0 字节（RX=0 字节）。"
          "串口已打开，但本进程尚未读到任何字节。核对 ls -l /dev/serial/by-id/、"
          "ls -l /dev/gimbal 与固件状态，不据此认定硬件故障。"
          "bridge 继续等待，不主动探测或发送。", waited);
      } else {
        const auto last_rx = last_rx_ns_.load();
        const double rx_age = static_cast<double>(steadyNowNs() - last_rx) / 1e9;
        RCLCPP_WARN(
          get_logger(), "已等待 %.0f 秒仍没有合格 0x01；累计 RX=%lu 字节，最近 RX 距今 %.3f 秒，"
          "帧头候选 %lu，等待帧头噪声字节 %lu，地址拒绝 %lu，功能拒绝 %lu，长度拒绝 %lu，"
          "校验错 %lu，应用层拒绝 %lu。已收到数据，但尚未通过预期协议；先核对端口身份及固件协议。"
          "bridge 继续等待，不主动探测或发送。",
          waited, static_cast<unsigned long>(rx), rx_age,
          static_cast<unsigned long>(n_headers_seen_.load()),
          static_cast<unsigned long>(n_noise_bytes_.load()),
          static_cast<unsigned long>(n_bad_addr_.load()),
          static_cast<unsigned long>(n_bad_func_.load()),
          static_cast<unsigned long>(n_bad_len_.load()),
          static_cast<unsigned long>(n_bad_checksum_.load()),
          static_cast<unsigned long>(n_bad_app_.load()));
      }
    }
  }
}

void BridgeNode::checkUartErrors()
{
  if (uart_counters_unsupported_) {return;}
  const auto result = serial_.readErrorCounters();
  if (result.status == UartCounterStatus::Unsupported) {
    uart_counters_unsupported_ = true;
    RCLCPP_INFO(
      get_logger(), "UART 驱动不支持错误计数查询（errno=%d），继续收发并保留协议帧校验；"
      "底层错误计数不可用，不代表没有错误。", result.error_number);
    return;
  }
  if (result.status == UartCounterStatus::Error) {
    const auto query_time = std::chrono::steady_clock::now();
    // 使用单调时钟限频，仿真 ROS 时钟暂停也不会导致重复刷屏。
    if (!uart_query_failed_ || query_time - last_uart_query_warning_ >= 5s) {
      RCLCPP_WARN(
        get_logger(), "UART 错误计数查询失败（errno=%d: %s），继续收发，下次诊断重试。",
        result.error_number, std::strerror(result.error_number));
      last_uart_query_warning_ = query_time;
    }
    uart_query_failed_ = true;
    return;
  }
  if (uart_query_failed_) {
    RCLCPP_INFO(get_logger(), "UART 错误计数查询已恢复。");
    uart_query_failed_ = false;
  }
  if (!have_uart_baseline_) {
    last_uart_counters_ = result.counters;
    have_uart_baseline_ = true;
    RCLCPP_INFO(
      get_logger(), "UART 驱动错误计数可读取，已建立会话基线；每秒检查增量。"
      "各字段是否上报取决于驱动，零计数不保证没有底层错误（8N1 不启用线路奇偶校验）。");
    return;
  }
  const auto & current = result.counters;
  const UartErrorCounters delta{
    current.frame - last_uart_counters_.frame,
    current.parity - last_uart_counters_.parity,
    current.overrun - last_uart_counters_.overrun,
    current.brk - last_uart_counters_.brk,
    current.buf_overrun - last_uart_counters_.buf_overrun};
  last_uart_counters_ = current;
  if (delta.frame || delta.parity || delta.overrun || delta.brk || delta.buf_overrun) {
    RCLCPP_WARN(
      get_logger(), "UART 错误增量：frame=%u parity=%u overrun=%u brk=%u buf_overrun=%u"
      "（驱动计数，不是完整坏协议帧数）",
      static_cast<unsigned>(delta.frame), static_cast<unsigned>(delta.parity),
      static_cast<unsigned>(delta.overrun), static_cast<unsigned>(delta.brk),
      static_cast<unsigned>(delta.buf_overrun));
  }
}

}  // namespace gimbal_serial
