#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gimbal_serial/protocol.hpp"
#include "gimbal_serial/serial_port.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace gimbal_serial
{

// 串口打不开时抛出。device_absent 为 true 表示设备文件还不存在（可等待插上后重试）；
// 被占用（EBUSY、EWOULDBLOCK）等其他原因为 false，进程必须直接退出，不能重试。
class SerialOpenError : public std::runtime_error
{
public:
  SerialOpenError(const std::string & what, bool absent)
  : std::runtime_error(what), device_absent(absent) {}
  bool device_absent;
};

// 进程退出码：launch 的 respawn 只在 kExitDeviceAbsent 时才会重启。
constexpr int kExitFatal = 1;
constexpr int kExitDeviceAbsent = 3;

// 串口 <-> ROS 2 的桥。
//   发布：imu_topic 参数指定的话题（默认 /red_standard_robot1/gimbal，sensor_msgs/Imu）、
//         TF world -> gimbal_link
//   可选订阅：cmd_vel_topic 参数指定的话题（默认 /offboardlink/cmd_vel_gimbal，Twist），
//         每收到一条新消息转发一帧 0x30，不做保活
// 串口打开失败时构造函数抛 std::runtime_error（第 2 节：直接报错退出）。
class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    UartCounterReader counter_reader = {});
  ~BridgeNode() override;

private:
  void readLoop();
  void onFrame(const std::uint8_t * frame, std::size_t n);
  void onCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void onDiagTimer();
  void checkUartErrors();
  proto::AxisSign readAxisSign();
  double readSign(const std::string & name);

  std::mutex command_gate_mutex_;
  SerialPort serial_;
  proto::FrameParser parser_{proto::Direction::ToPc};
  proto::ImuTracker tracker_;
  proto::AxisSign axis_sign_{{1.0, 1.0, 1.0}};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr reset_pub_;
  std::atomic<std::int64_t> last_imu_ns_{0};
  std::atomic<std::int64_t> command_ready_ns_{0};
  std::atomic<std::uint64_t> n_commands_{0};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  bool publish_tf_ = true;
  std::string imu_topic_;
  double yaw_sign_ = 1.0;
  double pitch_sign_ = 1.0;
  double imu_wait_warn_s_ = 15.0;

  std::thread reader_;
  std::atomic<bool> running_{false};

  // 统计（读线程写，定时器读）
  std::atomic<std::uint64_t> n_imu_{0};
  std::atomic<std::uint64_t> n_resets_{0};
  std::uint64_t epoch_base_{0};  // per bridge session, never restart the epoch at zero
  std::atomic<std::uint64_t> n_dropped_{0};
  std::atomic<std::uint64_t> n_bad_checksum_{0};
  std::atomic<std::uint64_t> n_bad_app_{0};
  std::atomic<std::uint64_t> n_rx_bytes_{0};
  std::atomic<std::int64_t> last_rx_ns_{0};  // steady clock; 0 表示尚未读到任何字节
  std::atomic<std::uint64_t> n_headers_seen_{0};
  std::atomic<std::uint64_t> n_noise_bytes_{0};
  std::atomic<std::uint64_t> n_bad_addr_{0};
  std::atomic<std::uint64_t> n_bad_func_{0};
  std::atomic<std::uint64_t> n_bad_len_{0};
  std::uint64_t last_reported_[7] = {0, 0, 0, 0, 0, 0, 0};
  std::chrono::steady_clock::time_point start_;
  bool warned_no_imu_ = false;

  // 构造函数建立基线后只由读线程访问；析构先 join 再关闭串口。
  bool uart_counters_unsupported_ = false;
  bool have_uart_baseline_ = false;
  UartErrorCounters last_uart_counters_{};
  bool uart_query_failed_ = false;
  std::chrono::steady_clock::time_point last_uart_query_warning_{};
};

}  // namespace gimbal_serial
