// ImuSample -> sensor_msgs/Imu 和 TF。第 7 节的字段映射只在这里实现一份，
// 节点与单元测试共用，golden 测试因此走的就是真实发布路径。
#pragma once

#include <array>
#include <cmath>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gimbal_serial/protocol.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace gimbal_serial
{

constexpr const char * kImuFrameId = "gimbal_link";
constexpr const char * kWorldFrameId = "world";

// 归一化后的四元数，顺序 w x y z。调用前 decodeImu 已保证范数 >= 0.5。
inline std::array<double, 4> normalizedQuat(const proto::ImuSample & s)
{
  const double n = proto::quatNorm(s);
  return {{s.q[0] / n, s.q[1] / n, s.q[2] / n, s.q[3] / n}};
}

// 对角协方差，行主序 3x3，非对角线为 0。
inline std::array<double, 9> diagCov(double xx, double yy, double zz)
{
  return {{xx, 0, 0, 0, yy, 0, 0, 0, zz}};
}

inline sensor_msgs::msg::Imu makeImuMsg(
  const proto::ImuSample & s, const proto::AxisSign & sign,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::Imu m;
  m.header.stamp = stamp;
  m.header.frame_id = kImuFrameId;

  const auto q = normalizedQuat(s);
  m.orientation.w = q[0];
  m.orientation.x = q[1];
  m.orientation.y = q[2];
  m.orientation.z = q[3];

  const auto w = proto::imuAngularVelocity(s, sign);
  m.angular_velocity.x = w[0];
  m.angular_velocity.y = w[1];
  m.angular_velocity.z = w[2];

  const auto a = proto::imuLinearAcceleration(s, sign);
  m.linear_acceleration.x = a[0];
  m.linear_acceleration.y = a[1];
  m.linear_acceleration.z = a[2];

  m.orientation_covariance = diagCov(1.0e-4, 1.0e-4, 1.0e-2);
  m.angular_velocity_covariance = diagCov(1.0e-5, 1.0e-5, 1.0e-5);
  m.linear_acceleration_covariance = diagCov(1.0e-2, 1.0e-2, 1.0e-2);
  return m;
}

// world -> gimbal_link，rotation = q，translation = 0。
inline geometry_msgs::msg::TransformStamped makeTf(
  const proto::ImuSample & s, const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = kWorldFrameId;
  t.child_frame_id = kImuFrameId;
  const auto q = normalizedQuat(s);
  t.transform.rotation.w = q[0];
  t.transform.rotation.x = q[1];
  t.transform.rotation.y = q[2];
  t.transform.rotation.z = q[3];
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  return t;
}

}  // namespace gimbal_serial
