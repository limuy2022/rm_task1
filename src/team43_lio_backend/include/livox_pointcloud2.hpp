#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rm_task1 {

// Livox 官方 PointXYZRTLT：timestamp 为绝对纳秒，不是相对 header 的偏移。
// 来源：Livox-SDK/livox_ros_driver2 的 src/lddc.cpp。
// 按 fields 读取而非强转 packed struct，兼容非对齐字段、行填充和端序。
inline std::uint32_t point_field_offset(const sensor_msgs::msg::PointCloud2& msg,
                                      const std::string& name, std::uint8_t type,
                                      std::uint32_t size) {
  const sensor_msgs::msg::PointField* found = nullptr;
  for (const auto& field : msg.fields) {
    if (field.name != name) continue;
    if (found) throw std::invalid_argument("PointCloud2 字段重复：" + name);
    found = &field;
  }
  if (!found || found->datatype != type || found->count != 1 ||
      found->offset > msg.point_step || size > msg.point_step - found->offset)
    throw std::invalid_argument("PointCloud2 缺少合法 Livox 字段：" + name);
  return found->offset;
}

template <class T>
T read_point_scalar(const std::uint8_t* data, bool big_endian) {
  std::array<std::uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), data, bytes.size());
  if (big_endian != (std::endian::native == std::endian::big))
    std::reverse(bytes.begin(), bytes.end());
  T value;
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

// 先验证消息结构再遍历；布局错误拒收整帧，坏点跳过，不伪造逐点时间。
template <class Emit>
void visit_livox_pointcloud2(const sensor_msgs::msg::PointCloud2& msg, Emit emit) {
  using Field = sensor_msgs::msg::PointField;
  const auto x = point_field_offset(msg, "x", Field::FLOAT32, 4);
  const auto y = point_field_offset(msg, "y", Field::FLOAT32, 4);
  const auto z = point_field_offset(msg, "z", Field::FLOAT32, 4);
  const auto tag = point_field_offset(msg, "tag", Field::UINT8, 1);
  const auto timestamp = point_field_offset(msg, "timestamp", Field::FLOAT64, 8);
  if (msg.point_step == 0 ||
      std::uint64_t(msg.width) * msg.point_step > msg.row_step ||
      std::uint64_t(msg.height) * msg.row_step > msg.data.size())
    throw std::invalid_argument("PointCloud2 行步长或数据长度无效");
  if (msg.width == 0 || msg.height == 0) return;
  for (std::uint32_t row = 0; row < msg.height; ++row) {
    for (std::uint32_t col = 0; col < msg.width; ++col) {
      const auto* point = msg.data.data() + std::size_t(row) * msg.row_step +
                          std::size_t(col) * msg.point_step;
      const auto echo = point[tag] & 0x30;
      if (echo != 0 && echo != 0x10) continue;
      const float px = read_point_scalar<float>(point + x, msg.is_bigendian);
      const float py = read_point_scalar<float>(point + y, msg.is_bigendian);
      const float pz = read_point_scalar<float>(point + z, msg.is_bigendian);
      const double ns = read_point_scalar<double>(point + timestamp, msg.is_bigendian);
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
          !std::isfinite(ns) || ns < 0 || ns >= 4294967296.0 * 1e9) continue;
      emit(ns * 1e-9, px, py, pz);
    }
  }
}

}  // namespace rm_task1
