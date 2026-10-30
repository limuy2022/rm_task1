// 真实 ROS 消息布局回归：时间单位、非对齐、端序、行填充和拒收边界。
#include "livox_pointcloud2.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <tuple>
#include <vector>

using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;
using Sample = std::tuple<double, float, float, float>;

Cloud cloud(bool big_endian = false) {
  Cloud msg;
  msg.header.stamp.sec = 1700000000;
  msg.height = 1;
  msg.width = 2;
  msg.point_step = 26;
  msg.row_step = 52;
  msg.is_bigendian = big_endian;
  for (const auto& [name, offset, type] : std::vector<std::tuple<std::string, unsigned, unsigned>>{
      {"x", 0, Field::FLOAT32}, {"y", 4, Field::FLOAT32}, {"z", 8, Field::FLOAT32},
      {"intensity", 12, Field::FLOAT32}, {"tag", 16, Field::UINT8},
      {"line", 17, Field::UINT8}, {"timestamp", 18, Field::FLOAT64}}) {
    Field field;
    field.name = name; field.offset = offset; field.datatype = type; field.count = 1;
    msg.fields.push_back(field);
  }
  msg.data.resize(msg.row_step);
  return msg;
}

template <class T>
void put(Cloud& msg, std::size_t offset, T value) {
  std::uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  if (msg.is_bigendian != (std::endian::native == std::endian::big))
    std::reverse(std::begin(bytes), std::end(bytes));
  std::copy(std::begin(bytes), std::end(bytes), msg.data.begin() + offset);
}

void point(Cloud& msg, std::size_t offset, double time, std::uint8_t tag = 0x10) {
  put(msg, offset, 1.25f); put(msg, offset + 4, -2.5f); put(msg, offset + 8, .75f);
  msg.data[offset + 16] = tag;
  put(msg, offset + 18, time);
}

std::vector<Sample> decode(const Cloud& msg) {
  std::vector<Sample> result;
  rm_task1::visit_livox_pointcloud2(msg, [&](double t, float x, float y, float z) {
    result.emplace_back(t, x, y, z);
  });
  return result;
}

TEST(LivoxPointCloud2, OfficialAbsoluteNanosecondsNotHeaderOffset) {
  auto msg = cloud();
  point(msg, 0, 1700000000000000000.0);
  point(msg, 26, 1700000000010000000.0, 0);
  const auto result = decode(msg);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_DOUBLE_EQ(std::get<0>(result[0]), 1700000000.0);
  EXPECT_NEAR(std::get<0>(result[1]) - std::get<0>(result[0]), .01, 1e-6);
  EXPECT_FLOAT_EQ(std::get<1>(result[0]), 1.25f);
  EXPECT_FLOAT_EQ(std::get<2>(result[0]), -2.5f);
  EXPECT_FLOAT_EQ(std::get<3>(result[0]), .75f);
}

TEST(LivoxPointCloud2, BigEndianUnalignedTimestamp) {
  auto msg = cloud(true);
  point(msg, 0, 1000000000.0); point(msg, 26, 2000000000.0);
  const auto result = decode(msg);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_DOUBLE_EQ(std::get<0>(result[1]), 2.0);
  EXPECT_FLOAT_EQ(std::get<2>(result[1]), -2.5f);
}

TEST(LivoxPointCloud2, UsesFieldOffsetsAndRowPadding) {
  auto msg = cloud();
  msg.height = 2; msg.width = 1; msg.row_step = 32;
  msg.data.resize(64);
  point(msg, 0, 1000000000.0); point(msg, 32, 2000000000.0);
  // 字段顺序与坐标存储偏移均可变化，不能依赖固定 struct 布局。
  msg.fields[0].offset = 4; msg.fields[1].offset = 0;
  std::reverse(msg.fields.begin(), msg.fields.end());
  const auto result = decode(msg);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_FLOAT_EQ(std::get<1>(result[0]), -2.5f);
  EXPECT_DOUBLE_EQ(std::get<0>(result[1]), 2.0);
}

TEST(LivoxPointCloud2, MissingOrInvalidTimeFieldRejectsFrame) {
  auto msg = cloud();
  msg.fields.pop_back();
  EXPECT_THROW(decode(msg), std::invalid_argument);
  msg = cloud(); msg.fields.back().datatype = Field::FLOAT32;
  EXPECT_THROW(decode(msg), std::invalid_argument);
  msg = cloud(); msg.fields.back().count = 2;
  EXPECT_THROW(decode(msg), std::invalid_argument);
  msg = cloud(); msg.fields.back().offset = 20;
  EXPECT_THROW(decode(msg), std::invalid_argument);
  msg = cloud(); msg.fields.push_back(msg.fields.back());
  EXPECT_THROW(decode(msg), std::invalid_argument);
}

TEST(LivoxPointCloud2, TruncatedAndOverflowingLayoutRejectsBeforeReading) {
  auto msg = cloud(); msg.data.pop_back();
  EXPECT_THROW(decode(msg), std::invalid_argument);
  msg = cloud(); msg.row_step = 25;
  EXPECT_THROW(decode(msg), std::invalid_argument);
  msg = cloud(); msg.width = std::numeric_limits<std::uint32_t>::max();
  EXPECT_THROW(decode(msg), std::invalid_argument);
  msg = cloud(); msg.height = std::numeric_limits<std::uint32_t>::max();
  EXPECT_THROW(decode(msg), std::invalid_argument);
  msg = cloud(); msg.point_step = 0;
  EXPECT_THROW(decode(msg), std::invalid_argument);
}

TEST(LivoxPointCloud2, RejectsInvalidEchoCoordinatesAndTimeWithoutFabricatingPoints) {
  auto msg = cloud();
  point(msg, 0, 1000000000., 0x20); point(msg, 26, 2000000000., 0x30);
  EXPECT_TRUE(decode(msg).empty());
  point(msg, 0, 1000000000.); put(msg, 0, std::numeric_limits<float>::quiet_NaN());
  point(msg, 26, std::numeric_limits<double>::infinity());
  EXPECT_TRUE(decode(msg).empty());
  point(msg, 0, -1.); point(msg, 26, 4294967296.0 * 1e9);
  EXPECT_TRUE(decode(msg).empty());
}

TEST(LivoxPointCloud2, EmptyOfficialMessageDoesNotEmitPoints) {
  auto msg = cloud(); msg.width = 0; msg.row_step = 0; msg.data.clear();
  EXPECT_TRUE(decode(msg).empty());
  msg.height = std::numeric_limits<std::uint32_t>::max();
  EXPECT_TRUE(decode(msg).empty());
}
