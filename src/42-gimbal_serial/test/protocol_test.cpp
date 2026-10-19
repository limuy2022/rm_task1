// 读取 protocol/golden_frames.txt，走 bridge 真实使用的解码与发布路径：
//   FrameParser（第 3.4 节状态机加应用层解码）-> decodeImu -> makeImuMsg / makeTf。
// 覆盖全部 accept、reject 用例，物理分组检查，流式测试，以及复位检测。
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "gimbal_serial/imu_convert.hpp"
#include "gimbal_serial/protocol.hpp"
#include "gimbal_serial/serial_port.hpp"

using namespace gimbal_serial;  // NOLINT

namespace
{

struct GoldenCase
{
  std::string name, dir, func, expect, reason, group, note;
  std::vector<std::uint8_t> hex;
  std::map<std::string, std::string> decoded;
  std::map<std::string, double> state;
  bool has_state = false;
};

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {return "";}
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::map<std::string, std::string> kvPairs(const std::string & s)
{
  std::map<std::string, std::string> m;
  std::istringstream is(s);
  std::string tok;
  while (is >> tok) {
    const auto eq = tok.find('=');
    if (eq != std::string::npos) {m[tok.substr(0, eq)] = tok.substr(eq + 1);}
  }
  return m;
}

std::vector<GoldenCase> loadGolden()
{
  const char * env = std::getenv("GOLDEN_FRAMES");
  const std::string path = env ? env : GOLDEN_FRAMES_PATH;
  std::vector<GoldenCase> cases;
  // 找不到黄金向量时返回空表，依赖它的用例由 SKIP_IF_NO_GOLDEN 跳过（CMake 只给 WARNING，不让 build 失败）。
  if (path.empty()) {return cases;}
  std::ifstream f(path);
  if (!f.good()) {return cases;}
  GoldenCase cur;
  bool open = false;
  std::string line;
  while (std::getline(f, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') {continue;}
    if (line == "end") {
      cases.push_back(cur);
      cur = GoldenCase();
      open = false;
      continue;
    }
    const auto colon = line.find(':');
    const std::string key = line.substr(0, colon);
    const std::string val = colon == std::string::npos ? "" : trim(line.substr(colon + 1));
    open = true;
    if (key == "case") {cur.name = val;}
    else if (key == "dir") {cur.dir = val;}
    else if (key == "func") {cur.func = val;}
    else if (key == "expect") {cur.expect = val;}
    else if (key == "reason") {cur.reason = val;}
    else if (key == "group") {cur.group = val;}
    else if (key == "note") {cur.note = val;}
    else if (key == "hex") {
      std::istringstream is(val);
      std::string tok;
      while (is >> tok) {
        cur.hex.push_back(static_cast<std::uint8_t>(std::strtol(tok.c_str(), nullptr, 16)));
      }
    } else if (key == "decoded") {
      if (val != "-") {cur.decoded = kvPairs(val);}
    } else if (key == "state") {
      if (val != "-") {
        for (const auto & kv : kvPairs(val)) {cur.state[kv.first] = std::strtod(kv.second.c_str(), nullptr);}
        cur.has_state = true;
      }
    }
  }
  EXPECT_FALSE(open) << "last block has no 'end'";
  return cases;
}

const std::vector<GoldenCase> & golden()
{
  static const std::vector<GoldenCase> g = loadGolden();
  return g;
}

// 黄金向量缺失时跳过，并给出醒目提示（不是通过）。
#define SKIP_IF_NO_GOLDEN() \
  do { \
    if (golden().empty()) { \
      GTEST_SKIP() << "!!! golden_frames.txt NOT FOUND: golden-vector test SKIPPED, NOT verified. " \
        "Set GOLDEN_FRAMES_DIR=<suite>/protocol and re-run cmake, or GOLDEN_FRAMES=<file> at run time. !!!"; \
    } \
  } while (0)

proto::Direction dirOf(const GoldenCase & c)
{
  return c.dir == "mcu2pc" ? proto::Direction::ToPc : proto::Direction::ToMcu;
}

using Frames = std::vector<std::vector<std::uint8_t>>;

Frames feedAll(proto::FrameParser & p, const std::vector<std::uint8_t> & bytes)
{
  Frames out;
  for (auto b : bytes) {
    p.feed(b, [&out](const std::uint8_t * f, std::size_t n) {out.emplace_back(f, f + n);});
  }
  return out;
}

double num(const GoldenCase & c, const std::string & key)
{
  const auto it = c.decoded.find(key);
  EXPECT_TRUE(it != c.decoded.end()) << c.name << " missing " << key;
  return it == c.decoded.end() ? 0.0 : std::strtod(it->second.c_str(), nullptr);
}

builtin_interfaces::msg::Time someStamp()
{
  builtin_interfaces::msg::Time t;
  t.sec = 1234;
  t.nanosec = 567;
  return t;
}

proto::ImuSample decodeOrDie(const GoldenCase & c)
{
  proto::ImuSample s;
  EXPECT_TRUE(proto::decodeImu(c.hex.data(), s)) << c.name;
  return s;
}

const GoldenCase & byName(const std::string & name)
{
  for (const auto & c : golden()) {
    if (c.name == name) {return c;}
  }
  ADD_FAILURE() << "no golden case " << name;
  static GoldenCase none;
  return none;
}

// ZYX 分解（无 roll）：第 6.2 节。
double yawOf(const std::array<double, 4> & q)
{
  return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}
double pitchOf(const std::array<double, 4> & q)
{
  return std::asin(2 * (q[0] * q[2] - q[3] * q[1]));
}

}  // namespace

TEST(Golden, FileLoaded)
{
  SKIP_IF_NO_GOLDEN();
  EXPECT_EQ(golden().size(), 44u);
  int accept = 0, reject = 0;
  for (const auto & c : golden()) {(c.expect == "accept" ? accept : reject)++;}
  EXPECT_EQ(accept, 8 + 13);
  EXPECT_GE(reject, 20);
}

// 第 9.2 节：逐帧。accept 必须解出与 decoded 一致的数值，reject 必须没有输出。
TEST(Golden, PerFrame)
{
  SKIP_IF_NO_GOLDEN();
  for (const auto & c : golden()) {
    SCOPED_TRACE(c.name);
    proto::FrameParser p(dirOf(c));
    const Frames out = feedAll(p, c.hex);
    if (c.expect == "reject") {
      EXPECT_TRUE(out.empty());
      EXPECT_EQ(p.stats().frames_ok, 0u);
      if (c.reason == "bad_checksum") {EXPECT_EQ(p.stats().bad_checksum, 1u);}
      if (c.reason == "bad_quat" || c.reason == "non_finite") {
        EXPECT_EQ(p.stats().bad_checksum, 0u);
        EXPECT_EQ(p.stats().bad_app, 1u);  // 校验正确，被应用层丢弃
      }
      continue;
    }
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0], c.hex);

    if (c.dir == "pc2mcu") {
      float y = 0, pi = 0;
      ASSERT_TRUE(proto::decodeCmd(out[0].data(), y, pi));
      EXPECT_NEAR(y, num(c, "yaw_omega"), 1e-6);
      EXPECT_NEAR(pi, num(c, "pitch_omega"), 1e-6);
      // 发送侧：encodeCmd 与黄金帧逐字节相同
      const auto enc = proto::encodeCmd(y, pi);
      EXPECT_EQ(std::vector<std::uint8_t>(enc.begin(), enc.end()), c.hex);
      continue;
    }

    proto::ImuSample s;
    ASSERT_TRUE(proto::decodeImu(out[0].data(), s));
    EXPECT_EQ(s.seq, static_cast<std::uint16_t>(num(c, "seq")));
    EXPECT_EQ(s.stamp_us, static_cast<std::uint32_t>(std::strtoull(c.decoded.at("stamp_us").c_str(), nullptr, 10)));
    const char * araw[3] = {"ax_raw", "ay_raw", "az_raw"};
    const char * graw[3] = {"gx_raw", "gy_raw", "gz_raw"};
    const char * qraw[4] = {"qw_raw", "qx_raw", "qy_raw", "qz_raw"};
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(s.accel_raw[i], static_cast<int>(num(c, araw[i])));
      EXPECT_EQ(s.gyro_raw[i], static_cast<int>(num(c, graw[i])));
    }
    for (int i = 0; i < 4; ++i) {EXPECT_EQ(s.q_raw[i], static_cast<int>(num(c, qraw[i])));}

    // 真实发布路径：makeImuMsg 里的 accel、gyro、orientation 解码，不取负，不绕过。
    const auto m = makeImuMsg(s, {{1.0, 1.0, 1.0}}, someStamp());
    EXPECT_NEAR(m.linear_acceleration.x, num(c, "ax"), 1e-6);
    EXPECT_NEAR(m.linear_acceleration.y, num(c, "ay"), 1e-6);
    EXPECT_NEAR(m.linear_acceleration.z, num(c, "az"), 1e-6);
    EXPECT_NEAR(m.angular_velocity.x, num(c, "gx"), 1e-6);
    EXPECT_NEAR(m.angular_velocity.y, num(c, "gy"), 1e-6);
    EXPECT_NEAR(m.angular_velocity.z, num(c, "gz"), 1e-6);
    // orientation 是归一化后的 q；decoded 里的 qw..qz 是 raw/32760，两者只差同一个范数
    const double qw = num(c, "qw"), qx = num(c, "qx"), qy = num(c, "qy"), qz = num(c, "qz");
    const double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    EXPECT_NEAR(m.orientation.w * n, qw, 1e-6);
    EXPECT_NEAR(m.orientation.x * n, qx, 1e-6);
    EXPECT_NEAR(m.orientation.y * n, qy, 1e-6);
    EXPECT_NEAR(m.orientation.z * n, qz, 1e-6);
    const double on = std::sqrt(
      m.orientation.w * m.orientation.w + m.orientation.x * m.orientation.x +
      m.orientation.y * m.orientation.y + m.orientation.z * m.orientation.z);
    EXPECT_NEAR(on, 1.0, 1e-12);
  }
}

// 第 9.4 节：垃圾字节加同一 dir 全部块的拼接，输出恰好是 accept 的帧，顺序相同。
TEST(Golden, Stream)
{
  SKIP_IF_NO_GOLDEN();
  for (const std::string dir : {"pc2mcu", "mcu2pc"}) {
    SCOPED_TRACE(dir);
    std::vector<std::uint8_t> stream = {0x55, 0x00, 0xAA, 0x12, 0x99};
    Frames expect;
    for (const auto & c : golden()) {
      if (c.dir != dir) {continue;}
      stream.insert(stream.end(), c.hex.begin(), c.hex.end());
      if (c.expect == "accept") {expect.push_back(c.hex);}
    }
    proto::FrameParser p(dir == "pc2mcu" ? proto::Direction::ToMcu : proto::Direction::ToPc);
    const Frames out = feedAll(p, stream);
    EXPECT_EQ(out.size(), dir == "pc2mcu" ? 8u : 13u);
    EXPECT_EQ(out, expect);
    EXPECT_EQ(p.stats().frames_ok, expect.size());
  }
}

// 第 3.3 节：接收缓冲有界，乱码不越界；假帧头之后仍能恢复同步。
TEST(Parser, ResyncAndBounded)
{
  SKIP_IF_NO_GOLDEN();
  const GoldenCase & c = byName("imu_static_level");
  proto::FrameParser p(proto::Direction::ToPc);
  std::vector<std::uint8_t> bytes;
  for (int i = 0; i < 100000; ++i) {bytes.push_back(i % 3 == 0 ? 0xAA : static_cast<std::uint8_t>(i * 7));}
  bytes.insert(bytes.end(), {0xAA, 0xAA, 0xFF, 0xAA, 0xFF, 0x01, 0x1A});  // 假帧头，进入 BODY
  // 假帧体会按整帧丢弃（校验失败），不回扫。喂满 60 个 0x00，保证状态机回到 HEAD。
  bytes.insert(bytes.end(), 60, 0x00);
  bytes.insert(bytes.end(), {0xAA, 0xAA, 0xAA});  // 重复帧头，之后的真帧仍要能收到
  bytes.insert(bytes.end(), c.hex.begin(), c.hex.end());
  const Frames out = feedAll(p, bytes);
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out.back(), c.hex);
}

TEST(Parser, ChunkedFeed)
{
  SKIP_IF_NO_GOLDEN();
  // 每次一个字节和一次全喂结果相同（串口按任意块到达）
  std::vector<std::uint8_t> stream;
  for (const auto & c : golden()) {
    if (c.dir == "mcu2pc") {stream.insert(stream.end(), c.hex.begin(), c.hex.end());}
  }
  proto::FrameParser a(proto::Direction::ToPc), b(proto::Direction::ToPc);
  const Frames whole = feedAll(a, stream);
  Frames pieces;
  for (std::size_t i = 0; i < stream.size(); i += 5) {
    std::vector<std::uint8_t> chunk(
      stream.begin() + i, stream.begin() + std::min(stream.size(), i + 5));
    const Frames part = feedAll(b, chunk);
    pieces.insert(pieces.end(), part.begin(), part.end());
  }
  EXPECT_EQ(whole, pieces);
}

// 第 9.3 节 static：accel 约等于 R(q)^T (0,0,+g)，容差 0.1 m/s^2；水平时 q 单位。
TEST(GoldenPhysical, StaticAccelMatchesGravity)
{
  SKIP_IF_NO_GOLDEN();
  for (const auto & c : golden()) {
    if (c.group != "static") {continue;}
    SCOPED_TRACE(c.name);
    const proto::ImuSample s = decodeOrDie(c);
    const auto q = normalizedQuat(s);
    const double g = 9.80665;
    // R^T * (0,0,g) 的第三列：R(q) 的第 3 行
    const double ex = g * 2 * (q[1] * q[3] - q[0] * q[2]);
    const double ey = g * 2 * (q[2] * q[3] + q[0] * q[1]);
    const double ez = g * (1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    const auto a = proto::imuLinearAcceleration(s);
    EXPECT_NEAR(a[0], ex, 0.1);
    EXPECT_NEAR(a[1], ey, 0.1);
    EXPECT_NEAR(a[2], ez, 0.1);
  }
  const auto level = decodeOrDie(byName("imu_static_level"));
  EXPECT_NEAR(proto::quatNorm(level), 1.0, 1e-4);
  const auto tilt = proto::imuLinearAcceleration(decodeOrDie(byName("imu_static_tilt_pitch")));
  EXPECT_LT(tilt[0], -1.0);  // 低头（theta > 0）时 ax 为负：比力，不取负
  EXPECT_GT(tilt[2], 9.0);
}

TEST(GoldenPhysical, YawAndPitchPositive)
{
  SKIP_IF_NO_GOLDEN();
  {
    const auto a = decodeOrDie(byName("phys_yaw_pos_a"));
    const auto b = decodeOrDie(byName("phys_yaw_pos_b"));
    const auto ga = proto::imuAngularVelocity(a), gb = proto::imuAngularVelocity(b);
    EXPECT_GT(ga[2], 0.0);
    EXPECT_GT(gb[2], 0.0);  // 绕 +z 正转：gz > 0，gyro 不取负
    const auto qa = normalizedQuat(a), qb = normalizedQuat(b);
    const double dpsi = yawOf(qb) - yawOf(qa);
    EXPECT_GT(dpsi, 0.0);  // q 的 yaw 增大
    const double dt = ((b.stamp_us - a.stamp_us) & 0xFFFFFFFFu) * 1e-6;
    EXPECT_NEAR(dt, 0.002, 1e-9);
    const double theta = pitchOf(qa);
    EXPECT_NEAR(dpsi, ga[2] / std::cos(theta) * dt, 0.1 * std::abs(dpsi));
  }
  {
    const auto a = decodeOrDie(byName("phys_pitch_pos_a"));
    const auto b = decodeOrDie(byName("phys_pitch_pos_b"));
    const auto ga = proto::imuAngularVelocity(a), gb = proto::imuAngularVelocity(b);
    EXPECT_GT(ga[1], 0.0);
    EXPECT_GT(gb[1], 0.0);
    const double dtheta = pitchOf(normalizedQuat(b)) - pitchOf(normalizedQuat(a));
    EXPECT_GT(dtheta, 0.0);
    const double dt = ((b.stamp_us - a.stamp_us) & 0xFFFFFFFFu) * 1e-6;
    EXPECT_NEAR(dtheta, ga[1] * dt, 0.1 * std::abs(dtheta));
  }
}

// 第 6.3 节 wrap_pair、reset_pair。
TEST(ImuTracker, WrapPairIsNotReset)
{
  SKIP_IF_NO_GOLDEN();
  const auto a = decodeOrDie(byName("imu_wrap_a"));
  const auto b = decodeOrDie(byName("imu_wrap_b"));
  proto::ImuTracker t;
  EXPECT_TRUE(t.update(a.seq, a.stamp_us).first);
  const auto s = t.update(b.seq, b.stamp_us);
  EXPECT_FALSE(s.reset);
  EXPECT_TRUE(s.has_dt);
  EXPECT_EQ(s.dt_us, 2000u);  // 无符号减法
  EXPECT_EQ(s.dropped, 0u);   // dseq = 1
  EXPECT_FALSE(s.gap);
}

TEST(ImuTracker, ResetPair)
{
  SKIP_IF_NO_GOLDEN();
  const auto a = decodeOrDie(byName("imu_reset_a"));
  const auto b = decodeOrDie(byName("imu_reset_b"));
  proto::ImuTracker t;
  t.update(a.seq, a.stamp_us);
  const auto s = t.update(b.seq, b.stamp_us);
  EXPECT_TRUE(s.reset);
  EXPECT_FALSE(s.has_dt);  // 不参与求 dt
  EXPECT_EQ(s.dropped, 0u);  // 不计丢帧
  EXPECT_EQ(t.resets(), 1u);
  EXPECT_EQ(t.dropped(), 0u);
  // 用这一帧重设 prev：下一帧的 dt 从复位帧算起
  const auto n = t.update(1, 2002000u);
  EXPECT_FALSE(n.reset);
  EXPECT_TRUE(n.has_dt);
  EXPECT_EQ(n.dt_us, 2000u);
}

TEST(ImuTracker, DropsGapsAndResetConditions)
{
  SKIP_IF_NO_GOLDEN();
  proto::ImuTracker t;
  t.update(10, 1000000);
  auto s = t.update(14, 1008000);  // 丢 3 帧
  EXPECT_FALSE(s.reset);
  EXPECT_EQ(s.dropped, 3u);
  EXPECT_EQ(t.dropped(), 3u);
  s = t.update(15, 1008000);  // dt = 0 不特殊处理
  EXPECT_TRUE(s.has_dt);
  EXPECT_EQ(s.dt_us, 0u);
  s = t.update(16, 1008000u + 150000u);  // dt > 100 ms
  EXPECT_TRUE(s.gap);
  EXPECT_FALSE(s.reset);
  // seq 回退 99 不算复位，回退 100 算
  proto::ImuTracker t2;
  t2.update(1000, 5000000);
  s = t2.update(901, 5002000);
  EXPECT_FALSE(s.reset);
  EXPECT_EQ(s.dropped, 0u);
  s = t2.update(801, 5004000);
  EXPECT_TRUE(s.reset);
  // stamp 倒退（dt >= 0x80000000）算复位，即使 seq 正常递增
  proto::ImuTracker t3;
  t3.update(50, 9000000);
  s = t3.update(51, 8999000);
  EXPECT_TRUE(s.reset);
  EXPECT_FALSE(s.has_dt);
  // seq 回绕（65535 -> 2）是丢 2 帧，不是复位
  proto::ImuTracker t4;
  t4.update(65535, 100);
  s = t4.update(2, 3100);
  EXPECT_FALSE(s.reset);
  EXPECT_EQ(s.dropped, 2u);
}

// 第 7 节字段映射。
TEST(ImuMsg, ContractFields)
{
  SKIP_IF_NO_GOLDEN();
  const auto s = decodeOrDie(byName("imu_negative_mix"));
  const auto stamp = someStamp();
  const auto m = makeImuMsg(s, {{1.0, 1.0, 1.0}}, stamp);
  EXPECT_EQ(m.header.frame_id, "gimbal_link");
  EXPECT_EQ(m.header.stamp.sec, stamp.sec);
  EXPECT_EQ(m.header.stamp.nanosec, stamp.nanosec);
  const std::array<double, 9> oc = {1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-2};
  const std::array<double, 9> gc = {1e-5, 0, 0, 0, 1e-5, 0, 0, 0, 1e-5};
  const std::array<double, 9> ac = {1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2};
  EXPECT_EQ(m.orientation_covariance, oc);
  EXPECT_EQ(m.angular_velocity_covariance, gc);
  EXPECT_EQ(m.linear_acceleration_covariance, ac);
  // accel、gyro 三轴都不取负
  EXPECT_NEAR(m.linear_acceleration.x, -1000 * proto::kAccelScale, 1e-12);
  EXPECT_NEAR(m.linear_acceleration.y, 2000 * proto::kAccelScale, 1e-12);
  EXPECT_NEAR(m.linear_acceleration.z, -3000 * proto::kAccelScale, 1e-12);
  EXPECT_NEAR(m.angular_velocity.x, -100 * proto::kGyroScale, 1e-12);
  EXPECT_NEAR(m.angular_velocity.y, 200 * proto::kGyroScale, 1e-12);
  EXPECT_NEAR(m.angular_velocity.z, -300 * proto::kGyroScale, 1e-12);

  // TF：父 world，子 gimbal_link，平移 0，旋转与 Imu.orientation 相同，时间戳相同
  const auto t = makeTf(s, stamp);
  EXPECT_EQ(t.header.frame_id, "world");
  EXPECT_EQ(t.child_frame_id, "gimbal_link");
  EXPECT_EQ(t.header.stamp.sec, stamp.sec);
  EXPECT_EQ(t.transform.translation.x, 0.0);
  EXPECT_EQ(t.transform.translation.y, 0.0);
  EXPECT_EQ(t.transform.translation.z, 0.0);
  EXPECT_EQ(t.transform.rotation.w, m.orientation.w);
  EXPECT_EQ(t.transform.rotation.x, m.orientation.x);
  EXPECT_EQ(t.transform.rotation.y, m.orientation.y);
  EXPECT_EQ(t.transform.rotation.z, m.orientation.z);
}

// imu_axis_sign 只作用于 accel 和 gyro，不作用于 q。
TEST(ImuMsg, AxisSignAffectsOnlyAccelAndGyro)
{
  SKIP_IF_NO_GOLDEN();
  const auto s = decodeOrDie(byName("imu_negative_mix"));
  const auto base = makeImuMsg(s, {{1.0, 1.0, 1.0}}, someStamp());
  const auto flip = makeImuMsg(s, {{-1.0, 1.0, -1.0}}, someStamp());
  EXPECT_EQ(flip.linear_acceleration.x, -base.linear_acceleration.x);
  EXPECT_EQ(flip.linear_acceleration.y, base.linear_acceleration.y);
  EXPECT_EQ(flip.linear_acceleration.z, -base.linear_acceleration.z);
  EXPECT_EQ(flip.angular_velocity.x, -base.angular_velocity.x);
  EXPECT_EQ(flip.angular_velocity.y, base.angular_velocity.y);
  EXPECT_EQ(flip.angular_velocity.z, -base.angular_velocity.z);
  EXPECT_EQ(flip.orientation.w, base.orientation.w);
  EXPECT_EQ(flip.orientation.x, base.orientation.x);
  EXPECT_EQ(flip.orientation.y, base.orientation.y);
  EXPECT_EQ(flip.orientation.z, base.orientation.z);
}

// 第 6.1 节：q 用浮点除法；q 范数 < 0.5 丢弃；极值不溢出。
TEST(ImuDecode, QuaternionRules)
{
  SKIP_IF_NO_GOLDEN();
  const GoldenCase & c = byName("imu_extreme_raw");
  const auto s = decodeOrDie(c);
  EXPECT_EQ(s.seq, 32769u);
  EXPECT_EQ(s.stamp_us, 2147483649u);
  EXPECT_EQ(s.accel_raw[0], -32768);
  EXPECT_EQ(s.accel_raw[1], 32767);
  EXPECT_DOUBLE_EQ(s.q[0], -1.0);
  // 半幅 q（不是 {-1,0,1}）：整数除法 bug 会把它变成 0
  const auto half = decodeOrDie(byName("imu_static_tilt_pitch"));
  EXPECT_NEAR(half.q[0], 32596 / 32760.0, 1e-12);
  EXPECT_GT(half.q[2], 0.05);
  EXPECT_LT(half.q[2], 0.2);
  // q 全零：帧层收下，应用层丢弃
  proto::ImuSample dummy;
  EXPECT_FALSE(proto::decodeImu(byName("rej_imu_q_zero").hex.data(), dummy));
}

// 第 8 节 cmd_vel_gimbal 转发：解码后的关节角速度与 Twist 的对应关系由节点实现，
// 这里只验证 encodeCmd 与限幅之前的 float 语义。
TEST(Cmd, EncodeMatchesSpecExample)
{
  const auto f = proto::encodeCmd(3.0f, -0.125f);
  const std::vector<std::uint8_t> want = {0xAA, 0xEE, 0x30, 0x08, 0x00, 0x00, 0x40,
    0x40, 0x00, 0x00, 0x00, 0xBE, 0x0E, 0xD8};
  EXPECT_EQ(std::vector<std::uint8_t>(f.begin(), f.end()), want);
  // 手算示例（第 3.2 节）：全零指令 D0 5A
  const auto z = proto::encodeCmd(0.0f, 0.0f);
  EXPECT_EQ(z[12], 0xD0);
  EXPECT_EQ(z[13], 0x5A);
}

TEST(PortSelect, Priority)
{
  const std::vector<std::string> scan = {"/dev/ttyUSB0", "/dev/ttyACM0"};
  // 显式 port 优先，RM_SIM_SERIAL 被忽略
  auto c = selectPort("/dev/ttyS3", "/tmp/ttyVGIMBAL", true, scan);
  EXPECT_EQ(c.path, "/dev/ttyS3");
  EXPECT_EQ(c.source, "param");
  // auto 或空：RM_SIM_SERIAL 优先于 /dev/gimbal，即使文件还不存在
  for (const char * p : {"auto", ""}) {
    c = selectPort(p, "/tmp/ttyVGIMBAL", true, scan);
    EXPECT_EQ(c.path, "/tmp/ttyVGIMBAL");
    EXPECT_EQ(c.source, "RM_SIM_SERIAL");
  }
  c = selectPort("auto", "", true, scan);
  EXPECT_EQ(c.path, "/dev/gimbal");
  c = selectPort("auto", "", false, scan);
  EXPECT_EQ(c.path, "/dev/ttyUSB0");
  c = selectPort("auto", "", false, {});
  EXPECT_TRUE(c.path.empty());
}
