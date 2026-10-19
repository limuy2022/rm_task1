// OffboardLink 帧层与应用层解码，依据 protocol/PROTOCOL.md v1.4。
// 纯 C++17 头文件，不依赖 ROS，单元测试和节点共用同一份代码。
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace gimbal_serial
{
namespace proto
{

constexpr std::uint8_t kHead = 0xAA;
constexpr std::uint8_t kAddrPcToMcu = 0xEE;  // 上行：PC -> MCU
constexpr std::uint8_t kAddrMcuToPc = 0xFF;  // 下行：MCU -> PC
constexpr std::uint8_t kFuncImu = 0x01;
constexpr std::uint8_t kFuncCmd = 0x30;
constexpr std::uint8_t kImuLen = 26;
constexpr std::uint8_t kCmdLen = 8;
constexpr std::size_t kHeadLen = 4;    // AA addr func len
constexpr std::size_t kVerifyLen = 2;  // sum sum_sum
constexpr std::size_t kFrameMax = 60;  // OFFBOARDLINK_FRAME_MAX_LEN
constexpr std::size_t kImuFrameLen = kImuLen + kHeadLen + kVerifyLen;  // 32
constexpr std::size_t kCmdFrameLen = kCmdLen + kHeadLen + kVerifyLen;  // 14

// 第 6.1 节换算系数，取完整数值。
constexpr double kAccelScale = 0.00718260498046875;
constexpr double kGyroScale = 0.00106526443603169529841533860381;
constexpr double kQuatScale = 32760.0;
constexpr double kQuatNormMin = 0.5;

// 第 6.3 节：dt 大于 100 ms 视为断流。
constexpr std::uint32_t kGapUs = 100000;

// ---------------------------------------------------------------- 校验

inline void calcVerify(
  const std::uint8_t * buf, std::size_t len, std::uint8_t & sum, std::uint8_t & sum_sum)
{
  sum = 0;
  sum_sum = 0;
  for (std::size_t i = 0; i < len + kHeadLen; ++i) {
    sum = static_cast<std::uint8_t>(sum + buf[i]);
    sum_sum = static_cast<std::uint8_t>(sum_sum + sum);
  }
}

inline bool frameOk(const std::uint8_t * buf, std::size_t len)
{
  std::uint8_t s, ss;
  calcVerify(buf, len, s, ss);
  return s == buf[len + kHeadLen] && ss == buf[len + kHeadLen + 1];
}

// ---------------------------------------------------------------- 小端读写

inline std::uint16_t readU16(const std::uint8_t * p)
{
  return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}

// 先按 u16 组装，再转成 i16（第 6.1 节）。
inline std::int16_t readI16(const std::uint8_t * p)
{
  return static_cast<std::int16_t>(readU16(p));
}

// 用无符号类型拼接，最高位置位时不会有符号溢出。
inline std::uint32_t readU32(const std::uint8_t * p)
{
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

inline float readF32(const std::uint8_t * p)
{
  const std::uint32_t u = readU32(p);
  float f;
  std::memcpy(&f, &u, sizeof f);
  return f;
}

inline void writeF32(std::uint8_t * p, float f)
{
  std::uint32_t u;
  std::memcpy(&u, &f, sizeof u);
  p[0] = static_cast<std::uint8_t>(u);
  p[1] = static_cast<std::uint8_t>(u >> 8);
  p[2] = static_cast<std::uint8_t>(u >> 16);
  p[3] = static_cast<std::uint8_t>(u >> 24);
}

// ---------------------------------------------------------------- 0x30 指令

// 整帧 14 字节。yaw_omega、pitch_omega 是关节角速度（rad/s）。
inline std::array<std::uint8_t, kCmdFrameLen> encodeCmd(float yaw_omega, float pitch_omega)
{
  std::array<std::uint8_t, kCmdFrameLen> f{};
  f[0] = kHead;
  f[1] = kAddrPcToMcu;
  f[2] = kFuncCmd;
  f[3] = kCmdLen;
  writeF32(&f[4], yaw_omega);
  writeF32(&f[8], pitch_omega);
  calcVerify(f.data(), kCmdLen, f[12], f[13]);
  return f;
}

// 应用层解码：两个 float 都必须是有限值。frame 为整帧。
inline bool decodeCmd(const std::uint8_t * frame, float & yaw_omega, float & pitch_omega)
{
  const float y = readF32(frame + kHeadLen);
  const float p = readF32(frame + kHeadLen + 4);
  if (!std::isfinite(y) || !std::isfinite(p)) {return false;}
  yaw_omega = y;
  pitch_omega = p;
  return true;
}

// ---------------------------------------------------------------- 0x01 IMU

using AxisSign = std::array<double, 3>;

struct ImuSample
{
  std::uint16_t seq = 0;
  std::uint32_t stamp_us = 0;
  std::array<std::int16_t, 3> accel_raw{};
  std::array<std::int16_t, 3> gyro_raw{};
  std::array<std::int16_t, 4> q_raw{};  // w x y z
  std::array<double, 4> q{};            // raw / 32760，尚未归一化
};

// accel：比力，raw × ACCEL_SCALE，三轴都不取负。sign 只来自 imu_axis_sign 参数。
inline std::array<double, 3> imuLinearAcceleration(
  const ImuSample & s, const AxisSign & sign = {{1.0, 1.0, 1.0}})
{
  return {{sign[0] * s.accel_raw[0] * kAccelScale,
    sign[1] * s.accel_raw[1] * kAccelScale,
    sign[2] * s.accel_raw[2] * kAccelScale}};
}

// gyro：去过零偏的机体角速度，raw × GYRO_SCALE，三轴都不取负。
inline std::array<double, 3> imuAngularVelocity(
  const ImuSample & s, const AxisSign & sign = {{1.0, 1.0, 1.0}})
{
  return {{sign[0] * s.gyro_raw[0] * kGyroScale,
    sign[1] * s.gyro_raw[1] * kGyroScale,
    sign[2] * s.gyro_raw[2] * kGyroScale}};
}

inline double quatNorm(const ImuSample & s)
{
  return std::sqrt(s.q[0] * s.q[0] + s.q[1] * s.q[1] + s.q[2] * s.q[2] + s.q[3] * s.q[3]);
}

// 应用层解码：frame 为整帧（len 已由帧层保证为 26）。
// 四元数范数 < 0.5 视为损坏，返回 false（第 6.1 节）。
inline bool decodeImu(const std::uint8_t * frame, ImuSample & out)
{
  const std::uint8_t * d = frame + kHeadLen;
  ImuSample s;
  s.seq = readU16(d + 0);
  s.stamp_us = readU32(d + 2);
  for (std::size_t i = 0; i < 3; ++i) {
    s.accel_raw[i] = readI16(d + 6 + 2 * i);
    s.gyro_raw[i] = readI16(d + 12 + 2 * i);
  }
  for (std::size_t i = 0; i < 4; ++i) {
    s.q_raw[i] = readI16(d + 18 + 2 * i);
    s.q[i] = s.q_raw[i] / kQuatScale;  // 浮点除法
  }
  if (!(quatNorm(s) >= kQuatNormMin)) {return false;}
  out = s;
  return true;
}

// ---------------------------------------------------------------- 接收状态机（第 3.4 节）

enum class Direction
{
  ToPc,   // 只收 addr 0xFF、func 0x01、len 26
  ToMcu   // 只收 addr 0xEE、func 0x30、len 8（测试和虚拟云台用）
};

struct ParserStats
{
  std::uint64_t frames_ok = 0;
  std::uint64_t bad_checksum = 0;
  std::uint64_t bad_app = 0;  // 校验正确，但应用层解码失败（非有限值、q 范数）
  // 以下是字节流诊断计数，不改变原有解析/重同步行为，也不表示已收到完整帧。
  std::uint64_t bytes_received = 0;  // 每次 feed 恰好计入一个字节
  std::uint64_t headers_seen = 0;  // 在寻找帧头或重同步时接受的 0xAA 候选
  std::uint64_t noise_bytes = 0;  // 等待帧头时遇到的非 0xAA 字节（不是完整坏帧数）
  std::uint64_t bad_addr = 0;
  std::uint64_t bad_func = 0;
  std::uint64_t bad_len = 0;
};

class FrameParser
{
public:
  explicit FrameParser(Direction dir)
  : target_addr_(dir == Direction::ToPc ? kAddrMcuToPc : kAddrPcToMcu),
    known_func_(dir == Direction::ToPc ? kFuncImu : kFuncCmd),
    known_len_(dir == Direction::ToPc ? kImuLen : kCmdLen)
  {
  }

  // 逐字节喂入。整帧通过帧层校验和应用层解码后调用 on_frame(const uint8_t * frame, size_t n)。
  template<class F>
  void feed(std::uint8_t b, F && on_frame)
  {
    ++stats_.bytes_received;
    switch (state_) {
      case State::Head:
        if (b == kHead) {
          ++stats_.headers_seen;
          buf_[0] = b; n_ = 1; state_ = State::Addr;
        } else {
          ++stats_.noise_bytes;
        }
        break;
      case State::Addr:
        if (b == target_addr_) {buf_[n_++] = b; state_ = State::Func;} else {
          ++stats_.bad_addr;
          restartWith(b);
        }
        break;
      case State::Func:
        if (b == known_func_) {buf_[n_++] = b; state_ = State::Len;} else {
          ++stats_.bad_func;
          restartWith(b);
        }
        break;
      case State::Len:
        if (b == known_len_) {
          buf_[n_++] = b;
          remain_ = static_cast<std::size_t>(b) + kVerifyLen;
          state_ = State::Body;
        } else {
          ++stats_.bad_len;
          restartWith(b);
        }
        break;
      case State::Body:
        buf_[n_++] = b;
        if (--remain_ == 0) {
          const std::size_t len = buf_[3];
          if (!frameOk(buf_, len)) {
            ++stats_.bad_checksum;
          } else if (!appDecodeOk()) {
            ++stats_.bad_app;
          } else {
            ++stats_.frames_ok;
            on_frame(static_cast<const std::uint8_t *>(buf_), n_);
          }
          state_ = State::Head;
          n_ = 0;
        }
        break;
    }
  }

  void reset() {state_ = State::Head; n_ = 0;}
  const ParserStats & stats() const {return stats_;}

private:
  enum class State { Head, Addr, Func, Len, Body };

  void restartWith(std::uint8_t b)
  {
    state_ = State::Head;
    n_ = 0;
    if (b == kHead) {
      ++stats_.headers_seen;
      buf_[0] = b; n_ = 1; state_ = State::Addr;
    }
  }

  bool appDecodeOk() const
  {
    if (known_func_ == kFuncImu) {
      ImuSample tmp;
      return decodeImu(buf_, tmp);
    }
    float y, p;
    return decodeCmd(buf_, y, p);
  }

  std::uint8_t target_addr_;
  std::uint8_t known_func_;
  std::uint8_t known_len_;
  State state_ = State::Head;
  std::size_t n_ = 0;
  std::size_t remain_ = 0;
  std::uint8_t buf_[kFrameMax] = {};  // n_ 最大 4 + 54 + 2 = 60，不会越界
  ParserStats stats_;
};

// ---------------------------------------------------------------- seq / stamp 跟踪（第 6.3 节）

struct ImuStep
{
  bool first = false;         // 没有上一帧
  bool reset = false;         // 判定 MCU 已复位：不计丢帧，不求 dt
  bool has_dt = false;
  std::uint32_t dt_us = 0;
  std::uint32_t dropped = 0;  // 丢帧数
  bool gap = false;           // dt > 100 ms（断流）
};

class ImuTracker
{
public:
  ImuStep update(std::uint16_t seq, std::uint32_t stamp_us)
  {
    ImuStep r;
    if (!have_prev_) {
      r.first = true;
    } else {
      const std::uint32_t dt = static_cast<std::uint32_t>(stamp_us - prev_stamp_);  // 无符号减法
      const std::int16_t dseq =
        static_cast<std::int16_t>(static_cast<std::uint16_t>(seq - prev_seq_));
      if (dt >= 0x80000000u || dseq <= -100) {
        r.reset = true;
        ++resets_;
      } else {
        r.has_dt = true;
        r.dt_us = dt;
        r.dropped = dseq > 1 ? static_cast<std::uint32_t>(dseq - 1) : 0u;
        r.gap = dt > kGapUs;
        dropped_total_ += r.dropped;
        if (r.gap) {++gaps_;}
      }
    }
    have_prev_ = true;
    prev_seq_ = seq;
    prev_stamp_ = stamp_us;
    return r;
  }

  std::uint64_t resets() const {return resets_;}
  std::uint64_t dropped() const {return dropped_total_;}
  std::uint64_t gaps() const {return gaps_;}

private:
  bool have_prev_ = false;
  std::uint16_t prev_seq_ = 0;
  std::uint32_t prev_stamp_ = 0;
  std::uint64_t resets_ = 0, dropped_total_ = 0, gaps_ = 0;
};

}  // namespace proto
}  // namespace gimbal_serial
