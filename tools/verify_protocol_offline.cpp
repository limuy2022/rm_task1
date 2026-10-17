// Compile real production protocol/SerialPort code; use only a caller-created PTY.
// No ROS, gtest, hardware port discovery, auto device or physical /dev path.
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <map>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "gimbal_serial/protocol.hpp"
#include "gimbal_serial/serial_port.hpp"

namespace proto = gimbal_serial::proto;
using Bytes = std::vector<std::uint8_t>;
using Frames = std::vector<Bytes>;

struct Golden {
  std::string name, direction, expect, reason, group;
  Bytes bytes;
  std::map<std::string, double> decoded;
};

void require(bool ok, const std::string & message)
{
  if (!ok) {throw std::runtime_error(message);}
}

void near(double value, double expected, const std::string & message)
{
  require(std::isfinite(value) && std::abs(value - expected) <= 1e-6, message);
}

std::string trim(const std::string & value)
{
  const auto a = value.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) {return "";}
  return value.substr(a, value.find_last_not_of(" \t\r\n") - a + 1);
}

std::vector<Golden> loadGolden(const char * path)
{
  std::ifstream input(path);
  require(input.good(), "golden file missing; skipping is forbidden");
  std::vector<Golden> cases;
  Golden current;
  for (std::string line; std::getline(input, line); ) {
    line = trim(line);
    if (line.empty() || line[0] == '#') {continue;}
    if (line == "end") {
      require(!current.name.empty() && !current.bytes.empty(), "malformed golden block");
      cases.push_back(current); current = Golden{}; continue;
    }
    const auto colon = line.find(':');
    require(colon != std::string::npos, "malformed golden field");
    const auto key = line.substr(0, colon), value = trim(line.substr(colon + 1));
    if (key == "case") {current.name = value;}
    else if (key == "dir") {current.direction = value;}
    else if (key == "expect") {current.expect = value;}
    else if (key == "reason") {current.reason = value;}
    else if (key == "group") {current.group = value;}
    else if (key == "hex") {
      std::istringstream words(value);
      for (std::string word; words >> word; ) {
        const auto byte = std::stoul(word, nullptr, 16);
        require(byte <= 255, "golden byte overflow");
        current.bytes.push_back(static_cast<std::uint8_t>(byte));
      }
    } else if (key == "decoded" && value != "-") {
      std::istringstream words(value);
      for (std::string word; words >> word; ) {
        const auto eq = word.find('=');
        require(eq != std::string::npos, "malformed decoded field");
        current.decoded[word.substr(0, eq)] = std::stod(word.substr(eq + 1));
      }
    }
  }
  require(current.name.empty(), "unterminated golden block");
  require(cases.size() == 44, "expected all 44 official v1.4 golden cases");
  return cases;
}

Frames feed(proto::FrameParser & parser, const Bytes & bytes)
{
  Frames frames;
  for (auto byte : bytes) {
    parser.feed(byte, [&frames](const std::uint8_t * p, std::size_t n) {
      frames.emplace_back(p, p + n);
    });
  }
  return frames;
}

const Golden & named(const std::vector<Golden> & cases, const std::string & name)
{
  for (const auto & c : cases) {if (c.name == name) {return c;}}
  throw std::runtime_error("missing golden " + name);
}

proto::ImuSample imu(const Golden & c)
{
  proto::ImuSample result;
  require(proto::decodeImu(c.bytes.data(), result), "IMU application decode " + c.name);
  return result;
}

void checkCases(const std::vector<Golden> & cases)
{
  for (const auto & c : cases) {
    proto::FrameParser parser(c.direction == "mcu2pc" ? proto::Direction::ToPc : proto::Direction::ToMcu);
    const auto frames = feed(parser, c.bytes);
    if (c.expect == "reject") {
      require(frames.empty(), "rejected golden accepted: " + c.name);
      if (c.reason == "bad_checksum") {require(parser.stats().bad_checksum == 1, c.name);}
      if (c.reason == "bad_quat" || c.reason == "non_finite") {
        require(parser.stats().bad_app == 1, "application rejection " + c.name);
      }
      continue;
    }
    require(frames == Frames{c.bytes}, "accepted golden not exact: " + c.name);
    if (c.direction == "pc2mcu") {
      float yaw = 0, pitch = 0;
      require(proto::decodeCmd(c.bytes.data(), yaw, pitch), c.name);
      near(yaw, c.decoded.at("yaw_omega"), c.name + " yaw");
      near(pitch, c.decoded.at("pitch_omega"), c.name + " pitch");
      const auto encoded = proto::encodeCmd(yaw, pitch);
      require(Bytes(encoded.begin(), encoded.end()) == c.bytes, "encode " + c.name);
    } else {
      const auto s = imu(c);
      near(s.seq, c.decoded.at("seq"), c.name + " seq");
      near(s.stamp_us, c.decoded.at("stamp_us"), c.name + " stamp");
      const char * ar[] = {"ax_raw", "ay_raw", "az_raw"};
      const char * gr[] = {"gx_raw", "gy_raw", "gz_raw"};
      const char * qr[] = {"qw_raw", "qx_raw", "qy_raw", "qz_raw"};
      const char * an[] = {"ax", "ay", "az"};
      const char * gn[] = {"gx", "gy", "gz"};
      const char * qn[] = {"qw", "qx", "qy", "qz"};
      const auto acceleration = proto::imuLinearAcceleration(s);
      const auto gyro = proto::imuAngularVelocity(s);
      for (int i = 0; i < 3; ++i) {
        near(s.accel_raw[i], c.decoded.at(ar[i]), c.name + ar[i]);
        near(s.gyro_raw[i], c.decoded.at(gr[i]), c.name + gr[i]);
        near(acceleration[i], c.decoded.at(an[i]), c.name + an[i]);
        near(gyro[i], c.decoded.at(gn[i]), c.name + gn[i]);
      }
      for (int i = 0; i < 4; ++i) {
        near(s.q_raw[i], c.decoded.at(qr[i]), c.name + qr[i]);
        near(s.q[i], c.decoded.at(qn[i]), c.name + qn[i]);
      }
    }
  }
}

Bytes streamFor(const std::vector<Golden> & cases, const std::string & direction)
{
  Bytes bytes{0x55, 0x00, 0xAA, 0x12, 0x99};
  for (const auto & c : cases) {
    if (c.direction == direction) {bytes.insert(bytes.end(), c.bytes.begin(), c.bytes.end());}
  }
  return bytes;
}

Frames acceptedFor(const std::vector<Golden> & cases, const std::string & direction)
{
  Frames frames;
  for (const auto & c : cases) {
    if (c.direction == direction && c.expect == "accept") {frames.push_back(c.bytes);}
  }
  return frames;
}

void checkStreams(const std::vector<Golden> & cases)
{
  for (const std::string direction : {"pc2mcu", "mcu2pc"}) {
    const auto input = streamFor(cases, direction);
    const auto expected = acceptedFor(cases, direction);
    for (std::size_t chunk : {1u, 2u, 5u, 31u, 4096u}) {
      proto::FrameParser parser(direction == "mcu2pc" ? proto::Direction::ToPc : proto::Direction::ToMcu);
      Frames output;
      for (std::size_t i = 0; i < input.size(); i += chunk) {
        const auto end = std::min(input.size(), i + chunk);
        auto part = feed(parser, Bytes(input.begin() + i, input.begin() + end));
        output.insert(output.end(), part.begin(), part.end());
      }
      require(output == expected, "chunked/noisy stream " + direction);
    }
  }
  Bytes noise;
  for (int i = 0; i < 100000; ++i) {noise.push_back(i % 3 == 0 ? 0xAA : static_cast<std::uint8_t>(i * 7));}
  noise.insert(noise.end(), {0xAA, 0xAA, 0xFF, 0xAA, 0xFF, 0x01, 0x1A});
  noise.insert(noise.end(), 60, 0);
  const auto good = named(cases, "imu_static_level").bytes;
  noise.insert(noise.end(), good.begin(), good.end());
  proto::FrameParser parser(proto::Direction::ToPc);
  const auto output = feed(parser, noise);
  require(!output.empty() && output.back() == good, "resynchronization after 100000 noise bytes");
}

void checkTracker(const std::vector<Golden> & cases)
{
  proto::ImuTracker tracker;
  auto a = imu(named(cases, "imu_wrap_a")), b = imu(named(cases, "imu_wrap_b"));
  require(tracker.update(a.seq, a.stamp_us).first, "wrap first");
  auto step = tracker.update(b.seq, b.stamp_us);
  require(!step.reset && step.has_dt && step.dt_us == 2000 && step.dropped == 0, "counter wrap");
  proto::ImuTracker reset;
  a = imu(named(cases, "imu_reset_a")); b = imu(named(cases, "imu_reset_b"));
  reset.update(a.seq, a.stamp_us); step = reset.update(b.seq, b.stamp_us);
  require(step.reset && !step.has_dt && step.dropped == 0, "MCU reset");
  step = reset.update(static_cast<std::uint16_t>(b.seq + 1), b.stamp_us + 2000);
  require(!step.reset && step.has_dt && step.dt_us == 2000, "post-reset baseline");
  proto::ImuTracker edge;
  edge.update(1000, 1000000);
  require(edge.update(900, 1002000).reset, "signed sequence rollback boundary");
  proto::ImuTracker duplicate;
  duplicate.update(10, 5000);
  step = duplicate.update(10, 5000);
  require(!step.reset && step.dt_us == 0 && step.dropped == 0, "zero dt must not underflow drops");
  step = duplicate.update(12, 205000);
  require(step.gap && step.dropped == 1 && step.dt_us == 200000, "gap and missing frame");
}

void masterWrite(int fd, const Bytes & bytes)
{
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto n = write(fd, bytes.data() + offset, bytes.size() - offset);
    if (n > 0) {offset += static_cast<std::size_t>(n); continue;}
    if (n < 0 && errno == EINTR) {continue;}
    throw std::runtime_error("PTY master write failed");
  }
}

Bytes portRead(gimbal_serial::SerialPort & port, std::size_t count)
{
  Bytes bytes;
  for (int tries = 0; bytes.size() < count && tries < 50; ++tries) {
    std::uint8_t buffer[37];
    const auto n = port.readSome(buffer, sizeof(buffer), 20);
    require(n >= 0, "PTY SerialPort read failed");
    bytes.insert(bytes.end(), buffer, buffer + n);
  }
  require(bytes.size() == count, "PTY read timeout/extra bytes");
  return bytes;
}

void checkPty(const std::vector<Golden> & cases, const char * path, int master)
{
  // Validate against the inherited master; never accept an arbitrary real port.
  const char * generated = ptsname(master);
  require(generated && std::string(generated) == path, "path is not the caller-created PTY");
  require(isatty(master), "master must be a PTY");
  gimbal_serial::SerialPort port, second;
  std::string error;
  require(port.open(path, 115200, error), "PTY open: " + error);
  require(!second.open(path, 115200, error), "PTY exclusive open did not reject second owner");
  // 身份解析对 PTY 的降级：resolved 指向 /dev/pts，且不会误配任何真实 by-id 条目。
  const auto id_pty = gimbal_serial::inspectPortIdentity(path, "/dev/serial/by-id");
  require(id_pty.by_id.empty(), "PTY must not match a real by-id entry");
  require(id_pty.resolved.rfind("/dev/pts/", 0) == 0, "PTY resolves under /dev/pts");
  const Bytes raw{0xAA, 0xFF, 0x0D, 0x0A, 0x03, 0x11, 0x13, 0x00, 0x80};
  masterWrite(master, raw);
  require(portRead(port, raw.size()) == raw, "raw mode changed binary/control bytes");
  const auto input = streamFor(cases, "mcu2pc");
  proto::FrameParser parser(proto::Direction::ToPc);
  Frames frames;
  for (std::size_t i = 0; i < input.size(); i += 7) {
    const auto end = std::min(input.size(), i + 7);
    const Bytes chunk(input.begin() + i, input.begin() + end);
    masterWrite(master, chunk);
    const auto output = feed(parser, portRead(port, chunk.size()));
    frames.insert(frames.end(), output.begin(), output.end());
  }
  require(frames == acceptedFor(cases, "mcu2pc"), "PTY fragmented IMU golden stream");
  const auto command = named(cases, "cmd_mixed").bytes;
  require(port.writeAll(command.data(), command.size()), "SerialPort command write failed");
  Bytes received;
  for (int tries = 0; tries < 50 && received.size() < command.size(); ++tries) {
    pollfd wait{master, POLLIN, 0};
    if (poll(&wait, 1, 20) > 0) {
      std::uint8_t buffer[64];
      const auto n = read(master, buffer, sizeof(buffer));
      require(n > 0, "PTY master command read failed");
      received.insert(received.end(), buffer, buffer + n);
    }
  }
  require(received == command, "command bytes did not cross actual SerialPort exactly");
  std::uint8_t buffer[8];
  require(port.readSome(buffer, sizeof(buffer), 10) == 0, "empty PTY timeout");
  require(!second.open(path, 12345, error), "unsupported baud must reject before open");
  port.close();
  require(second.open(path, 115200, error), "PTY cannot reopen after close: " + error);
  close(master);
  bool disconnected = false;
  for (int i = 0; i < 10 && !disconnected; ++i) {
    disconnected = second.readSome(buffer, sizeof(buffer), 20) < 0;
  }
  require(disconnected, "closed PTY peer must be fatal");
}

// PortIdentity：by-id 反查、同芯片按设备去重、目录缺失/幽灵路径的优雅降级。
// 用临时目录模拟 /dev/serial/by-id；glob 排序下同类芯片列表应为 [CH340, CP2102]。
void checkPortIdentity()
{
  char dir_tpl[] = "/tmp/verify_byid_XXXXXX";
  const char * dir = mkdtemp(dir_tpl);
  require(dir != nullptr, "mkdtemp failed");
  const std::string by_id = std::string(dir) + "/by-id";
  require(mkdir(by_id.c_str(), 0755) == 0, "mkdir by-id failed");
  const std::string cp_target = std::string(dir) + "/dev_cp";
  const std::string ch_target = std::string(dir) + "/dev_ch";
  const std::string ft_target = std::string(dir) + "/dev_ft";
  for (const auto & t : {cp_target, ch_target, ft_target}) {
    const int fd = ::open(t.c_str(), O_CREAT | O_RDWR, 0644);
    require(fd >= 0, "identity target create failed");
    ::close(fd);
  }
  const std::string cp1 = by_id + "/usb-Silicon_Labs_CP2102_A-if00-port0";
  const std::string cp2 = by_id + "/usb-Silicon_Labs_CP2104_B-if00-port0";  // 同一设备的第二接口
  const std::string ch = by_id + "/usb-QinHeng_CH340_C-if00-port0";
  const std::string ft = by_id + "/usb-FTDI_FT232_D-if00-port0";
  require(symlink(cp_target.c_str(), cp1.c_str()) == 0, "symlink cp1 failed");
  require(symlink(cp_target.c_str(), cp2.c_str()) == 0, "symlink cp2 failed");
  require(symlink(ch_target.c_str(), ch.c_str()) == 0, "symlink ch failed");
  require(symlink(ft_target.c_str(), ft.c_str()) == 0, "symlink ft failed");

  require(gimbal_serial::isGimbalRuleChipByIdName("USB-SILICON_LABS_CP2102"), "chip case-insensitive");
  require(gimbal_serial::isGimbalRuleChipByIdName("usb-wch_CH9102"), "chip ch9102 variant");
  require(!gimbal_serial::isGimbalRuleChipByIdName("usb-FTDI_FT232"), "chip ftdi excluded");

  const auto id = gimbal_serial::inspectPortIdentity(cp_target, by_id);
  require(id.resolved == cp_target, "realpath of plain file target");
  require(id.by_id == cp1, "by-id reverse lookup picks first sorted entry");
  require(
    id.rule_chip_by_ids.size() == 2 && id.rule_chip_by_ids[0] == ch &&
      id.rule_chip_by_ids[1] == cp1,
    "rule-chip adapters deduped by device");
  const auto id_ft = gimbal_serial::inspectPortIdentity(ft_target, by_id);
  require(id_ft.by_id == ft && id_ft.rule_chip_by_ids.size() == 2, "ftdi lookup, machine-wide chips");
  const auto id_missing = gimbal_serial::inspectPortIdentity(cp_target, std::string(dir) + "/no_such");
  require(
    id_missing.by_id.empty() && id_missing.rule_chip_by_ids.empty() && !id_missing.resolved.empty(),
    "missing by-id dir degrades to empty");
  const auto id_ghost = gimbal_serial::inspectPortIdentity("/dev/definitely_not_here", by_id);
  require(id_ghost.resolved.empty() && id_ghost.by_id.empty(), "ghost path resolves empty");

  for (const auto & e : {cp1, cp2, ch, ft}) {::unlink(e.c_str());}
  for (const auto & t : {cp_target, ch_target, ft_target}) {::unlink(t.c_str());}
  ::rmdir(by_id.c_str());
  ::rmdir(dir);
}

int main(int argc, char ** argv)
{
  try {
    require(argc == 4, "expected golden-file and caller-created PTY slave/master arguments");
    const auto cases = loadGolden(argv[1]);
    checkCases(cases);
    checkStreams(cases);
    checkTracker(cases);
    checkPortIdentity();
    checkPty(cases, argv[2], std::stoi(argv[3]));
    std::cout << "{\"passed\":true,\"golden_cases\":44,\"stream_chunk_sizes\":5,"
      "\"noise_bytes\":100000,\"pty_baud\":115200,\"physical_devices_opened\":0,"
      "\"tracker_wrap_reset_gap\":true,\"pty_raw_exclusive_reopen_disconnect\":true,"
      "\"port_identity\":\"by-id reverse lookup + rule-chip dedup on temp symlinks; PTY degrades\"}\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
