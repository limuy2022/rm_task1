// 极小的 termios 串口封装：独占打开（TIOCEXCL + flock）、raw 模式、非阻塞读写。
// 不依赖 ROS。依据 protocol/PROTOCOL.md 第 2 节。
#pragma once

#include <errno.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace gimbal_serial {

// Linux TIOCGICOUNT 返回驱动累计值；查询成功也不代表每个字段都被驱动上报。
// 使用无符号 32 位保存内核 int 的位模式，允许以模 2^32 差值处理计数回绕。
struct UartErrorCounters {
    std::uint32_t frame = 0;
    std::uint32_t parity = 0;
    std::uint32_t overrun = 0;
    std::uint32_t brk = 0;
    std::uint32_t buf_overrun = 0;
};

enum class UartCounterStatus { Ok, Unsupported, Error };

struct UartCounterResult {
    UartCounterStatus status = UartCounterStatus::Error;
    UartErrorCounters counters{};
    int error_number = 0;
};

// 默认使用真实 ioctl；测试可注入驱动快照，不需要接触真实串口。
using UartCounterReader = std::function<UartCounterResult(int)>;

class SerialPort {
  public:
    explicit SerialPort(UartCounterReader counter_reader = {});
    ~SerialPort() { close(); }
    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    // 打开并配置为 baud、8N1、无流控、raw，并开启 INPCK 以接收驱动错误事件。
    // 不启用 PARMRK：合法字节原样接收，驱动标记的坏字节由内核转 NUL
    // 后交协议校验。 任何一步失败都返回 false，err 写明是哪一步，不重试。
    // 步骤：open(O_RDWR|O_NOCTTY|O_NONBLOCK) -> ioctl(TIOCEXCL) ->
    // flock(LOCK_EX|LOCK_NB) -> termios。
    bool open(const std::string &path, int baud, std::string &err);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // 最近一次 open 失败时的 errno（成功时为 0）。
    int lastErrno() const { return last_errno_; }
    // 只有“设备还不存在”（ENOENT、ENODEV、ENXIO）才算可以等待重试；
    // EBUSY、EWOULDBLOCK/EAGAIN（被别的进程占用）等一律不重试，直接退出（第 2
    // 节）。
    static bool isDeviceAbsentErrno(int e) {
        return e == ENOENT || e == ENODEV || e == ENXIO;
    }

    // 最多等待 timeout_ms。返回读到的字节数；0 表示超时；-1
    // 表示致命错误（设备消失、对端关闭）。
    ssize_t readSome(std::uint8_t *buf, std::size_t cap, int timeout_ms);

    // 写完全部字节（缓冲满时等待），成功返回 true。线程安全（内部有互斥锁）。
    bool writeAll(const std::uint8_t *data, std::size_t size);

    // 只查询驱动错误计数，不读取/清空串口缓冲区；失败不影响串口收发。
    UartCounterResult readErrorCounters() const;

    static bool baudSupported(int baud);

  private:
    int fd_ = -1;
    int last_errno_ = 0;
    std::string path_;
    std::mutex write_mutex_;
    UartCounterReader counter_reader_;
};

// 串口选择（纯函数，便于测试）。
//   param 非空且不是 "auto"：原样使用，RM_SIM_SERIAL 被忽略。
//   否则 env 非空：使用 env（虚拟云台的兼容做法，即使文件暂不存在也直接用，由
//   open 报错，
//     避免在虚拟联调时悄悄改开真实设备）。
//   否则 /dev/gimbal 存在：使用它。
//   否则用 candidates（/dev/ttyUSB*、/dev/ttyACM* 的有序枚举）的第一项。
struct PortChoice {
    std::string path;
    std::string source; // "param" / "RM_SIM_SERIAL" / "udev" / "scan" / "none"
};

PortChoice selectPort(const std::string &param, const std::string &env,
                      bool gimbal_exists,
                      const std::vector<std::string> &candidates);

// 端口身份（纯函数，仅诊断日志用，不影响选口与读写）。by_id_dir 参数便于测试。
// 背景：99-gimbal.rules 按 VID:PID 匹配 CP210x/CH340，同型号多颗适配器时
// /dev/gimbal
// 指向后插的那颗——打开成功不代表打开的是云台。全部字段解析失败时为空，优雅降级。
struct PortIdentity {
    std::string resolved; // realpath 后的设备节点（/dev/gimbal ->
                          // /dev/ttyUSB2）；失败为空
    std::string
        by_id; // by_id_dir 下指向同一设备的稳定名；找不到为空（PTY 即此情况）
    // 本机所有 CP210x/CH340 类适配器的 by-id 名，按 resolved
    // 去重（多接口适配器只算一颗）
    std::vector<std::string> rule_chip_by_ids;
};

PortIdentity
inspectPortIdentity(const std::string &path,
                    const std::string &by_id_dir = "/dev/serial/by-id");

// by-id 条目名是否属于 99-gimbal.rules 会匹配的芯片（CP210x / CH340 /
// CH9102）。
bool isGimbalRuleChipByIdName(const std::string &by_id_basename);

} // namespace gimbal_serial
