# 云台套件通信协议与接口契约

版本 v1.4，日期 2026-09-30。

变更记录：

- v1.4（只改 10.3 的伪代码位置和 10.4 的措辞，数值和语义不变）：
  - 10.3 节伪代码里的 CAN 掉线块原来放在“以下是 ACTIVE / DECEL”之后，只覆盖 ACTIVE 和 DECEL，与 10.4 正文（BOOT 以外的任何状态）不一致。现移到 BOOT 分支之后、FAULT 和 IDLE 分支之前，BOOT 以外的任何状态下出现 CAN 掉线，每个 tick 都执行 relax 加 disarm。
  - 10.4 节补一句：FAULT 状态下掉线仍保持 FAULT，不转成 IDLE，恢复后按 10.1 的规则（指令流静默满 `cmd_timeout`）清除。
  - 固件实现（`GimbalSafety.cpp`）和主机测试早已按此行为，本版只是让文档与之对齐。

- v1.3（只改 10.4 节，数值不动）：
  - 10.4 节 CAN 掉线判据由“任一使能中的电机反馈中断”改为“BOOT 以外的任何状态下，任一电机的反馈不新鲜，每个 tick 都执行 disarm，静默计时要在反馈恢复之后才开始”。理由：急停期间上位机持续发指令，恢复供电后不得自动使能。
  - 10.4 节补充固件专用条款：DM 反馈 ERR 码高 4 位（0 禁能、1 使能、8~E 故障），ACTIVE/DECEL 下 ERR 不等于 1 且已过 `dm_enable_hold` 时按 CAN 掉线处理。
- v1.2（只补契约空白，数值和已有语义不变）：
  - 第 7 节新增虚拟相机接口（话题、PolygonStamped 像素坐标、角点顺序、frame）和真实相机话题名由 camera_setup 决定、课 2 输入话题可用参数切换的说明。
  - 第 7 节新增 bridge 参数表：`yaw_sign`、`pitch_sign`，默认 1，只作用于转发 `cmd_vel_gimbal` 的那一路。
  - 第 2 节写明独占打开同时用 TIOCEXCL 和 flock，root 绕得过 TIOCEXCL 但绕不过 flock，任一拒绝就报错退出，不重试。
  - 第 10.3 节 runaway：历史记录不足 `err_grow_dt` 时不判定为误差增长，固件和虚拟云台必须一致。
  - 第 7 节新增外参 yaml 格式：`camera_xyz`（m）、`camera_rpy`（rad），文件 `gimbal_edu_description/config/{default,virtual}.yaml`。
  - 第 7 节新增时钟约定：时长用单调时钟，header.stamp 用系统时钟，WSL2 系统时钟会跳变。
- v1.1（依据评审意见 D1–D14）：
  - IMU 去重改为复位检测：不再按 dt 丢弃“重复或乱序”帧，stamp 倒退或 seq 大幅回退判定 MCU 已复位（6.3 节）。黄金向量新增 `reset_pair`。
  - 重新上膛（安全）：BOOT、CAN 掉线、FAULT 之后要先观察到指令流静默 ≥ `cmd_timeout`，此后的第一帧有效指令才使能；FAULT 不再要求断电重启（10 节，参数 `arm_requires_silence`）。
  - 固件在 IMU 就绪之前不发 0x01（维持必须），补写“上电后约 N 秒才有 IMU 数据”（3.5、5.2 节）。
  - 方向参数收窄：`*_dir` 固定 +1，`*_sign` 是 PC 侧“靠传参”的旋钮，`*_motor_sign` 只管硬件极性；编码器换算符号取 motor_sign，必要时加 `enc_sign`（8 节）。新增“实机首次上电流程”（13 节）。
  - 第 11 节默认值标为“初值，实机未验证”，并写出校核式，`limit_margin` 由 0.15 改为 0.20 rad。
  - 12 节差异表加两行：ling_vision 对 accel x、y 取负；`imu_payload_mode` 与 WorldTranslation 分支。bridge 的黄金测试必须走真实 accel 解码路径。
  - 黄金向量：物理用例加 `state` 字段（`psi theta psid thetad`），由 state 生成的帧与 hex 每个 raw 相差不超过 ±1 LSB；新增反例 `rej_imu_q_zero`（reason `bad_quat`）；`decode_frame` 加 q 范数检查。9.3 节措辞相应收窄。
  - yaw 多圈：`yaw_limit + limit_fault_margin < π`；上电与每次 CAN 反馈恢复后由单圈绝对值重建 θ_yaw，重建完成才允许使能（10.3 节）。
  - 9.4 节流式测试的输出指“状态机加应用层解码之后的帧”，3.4 节注释同步。
  - yaw 的 ω_meas 公式里的 θ 取 pitch 关节角 `theta[1]`（6.2、10.3 节）。
  - `on_frame` 在 ISR 中运行，只原子地写 `cmd_target`、`t_last_valid` 和使能请求标志，状态转移与 enable 放在 tick（10.3 节）。
  - 取整统一为 `lroundf`（远离零舍入），编码侧测试允许 ±1 LSB（6.1 节）。
- v1.0：初版。

这份文档是固件、bridge、虚拟云台、学员答案四方实现的唯一依据。文档与源码或计划冲突时，先以本文为准，再回头改代码。协议数值细节由 `golden_frames.txt` 兜底，四份实现都必须跑通它。

相关文件：

- `protocol/golden_frames.txt`：黄金向量，由 `protocol/gen_golden.py` 生成。
- `protocol/func_id_table.md`：两个源仓库已占用的功能号，加上本套件的 0x30。

关键词约定：必须、不得表示硬性要求；应该表示默认做法，偏离要写明原因。

## 1. 范围与约定

- 协议沿用 OffboardLink 帧格式。核实过的源码：ling_vision `robo-bridge/include/robo_bridge/OffboardLink.h`，SwordInfantry `Libraries/Offboard/MessageBase.h`、`OffboardLink.h`。两仓的帧头、地址、校验算法完全一致。
- 本套件只用两个功能号：0x30（PC→MCU，角速度指令）和 0x01（MCU→PC，IMU）。
- 所有多字节整数和浮点数都是小端。float 指 IEEE 754 binary32。
- 坐标系遵守 REP-103，右手系：x 向前，y 向左，z 向上。
  - 绕 +z 逆时针为正，即 yaw 增大。
  - 绕 +y 转动，x 轴向下为正，即"低头为正"，pitch 增大。
- 帧名词：
  - `world`：惯性系，z 向上。没有磁力计，所以 world 的 yaw 零点是"IMU 就绪时云台的朝向"，之后 yaw 靠陀螺积分，会缓慢漂移。roll、pitch 由重力锁定，可观。
  - `gimbal_link`：固连在 IMU 上的机体系。IMU 所在的 C 板装在 pitch 级上，所以 gimbal_link 同时随 yaw、pitch 转动。
- 关节：yaw 关节轴是世界 z 轴，pitch 关节轴是机体 y 轴，云台没有 roll 关节。姿态按 ZYX（先 yaw 后 pitch）分解，roll 恒为 0：
  R(q) = Rz(ψ) · Ry(θ)。

## 2. 传输

| 项 | 取值 |
|---|---|
| 接口 | C 板 USART1，经 USB-TTL 接 PC（具体丝印以 gimbal-edu-firmware 仓库的 `docs/PARAMS.md` 为准） |
| 参数 | 921600 bps，8N1，无流控，TTL 3.3V，TX/RX 交叉，共地 |
| PC 设备名 | 真机 `/dev/gimbal`（udev 别名，`setup/99-gimbal.rules`）；虚拟云台 `/tmp/ttyVGIMBAL` |
| 带宽占用 | 0x01 帧 32 字节 × 500 Hz = 16 kB/s，约占线路 17%（每字节 10 位） |

PC 侧必须独占打开串口。原因：残留进程会分走字节流，帧就凭空丢掉了，症状看起来像"校验错很多"。

- C/C++：`open(O_RDWR | O_NOCTTY | O_NONBLOCK)` 之后，先 `ioctl(fd, TIOCEXCL)`，再 `flock(fd, LOCK_EX | LOCK_NB)`。任何一步失败都直接报错退出，不要静默重试。
  - 两种机制都要用：TIOCEXCL 拒绝其他非 root 进程再打开该设备，但 root 可以绕过 TIOCEXCL；flock 是建议锁，root 也绕不过（前提是对方也用 flock）。所以 TIOCEXCL 与 flock 缺一不可。
  - 只要有一种机制拒绝（TIOCEXCL 或 flock 返回错误），就报错退出，不重试、不降级为共享打开。
- Python：`serial.Serial(port, 921600, exclusive=True)`。
- 串口设成 raw 模式（`cfmakeraw`），关掉回显和行缓冲。
- bridge、`gimbal_serial`、学员节点三者同一时刻只能有一个打开串口。课 4、5 由学员节点自己收发串口，所以那两节课不启动 bridge。

## 3. 帧层

### 3.1 字段表

```
偏移  0     1     2     3     4 .. 4+len-1   4+len   5+len
      AA    addr  func  len   data           sum     sum_sum
```

| 偏移 | 字段 | 大小 | 说明 |
|---|---|---|---|
| 0 | head | 1 | 固定 0xAA |
| 1 | addr | 1 | 0xEE：PC→MCU（上行）；0xFF：MCU→PC（下行） |
| 2 | func | 1 | 功能号 |
| 3 | len | 1 | data 区字节数 |
| 4 | data | len | 载荷，小端 |
| 4+len | sum | 1 | 校验和 |
| 5+len | sum_sum | 1 | 校验和的累加和 |

- 帧总长 = len + 6，**不得超过 60 字节**（`OFFBOARDLINK_FRAME_MAX_LEN`），所以 len ≤ 54。
- addr 与 func 绑定：0x30 只能出现在 0xEE 上，0x01 只能出现在 0xFF 上。addr 不匹配的帧丢弃。
- 每个 func 的 len 是固定值（0x30 为 8，0x01 为 26）。len 与规定值不符的帧丢弃。
- 接收方遇到未知 func 丢弃，不报错，不影响后续帧。

### 3.2 校验算法

sum 和 sum_sum 都是 uint8，覆盖从 head 到 data 最后一个字节，即 `buf[0 .. len+3]`，不含这两个校验字节本身。sum_sum 累加的是**每一步更新之后**的 sum。

```
def calc_verify(buf, len):        # buf[0] 是 0xAA
    sum = 0
    sum_sum = 0
    for i in 0 .. len + 3:        # 含两端，共 len + 4 个字节
        sum     = (sum + buf[i]) & 0xFF
        sum_sum = (sum_sum + sum) & 0xFF
    return sum, sum_sum

def pack_frame(addr, func, data):
    buf = [0xAA, addr, func, len(data)] + data
    s, ss = calc_verify(buf, len(data))
    return buf + [s, ss]

def frame_ok(buf, len):
    s, ss = calc_verify(buf, len)
    return s == buf[len + 4] and ss == buf[len + 5]
```

手算示例：`AA EE 30 08` 加 8 个 0x00 数据字节。

- sum = (0xAA + 0xEE + 0x30 + 0x08) & 0xFF = 0xD0。
- sum_sum：前 4 步的 sum 依次为 0xAA、0x98、0xC8、0xD0，它们的和是 0x2DA；后面 8 个 0x00 字节各再加一次 0xD0，共加 0x680，合计 0x95A，取低 8 位得 0x5A。
- 整帧：`AA EE 30 08 00 00 00 00 00 00 00 00 D0 5A`，就是黄金向量里的 `cmd_zero`。

### 3.3 收发规则

- 一次只发一整帧，帧与帧之间没有间隔要求。
- 校验失败、len 不符、func 未知、addr 不符、head 不符，都是"丢弃"：不产生输出，不刷新任何超时计时，不改变任何状态。
- 校验失败时丢弃整帧，回到寻找帧头状态，不回扫已收字节。实现回扫也可以，只要黄金向量的流式测试仍然通过。
- 接收缓冲区必须限制下标不超过 60。源码里的解析器在找不到帧头时下标会一直增长，见第 12 节。

### 3.4 接收状态机

逐字节喂入。`target_addr` 对固件是 0xEE，对 PC 是 0xFF。`known_len[func]` 是本端订阅的功能号及其固定 len：固件只有 {0x30: 8}，PC 只有 {0x01: 26}。

```
state = HEAD; n = 0

def restart_with(b):                     # 当前字节 b 可能就是新帧头
    state = HEAD; n = 0
    if b == 0xAA: buf[0] = b; n = 1; state = ADDR

def feed(b):
    if state == HEAD:
        if b == 0xAA: buf[0] = b; n = 1; state = ADDR
    elif state == ADDR:
        if b == target_addr: buf[n++] = b; state = FUNC
        else: restart_with(b)
    elif state == FUNC:
        if b in known_len:   buf[n++] = b; state = LEN
        else: restart_with(b)
    elif state == LEN:
        if b == known_len[buf[2]]: buf[n++] = b; remain = b + 2; state = BODY
        else: restart_with(b)
    else:                                # BODY
        buf[n++] = b; remain -= 1
        if remain == 0:
            if frame_ok(buf, buf[3]) and app_decode_ok(buf): output(buf)   # 帧层校验，再加应用层解码
            state = HEAD; n = 0          # 校验失败也整帧丢弃
```

n 最大为 60（len 最大 54 时 4 + 54 + 2），不会越界。

`app_decode_ok` 是应用层解码：0x30 要求两个 float 都是有限值（isfinite）；0x01 要求解码后的四元数范数 ≥ 0.5（`rej_imu_q_zero` 这类校验正确但 q 全零的帧在这里被丢）。校验通过而应用层解码失败的帧，同样不产生输出，不刷新任何计时。第 9.4 节的流式测试比较的，就是 `output(buf)` 之后的帧，不是只过了帧层校验的帧。

### 3.5 发送规则

- 固件 0x01 只在 IMU 就绪之后才开始发（必须，见第 10 节）。上电后要等到温和标定完成，约 N 秒后才有 IMU 数据（N 见 5.2 节）。所以 PC 收到第一帧 0x01，就等于知道 IMU 已经就绪。
- PC 发 0x30 建议 20 到 100 Hz 连续发送，间隔要远小于 500 ms 的固件超时。
- 程序退出或目标跟丢时，PC 发 0x30 零指令，并且可以连续发几帧。即便退出前一帧都没发出去，固件也会在 500 ms 后减速、3.5 s 后松弛（第 10 节）。
- 例外：bridge 的可选订阅 `/offboardlink/cmd_vel_gimbal` 只在收到新消息时转发一帧，不做保活。这是为了保持 bridge 简单；持续跟踪要自己发布话题或使用课 5 的节点。

## 4. 0x30 角速度指令（PC→MCU）

功能号 0x30，addr 0xEE，len 8，整帧 14 字节。

| 帧内偏移 | data 偏移 | 类型 | 字段 | 单位 | 说明 |
|---|---|---|---|---|---|
| 0 | — | u8 | head | | 0xAA |
| 1 | — | u8 | addr | | 0xEE |
| 2 | — | u8 | func | | 0x30 |
| 3 | — | u8 | len | | 8 |
| 4–7 | 0 | float32 LE | yaw_omega | rad/s | 绕 yaw 关节轴（世界 z）的关节角速度，绕 +z 逆时针为正 |
| 8–11 | 4 | float32 LE | pitch_omega | rad/s | 绕 pitch 关节轴（机体 y）的关节角速度，低头为正 |
| 12 | — | u8 | sum | | |
| 13 | — | u8 | sum_sum | | |

语义：

- 这是**关节角速度**，不是机体系下的陀螺分量。yaw_omega 就是 ψ̇，pitch_omega 就是 θ̇。pitch 抬起时，云台机体系 z 轴陀螺读数 gz = ψ̇·cosθ，与 yaw_omega 不相等，这一点见第 6.2 节。
- 上表的正方向是本套件的取值：`yaw_dir = pitch_dir = +1`，固定不变。`*_dir` 只有把协议挪作他用时才会改，改了就会翻转线上正方向，见第 8 节。
- 固件只接收 len == 8 且两个 float 都是有限值的帧，其余直接丢弃。有限值指非 NaN、非 ±inf。
- 大幅度的合法值（例如 100 rad/s）在帧层是接受的，由应用层限幅到 `omega_max`，不是丢弃。
- 零指令 `(0, 0)` 是合法的：它照常刷新看门狗，电机保持使能，速度环保持 0。

示例：yaw = 3.0，pitch = −0.125（黄金向量 `cmd_mixed`）：

```
AA EE 30 08 00 00 40 40 00 00 00 BE 0E D8
```

## 5. 0x01 IMU（MCU→PC）

功能号 0x01，addr 0xFF，len 26，整帧 32 字节，500 Hz。**字节布局与两个源仓库完全一致，不变。**

| 帧内偏移 | data 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|---|
| 0 | — | u8 | head | 0xAA |
| 1 | — | u8 | addr | 0xFF |
| 2 | — | u8 | func | 0x01 |
| 3 | — | u8 | len | 26（0x1A） |
| 4–5 | 0 | u16 LE | seq | 每发一帧加 1，从 0 起，65535 后回绕到 0 |
| 6–9 | 2 | u32 LE | stamp_us | 板载单调时钟，微秒，约 71.6 分钟回绕一次 |
| 10–11 | 6 | i16 LE | ax_raw | 比力 x，gimbal_link 系 |
| 12–13 | 8 | i16 LE | ay_raw | 比力 y |
| 14–15 | 10 | i16 LE | az_raw | 比力 z |
| 16–17 | 12 | i16 LE | gx_raw | 去过零偏的机体角速度 x |
| 18–19 | 14 | i16 LE | gy_raw | 机体角速度 y |
| 20–21 | 16 | i16 LE | gz_raw | 机体角速度 z |
| 22–23 | 18 | i16 LE | qw_raw | 四元数 w，除以 32760 |
| 24–25 | 20 | i16 LE | qx_raw | 四元数 x |
| 26–27 | 22 | i16 LE | qy_raw | 四元数 y |
| 28–29 | 24 | i16 LE | qz_raw | 四元数 z |
| 30 | — | u8 | sum | |
| 31 | — | u8 | sum_sum | |

data 合计 2 + 4 + 6 + 6 + 8 = 26 字节。

### 5.1 语义修正

现状的固件回传的 accel、gyro 是 BMI088 的原始值，q 来自 AHRS，三者不在同一个坐标系里；ling_vision 的 bridge 又只对 gyro 的 x、y 取负来凑轴。本套件把它改成：

- 固件用**同一个安装旋转**（整个工程只有一处来源）同时处理 AHRS 的输入和回传的 accel、gyro。三者都落在 gimbal_link（REP-103）机体轴上。
- gyro 是去过零偏后的值。
- bridge 因此不再对陀螺取负。它的 `imu_axis_sign` 参数只作用于 accel 和 gyro，不碰 q，本套件固定为 (1, 1, 1)。

### 5.2 就绪与首帧

固件在 IMU 就绪（到温、零偏标定完成、AHRS 用加计平均值完成初始化）之前**不发送** 0x01，这一条是必须。收到第一帧 0x01，就说明 q 已经可用。

**上电后要等到温和标定完成，约 N 秒后才有 IMU 数据。** N 是到温加零偏标定所需的时间，实机值由固件 `PARAMS.md` 记录（尚未实测），虚拟云台是 `imu_ready_delay`，默认 1.0 s。PC 侧等 IMU 的超时必须明显大于 N，不得因为上电后一阵子没有 0x01 就判定串口故障。

## 6. 换算系数与语义

### 6.1 换算系数

| 量 | 换算 | 系数 | 来源 |
|---|---|---|---|
| accel | m/s² = raw × ACCEL_SCALE | 0.00718260498046875（= 24·9.80665/32768） | 24 g 量程 |
| gyro | rad/s = raw × GYRO_SCALE | 0.00106526443603169529841533860381（= 2000°/s 转弧度 / 32768） | ±2000 dps 量程 |
| q | 分量 = raw / 32760.0 | 32760 | |

- 文中和计划里写的 0.0071826、0.0010653 是这两个系数的四舍五入，实现时用上表的完整数值。
- 量程：accel ±235.36 m/s²，gyro ±34.9 rad/s。
- 编码时 raw 用 `lroundf`（四舍五入，恰好 .5 时远离零）取整后限幅到 int16，再拆字节。四元数分量限幅到 ±32760，不要取 −32768。解码时先按 u16 组装，再转成 i16。
- 取整方式统一为 `lroundf`，不用 `lrintf`（默认舍入模式下就近取偶）。float32 与 double 的中间结果会在 .5 边界上分叉，所以**编码侧**的比较（固件、虚拟云台由 state 生成的帧对黄金 hex）允许每个 raw 相差 ±1 LSB；解码侧必须精确。
- 除以 32760 必须用浮点除法。SwordInfantry 的 `IMUData.h` 里那处整数除法会让 q 只剩 {−1, 0, 1}，是 bug，见第 12 节。
- 解码后的四元数不一定恰好是单位长度。使用前要归一化。范数小于 0.5 的帧视为损坏，丢弃并计数（黄金向量 `rej_imu_q_zero`，q 全零）。

### 6.2 各量的语义

**accel：gimbal_link 轴上的比力。**

比力是加速度计实际测到的量，等于运动加速度减去重力加速度。静止时它指向"向上"：

- f_b = R(q)ᵀ · (a_world − g_world)，其中 g_world = (0, 0, −g)，g = 9.80665。
- 静止水平：accel ≈ (0, 0, +g)。
- 静止但 pitch = θ：accel ≈ (−g·sinθ, 0, g·cosθ)。低头（θ > 0）时 ax 为负。
- 这里的 accel 不是"减去重力后的线加速度"，也不是世界系量。ROS 的 `linear_acceleration` 直接放这个值。

**gyro：去过零偏的机体角速度。**

坐标轴是 gimbal_link 的 x、y、z，单位 rad/s，正方向按右手定则。对 ZYX 姿态：

- gyro = (−ψ̇·sinθ, θ̇, ψ̇·cosθ)。
- 绕 +z 正转（ψ̇ > 0，θ = 0）时 gz > 0，q 的 yaw 增大。
- pitch 增大（θ̇ > 0）时 gy > 0，q 的 pitch 增大。
- 从 gyro 反推关节角速度：θ̇ = gy；ψ̇ = gz·cosθ − gx·sinθ，这里的 θ 取 **pitch 关节角** `theta[1]`（编码器角度乘 motor_sign 之后的值），不是从 q 反解的角。固件和虚拟云台的失控保护都用这两个量当作测量值。
- "yaw 轴即世界 z 轴"这一说法以底座水平为前提。底座倾斜时，yaw 关节轴不再是世界 z，上面的 ψ̇ 是绕 yaw 关节轴的分量。

**q：world→gimbal_link，顺序 wxyz，每个分量除以 32760。**

名称沿用 TF 的"world→gimbal_link"（父系 world，子系 gimbal_link）。数学含义是**机体到世界的旋转** R_world_gimbal：v_world = R(q) · v_gimbal。这与 `sensor_msgs/Imu.orientation` 的定义一致，也与 TF 中 parent=world、child=gimbal_link、rotation=q 的含义一致。

- 上电就绪时的 q 是由重力确定的 roll、pitch，加上 yaw = 0。
- 由 q 反解欧拉角（无 roll）：
  - ψ = atan2(2(wz + xy), 1 − 2(y² + z²))
  - θ = asin(2(wy − zx))

### 6.3 seq 与 stamp_us 的回绕

- seq 是 u16，每帧加 1，每 65536 帧回绕一次（500 Hz 下约 131 s）。没有判定复位时，丢帧数 = ((seq − prev_seq) mod 65536) − 1；实现上先把 (seq − prev_seq) 按 int16 解读，再取 max(0, ·− 1)。
- stamp_us 是 u32 微秒，2³² µs ≈ 4294.97 s ≈ 71.58 分钟回绕一次。
- 求间隔必须用无符号减法：`dt_us = (uint32_t)(stamp - prev_stamp)`。不要转成 int32 再比大小，也不要转 double 再减。
- 回绕的例子（黄金向量 `imu_wrap_a`、`imu_wrap_b`）：
  - 前一帧 seq = 65535，stamp_us = 4294966000；
  - 后一帧 seq = 0，stamp_us = 704；
  - 于是 dseq = 1，dt_us = 2000。
- 串口是字节流，不会乱序，也不会把同一帧投递两次，所以**没有**"重复或乱序就丢弃"这条规则。dt_us 为 0 不作特殊处理，但用 dt 做除法之前实现要自己防护。
- **MCU 复位检测**：以下任一条成立，判定 MCU 已复位：
  - stamp 倒退：`dt_us = (uint32_t)(stamp − prev_stamp) ≥ 0x80000000`；
  - seq 大幅回退：`(int16_t)(seq − prev_seq) ≤ −100`。

  判定之后：记一条日志；用这一帧重设 `prev_stamp` 和 `prev_seq`；同时重置滤波器（AHRS、低通、差分等）；这一帧**不计入丢帧，也不参与求 dt**，第一个 dt 从下一帧算起。这一帧本身照常是有效帧：帧层和应用层都必须 accept，也照常发布。
- 复位的例子（黄金向量 `imu_reset_a`、`imu_reset_b`，组 `reset_pair`）：
  - 前一帧 seq = 5000，stamp_us = 60000000；
  - 后一帧 seq = 0，stamp_us = 2000000；
  - 两帧都是 accept；第二帧判定为复位，丢帧数 0，没有 dt。
- 没有判定复位时，dt_us 大于 100 ms 视为断流，记一次日志，滤波器重置 dt。
- stamp_us 只用来排序和求 dt，**不映射成 ROS 时间**。ROS 时间戳的规则见第 7 节。

## 7. ROS 侧契约

bridge（课 1–3、固件联调）和课 5 答案（`gimbal_serial` + `class5_tracking`）必须发布完全一样的东西。

| 项 | 取值 |
|---|---|
| IMU 话题 | `/offboardlink/imu_data`，`sensor_msgs/msg/Imu` |
| IMU frame_id | `gimbal_link` |
| QoS | 可靠、volatile、keep_last 深度 200（与默认订阅和 SensorData 订阅都兼容） |
| TF | 父 `world`，子 `gimbal_link`，rotation = q，**translation = (0, 0, 0)** |
| TF 发布方 | **只能由 IMU 一方发布**：课 1–3 是 bridge，课 5 是学员节点。不得再有第二个节点发布以 gimbal_link 为子系的 TF |
| header.stamp | **主机接收时刻**：帧通过校验之后立刻取 `node->now()`。Imu 和 TF 用同一个值，不加偏移 |
| 频率 | 与固件一致，500 Hz；每收到一个有效 0x01 帧，发一条 Imu、发一个 TF |

字段映射：

- `orientation` = 归一化后的 q（x、y、z、w 按 ROS 的字段名放，注意协议顺序是 wxyz）。
- `angular_velocity` = gyro_raw × GYRO_SCALE，三轴都不取负。
- `linear_acceleration` = accel_raw × ACCEL_SCALE。
- 三个 covariance 都是 3×3 行主序，只有对角线，非对角线为 0：

| 字段 | 对角线 (xx, yy, zz) | 单位 | 依据 |
|---|---|---|---|
| `orientation_covariance` | (1.0e-4, 1.0e-4, 1.0e-2) | rad² | roll、pitch 由重力锁定；yaw 没有磁力计，会漂，给大一个量级 |
| `angular_velocity_covariance` | (1.0e-5, 1.0e-5, 1.0e-5) | (rad/s)² | BMI088 陀螺噪声加量化 |
| `linear_acceleration_covariance` | (1.0e-2, 1.0e-2, 1.0e-2) | (m/s²)² | 云台振动下的经验量级 |

- `imu_axis_sign`（bridge 参数）固定为 (1, 1, 1)，只作用于 accel 与 gyro，不作用于 q。
- 旧版 ling_vision bridge 的父系是 `odom`，平移来自 `getImuWorldTranslation`，时间戳带 `camera_imu_stamp_offset`。本套件三处都要去掉：父系改 `world`，平移恒 0，不加偏移。
- 其他约束：
  - `gimbal_edu_description` 的 URDF 根保持 `gimbal_link`，不改成 world，以免和 IMU 发布的 world→gimbal_link 冲突。
  - 课 1 的 Mahony 节点发布 world→`gimbal_link_ahrs`。虚拟云台的真值发布 world→`gimbal_link_gt`。这两个子系名字和 `gimbal_link` 都不同，不会冲突。
  - 主机侧 USB 和 UART 的收包会成簇到达，header.stamp 因此不是严格等间隔。需要精确 dt 的处理，直接读串口的课 5 用 stamp_us 差，走话题的课 1 用 header.stamp 差。

**bridge 参数（PC 侧 ROS 参数）**

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `imu_axis_sign` | 三个 ±1 | (1, 1, 1) | 见上，只作用于 accel 与 gyro |
| `yaw_sign` | ±1 | 1 | 只作用于转发 `/offboardlink/cmd_vel_gimbal` 的那一路：线上 yaw_omega = `yaw_sign` × angular.z |
| `pitch_sign` | ±1 | 1 | 同上：线上 pitch_omega = `pitch_sign` × angular.y |

`yaw_sign`、`pitch_sign` 不影响 IMU 的任何输出，语义见第 8 节。

**虚拟相机接口（虚拟云台发布）**

| 话题 | 内容 |
|---|---|
| `/virtual_gimbal/marker_corners` | `geometry_msgs/msg/PolygonStamped`，四个角点的像素坐标 |
| `/virtual_gimbal/camera_info` | 相机内参 |
| `/virtual_gimbal/image_raw` | 渲染图像 |
| `/virtual_gimbal/status` | 云台状态（含第 10 节的状态机状态） |
| `/virtual_gimbal/target_gt` | 目标真值，供评估使用 |

`marker_corners` 的约定：

- 像素坐标放在 `polygon.points[i].x`、`.y`，单位像素，`z = 0`。
- 角点顺序固定为 TL、TR、BR、BL（左上、右上、右下、左下）。
- `header.frame_id` 为 `camera_optical_frame`。
- 话题名固定，不随参数改变。

真实相机：话题名由 camera_setup 决定，本协议不固定。课 2 的学员答案必须把输入话题做成 ROS 参数，这样不改代码，只换参数就能在虚拟相机的话题和真实相机的话题之间切换。

**外参 yaml 格式**

相机相对云台的外参用普通 yaml（不是 ROS 参数文件格式，没有 `ros__parameters` 层），顶层两个键：

```
camera_xyz: [x, y, z]      # 单位 m
camera_rpy: [r, p, y]      # 单位 rad
```

文件位置：`gimbal_edu_description/config/default.yaml`（真机）和 `gimbal_edu_description/config/virtual.yaml`（虚拟云台）。

**时钟**

- 仿真、轨迹生成、评估时长一律用单调时钟（steady clock），不受系统时间调整影响。
- ROS 消息的 `header.stamp` 仍用系统时钟（`node->now()`），不改成单调时钟。
- WSL2 的系统时钟会跳变（例如休眠恢复、与 Windows 主机时间同步时），所以不能用系统时钟量时长。

## 8. 方向与三类参数

默认全部遵守 REP-103。一共有三类符号参数，各管一件事，不要混用。

| 参数 | 所在位置 | 取值 | 管什么 | 默认 |
|---|---|---|---|---|
| `yaw_dir`、`pitch_dir` | 固件 `PARAMS.md`；虚拟云台 yaml（同名） | ±1 | **协议约定**：线上 omega 的正方向与 REP-103 关节正方向的关系。固件内部 ω_cmd = dir · ω_wire。**本套件固定为 +1**，只有把协议挪作他用时才改；学员、讲师、调试都不改它 | +1（固定） |
| `yaw_sign`、`pitch_sign` | PC 侧 ROS 参数：bridge 转发 `cmd_vel_gimbal` 时、`class5_tracking`、示例节点 | ±1 | **PC 侧发送前乘的符号**：ω_wire = sign · ω_ctrl。这是"靠传参"的旋钮，用来补偿相机安装、控制律约定这类应用层差异；学员和讲师可以换讲法（例如让学员自己传参把方向调对，或者用它演示符号约定对闭环的影响） | +1 |
| `yaw_motor_sign`、`pitch_motor_sign` | **只在固件** `PARAMS.md` | ±1 | **只管硬件极性**：正的关节控制量对应的电机指令符号，同时决定编码器角度换成 REP-103 关节角的符号。用实机点动、对照陀螺符号来定 | 按实机 |

信号链（一个方向的完整闭环）：

```
PC 控制量 ω_ctrl
   × sign            (PC)
线上 ω_wire  ── 0x30 ──▶
   × dir             (固件 / 虚拟云台，固定 +1)   得到 REP-103 关节坐标系下的 ω_cmd
   限幅、软限位、斜坡  (第 10 节)           得到 ω_ref
   速度环：ω_ref 减去 ω_meas，ω_meas 由 IMU 得到（REP-103，与 dir、motor_sign 无关）
   × motor_sign      (仅固件)               变成电机的原始指令极性
电机
   编码器角度 × motor_sign 得到 REP-103 关节角 θ_joint，供软限位和失控保护使用
```

编码器符号：编码器角度换算到 REP-103 关节角时，符号取 `motor_sign`。前提是 GM6020 和 DM-J4310 的"正向指令"与"编码器正向"一致。实测不一致时，在固件 `PARAMS.md` 里为对应关节加 `enc_sign`，θ_joint = enc_sign · (θ_enc − θ_zero)，`enc_sign` 只影响这一步，不影响电机输出极性。

必须遵守的要点：

- IMU 的三项回传（accel、gyro、q）永远是 REP-103，不受 `*_dir`、`*_sign`、`*_motor_sign` 任何一项影响。改极性只改"指令怎么变成运动"，不改"运动怎么被测量"。
- 软限位与失控保护的判断量，必须和电机输出处于**同一符号系**：即经过 `*_dir` 和 `*_motor_sign`（或 `enc_sign`）变换之后的 REP-103 关节量。第 10 节的伪代码里，`theta` 与 `omega_meas` 都是这个符号系。
- 闭环总符号 = `sign · dir`，`dir` 固定为 +1，实际可调的只有 `sign`（PC 侧）和 `motor_sign`（硬件）。一次调试只动其中一个，不要同时动多个。
- 验证顺序（第 13 节）：先用 `*_motor_sign` 让电机极性正确（第 2 步）；再**验证**（不是确定）方向：正的 `yaw_omega` 应得到 gz > 0，正的 `pitch_omega` 应得到 gy > 0（第 3 步）。不符时查安装旋转和 `*_motor_sign`，**禁止改 `*_dir`**。`*_sign` 留 +1，只在学员自己的相机安装与控制律不同时才改。
- 课 5 端到端测试里，`yaw_sign` 取反和 `kp = 0` 两种情况必须被判失败。

## 9. 黄金向量

`protocol/golden_frames.txt` 由 `protocol/gen_golden.py` 确定性地生成：无随机数、无时间戳，两次生成的文件逐字节相同。重新生成并自检：

```
python3 protocol/gen_golden.py            # 生成并自检
python3 protocol/gen_golden.py --check    # 只重新解析已有文件并自检
```

四份实现（固件真实头文件加 HAL 桩、bridge、虚拟云台、`gimbal_serial`）都要读这个文件并全部通过；`reject` 用例里 reason 为 `bad_quat`、`non_finite` 的帧要求应用层丢弃。固件一侧编译的必须是真实头文件，不是另抄的一份。

### 9.1 文件格式

文本，UTF-8，LF 换行，`#` 开头的行是注释，空行忽略。每个用例一个块，一行一个字段，形如 `key: value`，以单独一行 `end` 结束，字段顺序固定：

| 字段 | 内容 |
|---|---|
| `case` | 用例名 |
| `dir` | `pc2mcu`（addr 0xEE）或 `mcu2pc`（addr 0xFF） |
| `func` | 帧里的功能号，如 `0x30` |
| `expect` | `accept` 或 `reject` |
| `reason` | reject 的原因说明，只作说明；accept 为 `-` |
| `group` | 成组用例的组名，没有则为 `-` |
| `hex` | 整帧字节，大写十六进制，空格分隔，含 AA 头和两个校验字节 |
| `decoded` | accept：空格分隔的 `key=value`；reject：`-` |
| `state` | 物理用例的关节状态：`psi=.. theta=.. psid=.. thetad=..`（rad、rad/s，`%.9g`）；没有则为 `-` |
| `note` | 英文说明 |

`decoded` 的键：

- 0x30：`yaw_omega`、`pitch_omega`（rad/s，float32 的精确值，`%.9g` 打印）。
- 0x01：`seq`、`stamp_us`，六个原始整数 `ax_raw ay_raw az_raw gx_raw gy_raw gz_raw`，四个 `qw_raw qx_raw qy_raw qz_raw`，然后是物理量 `ax ay az`（m/s²）、`gx gy gz`（rad/s）、`qw qx qy qz`。

比较规则：整数字段必须完全相等；浮点字段绝对误差不超过 1e-6。

解析提示：C 里用 `fgets` 读行，`strncmp` 匹配前缀；`hex` 用 `strtol(tok, NULL, 16)` 逐 token 转；`decoded` 按空格切分后再按 `=` 切分。Python 里 `line.partition(": ")` 就够了。

### 9.2 逐帧测试

对每个块，用被测实现的**帧解析加应用层解码**（不是黄金脚本里的函数）处理 `hex`：

- `expect: accept`：必须解出与 `decoded` 一致的数值。
- `expect: reject`：必须不产生输出（不触发回调，不刷新看门狗，不改变状态）。

bridge 的黄金测试必须走**真实的 accel 解码路径**：把 `hex` 喂给 bridge 里实际用来生成 `linear_acceleration` 的那段解码（去掉 `imu_payload_mode` 和 WorldTranslation 之后的固定比力解码，不取负），再和 `decoded` 里的 `ax ay az` 比较。不得绕过这段代码，直接拿 raw 乘系数。见第 12 节。

`reject` 用例覆盖：校验错（sum、sum_sum、数据位翻转）、len 不等于规定值（0、4、12、55 超过 60 字节上限，以及 0x01 的 24、28）、NaN（安静、信号）、±inf、head 不是 0xAA、未知功能号 0x31、addr 与功能号不匹配、0x01 的 q 全零（`rej_imu_q_zero`，reason 为 `bad_quat`）。其中 NaN、inf、q 全零的帧**校验是对的**，帧层能收下来，但应用层解码必须丢弃，所以仍是 reject。

### 9.3 分组用例

| group | 内容 | 检查 |
|---|---|---|
| `static` | 静止水平、静止低头 0.2 rad、带确定性噪声 | accel ≈ R(q)ᵀ·(0, 0, +g)，容差 0.1 m/s²；水平时 q 为单位四元数。水平和低头两帧带 state |
| `physical_yaw_pos` | 连续两帧 0x01，相隔 2 ms，绕 +z 正转 3 rad/s | 两帧 gz > 0；q 的 yaw 增大；Δψ ≈ gz / cosθ · dt，容差 10%；accel 与 q 一致；带 state |
| `physical_pitch_pos` | 连续两帧，pitch 以 +2 rad/s 增大 | 两帧 gy > 0；q 的 pitch 增大；Δθ ≈ gy · dt，容差 10%；带 state |
| `wrap_pair` | seq 65535→0，stamp_us 4294966000→704 | dseq = 1，dt_us = 2000（无符号减法）；**不是**复位 |
| `reset_pair` | seq 5000→0，stamp_us 60000000→2000000，两帧都是 accept | 第二帧判定为 MCU 复位（6.3 节）：不计丢帧，不参与求 dt，滤波器重置 |

`extreme` 组不做物理检查，只测高位与符号扩展：seq、stamp 的最高位置位，int16 取 −32768、32767，q 取 (−1, 0, 0, 0)。

**state 字段。** 带 state 的用例（`psi theta psid thetad`，即 yaw、pitch 关节角与角速度，rad、rad/s）给出这一帧背后的关节状态。虚拟云台的物理模型由 state 生成的 0x01 帧，必须与 `hex` 一致：每个 raw（六个 accel、gyro 与四个 q）相差不超过 ±1 LSB。这钉住的是**虚拟云台**的符号约定（gyro 符号、q 顺序、旋转方向）：任何一处搞反，这几个用例会直接失败。

这几个用例**不能**直接钉住固件的符号，因为固件的 accel、gyro、q 来自安装旋转和真实传感器，不是由 state 生成的。固件一侧由两处来钉：`host_test`（给定安装旋转和模拟传感器输入，检查上报的 accel、gyro、q 与本节的符号约定一致）和首次上电第 1 步的手转核对（第 13 节）。bridge、`gimbal_serial` 与学员答案靠逐帧解码和物理组检查（gz > 0、q 的 yaw 增大等）来保证解码方向正确。

### 9.4 流式测试

同一 `dir` 的全部块按文件顺序拼接，前面加垃圾字节 `55 00 AA 12 99`，逐字节喂给第 3.4 节的状态机。这里的输出指**状态机加应用层解码（isfinite 和 q 范数检查）之后的帧**，不是只过了帧层校验的帧。输出必须恰好是该 `dir` 里 `expect: accept` 的帧，顺序相同：

- 固件和虚拟云台的解析器（target_addr 0xEE）喂 `pc2mcu` 的块，应输出 8 个 0x30 帧。
- PC 侧解析器（target_addr 0xFF）喂 `mcu2pc` 的块，应输出全部 accept 的 0x01 帧（13 个）。NaN、inf、q 全零的帧能通过帧层，但被应用层解码丢弃，所以不在输出里。

生成脚本保证任何帧的内部都不含"AA 加 0xEE 或 0xFF"的伪帧头，所以整帧丢弃与回扫两种实现都能通过。

## 10. 安全语义

固件与虚拟云台必须同构：同样的输入序列，得到同样的状态转移和同样的动作时序。"CAN 掉线处理"和"CAN 反馈恢复后重建 θ"只适用于固件，见 10.4。

### 10.1 状态

| 状态 | 电机 | 进入条件 | 说明 |
|---|---|---|---|
| BOOT | 松弛 | 上电 | IMU 未就绪。**拒收 0x30**：不使能，指令内容不会被使用，不刷新看门狗（看门狗只在 ACTIVE、DECEL 里看）。有效帧的到达时刻仍会记入 `t_last_valid`，只用于下面的静默判定 |
| IDLE | 松弛 | IMU 就绪；或超时松弛；或（仅固件）CAN 掉线；或 FAULT 清除 | 带一个 `armed` 标志。未上膛（`armed = false`）时，有效指令只会重置静默计时，不使能；已上膛时，等第一帧有效指令 |
| ACTIVE | 使能，速度环 | 在 IDLE 已上膛，且上个周期之后收到了有效指令，并满足使能条件；或在 DECEL 收到有效指令 | ω_ref 以斜坡跟随指令 |
| DECEL | 使能，速度环 | ACTIVE 且 500 ms 没有有效帧 | 目标强制为 0，斜坡减速 |
| FAULT | 松弛，**锁存** | 失控保护或软限位越界 | 不需要断电。指令流静默 ≥ `cmd_timeout`（从进入 FAULT 起算）后回到 IDLE，并处于已上膛状态，见下面的重新上膛规则 |

"松弛"的含义：固件不输出力矩（GM6020 输出零电压，DM-J4310 发禁能命令），清除所有积分；虚拟云台的关节不再受驱动，角速度按 `tau_relax` 一阶衰减到 0。

有效指令的定义：通过帧层校验，且 len 等于 8，且 yaw_omega、pitch_omega 都是有限值。不满足任何一条的帧一律丢弃，不产生任何副作用。

**重新上膛（安全）。** 以下三种情况之后，要进入 ACTIVE，必须先观察到指令流静默至少 `cmd_timeout`（0.5 s），此后到达的**第一帧**有效指令才会触发使能：

- BOOT 之后（上电，或 IWDG 复位）；
- CAN 掉线回到 IDLE 之后（仅固件）；
- FAULT 之后。FAULT 也用这条规则清除，不再要求断电重启。

这样物理急停后恢复 24V、MCU 复位、故障清除之后，只要上位机还在持续发指令，电机就不会自己动起来；要让它动，必须停掉上位机程序至少 0.5 s，再重新发。

参数 `arm_requires_silence`，默认 true。设为 false 时不看指令流，只要求从下面的 `t_disarm` 起过了 `cmd_timeout`，仅用于台架调试。

静默从哪一刻起算：`t_disarm` 是上电（取 0）、CAN 掉线、进入 FAULT 这三类事件的发生时刻，`quiet_ref = max(t_last_valid, t_disarm)`，`now − quiet_ref ≥ cmd_timeout` 时上膛。上位机在 BOOT 期间就没有发指令的话，IMU 就绪时静默已经满足，一进 IDLE 就是已上膛状态。

### 10.2 时序

```
上电          BOOT：电机松弛，0x30 一律拒收，也不发 0x01；未上膛
IMU 就绪      → IDLE，开始发 0x01。若上电以来指令流静默 ≥ 0.5 s，此时已上膛；否则等到静默满 0.5 s 才上膛
已上膛后的首个有效 0x30  → 使能电机，ω_ref 从 0 起以斜坡上升到指令 → ACTIVE
之后每个有效帧 刷新看门狗，更新目标
最后一个有效帧之后 0.5 s（cmd_timeout）   → DECEL：目标置 0，斜坡减速到 0
最后一个有效帧之后 0.5 + 3.0 s（cmd_timeout + relax_after） → 松弛，回到 IDLE（静默已超过 0.5 s，所以是已上膛的）
DECEL 或 ACTIVE 期间只要再收到有效帧，回到 ACTIVE
FAULT         → 松弛；静默满 0.5 s 后回 IDLE 并已上膛，此后的第一帧有效指令才使能
CAN 掉线      → 松弛，回 IDLE，未上膛；静默满 0.5 s、反馈新鲜、θ 重建完成之后，第一帧有效指令才使能
```

- 看门狗只被有效帧刷新。零指令也算有效帧。
- 停止发送时，减速在 0.5 s 时开始，只要 `omega_max / omega_slew` 加上电机响应滞后不超过 0.2 s，就在 0.7 s 内停住（默认参数下约 0.13 s 加滞后）。验收时用这个数。
- 首次使能和从松弛重新使能，ω_ref 一律从 0 开始，所以不会因为积压的指令突然冲出去。
- 自动化测试（虚拟云台）要遵守同一条规则：等到第一帧 0x01 之后再发 0x30；如果之前发过指令，先停发 ≥ `cmd_timeout` 再发。

### 10.3 每个控制周期的处理

固件周期取 1 kHz；虚拟云台的仿真步长默认也是 1 kHz。下面的伪代码是规范。`theta[i]` 是 REP-103 关节角（编码器角度乘 `enc_sign`，默认等于 `*_motor_sign`，见第 8 节），`omega_meas[i]` 是 REP-103 关节角速度，下标 i：0 为 yaw，1 为 pitch：

- `omega_meas[1] = gy`。
- `omega_meas[0] = gz·cos(theta[1]) − gx·sin(theta[1])`。公式里的 θ 是 **pitch 关节角** `theta[1]`（编码器乘 motor_sign 之后的值），不是 yaw 角。"yaw 轴即世界 z 轴"这一说法以**底座水平**为前提；底座倾斜时，这个量是绕 yaw 关节轴的分量，不再是世界 z 分量。

`on_frame` 在串口接收的 ISR（虚拟云台是接收回调）里运行，**只做一件事**：原子地写 `cmd_target`、`t_last_valid` 和使能请求标志。它不读状态，不做状态转移，不调用 enable。状态转移和 enable 全部放在 tick 里做。

```
def on_frame(f):                          # ISR。f 已通过第 3 节的帧层校验
    if f.func != 0x30 or f.len != 8: return
    y = f32(f.data, 0); p = f32(f.data, 4)
    if not (isfinite(y) and isfinite(p)): return       # 直接丢弃
    w = [dir[0] * y, dir[1] * p]
    tgt = [clamp(w[i], -omega_max[i], omega_max[i]) for i in 0..1]          # omega 限幅
    atomic:                               # 关中断的临界区（两个 float 不是单条指令写入）
        cmd_target = tgt
        t_last_valid = now
        enable_request = true

def update_arm(t_last):                   # 只在 BOOT、IDLE、FAULT 里调用
    if armed: return
    ref = max(t_last, t_disarm) if arm_requires_silence else t_disarm
    if now - ref >= cmd_timeout: armed = true

def disarm():                             # 进入 FAULT、CAN 掉线时调用；上电时等价于 armed = false, t_disarm = 0
    armed = false; t_disarm = now

def tick(dt):
    atomic: req = enable_request; enable_request = false; t_last = t_last_valid; target_cmd = cmd_target
    if state == BOOT:
        relax_motors(); update_arm(t_last)
        if imu_ready(): state = IDLE
        return
    # ---- 仅固件：CAN 掉线，见 10.4。BOOT 以外的任何状态（IDLE、ACTIVE、DECEL、FAULT），每个 tick 都检查 ----
    if firmware and not can_fresh():                   # 任一电机反馈超过 can_loss_timeout（50 ms）
        relax_motors(); clear_integrators(); omega_ref = [0, 0]; theta_valid = [false, false]
        disarm()                                       # 每个 tick 都执行：静默计时要等反馈恢复之后才开始
        if state != FAULT: state = IDLE                # FAULT 状态下掉线仍保持 FAULT
        return
    if state == FAULT:
        relax_motors(); update_arm(t_last)
        if armed: state = IDLE            # FAULT 用同一条规则清除，不要求断电
        return
    if state == IDLE:
        if not armed: relax_motors(); update_arm(t_last); return
        if not (req and can_enable()): relax_motors(); return
        # 已上膛之后的第一帧有效指令：
        enable_motors(); omega_ref = [0, 0]; clear_integrators(); state = ACTIVE
    # ---- 以下是 ACTIVE / DECEL ----
    age = now - t_last
    if age > cmd_timeout + relax_after:                # 3.5 s
        relax_motors(); clear_integrators(); omega_ref = [0, 0]; state = IDLE; armed = true; return
    if age > cmd_timeout: state = DECEL; target = [0, 0]
    else:                 state = ACTIVE; target = target_cmd
    for i in 0..1:
        tgt = soft_limit(target[i], theta[i], lo[i], hi[i], limit_margin)     # 软限位
        omega_ref[i] = slew(omega_ref[i], tgt, omega_slew * dt)               # 斜坡
        if theta[i] > hi[i] + limit_fault_margin or theta[i] < lo[i] - limit_fault_margin:
            fault("limit"); return
        if runaway(i, dt): fault("runaway"); return                            # 失控保护
    run_velocity_loop(omega_ref, omega_meas)           # 输出经 motor_sign 变成电机指令

def can_enable():
    # 关节角已重建（theta_valid），都在限位区间加 limit_fault_margin 之内，
    # 且（仅固件）所有电机的 CAN 反馈新鲜（不超过 can_loss_timeout）
    return (theta_valid[0] and theta_valid[1]
            and all(lo[i] - limit_fault_margin <= theta[i] <= hi[i] + limit_fault_margin for i in 0..1)
            and (not firmware or can_fresh()))

def soft_limit(w, th, lo, hi, m):        # 越靠近限位越慢，限位处为 0；反向不受限
    if w > 0: return w * clamp((hi - th) / m, 0, 1)
    if w < 0: return w * clamp((th - lo) / m, 0, 1)
    return 0

def slew(cur, tgt, step):                # 每周期最多变化 step
    return cur + clamp(tgt - cur, -step, step)

def runaway(i, dt):
    e = abs(omega_ref[i] - omega_meas[i])
    if history_len(i) < err_grow_dt: grow = false                # 历史不足 err_grow_dt，不判定为增长
    else: grow = (e > err_thresh) and (e > e_at(i, err_grow_dt))   # e_at：err_grow_dt 之前的 e 值
    hold[i] = hold[i] + dt if grow else 0
    return abs(omega_meas[i]) > runaway_factor * omega_max[i] or hold[i] > err_grow_time

def fault(reason):
    relax_motors(); clear_integrators(); omega_ref = [0, 0]; state = FAULT; disarm()   # 锁存，并记录日志
```

要点：

- **ISR 与 tick 的分工**：`on_frame` 里没有状态判断。BOOT、FAULT、未上膛的 IDLE 期间收到的指令，`cmd_target` 照写但不会被使用，使能请求在 tick 开头被取走并丢弃；`t_last_valid` 照常更新，因为它同时是静默判定的依据。
- **omega 限幅**：合法但过大的指令被夹到 ±`omega_max`，不丢弃。
- **软限位**：pitch 和 yaw **都要有**。yaw 的线缆要穿过 yaw 轴，所以 yaw 也不能无限转。限位内侧的速度不受影响，越靠近限位速度越慢，限位处降为 0；`theta` 超出限位 `limit_fault_margin` 就进入 FAULT。松弛状态下手转云台不触发 FAULT，但满足 `theta` 在限位区间加 `limit_fault_margin` 之内才允许使能。
- **yaw 多圈与关节角重建（仅固件）**：yaw 限位必须满足 `yaw_limit_max + limit_fault_margin < π` 且 `yaw_limit_min − limit_fault_margin > −π`（默认 2.8 + 0.1 = 2.9 < π），这样合法关节角落在 (−π, π] 内，单圈绝对值没有歧义。上电时，以及每次 CAN 反馈从丢失恢复之后，第一帧新鲜反馈到达时，都用单圈绝对值按零点偏移重建：`theta[i] = wrap_pi(enc_sign[i] · (enc_single_turn[i] − enc_zero[i]))`，值域 (−π, π]，然后置 `theta_valid[i] = true`。运行中可以继续用多圈计数累加，但重建完成之前 `theta_valid` 为假，`can_enable()` 不放行；CAN 掉线时把 `theta_valid` 置假。pitch 同样重建（限位远小于 π，没有歧义）。虚拟云台没有编码器和 CAN，`theta_valid` 恒为真。
- **失控保护**：满足以下任一条就松弛并锁存：
  - 测得的 |ω| 超过 1.5 倍 `omega_max`；
  - 跟踪误差 |ω_ref − ω_meas| 大于 `err_thresh`，且比 `err_grow_dt` 之前的值更大，这个状态持续超过 `err_grow_time`（100 ms）。历史记录不足 `err_grow_dt` 时（刚使能，或误差历史刚被清空），不判定为误差增长。固件和虚拟云台必须一致。

  这条保护主要抓"电机极性接反"造成的正反馈。参数选择要让正常斜坡的稳态滞后 `omega_slew × τ` 小于 `err_thresh`，否则需要同时调大 `err_thresh`。
- FAULT 状态下 0x01 照常发送，0x30 不会使能。指令流静默满 `cmd_timeout` 之后回 IDLE，见 10.1。

### 10.4 CAN 掉线（仅固件）

虚拟云台没有 CAN，不实现这一节，也不需要留接口。

- BOOT 以外的任何状态下，任一电机的反馈不新鲜（超过 50 ms），每个 tick 都执行 disarm，静默计时要在反馈恢复之后才开始。理由：急停期间上位机持续发指令，恢复供电后不得自动使能。按掉电处理：清积分，ω_ref 清零，松弛，回到 IDLE，`disarm()`，`theta_valid` 置假。**不是**锁存故障。伪代码见 10.3 节 tick 里 BOOT 分支之后的那一块。
- FAULT 状态下掉线仍保持 FAULT：同样每个 tick 执行 relax 和 disarm，但不转成 IDLE；反馈恢复后按 10.1 的规则，指令流静默满 `cmd_timeout` 才清除 FAULT。
- 回到 IDLE 之后，必须先观察到指令流静默 ≥ `cmd_timeout`（重新上膛），所有电机反馈都新鲜（不超过 50 ms）并且 θ 已重建，此后到达的第一帧有效指令才重新使能，ω_ref 从 0 起步走斜坡。这样物理急停之后恢复 24V 供电，电机不会冲。
- 仅固件：DM 反馈 ERR 码的高 4 位，0 表示禁能、1 表示使能、8~E 表示故障。ACTIVE/DECEL 状态下，如果 ERR 不等于 1 且已经过了 `dm_enable_hold`，就按 CAN 掉线处理。
- 固件开 IWDG，卡死时复位到 BOOT。
- DM-J4310 自身的 CAN 超时在 DM 调参工具里设置；GM6020 在 CAN 掉线时的表现要实测，取决于电机固件，记录在 `PARAMS.md`。

### 10.5 与虚拟云台的对应

虚拟云台除电机与 CAN 以外必须逐条实现上面的状态和时序：

- 解析用 3.4 节的状态机，只订阅 0x30。
- `on_frame`、`tick` 的结构与上面相同：接收回调只写三个变量，状态转移在仿真步里做；含重新上膛规则（`arm_requires_silence`）。
- 关节按一阶响应跟踪 ω_ref：dω/dt = (ω_ref − ω)/`tau`；松弛时按 `tau_relax` 衰减。
- 就绪延时 `imu_ready_delay` 之前处于 BOOT，不发 0x01，拒收 0x30。
- 参数名与固件 `PARAMS.md` 同名（见第 11 节）。
- 可选的测试钩子（不属于协议）：给虚拟云台配 `yaw_motor_sign = -1`，模拟极性接反，失控保护应在约 100 ms 内触发 FAULT；之后停发指令 ≥ 0.5 s 再发，应能重新使能，不用重启节点。

## 11. 参考参数

名字固定，固件 `PARAMS.md` 与虚拟云台 yaml 同名同义。表里是虚拟云台的默认值，**都是初值，实机未验证**；固件的默认值在 `PARAMS.md`，首次上电用更保守的 `omega_max`。

| 参数 | 默认 | 含义 |
|---|---|---|
| `yaw_dir`、`pitch_dir` | +1、+1 | 第 8 节，固定 +1 |
| `omega_max_yaw`、`omega_max_pitch` | 2.0、2.0 rad/s | omega 限幅 |
| `omega_slew` | 15 rad/s² | 斜坡（加速和减速共用） |
| `cmd_timeout` | 0.5 s | 看门狗，超时后开始减速；也是重新上膛要求的静默时长 |
| `arm_requires_silence` | true | BOOT、CAN 掉线、FAULT 之后要先静默 ≥ `cmd_timeout` 才能使能（10.1 节） |
| `relax_after` | 3.0 s | 超时之后再过这么久松弛 |
| `yaw_limit_min`、`yaw_limit_max` | −2.8、+2.8 rad | 相对零点的关节角软限位，须满足 yaw_limit + `limit_fault_margin` < π |
| `pitch_limit_min`、`pitch_limit_max` | −0.5、+0.35 rad | 低头为正，所以上仰是负值 |
| `limit_margin` | 0.20 rad | 软限位的减速带宽度，须满足下面的校核式 |
| `limit_fault_margin` | 0.10 rad | 越出限位这么多进入 FAULT |
| `runaway_factor` | 1.5 | 失控保护：|ω_meas| 超过 factor × `omega_max` |
| `err_thresh` | 0.5 rad/s | 失控保护：跟踪误差门限 |
| `err_grow_dt` | 0.02 s | 失控保护：误差增长的比较间隔 |
| `err_grow_time` | 0.1 s | 失控保护：误差持续增大的时间 |
| `can_loss_timeout` | 0.05 s | 仅固件，第 10.4 节 |
| `yaw_enc_zero`、`pitch_enc_zero` | 按实机标定 | 仅固件（`PARAMS.md`），编码器单圈绝对角的零点偏移，rad（10.3 节） |
| `yaw_enc_sign`、`pitch_enc_sign` | 默认等于 `*_motor_sign` | 仅固件，只有 6020 或 DM4310 的正向指令与编码器正向不一致时才需要单独写出（第 8 节） |
| `tau` | 0.02 s | 仅虚拟云台，关节一阶响应时间常数 |
| `tau_relax` | 0.2 s | 仅虚拟云台，松弛后的衰减常数 |
| `imu_ready_delay` | 1.0 s | 仅虚拟云台，模拟 IMU 到温标定 |
| `imu_rate_hz` | 500 | 0x01 发送频率，可调 |

默认值必须同时满足下面三条校核（改任何一个参数之后重算）：

1. **减速带要装得下刹车距离**：以 `omega_max` 冲向限位时，斜坡减速的距离加上一阶响应的滞后距离，必须在 `limit_margin` 之内：

   `omega_max² / (2 · omega_slew) + omega_max · tau ≤ limit_margin`

   默认值代入：2.0² / (2 × 15) + 2.0 × 0.02 = 0.1333 + 0.04 = 0.1733 rad ≤ 0.20 rad，成立（原来的 0.15 rad 不成立，所以改为 0.20）。固件里的 `tau` 换成实测的电机响应滞后，须在 `PARAMS.md` 里重算。
2. **正常斜坡不触发失控保护**：`omega_slew · tau < err_thresh`，即 15 × 0.02 = 0.30 rad/s < 0.5 rad/s，成立。
3. **yaw 单圈可重建**：`yaw_limit + limit_fault_margin < π`，即 2.8 + 0.10 = 2.90 < 3.1416，成立（10.3 节）。

## 12. 与源码的核实记录与差异

已核实的事实：

- 两仓校验算法一致：`OffboardLink.h:60-68`（ling_vision）与 `MessageBase.h:76-84`（SwordInfantry）都是 sum 与 sum_sum，覆盖 `buf[0 .. data_len+3]`，两者取 &0xFF。
- 两仓 `IMUData` 都是 `MessageBase(0x01, 26)`，字段偏移相同。
- 地址：ling_vision 一侧 `TARGET = 0xFF`、`SOURCE = 0xEE`（PC）；Sword 一侧 `TARGET = 0xEE`、`SOURCE = 0xFF`（MCU）。与本文一致。
- 0x30、0x06 在两仓都没有被使用（见 `func_id_table.md`）。

**契约按正确语义写，不照抄下面这些源码问题**：

| 位置 | 问题 | 契约的做法 |
|---|---|---|
| Sword `msgs/IMUData.h:49-55` | q 用整数除以 32760，结果只剩 {−1, 0, 1} | 浮点除法（6.1 节） |
| Sword `msgs/IMUData.h:88-98` | float 直接赋给 uint8，越界是未定义行为 | 先 `lroundf` 转 int16，限幅，再拆字节 |
| Sword `msgs/IMUData.h:35` | stamp 由 int 左移 16 位拼出，高位置位时有符号溢出 | 用无符号类型拼接 |
| Sword `msgs/IMUData.h:37-47`、ling_vision `IMUData.h:89-93` | 由 int 拼出的值直接赋给 int16 | 先按 u16 组装，再转 i16 |
| ling_vision `IMUData.h:114-116` | gyro x、y 取负来凑轴 | 固件统一安装旋转，PC 侧不取负（5.1 节） |
| ling_vision `compatibility.hpp:37-38`（`imuLinearAcceleration`） | 对 accel 的 x、y 取负（`-raw[0]*scale, -raw[1]*scale`） | 删除取负。accel 按比力原样输出：`raw × ACCEL_SCALE`，三轴都不取负（6.2 节） |
| ling_vision `IMUData.h:110`、`compatibility.hpp` 的 `decodeImuPayloadWords`、`imu_payload_mode`、`WorldTranslation` 分支 | `imu_payload_mode` 参数把 accel 槽位解释成"世界平移"，`imuLinearAcceleration` 在该模式下返回 0 | 三处都删除：参数 `imu_payload_mode`、枚举与 `WorldTranslation` 分支、`decodeImuPayloadWords` 里的分模式逻辑。固定按比力解码；bridge 的黄金测试必须走这条真实的 accel 解码路径（9.2 节） |
| ling_vision `IMUData.h:125-133` | TF 父系是 odom，平移来自 `getImuWorldTranslation` | 父系 world，平移 0（第 7 节） |
| ling_vision `IMUData.h:104,124` | 时间戳加 `camera_imu_stamp_offset` | 不加偏移 |
| 两仓 `processByte` | `rec_buffer[60]` 写入前没有检查 `index`，找不到帧头时下标持续增长会越界；Sword 版校验失败不清 index，解析卡死；`sub_list[0xFF]` 下标越界 | 3.4 节状态机：所有分支都有界，校验失败整帧丢弃 |
| ling_vision `IMUData.h:158-181` | `packData` 布局与 `decode` 不一致（遗留未用路径） | 不使用；PC 侧不发 0x01 |
| Sword `ComputerVisionControlTask.cpp:241-255` | 回传的 accel、gyro 取的是 BMI088 Raw 指针，与 q 不在同一个系 | 5.1 节的统一安装旋转 |

（以上行号是本次阅读时的位置；固件实现者按内容而不是行号定位。）

## 13. 实机首次上电流程

第 8 节和第 10 节引用的"首次上电"就是这一节。步骤按顺序做，不要跳。整个流程用较小的 `omega_max`（例如 0.5 rad/s），云台周围留出空间，手边有 24V 总开关。

1. **只上 C 板供电（5V 或 USB），24V 断开，电机不上电。** 接好 USB-TTL。等到温和标定完成：固件在 IMU 就绪之前不发 0x01，所以**上电后要等到温和标定完成，约 N 秒后才有 IMU 数据**（N 见 5.2 节，以 `PARAMS.md` 为准）。收到第一帧 0x01 后，**用手转云台**核对符号：
   - 从上往下看逆时针转 yaw：gz > 0，q 的 yaw 增大；
   - 抬起再低头（pitch 增大）：gy > 0，q 的 pitch 增大；
   - 静止时 accel ≈ (0, 0, +g)。

   不符合就查固件的安装旋转（5.1 节），不要在 PC 侧取负来凑。
2. **上 24V，确定 `*_motor_sign`。** 在很低的 `omega_max` 下发很小的指令点动，让"正的关节控制量"对应"陀螺读数为正"：电机动起来的方向与陀螺符号相反，就翻转对应的 `*_motor_sign`。同时确认编码器的正向与正向指令一致（6020 与 DM4310 各自确认）；不一致就在 `PARAMS.md` 里单独写 `enc_sign`（第 8 节）。失控保护会在极性接反时约 100 ms 内进 FAULT，这是预期行为。
3. **验证方向（不是确定方向）。** `*_dir` 固定为 +1，`*_sign` 为 +1。发正的 `yaw_omega`，应得到 gz > 0；发正的 `pitch_omega`，应得到 gy > 0。不符合时，去查安装旋转（步骤 1）和 `*_motor_sign`（步骤 2），**禁止改 `*_dir`**。
4. **看门狗与软限位。** 发一段正弦或恒定指令，然后直接停发：云台应在 0.7 s 内停住，3.5 s 后松弛。缓慢把 pitch、yaw 推向软限位，速度应逐渐降到 0，不越界。
5. **重新上膛。** 云台运动时把 24V 断开再恢复（物理急停）：恢复 24V 后电机不动；停掉上位机程序 ≥ 0.5 s 再重启，才会使能。同样的现象也出现在 MCU 复位和 FAULT 之后。
