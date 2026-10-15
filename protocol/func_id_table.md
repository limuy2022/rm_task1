# OffboardLink 功能号表

协议版本 v1.4（以 protocol/PROTOCOL.md 为准），2026-09-30。本表列出两个源仓库已占用的功能号、本套件新增的 0x30，以及保留不动的 0x01。

数据来源与核实方式：

- ling_vision：`src/utils/robo-bridge/include/robo_bridge/msgs/*.h`，逐个 grep `MessageBase(0x..`。
- SwordInfantry：`Libraries/Offboard/msgs/*.h`，以及 `Modules/AutoAimer/CustomMsgs/`、`Vehicle/SwordInfantry/Control/CustomMsgs/` 下的自定义消息。
- 帧长 = data_len + 6（AA、addr、func、len 共 4 字节，加 2 字节校验）。
- 方向一栏：带"已核实"的是在收发代码里看到了 subscribe/publish 调用；带"推测"的只是消息在头文件里有定义，两个仓库的当前工程都没有使用，按命名推测。

第一批 haiku 扫描只覆盖了 `Libraries/Offboard/msgs/`，漏掉了 Sword 自定义消息目录里的 0x44、0xbb，也把 0xbd 误记成了 ling_vision 的 `Target`。下表已按源码更正。

## 1. 本套件使用

| 功能号 | 名称 | 方向 | data_len | 帧长 | 状态 |
|---|---|---|---|---|---|
| 0x01 | IMUData | MCU→PC（addr 0xFF） | 26 | 32 | 保留。字节布局与两个仓库一致，语义按 PROTOCOL.md 第 5 节修正 |
| 0x30 | GimbalOmegaCommand | PC→MCU（addr 0xEE） | 8 | 14 | 本套件新增。`yaw_omega f32 | pitch_omega f32`，rad/s |

0x30 和 0x06 在两个仓库里都没有被任何 `MessageBase(...)` 使用，grep 结果为空。选 0x30，是为了避开两个仓库现有的 0x01–0x05、0x10–0x17、0x20、0x40–0x44、0x50–0x52、0x60、0x78、0xa0、0xaf、0xbb–0xbf。

本套件的固件和 PC 侧只订阅或发布上面两个功能号。其他功能号在接收端一律按"未知功能号"丢弃，不报错。

## 2. 两仓已占用的功能号

| 功能号 | 名称 | data_len | 帧长 | 方向 | ling_vision | SwordInfantry | 备注 |
|---|---|---|---|---|---|---|---|
| 0x01 | IMUData | 26 | 32 | MCU→PC，已核实 | 有 | 有 | 两仓布局相同。Sword 的 pack/decode 有 bug（见 PROTOCOL.md 第 12 节） |
| 0x02 | ChassisVelYawRateCommand | 17 | 23 | PC→MCU，推测 | 无 | 有 | 底盘指令，云台套件不用 |
| 0x04 | GimbalConstantAimShootCommand | 14 | 20 | PC→MCU，已核实（ling_vision 发布） | 有 | 有 | 云台指令，两仓同号同长；本套件不用 |
| 0x05 | ChassisVelCommand | 13 | 19 | PC→MCU，推测 | 无 | 有 | |
| 0x10 | GameState | 4 | 10 | MCU→PC，推测 | 无 | 有 | 裁判系统 |
| 0x11 | RobotsHealth | 32 | 38 | MCU→PC，推测 | 无 | 有 | 裁判系统 |
| 0x12 | RobotStatus | 29 | 35 | MCU→PC，推测 | 无 | 有 | 裁判系统 |
| 0x13 | PowerHeatData | 14 | 20 | MCU→PC，推测 | 无 | 有 | 裁判系统 |
| 0x14 | Buff | 1 | 7 | MCU→PC，推测 | 无 | 有 | 裁判系统 |
| 0x15 | RobotHurt | 2 | 8 | MCU→PC，推测 | 无 | 有 | 裁判系统 |
| 0x16 | ShootData | 7 | 13 | MCU→PC，推测 | 无 | 有 | 裁判系统 |
| 0x17 | BulletRemaining | 6 | 12 | MCU→PC，推测 | 无 | 有 | 裁判系统 |
| 0x20 | RefereeWarning | 2 | 8 | MCU→PC，推测 | 无 | 有 | 裁判系统 |
| 0x40 | ReferenceDataMini | 14 | 20 | 推测 | 无 | 有 | |
| 0x41 | RobotAroundInformation | 5 | 11 | 推测 | 无 | 有 | |
| 0x44 | ChassisPowerData | 26 | 32 | 推测 | 无 | 有（`Vehicle/.../Control/CustomMsgs/`） | 与 0x01 同长，靠 func 区分 |
| 0x50 | MinipcHeartbeat | 3 | 9 | PC→MCU，推测 | 无 | 有 | |
| 0x51 | Heartbeat | 10 | 16 | MCU→PC，已核实（Sword 发布） | 无 | 有 | |
| 0x52 | HeartbeatReply | 2 | 8 | PC→MCU，推测 | 无 | 有 | |
| 0x60 | CommandData | 14 | 20 | 推测 | 无 | 有 | |
| 0x78 | BoosterSpeed | 18 | 24 | 推测 | 无 | 有 | |
| 0xa0 | GimbalPIDDebug | 34 | 40 | MCU→PC，已核实 | 有 | 有（`Modules/AutoAimer/CustomMsgs/`） | |
| 0xaf | AutoAimerFeedback | 14 | 20 | MCU→PC，已核实 | 有 | 有（同上） | |
| 0xbb | Target | 48 | 54 | PC→MCU，推测 | 无 | 有（同上） | Sword 的 0xbb 是 Target |
| 0xbc | AutoAimInfo | 9 | 15 | MCU→PC，已核实（Sword 发布） | 有（定义了，bridge 未订阅） | 有（同上） | |
| 0xbd | Target（ling_vision）/ TargetWithId（Sword） | 49 | 55 | PC→MCU，已核实（ling_vision 发布） | 有，名称为 `Target` | 有，名称为 `TargetWithId` | 两仓同号同长，名称不同 |
| 0xbf | AutoAimerCommand | 33 | 39 | PC→MCU，已核实 | 有 | 有（同上） | |

说明：

- 同一 data_len 出现在多个功能号上是允许的（0x01 与 0x44 都是 26），接收端必须先看 func，再核对 len。
- 两个仓库里都没有出现过 0x30 和 0x06。将来若并入其他工程，先在这张表里登记。

## 3. 保留与自由区

| 范围 | 用途 |
|---|---|
| 0x01、0x30 | 本套件使用 |
| 0x06、0x31–0x3F | 保留，暂不使用。黄金向量用 0x31 作为"未知功能号"反例，所以 0x31 永远不会被分配 |
