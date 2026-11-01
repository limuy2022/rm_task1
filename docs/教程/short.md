详细操作见：[比赛启动与收尾](/home/limuy/桌面/code/rm_task1/docs/教程/比赛启动与收尾.md)。

下午不需要重新建图、加载上午地图定位，也不需要在 RViz 中点击初始位置。**程序负责复制地图文件，LIO 负责定位，实时点云负责约束建议速度，裁判的消息负责触发评分输出。**

### 1. 开赛前：更新程序和检查文件

把最新代码上传到 NUC 的本队目录，并按[部署指南](/home/limuy/桌面/code/rm_task1/docs/教程/整体部署.md)重新编译安装。不要直接使用个人电脑上的编译产物。

进入本队目录：

```bash
cd ~/ws_championship/src/43-MyDuo
```

确认选定的地图文件都在：

```bash
ls -lh maps/43_map.pgm maps/43_map.yaml
```

`maps/` 只需要这两个文件。它们是第一张上午地图，已经配好文件名和 YAML 图片引用。

同时确认上午的 `start_mapping.bash`、独立 Livox 驱动或另一套 LIO 已经停止，避免重复启动。

### 2. 保持静止，启动比赛程序

先让**底盘静止、相机水平朝向车头**，然后运行：

```bash
bash tools/start_match.bash
```

这个终端一直保持运行。

程序会自动：

- 把两份地图复制到 `~/ws_championship/output/`，后续不会用运行时地图覆盖。
- 启动本队 Livox 驱动、LIO、云台串口桥及视觉处理链。
- 使用第一组同步姿态完成云台世界参考对齐。
- 默认开启运动和连续跟踪，但目标 ID 初始为 `-1`，等待裁判通知。

相机驱动由赛事方提供并启动。本队入口不会替代相机驱动。

**初始化期间先不要移动底盘或转动云台。** 等待 LIO 位姿输出，以及终端出现“按启动水平朝前参考完成世界对齐”提示。

### 3. 裁判通知目标 ID 后，设置目标

主终端继续运行，另开一个终端：

```bash
cd ~/ws_championship/src/43-MyDuo
bash tools/set_target.bash 0
```

这里的 `0` 换成裁判通知的 ID。

脚本会设置检测器和 PnP 节点的目标，并读回确认。成功后应显示两个节点的目标 ID 一致。

然后现场确认：

- 相机画面中确实有指定的 ArUco 码。
- 云台能够持续瞄准该码。
- 主终端没有反馈中断、限位或复位故障。

**无需再手动开启 `motion_enabled` 或 `continuous`，两者已经默认开启。** 如果出现故障锁停，不要只看“目标设置成功”就通知裁判开赛，应先处理故障。

### 4. 确认准备完成，再由裁判启动客户端

通知裁判前，至少确认：

1. LIO 和底盘定位持续输出。
2. 启动世界参考对齐已完成。
3. 指定 ID 跟踪正确，云台没有故障。

不需要等待地图重定位，也不需要等待两线模型生成。

裁判启动客户端后，系统发布本轮 `/judge/replay_start` 和 `/judge/tick`。程序自动：

- 以本轮 `replay_start` 对应时刻的底盘中心建立原点，以当时车头方向为 x 轴、左侧为 y 轴。
- 每收到一个 tick，向 CSV 写入一行九列数据并立即刷新。
- 时间列使用该拍的 stamp，按秒保留六位小数。

**不要在裁判开赛时重启程序，也不要自行发布 judge 消息。**

### 5. 比赛中检查输出，保持程序运行

另一个终端可以查看：

```bash
wc -l ~/ws_championship/output/43.csv
```

重复执行时，行数应随裁判节拍增加。若持续以 10Hz 发拍，每秒大约增加 10 行；具体以实际收到的拍数为准。

地图文件应已在启动时复制完成：

```bash
ls -lh ~/ws_championship/output/43_map.pgm \
       ~/ws_championship/output/43_map.yaml
```

三份交付文件是：

```text
~/ws_championship/output/43.csv
~/ws_championship/output/43_map.pgm
~/ws_championship/output/43_map.yaml
```

比赛过程中：

- 不关闭主终端、不重新运行第二份程序。
- 不手动删除或编辑 CSV。
- 不额外启动建图程序覆盖地图。
- 底盘仍由主办方控制；程序只输出建议速度。
- 实时点云无效、过期或通路受阻时，建议速度会归零，CSV 仍继续写。

复制的地图保留上午坐标；CSV 使用下午正式开赛坐标，两者的用途已经分开。

### 6. 比赛结束后停止程序

等裁判停止发拍，确认 CSV 行数不再增加，再在主终端按：

```text
Ctrl+C
```

正常退出会补发云台零速指令。确认现场云台实际停止后，备份：

- `output/` 中的三个交付文件。
- 本轮 `records/match_XXXXXXXX/` 日志目录。

如果需要中途立即关闭云台运动，在另一个终端执行：

```bash
cd ~/ws_championship/src/43-MyDuo
source tools/env.bash
ros2 param set --no-daemon \
  /team43_tracking/visual_servo_node motion_enabled false
```

新一轮应重新启动程序，重新保持静止完成参考对齐，再设置目标。
