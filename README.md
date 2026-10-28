# RoboMaster 赛题一

[![CI](https://github.com/limuy2022/rm_task1/actions/workflows/ci.yml/badge.svg)](https://github.com/limuy2022/rm_task1/actions/workflows/ci.yml)

当前已有独立图像跟踪程序：ArUco → 图像偏差 → P/PI/PD/PID → 两轴角速度 → C++ 串口桥。它不等待地图或底盘定位；默认运动关闭。云台通信已确认 ttyUSB0、IMU 轴 [1,1,1]；新相机 K/D 已更新。真实转动效果仍待 NUC 试验。

按[云台跟踪教程](docs/教程/云台跟踪.md)将源码包装到自己的独立 NUC 目录后运行：

```bash
bash "$HOME/ws_championship/src/43-visual-tracking/run_here.bash" target_id:=0
```

默认只看不动；真实串口模式加 `with_bridge:=true port:=/dev/ttyUSB0`，由人在参数终端开启 motion_enabled。停止在启动终端 Ctrl+C，或设置 motion_enabled=false。先做 3 秒单轴方向试验，再做两轴。参数默认值、调整现象和保存方法见[调参检查清单](docs/教程/上机调参检查清单.md)。

代码统一保留在 src（含 C++ 串口包 team43_serial_bridge）；配置/launch 在对应包内，离线工具在 tools，独立教程在 docs/教程，原理在 docs/原理。没有把代码再复制进文档目录。Point-LIO、目标坐标/预测、点云地图、建议速度和九列提交链已有实现，整车尺寸、真实误差、最终地图与正式 180 秒端到端流程仍待联调。主办方驾驶底盘，我们自行估计底盘位置。

Gazebo 场景、网格模型与仿真辅助模块由 [`team43_gazebo`](src/team43_gazebo/README.md) 包安装；SDF 使用 `package://` 引用模型，source 工作区后自动配置资源搜索路径。运动插件仍只在可选 `gazebo-sim` 镜像中构建。

本队队号为 **43**。自有软件包统一使用 `team43_<功能>`，源码目录、ROS 包名、Python 模块名保持一致；部署 ZIP 内也保留相同目录名。第三方子模块和 vendor 源码保留上游名称及许可证。

| 软件包 | 职责 |
|---|---|
| `team43_common` | 公共几何、时钟、通信协议和工具 |
| `team43_interfaces` | ROS 消息接口 |
| `team43_perception` | ArUco 识别与 PnP |
| `team43_estimation` | 目标状态估计与预测 |
| `team43_control` | 云台跟踪与控制 |
| `team43_serial_bridge` | C++ 实机串口桥 |
| `team43_gimbal_io` | Python 仿真串口收发与实机消息适配 |
| `team43_localization` | 底盘定位转换与云台运动学 |
| `team43_lio_backend` | Small Point-LIO 接入 |
| `team43_grid_mapper` | 点云建图与栅格地图输出 |
| `team43_chassis_advisor` | 底盘方向与速度建议 |
| `team43_judge_io` | 评分与 CSV 提交 |
| `team43_dataio` | 运行记录与轨迹标注 |
| `team43_debug` | 仿真调试与误差监测 |
| `team43_bringup` | 启动配置与系统集成 |

跟踪话题使用 `/team43_tracking/*`，建图话题使用 `/team43/*`；默认提交文件为 `43.csv` 和 `43_map.pgm/yaml`。迁移后应从源码重新构建并加载新安装空间，NUC 上仍按教程只构建自己的独立源码包。

验证命令与产物位置见[测试运行指南](docs/教程/测试运行指南.md)；既有测试成绩见[回归历史](docs/记录/测试与控制回归历史.md)。原始实验产物不提交 Git。

---

## 文档导航

- **[云台跟踪教程](docs/教程/云台跟踪.md)**：把源码包装到独立 NUC 目录运行；配套[上机调参检查清单](docs/教程/上机调参检查清单.md)（参数默认值、现象与保存方法）。
- **[云台台架试转步骤](docs/教程/云台台架试转步骤.md)**：bearing_bench / bearing_hardware 台架与整车试转手册。
- **[相机标定](docs/教程/相机标定.md)**与**[坐标外参与整车接入](docs/教程/坐标外参与整车接入.md)**：内参标定、外参测量与整车接入。
- **[底盘定位与点云建图](docs/教程/底盘定位与点云建图.md)**：Point-LIO + cloud_mapping 建图流程。
- **[定位建图仿真](docs/教程/定位建图仿真.md)**：已知房间与轨迹，经真实 ROS 2 进程验证建图、底盘中心转换和比赛地图导出；尚不覆盖 Point-LIO 原始输入。
- **[Gazebo 雷达惯性定位仿真](docs/教程/Gazebo雷达惯性定位仿真.md)**：三维模拟雷达与 IMU 驱动真实 Point-LIO，按独立 Gazebo 真值评价定位与地图。
- **[Gazebo 视觉定位联合仿真](docs/教程/Gazebo视觉定位联合仿真.md)**：真实渲染 ArUco，经 PnP、Point-LIO 和 TF 验证目标坐标，测试观测延迟、断流及旧帧重放。
- **[评分接口测试](docs/教程/评分接口测试.md)**：180 秒提交链三终端测试。
- **[测试运行指南](docs/教程/测试运行指南.md)**：验证体系分层、容器内运行命令、专项验收与换标定影响。
- **[云台控制与串口原理](docs/原理/云台控制与串口.md)**：识别→PD→0x30→下位机全链路原理与源码索引。
- 记录与模板：[G坐标核查历史](docs/记录/G坐标核查_历史.md)、[现场记录表模板](docs/记录模板/现场记录表.md)；验证结论在 docs/验证，原始产物在被 Git 忽略的 build/offline。

## 已实现能力

| 功能 | 修改与验证 |
|---|---|
| ArUco/PnP | 修复detectMarkers返回值错误；角点消息绑定真实ID与图像stamp；候选PnP、丢失/过期处理；实际OpenCV合成图验证 |
| 定位/时间/预测 | 传感器三维位姿含杠杆臂换到底盘中心；保留源stamp；按图像时刻查询完整TF，滤波留在观测时间；按tick采样当前和100ms预测 |
| 云台几何/控制 | 完整两轴几何正反解、同stamp底盘+IMU派生关节估计；控制限位不再使用惯性yaw充当机械角；控制/搜索默认关闭 |
| 地图/方向 | 保留真实地图尺寸、分辨率和origin yaw；处理旋转地图；输入无效/失联/全阻挡时不凭空给穿墙方向 |
| CSV/评分 | 九列逐拍flush；单写者、重启去重、损坏文件拒绝；正式评分补回底盘40%权重与归一化；缺参考不伪造总分 |
| 串口 | 生产C++解析/串口类经黄金帧、噪声、分片、OS伪终端及ASan/UBSan验证；桥会话/MCU重启使epoch变化 |

开发验证统一使用根 `Dockerfile` 的 ROS 2 Humble 容器（apt OpenCV 4.5.4），不在宿主安装 ROS，也不用 pip 覆盖系统库。构建镜像、全量验证及失败排查统一见[测试运行指南](docs/教程/测试运行指南.md)。

公共 NUC 不用 pip 覆盖 ROS 的 OpenCV/numpy，不全量构建，不改 .bashrc，不读取主办方保护目录，不向 /judge/* 发布测试消息。保留现有源码版权及许可证声明。
