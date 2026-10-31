# RoboMaster 赛题一

[![CI](https://github.com/limuy2022/rm_task1/actions/workflows/ci.yml/badge.svg)](https://github.com/limuy2022/rm_task1/actions/workflows/ci.yml)

第 43 队，ROS 2 Humble。程序包括 ArUco/PnP、云台图像 PID 跟踪、Small Point-LIO 底盘定位、点云建图、目标预测与九列 CSV 提交。底盘由主办方驾驶，本程序输出底盘方向和速度建议。

题目文字保存在[赛题原文](docs/赛题原文.md)，部署依据见[校内赛环境页](https://blog.secaumene.com/kit/campus-contest-env.html)。

按题目与环境核对的已复现缺陷、部署缺口及待实测风险见[代码审查记录](docs/记录/题目与校内赛环境代码审查.md)。

**比赛当天从[比赛启动与收尾](docs/教程/比赛启动与收尾.md)开始。**先运行`bash tools/start_match.bash`；裁判告知目标后执行`bash tools/set_target.bash ID`，确认跟踪后由裁判启动客户端。入口直接启动全链，不等待人工confirmed开关、旧epoch匹配或源图就绪；默认自动连续跟踪。本轮尺寸及授权近似已接入随包vehicle_field.yaml；个人vehicle43.yaml可覆盖默认值。启动前保持底盘静止、相机水平朝车头，以首组同步数据自动对齐。相机由赛事方启动。只读调试传`motion_enabled:=false`。

已确认串口 `/dev/ttyUSB0`、IMU 轴 `[1,1,1]`，新相机 K/D 已写入配置；用户确认现场可持续瞄准ArUco，实物码黑框外边长7.5cm（不含白边）。现用跟踪配置与覆盖范围需归档；最终装车几何、地图精度和NUC正式180秒端到端流程仍待现场验收。离线与仿真结果不代表实机成绩。

已探明的现场条件与原始证据集中在[现场已知前提](docs/记录/现场已知前提.md)。雷达链固定使用Livox PointCloud2，比赛命令沿用记录中的实际话题；LIO世界系统一命名为`lio_world`。

## 文档导航

| 要做什么 | 阅读入口 |
|---|---|
| 比赛开机、检查、启动、开赛和收尾 | **[比赛启动与收尾](docs/教程/比赛启动与收尾.md)** |
| 一次完成剩余实物验收、记录与交接 | **[实验室统一验证](docs/教程/实验室统一验证.md)** |
| 上传完整源码、构建、加载环境、Git 交接 | [整体部署](docs/教程/整体部署.md) |
| 不知道具体怎样摆姿态、开终端、采样和交接 | [剩余数据现场测量](docs/教程/未确认参数说明与确认方法.md#附录-b-剩余数据的现场测量与交接) |
| 查未确认参数的含义、实测方法和保存位置 | [未确认参数说明与确认方法](docs/教程/未确认参数说明与确认方法.md) |
| 首次试转、方向和停止验收 | [云台跟踪](docs/教程/云台跟踪.md) |
| 调增益、保存与恢复参数 | [上机调参检查清单](docs/教程/上机调参检查清单.md) |
| 相机内参、装车外参和本次上电对齐 | [相机标定](docs/教程/相机标定.md)、[坐标外参与整车接入](docs/教程/坐标外参与整车接入.md) |
| 理解雷达建图、实机操作和各份地图的去向 | [底盘定位与点云建图](docs/教程/底盘定位与点云建图.md) |
| 隔离域合成数据提交演练 | [评分接口测试](docs/教程/评分接口测试.md) |
| 开发测试、专项验证与仿真 | [测试运行指南](docs/教程/测试运行指南.md) |
| 查配置归属或控制原理 | [配置文件归属](docs/教程/配置文件归属.md)、[云台控制与串口](docs/原理/云台控制与串口.md) |

备选 PnP 视线控制只见[台架试转步骤](docs/教程/云台台架试转步骤.md)，不与当前图像控制混开。
实验过程和历史结论在 `docs/记录/`，现场填写[记录表](docs/记录模板/现场记录表.md)。Gazebo 资源说明见 [team43_gazebo](src/team43_gazebo/README.md)，各仿真操作从测试指南进入。

## 代码结构

源码在 `src/`，配置和 launch 在对应包内，离线工具在 [tools](tools/README.md)。第三方子模块 `gimbal-edu-tools/` 和 vendor 保留上游名称与许可证，克隆后执行 `git submodule update --init --recursive`。

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

跟踪话题为 `/team43_tracking/*`，源地图话题为 `/team43/slam_map`，比赛地图话题为 `/map/grid`。正式文件为 `~/ws_championship/output/43.csv`、`43_map.pgm`、`43_map.yaml`。

开发验证统一在根 `Dockerfile` 的 Humble 容器运行，命令见[测试运行指南](docs/教程/测试运行指南.md)。公共 NUC 不用 pip 覆盖 OpenCV/numpy，不在公共工作区全量构建，不改 `.bashrc`，不读取主办方保护目录，不向 `/judge/*` 发布测试消息。
