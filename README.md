# RoboMaster 赛题一

[![CI](https://github.com/limuy2022/rm_task1/actions/workflows/ci.yml/badge.svg)](https://github.com/limuy2022/rm_task1/actions/workflows/ci.yml)

第 43 队，ROS 2 Humble。程序包括 ArUco/PnP、云台图像 PID 跟踪、Small Point-LIO 底盘定位、点云建图、目标预测与九列 CSV 提交。底盘由主办方驾驶，本程序输出底盘方向和速度建议。

**比赛当天从[比赛启动与收尾](docs/教程/比赛启动与收尾.md)开始。**当前实机组合是 `visual_tracking` + `lio` + `cloud_mapping` + `processing(submission:=true)`；相机、雷达沿用现场驱动。运动默认关闭，须由人在参数终端显式开启。`competition.launch.py` 是历史集成入口，不作为当前实机启动命令。

已确认串口 `/dev/ttyUSB0`、IMU 轴 `[1,1,1]`，新相机 K/D 已写入配置。真实跟踪效果、最终装车外参、地图精度和 NUC 正式 180 秒端到端流程仍待现场验收；离线与仿真结果不代表实机成绩。

已探明的现场条件与原始证据集中在[现场已知前提](docs/记录/现场已知前提.md)。定位接收层默认自动识别 Livox PointCloud2 / CustomMsg，比赛命令沿用记录中的实际话题。

## 文档导航

| 要做什么 | 阅读入口 |
|---|---|
| 比赛开机、检查、启动、开赛和收尾 | **[比赛启动与收尾](docs/教程/比赛启动与收尾.md)** |
| 上传完整源码、构建、加载环境、Git 交接 | [整体部署](docs/教程/整体部署.md) |
| 首次试转、方向和停止验收 | [云台跟踪](docs/教程/云台跟踪.md) |
| 调增益、保存与恢复参数 | [上机调参检查清单](docs/教程/上机调参检查清单.md) |
| 相机内参、装车外参和本次上电对齐 | [相机标定](docs/教程/相机标定.md)、[坐标外参与整车接入](docs/教程/坐标外参与整车接入.md) |
| 地面确认、源地图与定位排查 | [底盘定位与点云建图](docs/教程/底盘定位与点云建图.md) |
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
