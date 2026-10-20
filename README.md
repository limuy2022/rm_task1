# RoboMaster 赛题一

[![CI](https://github.com/limuy2022/rm_task1/actions/workflows/ci.yml/badge.svg)](https://github.com/limuy2022/rm_task1/actions/workflows/ci.yml)

当前已有独立图像跟踪程序：ArUco → 图像偏差 → P/PI/PD/PID → 两轴角速度 → C++ 串口桥。它不等待地图或底盘定位；默认运动关闭。云台通信已确认 ttyUSB0、IMU 轴 [1,1,1]；新相机 K/D 已更新。真实转动效果仍待 NUC 试验。

按[云台跟踪教程](docs/教程/云台跟踪.md)将源码包装到自己的独立 NUC 目录后运行：

```bash
bash "$HOME/ws_championship/src/47-visual-tracking-20261018-r3/run_here.bash" target_id:=0
```

默认只看不动；真实串口模式加 `with_bridge:=true port:=/dev/ttyUSB0`，由人在参数终端开启 motion_enabled。停止在启动终端 Ctrl+C，或设置 motion_enabled=false。先做 3 秒单轴方向试验，再做两轴。参数默认值、调整现象和保存方法见[调参检查清单](docs/教程/上机调参检查清单.md)。

代码统一保留在 src（含 C++ 串口包 42-gimbal_serial）；配置/launch 在对应包内，离线工具在 tools，独立教程在 docs/教程，原理在 docs/原理。没有把代码再复制进文档目录。Point-LIO、目标坐标/预测、点云地图、建议速度和九列提交链已有实现，整车尺寸、真实误差、最终地图与正式 180 秒端到端流程仍待联调。主办方驾驶底盘，我们自行估计底盘位置。

本机完成实际写出器 180 秒/1800 行基线：漏拍 0，读者可见延迟平均 1.73ms、最大 31.62ms；没有 ROS/DDS，也不代表 NUC 验收。点云地图、预测对比与其它离线结果见 docs/验证 的 JSON；原始实验产物不提交 Git。

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
- **[测试运行指南](docs/教程/测试运行指南.md)**：验证体系分层、容器内运行命令、新增测试清单与换标定影响。
- **[云台控制与串口原理](docs/原理/云台控制与串口.md)**：识别→PD→0x30→下位机全链路原理与源码索引。
- 记录与模板：[G坐标核查历史](docs/记录/G坐标核查_历史.md)、[现场记录表模板](docs/记录模板/现场记录表.md)；离线验证产物在 docs/验证。

## 本批实际完成

| 功能 | 修改与验证 |
|---|---|
| ArUco/PnP | 修复detectMarkers返回值错误；角点消息绑定真实ID与图像stamp；候选PnP、丢失/过期处理；实际OpenCV合成图验证 |
| 定位/时间/预测 | 传感器三维位姿含杠杆臂换到底盘中心；保留源stamp；按图像时刻查询完整TF，滤波留在观测时间；按tick采样当前和100ms预测 |
| 云台几何/控制 | 完整两轴几何正反解、同stamp底盘+IMU派生关节估计；控制限位不再使用惯性yaw充当机械角；控制/搜索默认关闭 |
| 地图/方向 | 保留真实地图尺寸、分辨率和origin yaw；处理旋转地图；输入无效/失联/全阻挡时不凭空给穿墙方向 |
| CSV/评分 | 九列逐拍flush；单写者、重启去重、损坏文件拒绝；正式评分补回底盘40%权重与归一化；缺参考不伪造总分 |
| 串口 | 生产C++解析/串口类经黄金帧、噪声、分片、OS伪终端及ASan/UBSan验证；桥会话/MCU重启使epoch变化 |

10月17日控制追踪增量：修复高度速度被低通滞后放大的问题；控制回归18项、适配/门控13项、既有控制单元39项通过。没有改变放行开关和增益；未部署、未控制真实电机。world控制仍有装配/高度同步/机械可达路径限制，见转向教程第11节。

当前验证统一在 ROS2 Humble 容器中复跑（与 NUC 同源环境，apt OpenCV 4.5.4；`Dockerfile` 见仓库根）：

```bash
docker build -t rm-task1-humble-test .
docker run --rm -v "$PWD":/workspace -w /workspace \
  --user "$(id -u):$(id -g)" -e HOME=/tmp rm-task1-humble-test bash -c '
    source /opt/ros/humble/setup.bash &&
    colcon --log-base /tmp/clog build --build-base /tmp/cb \
           --install-base /tmp/ci --symlink-install &&
    source /tmp/ci/setup.bash &&
    colcon --log-base /tmp/clog test --build-base /tmp/cb \
           --install-base /tmp/ci --packages-select gimbal_serial_42 &&
    colcon --log-base /tmp/clog test-result --test-result-base /tmp/cb --verbose &&
    python3 -B -m pytest -q src --import-mode=importlib &&
    python3 -B tools/verify_all_offline.py'
```

测试环境即运行环境：被测的 rclpy / OpenCV / numpy / pyserial 等全部来自 humble 镜像的 apt 包，不用 pip 覆盖（**不要在NUC用pip覆盖ROS的OpenCV/numpy**）。串口伪终端验证所需的 C++ 编译器镜像内已有；脚本本身不安装依赖、不假装通过。日志及合成产物放在被 Git 忽略的`build/offline/`。

公共 NUC 不用 pip 覆盖 ROS 的 OpenCV/numpy，不全量构建，不改 .bashrc，不读取主办方保护目录，不向 /judge/* 发布测试消息。保留现有源码版权及许可证声明。
