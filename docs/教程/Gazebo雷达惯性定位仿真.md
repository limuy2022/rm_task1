# Gazebo 原始雷达、IMU 与 Point-LIO 仿真

本试验在 Gazebo Fortress 中产生三维雷达回波和 IMU，实际运行仓库导航压缩包
里的 Point-LIO，再把其已配准点云与位姿送给生产 `cloud_mapping_node`。
Gazebo 真值只用于评价误差，不替代定位输入。这比[轻量接入仿真](定位建图仿真.md)
多覆盖了雷达、IMU 到定位输出这一段。

## 1 工具与源码来源

- 环境：根 `Dockerfile` 的可选 `gazebo-sim` 目标；默认 `test` 目标仍使用原来的测试环境。
- 仿真：Gazebo Fortress / Ogre2 / EGL 无窗口渲染，`ros_gz_bridge` 负责 ROS 2 通信。
- 定位：`archives/robomaster_navigation.tar.gz` 中的 `point_lio`，在镜像构建副本上加入平面有效性修复；归档及保留的原始源码不变。见[CI 修复记录](../记录/Point-LIO稀疏场景失败归因.md#8-云端-ci-回归修复)。
- 消息：同一压缩包中的 `livox_ros_driver2`；编译驱动以取得真实消息定义，但不启动雷达驱动。
- 评价：`tools/simulate_gazebo_lio.py`，输出解析匹配的真值、定位轨迹、地图与 JSON。

源码在镜像内部 `/opt/lio_sim` 中编译，构建产物在镜像 `/tmp` 中产生并清理；
不会把第三方包混进本仓库 `src/`。运行时仓库仍挂载为 `/workspace`，自己的
colcon build/install/log 仍放 `/tmp`。不在宿主安装 ROS，不使用 pip。
压缩包的 SHA256 写进每轮报告，保留原源码版权和许可证。

## 2 场景与传感器模型

场景文件是 `tools/gazebo/lio_room.sdf`：8×6m 封闭房间、地面、2.5m 高的墙与
天花板，以及两个不对称箱体。刚性平台的 IMU 在世界 z=0.7m 处开始，初始
朝向为零。平台关闭重力，通过专用插件把速度目标转为力/力矩，再由物理引擎
积分运动；不直接设置位置或速度，确保 IMU 能测到加减速。第一轮不模拟轮胎/悬架。

雷达为规则 **16线**、每线180个方向、垂直角度 [-0.65,0.65]rad、40Hz 快照；
IMU 为200Hz。这里不是 MID360 非重复扫描模式，也没有声称覆盖材质、反射率、
遮挡噪声、时延和真实电机动态。

四个瞬时快照组成一个约10Hz的 `livox_ros_driver2/msg/CustomMsg`。每个快照
内部的点使用同一采样时间，快照之间的 `offset_time` 来自 Gazebo 原始时间戳。
不把一张快照随意补成连续扫描，也不使用真值去变换/配准雷达点。

雷达相对 IMU 的平移同时从生产 `point_lio_mid360.yaml` 读取到场景和仿真
参数文件。仿真副本默认使用 `use_imu_as_input=true`、`scan_line=16`、
`acc_norm=9.81`、同步换算的 `satu_acc=29.43` 和15cm体素。该版本直接用原始
加速度判断饱和，单位改为m/s²时不能沿用g输入的阈值3；仿真在启动前检查静止
重力不会被误判为饱和。`--imu-mode input|output` 可切换算法支持的
IMU 模式，`--scan-lines 16|32` 同时修改实际光束数和预处理参数。
这些改动只属于仿真副本，没有修改实车默认的 IMU 输出模式或其他参数。

IMU 原点初始化为 `camera_init`，故该场景地面高度为 -0.7m；这个数仅来自
场景定义，不能当成实车的地面测量。真值评分只应用已知初始坐标关系，没有
事后用轨迹拟合、ICP 或整体旋转对齐来消掉定位误差。

## 3 一条命令运行

先在仓库根目录构建可选镜像：

```bash
docker build --target gazebo-sim -t rm-task1-gazebo-sim .
```

使用软件渲染运行，无需传入宿主显示器或显卡。输出目录必须不存在，再次运行
换新编号；开始前确认子模块已检出。

```bash
docker run --rm -v "$PWD":/workspace -w /workspace \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=1 \
  -e IGN_PARTITION=rm_task1_gazebo_01 -e LIBGL_ALWAYS_SOFTWARE=1 \
  rm-task1-gazebo-sim bash -c '
    source /opt/ros/humble/setup.bash &&
    source /opt/lio_sim/install/setup.bash &&
    colcon --log-base /tmp/clog build --build-base /tmp/cb \
           --install-base /tmp/ci --symlink-install &&
    source /tmp/ci/setup.bash &&
    python3 -B tools/simulate_gazebo_lio.py \
      --output-dir build/offline/gazebo_lio_01 --imu-mode input --record-bag'
```

不要给这条软件渲染命令增加 `--device /dev/dri`。双显卡机器可能先枚举到
容器内不可用的 EGL 设备，使 Ogre2 在初始化阶段崩溃；硬件加速要另行验证。
默认 `--timeout 600` 是墙钟秒，`--real-time-factor 0.5` 是请求的运行倍率，
实际速度取决于 CPU 与渲染性能，不作为实时吞吐验收。

`--record-bag` 可省略；启用时录的是**仿真数据**，不能当真实雷达数据交付。
脚本捕获失败并写日志，退出时只停止自己启动的进程。

## 4 运动与验收

先等待 Point-LIO 输出足够的有效位姿，再保持静止3秒。随后沿四条直线和四次
90°转向走一个约1.05m边长的方形，最后静止3秒。速度和角速度有0.5秒余弦
过渡，避免不必要的瞬时跳变。实际路径以 Gazebo 的模型位姿为真值，不拿
命令积分的轨迹代替真值。

| 检查 | 工程回归门槛 |
|---|---|
| 平台确实运动 | 真值平面轨迹至少一个方向跨度 >0.8m |
| 真值回到起点 | 末尾静止窗口均值与起点距离 <5cm |
| 三维位置 | RMSE <15cm，最大误差 <35cm |
| 朝向 | 最大 yaw 误差 <0.15rad |
| 启动静止漂移 | 最大位置误差 <5cm |
| 估计返回误差 | 末尾静止窗口位置均值距起点 <20cm |
| 建图墙体 | 真值墙面10cm邻域覆盖率 >85% |
| 地图外部 | 离墙超过20cm的外部采样点未知比例 >98% |
| 静止 IMU | 加速度模均值与9.81之差 <0.1 |
| 运动 IMU | 水平加速度峰值 >0.4m/s²，防止只有运动位置而没有对应加速度 |
| 测量物理一致性 | IMU 加速度与真值运动的差值 RMSE <0.1m/s²；原始回波到真实场景表面的距离 P95 <4cm |
| 时间与输出 | 快照时间单调、批内有真实时间跨度、无倒序、地图 PGM 与 ROS 消息一致 |

这些门槛是运行前定义的合成场景回归标准，不是官方评分或真实 MID360 精度
保证。`passed` 要求所有检查通过；只看“有位姿”或地图外形不足以验收。

世界暂停后，脚本排空输入，再将本次地图节点的发布定时器切到墙钟以发布最后
快照，之后保存和评价。测量时间戳保持不变，保证地图消息、PGM 与报告一致。

## 5 本机实际结果（2026-10-20）

使用根 Dockerfile、软件渲染、施力控制和上述相同误差门槛，两种配置各完成一轮：

| 仿真配置 | 三维位置 RMSE | 最大位置误差 | 返回起点估计误差 | 墙体覆盖率 | 验收 |
|---|---:|---:|---:|---:|---|
| 16线 / IMU input（默认） | 7.16cm | 15.36cm | 1.14cm | 97.59% | 16项检查及正常退出均通过 |
| 32线 / IMU output | 7.18cm | 22.19cm | 0.97cm | 97.59% | 16项检查及正常退出均通过 |

后续归因发现上述早期output试验遗漏了饱和阈值单位换算，静止重力被误判为饱和；
该行仅保留为历史运行数据，不能作为完整IMU output验收。仿真已修正单位转换，
新对照同时发现接收队列丢样和局部平面检查缺口，详见[失败归因记录](../记录/Point-LIO稀疏场景失败归因.md)。

默认配置取得401条定位输出、约465万回波、8078条IMU消息和4038条真值消息。
原始回波到真实表面的距离 P95 为1.47cm，IMU 加速度一致性 RMSE 为0.0244m/s²。
源地图导出、时间顺序和所有进程正常退出也通过。

施力模型修正后的早期16线 / IMU output试验得到16.53cm位置RMSE，未达到15cm
门槛；保留该模式供对比，没有放宽门槛或修改正式运行参数。这些是单场景、无
额外噪声的各一次结果，不能据此宣称实车精度、模式优劣或 MID360 鲁棒性。

初版“直接设置速度”的控制方式没有提供对应平移加速度，因此已替换为实际
施力插件。增加测量物理一致性检查后，可以在验收中发现这类模型问题。

## 6 产物

- `result.json`：是否真正得到 Gazebo 数据和 Point-LIO 位姿、误差、检查、来源 SHA256、`point_lio_image_build` 修复清单、局限和退出码。
- `overview.png`：源地图，蓝线为真实房间边界，绿线为 Gazebo 真值，红线为 Point-LIO 输出。
- `trajectory.csv`：按定位消息原始时间插值匹配的真值、估计、位置与 yaw 误差。
- `gazebo_truth.csv`、`point_lio_odometry.csv`、`imu.csv`：独立记录的原始数值。
- `maps/47_slam_map.pgm/yaml`：生产建图节点写出的 `camera_init` 源地图。
- `world.sdf`、`point_lio.yaml` 和节点日志：实际使用的参数与失败证据。
- 可选 `sensor_bag/`：包含仿真时钟、雷达、IMU、真值和 Point-LIO 输出的完整 rosbag2 目录。

这些文件都位于 gitignore 的 `build/offline/`，不提交。该试验不运行云台视觉、
九列评分器，也不发布 `/judge/*` 或真实电机命令。

工具机制参考：[Gazebo 传感器](https://gazebosim.org/docs/fortress/sensors/)、
[Gazebo 无窗口渲染](https://gazebosim.org/api/sim/7/headless_rendering.html)、
[Point-LIO](https://github.com/hku-mars/Point-LIO)。
