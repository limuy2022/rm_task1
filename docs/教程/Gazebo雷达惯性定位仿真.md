# Gazebo 雷达、IMU 与 Small Point-LIO 仿真

链路：Gazebo Fortress三维雷达和IMU → Livox消息 → Small Point-LIO原始核心
→ 世界系点云、IMU body位姿 → 生产建图节点。真值只用于评价，不输入定位算法。

## 1 算法与环境

普通包 `src/team43_lio_backend` 直接包含 Small Point-LIO ROS 2 提交
`131fb586dc609521e7acf11a9c833121b2d49141`，直接编译未修改的核心，项目接口
负责消息、固定外参和入口审核。没有旧算法、补丁、回退目标或模式兼容分支。
来源和许可证见[接口说明](../../src/team43_lio_backend/README.md)及
[迁移验证记录](../记录/SmallPointLIO迁移验证.md)。

Livox驱动和消息单独归档于 `archives/livox_ros_driver2.tar.gz`，保留原始内容与
许可证；镜像不启动硬件驱动。所有构建产物位于容器临时目录，不用pip覆盖系统库。

## 2 标准场景与自定义场景

标准回归使用 `tools/gazebo/lio_room_regression.sdf`：8×6m房间、地面、天花板、
两箱体，平台初始IMU高度0.7m。它与用户编辑的 `lio_room.sdf` 和Blender模型分开。
视觉联合的features场景在标准房间中增加箱体。

平台通过力/力矩运动，保证IMU测到实际加减速，不模拟轮胎和悬架。
雷达默认16线、每线180个方向、40Hz快照，四个快照按真实时间组成约10Hz
Livox帧；IMU为200Hz。这不是MID360非重复扫描，也未模拟材质、噪声和时延。

雷达到IMU固定外参读取 `small_point_lio_mid360.yaml`；接口先把点统一到IMU，
核心使用单位外参，世界系原点为初始IMU。仿真配置副本中把IMU参数换算为m/s²，
体素与地图栅格参数不会改写实车配置。核心使用IMU测量更新，没有input/output切换。

`RM_LIO_WORLD` 可选择源SDF。自定义场景可用于RViz观察，但标准房间的墙面覆盖率、
边界和盒体回波检查不适用，脚本在评价阶段会明确拒绝，而不是认定算法定位失败。
报告记录源文件与生成场景的SHA256；对新模型验收需要另外定义几何评价。

## 3 运行

先确认子模块已检出。仓库根目录执行：

```bash
docker build --target gazebo-sim -t rm-task1-gazebo-sim .
docker run --rm -v "$PWD":/workspace -w /workspace \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=1 \
  -e IGN_PARTITION=rm_task1_gazebo_01 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e RM_LIO_WORLD=/workspace/tools/gazebo/lio_room_regression.sdf \
  rm-task1-gazebo-sim bash -c '
    source /opt/ros/humble/setup.bash &&
    colcon --log-base /tmp/clog build --build-base /tmp/cb \
           --install-base /tmp/ci --symlink-install &&
    source /tmp/ci/setup.bash &&
    python3 -B tools/simulate_gazebo_lio.py \
      --output-dir build/offline/gazebo_lio_01 --record-bag'
```

输出目录必须不存在，重复运行换新编号。软件渲染不传显卡或宿主显示器；不要
增加 `/dev/dri`。默认墙钟超时600秒，请求倍率0.5；CI用0.25和900秒，不代表实时
吞吐通过。`--scan-lines 32` 可改光束数，但当前通过结果仅覆盖16线。

## 4 验收与产物

等待有效定位后静止3秒，沿约1.05m边长方形运动，最后静止3秒。
位置RMSE须小于15cm、最大误差35cm、最大yaw误差0.15rad、静止误差5cm、
估计返回误差20cm；墙体覆盖率大于85%，墙外未知比例大于98%。
还检查原始回波、IMU与物理运动一致性、输入时间/接收完整性、PGM与ROS地图一致性，
以及所有进程正常退出。没有放宽原精度和地图门槛。

输出目录包含 `result.json`、`overview.png`、`trajectory.csv`、实际 `world.sdf`、
实际算法参数、节点日志、`maps/43_slam_map.pgm/yaml` 和可选 `sensor_bag/`。
报告的 `point_lio_image_build` 保存源码来源、固定提交和二进制SHA256。
这些均为仿真证据，不作为真实雷达数据或官方评分提交。

最新结果和测试边界以[迁移记录](../记录/SmallPointLIO迁移验证.md)为准。
真实MID360、32线、旋转雷达外参和NUC实时性能仍需各自验收。
