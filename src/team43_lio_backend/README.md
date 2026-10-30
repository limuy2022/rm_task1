# Small Point-LIO 定位包

这是普通的ROS 2 Humble包。Small Point-LIO的固定版本源码直接包含在
`vendor/small_point_lio/`，来源与提交号见 `upstream.json`，原始MIT许可证保留。
CMake直接编译算法核心和本包的ROS接口，不需要下载、解压或安装到固定目录。

## 比赛机编译与启动

在比赛机原本已加载ROS 2 Humble和Livox驱动环境的终端，进入本项目工作区：

```bash
colcon build --base-paths src --packages-select team43_lio_backend \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 launch team43_lio_backend lio.launch.py
```

只编译本包，不构建整个项目。不启动雷达驱动或电机。若编译提示找不到
`livox_ros_driver2`，先加载比赛机已有Livox工作区的 `install/setup.bash`。
不使用 `sudo`、不自动安装依赖，不修改 `.bashrc`。

默认输入为 `/livox/lidar`、`/livox/imu`，输出为
`/red_standard_robot1/aft_mapped_to_init` 和 `/red_standard_robot1/cloud_registered`。
按现场 record3 的 Livox PointCloud2 输入启动：

```bash
ros2 launch team43_lio_backend lio.launch.py \
  lidar_format:=auto \
  lidar_topic:=/red_standard_robot1/livox/lidar \
  imu_topic:=/red_standard_robot1/livox/imu
```

参数在 `config/small_point_lio_mid360.yaml`；也可用 `params_file:=/实际路径/配置.yaml`
指定自己的副本。默认 `acc_norm=1.0` 和 `satu_acc=3.0` 对应g单位；m/s²输入需要
同时换算。雷达到IMU的固定外参必须符合实际安装。

传感器接收默认使用 best-effort QoS。仅在输入发布端支持可靠传输时，才可在
配置副本中设置 `imu_qos_reliable: true`；Gazebo 回归会显式启用此选项，
避免共享 CPU 调度抖动导致 IMU 不可重传丢样。该选项不改变实机默认值或雷达 QoS。

`lidar_format` 默认为 `auto`，按配置话题的实际发布者类型选择 `custom` 或 `pointcloud2`；
也可手动指定后二者。只识别消息格式，仍使用你指定的 `lidar_topic`。
自动模式用墙钟轮询，驱动晚启动或 `/clock` 尚未发布时继续等待；连续两次发现结果一致才锁定。
使用展开并 remap 后的话题名，不把订阅者声明的类型误当发布者类型。
识别前不积压 IMU；无发布者时等待，同一类型恢复后继续。
选定前遇到不支持类型/多类型冲突会等待修正；选定后发现类型冲突或改变则停止向核心送入
点云和 IMU，日志提示重启定位并重建地图/坐标关联，不把不同流接入已有状态。
`lio.launch.py` 与 bringup 的 `lio_only.launch.py` 均支持此参数。
PointCloud2 按 [Livox 官方发布实现](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp)
读取 x/y/z（float32）、tag（uint8）和 timestamp（float64，绝对纳秒→秒），不再加 header 时间。
按 fields 的偏移解析，支持非对齐、端序及行填充；缺字段、类型/长度错误拒收整帧并报错，
非有限坐标/时间与无效回波跳过，不以帧时间替代缺失的逐点时间。强度和线号不参与当前定位。

两种输入共用固定外参将点统一到IMU坐标系，
核心使用单位外参，避免重复变换。输出为 `camera_init → body` 的IMU位姿和世界系
点云；body不是底盘中心。核心源码保持原样，不依赖Gazebo。

## 接收与回归测试

C++ gtest 覆盖官方时间单位、字节布局、端序、行填充及坏消息拒收；pytest 通过真实 DDS
验证两种输入类型、QoS、晚启动、remap、类型冲突与恢复；分别自动识别两种合成点云，
经 IMU 驱动原始核心，检查位姿与地图输出。
运行命令统一见[测试指南](../../docs/教程/测试运行指南.md)，不代表真实 NUC 精度已验收。
