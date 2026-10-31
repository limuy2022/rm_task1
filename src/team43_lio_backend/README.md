# Small Point-LIO 定位包

实机完整链与源地图/比赛地图的关系统一见[底盘定位与点云建图](../../docs/教程/底盘定位与点云建图.md)。本页仅说明LIO接收与算法接口。

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

此命令只编译并启动定位后端，不启动电机。硬件雷达驱动须先用`ros2 launch team43_bringup livox.launch.py config_file:=实际JSON路径`独立启动。硬件驱动的构建和环境加载见[整体部署](../../docs/教程/整体部署.md)。定位包本身仅依赖标准ROS传感器消息。
不使用 `sudo`、不自动安装依赖，不修改 `.bashrc`。

默认输入为 `/livox/lidar`、`/livox/imu`，输出为
`/red_standard_robot1/aft_mapped_to_init` 和 `/red_standard_robot1/cloud_registered`。
按现场 record3 的 Livox PointCloud2 输入启动：

```bash
ros2 launch team43_lio_backend lio.launch.py \
  lidar_topic:=/red_standard_robot1/livox/lidar \
  imu_topic:=/red_standard_robot1/livox/imu
```

参数在 `config/small_point_lio_mid360.yaml`；也可用 `params_file:=/实际路径/配置.yaml`
指定自己的副本。默认 `acc_norm=1.0` 和 `satu_acc=3.0` 对应g单位；m/s²输入需要
同时换算。雷达到IMU的固定外参必须符合实际安装。

传感器接收默认使用 best-effort QoS。仅在输入发布端支持可靠传输时，才可在
配置副本中设置 `imu_qos_reliable: true`；Gazebo 回归会显式启用此选项，
避免共享 CPU 调度抖动导致 IMU 不可重传丢样。该选项不改变实机默认值或雷达 QoS。

本队驱动固定发布PointCloud2，定位端直接订阅，不提供格式选择或自动识别参数。
ROS负责发布者晚启动及话题remap；首帧有效点云到达前有界缓存IMU，
收到点云后保留原始时间顺序送入核心，缓存容量沿用`sensor_qos_depth`。
PointCloud2 按 [Livox 官方发布实现](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp)
读取 x/y/z（float32）、tag（uint8）和 timestamp（float64，绝对纳秒→秒），不再加 header 时间。
按 fields 的偏移解析，支持非对齐、端序及行填充；缺字段、类型/长度错误拒收整帧并报错，
非有限坐标/时间与无效回波跳过，不以帧时间替代缺失的逐点时间。强度和线号不参与当前定位。

输入点通过固定外参统一到IMU坐标系，
核心使用单位外参，避免重复变换。输出为 `lio_world → body` 的IMU位姿和世界系
点云；body不是底盘中心。核心源码保持原样，不依赖Gazebo。

## 接收与回归测试

C++ gtest覆盖官方时间单位、字节布局、端序、行填充及坏消息拒收；pytest通过真实DDS
验证PointCloud2的QoS、晚启动、remap、坏帧拒收及恢复。合成点云与IMU驱动原始核心，
检查位姿和地图输出。
运行命令统一见[测试指南](../../docs/教程/测试运行指南.md)，不代表真实 NUC 精度已验收。


## 本场斜装输出调平与近似地面

`output_world_up`是算法初始世界系中的向上方向；适配层用最短旋转将其变为+z，同时旋转发布的body位姿和注册点云。默认参数为[0,0,1]；本场实机YAML按2026-10-31静止包填写[-0.0321952086,-0.6955923471,0.7122368773]，原始驱动数据和内部雷达↔IMU外参不变。航向不是仅靠重力测得，底盘朝向仍依赖安装外参和比赛起点。

用户授权跳过点地面，`approximate_floor_from_height=true`和`lidar_height_m=0.231`供配套cloud_mapping入口推导近似地面，floor_z约−0.182833m；不是直接−0.231m，因为世界原点为内部IMU，需计入光心偏移。近似来源写入status及源地图YAML，不能当作实测地面精度。

仿真平台已水平，Gazebo和合成安装验证显式使用output_world_up=[0,0,1]，不套用本场斜装参数。另可在隔离容器运行`tools/verify_lio_install.py --field-alignment --output-dir 新目录`，用合成输入验证非单位输出旋转同时作用于点云和位姿；不连接硬件。
