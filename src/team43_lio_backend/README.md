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
输入有命名空间时直接覆盖：

```bash
ros2 launch team43_lio_backend lio.launch.py \
  lidar_topic:=/red_standard_robot1/livox/lidar \
  imu_topic:=/red_standard_robot1/livox/imu
```

参数在 `config/small_point_lio_mid360.yaml`；也可用 `params_file:=/实际路径/配置.yaml`
指定自己的副本。默认 `acc_norm=1.0` 和 `satu_acc=3.0` 对应g单位；m/s²输入需要
同时换算。雷达到IMU的固定外参必须符合实际安装。

本包接口接收带逐点时间的Livox CustomMsg和IMU，按固定外参将点统一到IMU坐标系，
核心使用单位外参，避免重复变换。输出为 `camera_init → body` 的IMU位姿和世界系
点云；body不是底盘中心。核心源码保持原样，不依赖Gazebo。
