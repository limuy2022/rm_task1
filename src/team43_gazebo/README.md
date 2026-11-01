# Gazebo 走廊仿真

本包只保留一个场景：`worlds/corridor.sdf`，8m长、2.5m宽，含地面、天花板、两侧墙和两端墙，初始IMU高0.7m。墙体同时具有视觉和碰撞几何，全部使用SDF基本形体，无外部网格模型。长度为测试假设，宽度接近上午两图的约2.494m估计，不作为实物测绘。

主要入口为 `tools/simulate_gazebo_corridor.py`：Gazebo雷达/IMU → 真实Small Point-LIO → 生产实时防撞建议。检查近侧墙、近端墙、保护点云断流及恢复；真值只用于评价，不输入定位。`simulate_gazebo_vision.py`也使用同一走廊，仅添加相机和目标码。

场景通过ament索引从安装空间读取，`worlds/`安装到 `share/team43_gazebo/`。运行时生成的 `world.sdf` 仅修改传感器外参、线数和物理倍率等试验参数，不引入其他房间。

`plugins/`中的施力插件由根Dockerfile的 `gazebo-sim` 目标单独编译，安装到 `/opt/team43_gazebo/lib/`，SDF只引用库名。普通colcon构建只安装资源与Python辅助模块。所有仿真在隔离Docker、独立ROS域及IGN_PARTITION中运行，不接串口或真实电机。

完整命令与边界见[雷达走廊仿真教程](../../docs/教程/Gazebo雷达惯性定位仿真.md)。
