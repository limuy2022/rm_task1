# Gazebo 走廊实时防撞建议验证

唯一场景为 `src/team43_gazebo/worlds/corridor.sdf`：长8m、宽2.5m，两侧墙和两端墙有碰撞几何。8m是测试长度，不是实测两门间距；2.5m用于对照生产配置中从上午两图估计的约2.494m墙宽。

链路为 Gazebo Fortress 三维雷达和IMU → 官方格式PointCloud2 → 未修改核心的Small Point-LIO → 生产 `chassis_advisor_node`。规则16线雷达每40Hz采样，四次快照按真实时刻组合为约10Hz输入；不是MID360非重复扫描。

## 运行

仓库根目录构建根Dockerfile的可选镜像：

```bash
docker build --target gazebo-sim -t rm-task1-gazebo-sim .
docker run --rm -v "$PWD":/workspace -w /workspace \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID=94 -e ROS_LOCALHOST_ONLY=1 \
  -e IGN_PARTITION=rm_task1_corridor_trial -e LIBGL_ALWAYS_SOFTWARE=1 \
  rm-task1-gazebo-sim bash -c '
    source /opt/ros/humble/setup.bash &&
    colcon --log-base /tmp/clog build --build-base /tmp/cb \
           --install-base /tmp/ci --symlink-install &&
    source /tmp/ci/setup.bash &&
    python3 -B tools/simulate_gazebo_corridor.py \
      --output-dir build/offline/gazebo_corridor_新编号 \
      --real-time-factor 0.25 --timeout 900'
```

输出目录必须为新目录。软件渲染不传宿主显卡或显示器，不安装pip库。请求倍率0.25给渲染留出时间，不能代表真实NUC实时吞吐通过。

## 场景与判据

平台独立按预设轨迹运动，模拟主办方控制底盘；**建议速度不控制平台**。目标为合成有效TargetState，不运行ArUco视觉或评分CSV。本试验map与LIO初始系重合，不替代正式replay_start链验收。

- 必须收到真实Gazebo雷达和LIO输出，独立真值只用于评价。
- 实际运动先靠近侧墙，再靠近端墙，确认建议可用且受到约束。
- 对每条有效建议，以独立Gazebo真值和真实墙面计算1.5秒路径。生产保护距离保持40cm，仿真评价允许5cm定位误差，真实墙面净距不得小于35cm。
- 仅中断防撞点云输入，底盘与目标数据继续，检查建议归零；恢复点云后须恢复非零建议。
- LIO最大位置误差须小于20cm。防撞通过而LIO误差超标时，整轮仍判失败，不能只汇报通过项。
- 所有子进程必须正常退出。

输出包括 `result.json`、`overview.png`、真实 `world.sdf`、生产节点参数和日志、`truth.csv`、`lio.csv`、`advice.csv`。真值从不输入定位；所有数据为仿真证据，不是真实场地采样。

## 边界

悬浮刚性平台不模拟轮胎、悬架、真实制动或完整电机闭环。当前无额外噪声注入。几何为简单走廊，不能保证稀疏走廊的实机LIO精度。Gazebo通过也不能保证主办方底盘会遵循建议。

雷达独立位姿/栅格和视觉联合入口也使用同一个 `corridor.sdf`；前者沿走廊往返，后者仅加入相机和目标码，不额外生成箱体房间。

仿真只调整坐标、单位、接口与模拟量程，LIO体素及匹配参数沿用装车配置，不另外设一套。当前精度及前后对照见[精度修复记录](../记录/LIO走廊精度修复_2026-11-01.md)。
