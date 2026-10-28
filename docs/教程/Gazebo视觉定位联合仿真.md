# Gazebo 视觉、雷达定位与目标坐标联合仿真

2026-10-28 起，默认镜像使用固定提交的 Small Point-LIO 原始核心和项目接口层，
不再运行旧版算法补丁；算法与场景选择见[雷达教程当前版本说明](Gazebo雷达惯性定位仿真.md)。
标准回归须设置 `RM_LIO_WORLD=/workspace/tools/gazebo/lio_room_regression.sdf`；
`room/features` 都以此原始房间为基础。自定义 Blender 场景不能沿用本页的几何验收。
当前仅有Small Point-LIO后端，不提供旧版算法或模式切换。

`tools/simulate_gazebo_vision.py` 复用[雷达惯性定位仿真](Gazebo雷达惯性定位仿真.md)，
增加真实渲染相机、静止 ArUco 码和跨进程视觉估计。它主要检查：移动底盘观察
静止目标时，目标的比赛坐标是否仍保持固定；旧观测是否会被拒绝，以及恢复
新观测后是否能重新跟踪。

## 1 实际运行的链路

```text
Gazebo 三维雷达 + IMU → Livox 消息适配 → Small Point-LIO（原始核心）
                                           ├→ cloud_mapping_node → 地图/PGM
                                           └→ base_pose_provider_node
                                                 → map_frame_broadcaster_node → TF
Gazebo RGB 相机 → aruco_detector_node → marker_pnp_node
                                           → 观测故障注入 → target_estimator_node
```

以上生产节点均作为独立 ROS 2 进程运行，通过真实 DDS 和 TF2 通信。相机图像
由 Gazebo/Ogre2 渲染，不用手工投影角点或目标坐标替代。CameraInfo 经 bridge
送到 PnP 节点；图像时间戳一直保留到 MarkerObservation，估计器按图像时刻查 TF。
Gazebo 真值只用于离线误差评价和阶段记录，不送给定位或目标估计器。

相机**固定在平台上**，没有运行云台关节、云台 IMU 姿态反解、视觉 PID 或电机
控制。这是整车坐标链的一个新增验证层，不能视为云台机械闭环验收。相机桥接
接口参考 [Gazebo Fortress 官方迁移说明](https://gazebosim.org/docs/fortress/migrating_gazebo_classic_ros2_packages/)。

## 2 场景和参数

- 房间沿用 8×6m 的雷达场景，默认 `--scene features` 增加五个不同方位的近距离箱体；
  `--scene room` 保留原来的两个箱体，用于比较场景几何的影响。
- 静止目标为 `DICT_4X4_50` 的 ID 0，黑色码边长 **0.1m**，白边外尺寸 0.14m。
  黑白格用真实三维薄片构成，无外部纹理依赖。评分参考点是黑色码平面中心，
  不是厚板背面或板模型原点。
- 相机为 800×600、15Hz、水平视场角 0.65rad 的无畸变针孔模型；内参以实际
  CameraInfo 为准。没有用实车标定参数冒充仿真内参。
- IMU 相对底盘中心平移为 `(0.08,-0.03,0.06)m`；相机相对 IMU 平移为
  `(0.15,-0.08,0.12)m`，RPY 为 `(0.02,-0.04,0.06)rad`。底盘转换和静态相机
  TF 共同使用这些场景定义，覆盖非零杠杆臂和三轴相机安装角。
- 雷达默认16线、IMU测量更新，其他参数沿用雷达仿真；支持 `--scan-lines 32`
  作对照，不修改实车 YAML。
- 比赛原点由生产起点节点使用显式 `base_init` 模式在启动静止阶段冻结；本次
  不验证 replay_start 的历史插值，后者由[定位建图接入仿真](定位建图仿真.md)覆盖。
- 目标估计节点的 `perception.r_min_sigma` 在仿真中默认设为 **0.03m**，CLI 为
  `--observation-sigma-floor`。它是组合观测误差下限的工程假设，避免将厘米级
  定位扰动当成毫米级像素测量；不是完整的 TF/定位协方差传播，实车默认仍为
  0.002m，真实配置需要实测依据。参数和场景均记录进报告。

平台用已有施力插件运动，等待定位就绪后静止3秒，再依次前后、左右往返、小角度
转向及再次前后往返，各段4秒，最后静止3秒。最大平移速度0.24m/s，最大角速度
0.06rad/s，均带0.5秒余弦过渡。真值 x 跨度约0.84m，目标始终位于相机视场内。

## 3 在 Docker 中运行

使用根 Dockerfile 的可选镜像；依赖安装和编译方式与雷达仿真完全相同：

```bash
docker build --target gazebo-sim -t rm-task1-gazebo-sim .
docker run --rm -v "$PWD":/workspace -w /workspace \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID=94 -e ROS_LOCALHOST_ONLY=1 \
  -e IGN_PARTITION=rm_task1_vision_01 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e RM_LIO_WORLD=/workspace/tools/gazebo/lio_room_regression.sdf \
  rm-task1-gazebo-sim bash -c '
    source /opt/ros/humble/setup.bash &&
    colcon --log-base /tmp/clog build --build-base /tmp/cb \
           --install-base /tmp/ci --symlink-install &&
    source /tmp/ci/setup.bash &&
    python3 -B tools/simulate_gazebo_vision.py \
      --output-dir build/offline/gazebo_vision_01 --record-bag'
```

输出目录必须不存在；重跑时换编号。`--record-bag` 可省略，开启后会录制图像、
雷达、IMU、定位、TF、故障前后的观测及目标状态。所有自定义话题在
`/sim/gazebo` 下；不访问串口，不发布 `/judge/*`，不修改运动开关。
沿用软件渲染，不传入 `/dev/dri`。默认请求0.5倍速、600秒墙钟超时。

## 4 故障、门槛和独立检查

故障只注入在 **PnP 输出与估计器输入之间**，不改变图像内容或原始时间戳。
以下时间均相对运动开始时刻：

| 故障 | 时间 | 具体行为 |
|---|---|---|
| 延迟 | 8～11秒 | 每条观测延迟0.45仿真秒送达，超过估计器0.3秒接收期限 |
| 断流 | 16～19秒 | 完全停止向估计器转发观测 |
| 旧帧重放 | 24～27秒 | 反复发送故障前最后一条有效观测，保持原 stamp |

每种故障的第1秒之后，目标无效比例应 >95%；故障结束第1～2.5秒，有效比例
应 >90%。同时检查确实注入超过20条消息、延迟超过期限、重放没有刷新 stamp。
正常窗口为1～8、12～16、20～24及28～35秒，有效比例应 >90%。统计包括
`track_status=2` 的仍有效外推，不能只挑新鲜帧来隐藏外推误差。

正常窗口的目标坐标 RMSE 应 <15cm、最大误差 <35cm；静止目标的100ms预测
RMSE 应 <20cm。独立使用 Gazebo 位姿审核原始 PnP 的三维几何，RMSE 应 <5cm；
此项只用于评分，不构成生产 TF 输入。所有有效 PnP 的 stamp 必须来自实际收到
的图像。定位、原始测量物理一致性和地图保存检查复用雷达仿真的既有门槛。

增加箱体会遮住部分墙面，因此 `features` 场景按 SDF 箱体和独立真值轨迹，在
雷达高度的水平面计算视线遮挡，评价至少一次几何可见的墙面。可见墙面10cm邻域
覆盖率仍须 >85%，并至少有100个可评价样本。报告同时保留**全墙面原始覆盖率**和
可见比例；此几何近似没有模拟逐条激光束，也不等于所有可见点都一定被扫描。
`room` 对照仍使用原来的全墙面检查。两个场景均检查墙外区域是否保持未知。

## 5 产物与验证结果

产物包含雷达教程的报告、地图和轨迹，以及 `vision_metrics`、图像内参、观测与
目标状态记录、故障窗口记录和 `vision_overview.png`。启用录包时保存仿真数据。
算法版本、当前测试成绩与边界见[迁移验证记录](../记录/SmallPointLIO迁移验证.md)。
旧算法、补丁、模式参数及专用诊断脚本已移除。当前联合验收只覆盖标准features
场景、16线和指定运动；用户自定义场景及真实MID360需要各自验证。
