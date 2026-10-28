# Small Point-LIO ROS 2 迁移验证（2026-10-28）

> 当前结构已简化：算法源码直接包含在 `src/team43_lio_backend/vendor/small_point_lio`，
> 使用普通colcon构建和 `ros2 launch team43_lio_backend lio.launch.py` 启动。
> 下文归档与独立环境布局为迁移过程记录；不再使用这些部署方式。


本次把默认可选仿真镜像和 `lio_only.launch.py` 从导航归档中的旧移植版切换为
Small Point-LIO 的 ROS 2 核心。没有部署或修改公共 NUC，没有操作真实电机。

## 1 版本选择

| 候选 | 本次检查 | 选择依据 |
|---|---|---|
| LihanChen2004/Point-LIO，默认 `grid_map_ros2` | 默认提交 `155f46d` 为2024-12-11；所读源码仍有 reserve 后下标访问及旧测量方差分支 | 没有证据表明直接更新这个默认分支能省掉原来的问题；不把结论推广到其所有分支 |
| dfloreaa/point_lio_ros2 | 默认提交 `a8e2d0d` 为2025-08-14，README 声明 Ubuntu22.04/Humble，含 Unitree 适配；本地源码仍见 crossmat_list/pbody_list 的 reserve 后下标写入 | 没有因换到该实现而避开向量尺寸问题，不以 star 数判断稳定性 |
| ctu-mrs/point-lio，`ros2` | 提交 `e7ff034` 为2026-04-23，有团队维护；依赖 MRS 库和其节点/参数/订阅框架，仍见旧测量方差公式 | 引入整套额外框架，不能作为轻量直接替换 |
| Yancey2023/small_point_lio，`ros2` | 提交 `131fb586dc609521e7acf11a9c833121b2d49141` 为2026-10-23；有 Humble/Jazzy/Lyrical、含/不含 Livox 的构建 CI；MIT | 选为候选并完成本地 Humble 编译、接口和集成验证 |

上游地址：
[Small Point-LIO](https://github.com/Yancey2023/small_point_lio/tree/131fb586dc609521e7acf11a9c833121b2d49141)、
[LihanChen2004](https://github.com/LihanChen2004/Point-LIO/tree/155f46da0b1eb3fc7c7facf43b00cb124631c807)、
[dfloreaa](https://github.com/dfloreaa/point_lio_ros2)、
[CTU ROS 2](https://github.com/ctu-mrs/point-lio/tree/ros2)。

“近期维护”和“存在CI”不等于所有设备和场景已经稳定。固定提交而非浮动拉取，
保留本地测试边界；未来更新仍须重新审阅和验证。

## 2 构建与接口

`archives/small_point_lio-131fb586.tar.gz` 是上游固定提交的完整 `git archive`，
SHA256 为 `8d87ab72df8ae9631712a980493276bb69ec066273a8bb11c7a0ffbe82a25871`，
保留许可证和第三方版权。Docker 构建先校验哈希，再直接编译未修改的算法核心、
预处理和体素采样源码。旧三个算法补丁没有用于新核心。

项目接口位于 `src/team43_lio_backend/`，代替上游 ROS 外壳，负责真实 Livox 消息、
固定外参、话题/frame、接收队列和入口审核。没有通过真值配准或轨迹拟合消除误差。
它不是旧 Point-LIO 的无差别升级，仍有我们自己的接入代码需要维护。

初轮直接接入核心的标准房间 RMSE 为4.741cm，虽通过原门槛，但静止存在与安装
偏移相近的固定误差。审阅发现核心初始化地图直接使用输入点，而后续点更新使用
雷达到IMU外参。最终接口把所有输入点先按已标定外参变换到IMU frame，再给核心
配置单位外参；初始地图和后续更新因此采用相同参考系。外参只作用一次，输出仍
为IMU body位姿和世界系点云；建图射线原点继续使用物理雷达相对body的位置。

该核心使用IMU测量更新，不把旧 `use_imu_as_input` 两模式映射成不存在的新模式。
仿真报告以 `imu_model` 标注实际模型，不再提供旧版模式兼容入口。新实机参数独立保存，g与m/s²的饱和阈值必须同步换算。

## 3 场景隔离

用户正在编辑 `tools/gazebo/lio_room.sdf` 及 Blender 模型，本次保留这些修改。
（后续资源整理已迁入 `src/team43_gazebo/worlds/lio_room.sdf`，Blender 源文件位于该包的 `blender/`。）
`lio_room_regression.sdf` 是仓库原始标准房间，SHA256 为
`de58aca4e8cce321ad4776b18c3f3d7480716b273c3aacf7bcf6eb3e634c793b`。
CI 显式设置 `RM_LIO_WORLD` 选择它；报告保留源场景和生成场景的哈希。

用户新场景不套用原房间边界、墙面覆盖率及盒体回波距离检查。脚本允许运行和
观察，但在评价阶段会明确拒绝不匹配的源场景；该拒绝不表示算法定位失败。
已单独验证这个拒绝路径。若要给新模型做精度/地图验收，应另建对应几何评价。

## 4 本轮验证

全部在根 Dockerfile 的 Humble 容器中执行；未用pip安装或覆盖系统库。

- 标准环境：colcon完整构建成功；串口C++ 36项无失败；pytest **553项通过**；
  `verify_all_offline.py --skip-pytest` **19项通过**。
- 最终雷达方形场景：2CPU、软件渲染、请求倍率0.25，全部22项检查及正常退出通过。
  位置RMSE **0.006708m**，最大误差 **0.017638m**，静止最大误差 **0.001500m**，
  估计返回误差 **0.001418m**；墙体覆盖率97.59%，墙外未知比例100%。
  实收410帧雷达、8207条IMU，与独立源端数量一致，无倒序，最大IMU间隔6ms。
  报告与地图在 `build/offline/small_lio_final_01/`，首轮在 `small_lio_trial_01/`。

- 最终视觉联合场景：同样2CPU、软件渲染、0.25倍率，全部38项检查及正常退出通过。
  定位RMSE **0.007101m**，最大误差 **0.015136m**；目标位置RMSE **0.026149m**，
  最大误差 **0.076187m**。622帧图像均产生有效PnP，延迟、断流、旧帧重放的失效
  与恢复比例均为100%。实收410帧雷达、8210条IMU，与源端一致，无倒序。
  报告在 `build/offline/small_lio_vision_01/`。

最终镜像可执行文件SHA256为
`b8da9859d267db0d4535e7f9b5be131328ac6f217ad663d9afc8ebb4f247d35d`，
与两轮最终回归报告一致。镜像已标记为 `rm-task1-gazebo-sim`。
标准测试及集成控制台日志保存在 `build/offline/small_lio_migration/`。
原始仿真bag、日志和图片均留在被gitignore的
`build/offline/`，不作为真实雷达数据提交。

## 5 使用与旧版清理

```bash
docker build --target gazebo-sim -t rm-task1-gazebo-sim .
```

加载 `/opt/ros/humble/setup.bash`、`/opt/lio_sim/install/setup.bash` 和本项目安装
环境后，`lio_only.launch.py` 运行新后端；`cloud_mapping.launch.py` 默认读取
`small_point_lio_mid360.yaml`。话题和IMU body参考点保持下游契约。

后续清理已删除旧算法源码归档、补丁、专用诊断脚本、Docker目标及回退launch。
驱动与消息定义单独保存在 `archives/livox_ros_driver2.tar.gz`，与新算法独立校验哈希。
仿真只运行IMU测量更新模型，旧版模式切换参数也已移除。

本次没有验证用户新模型、真实MID360、非单位雷达旋转外参、32线、高倍率回放或
NUC实时性能；也不代表比赛评分通过。比赛部署前仍需在许可环境中做真实传感器
数据对照和实时验收，不在比赛当天临时换版本。

## 6 旧版全部移除后的回归

已删除旧导航源码归档（包含旧Point-LIO）、旧参数、补丁及其C++测试、专用诊断工具、
回退launch、Docker回退目标和运行器兼容分支。当前只有Small Point-LIO；历史实验
文档单独标明不可按原命令运行。Livox驱动独立归档的SHA256为
`b438683b99951c2a962df0919106476baf00f3281a50e5691203dc510dd24d23`，许可证及
源码内容保留。镜像检查确认没有旧算法源码、安装包或补丁目录。

全部验证仍在Humble Docker环境执行：

- 完整pytest **554项通过**（删除旧launch用例，增加两项通用启动助手回归）；
  串口C++ **36项通过**；全量离线核验 **19项通过**。
- 雷达场景 `small_lio_clean_02`：全部22项与正常退出通过，位置RMSE为
  **0.006724m**，完整接收410帧雷达和8207条IMU。
- 视觉联合场景 `small_lio_vision_clean_03`：全部38项与正常退出通过，定位RMSE
  **0.006979m**，目标位置RMSE **0.026040m**；完整接收410帧雷达和8208条IMU。

清理中的首轮集成测试捕获了通用节点启动助手误用定位可执行文件名的问题，已修正
并用建图/PnP两类调用添加回归。失败产物保留在 `small_lio_clean_01` 和
`small_lio_vision_clean_01`。

联合场景 `small_lio_vision_clean_02` 的38项检查通过，但ros_gz_bridge在SIGINT后
发生段错误（退出码245），整轮仍判失败，没有放宽退出门槛。随后单独复跑03正常
退出；这不表示已经定位或修复该次退出异常的根因。全部日志和失败证据保留，
清理回归控制台日志位于 `build/offline/small_lio_cleanup/`。

用户编辑的SDF、Blender模型和VS Code设置未改动；测试使用独立标准房间。

## 7 简化为普通ROS包与比赛机启动

算法源码现直接包含在 `src/team43_lio_backend/vendor/small_point_lio`，保留原始
许可证、版本和全部源文件。删除了独立解包安装器、额外定位工作区和算法源码归档。
CMake从包内编译核心，launch/config随包安装，编译和运行均不依赖仿真目录。

比赛机只需在已加载Humble和已有Livox环境的终端执行：

```bash
colcon build --base-paths src --packages-select team43_lio_backend \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 launch team43_lio_backend lio.launch.py
```

验证在根Dockerfile的Humble容器完成。交付ZIP实际解压后，仅编译定位包，再从
空的 `/tmp` 工作目录直接launch，获得37条位姿、36条配准点云，坐标系、静止位姿、
时间递增与正常退出六项检查通过。该容器不存在 `/opt/lio_sim`。证据位于
`build/offline/lio_simple_delivery_01/`，完整标准构建后的同类检查也通过。
原始算法数值没有修改；这轮验证聚焦原生构建、打包、环境依赖和启动，不代表真实
MID360或比赛NUC已部署验收。

`tools/visual_tracking/use_here.bash` 保留已加载的Livox环境，避免清除定位动态库。
测试容器只预装Livox依赖；定位包与其他项目源码一样通过colcon构建。
