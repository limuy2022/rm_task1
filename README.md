# RoboMaster 赛题一

Ubuntu 22.04 / ROS2 Humble，队号暂用47。2026-10-16现场记录已证明专用视觉入口在NUC运行：识别63帧，其中PnP有效44帧、拒绝19帧；雷达IMU收到连续消息。用户补充确认首次视觉第4步与O/C通过，消息类型错误发生在后续重试；真实距离精度、云台反馈和整车链仍未验收。相机已固定在云台上，整套装置此前尚未安装到底盘；不把模板零值当成安装外参。

**当前现场任务仍是[G坐标核查](docs/当前任务/G坐标定义核查_实际操作.md)：首次视觉及O/C不重复验证。新增[云台转向教程](docs/当前任务/云台如何转向目标_源码与操作教程.md)，解释识别→PD→0x30→下位机边界，第10节只读核查已运行链路。**

## 从哪里开始

- **[发给B与GitHub提交说明](docs/当前任务/交接与GitHub提交.md)**：本轮资料包用途、30个文件提交范围和Desktop逐步操作。

- **[G专项教程](docs/当前任务/G坐标定义核查_实际操作.md)**：本轮现场先用这一份。只传watch_gimbal_axes.py，不重编已通过的视觉。
- **[云台转向教程](docs/当前任务/云台如何转向目标_源码与操作教程.md)**：看懂每层输入输出、PD公式、两条串口分支和必须靠固件确认的部分。
- [完整上机教程](docs/当前任务/现场上机教程_ABC.md)：后续完整部署/标定参考；当前不按全文重做。
- [实验室执行手册](docs/当前任务/实验室执行手册_ABC.md)：开头就是A/B/C现在该做的三项任务，后文包含物理测量、教程章节、命令、预期与失败排查。
- [现场记录表](docs/当前任务/现场记录表_ABC.md)：每次复制填写，空白模板保留。
- [文档入口](docs/README.md)：阶段一数学样例及当前资料位置。
- [10月17日现场复查](docs/当前任务/现场复查_20261017.md)：昨晚记录的结论、消息加载恢复、串口链接检查、两份内参对照和需要带回的数据。

当前先做：B在Windows的NoMachine中进入NUC，按G专项教程读取真实云台IMU或保存端口/桥诊断；A/C补G轴与原点依据。可按云台转向教程第10节只读记录已有进程/参数/话题。两份内参冲突、完整提交与整车任务保留，首次视觉/O/C不重做，消息类型问题仅复现时排查。

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
    python3 -B -m pytest -q src --import-mode=importlib &&
    python3 -B tools/verify_all_offline.py'
```

测试环境即运行环境：被测的 rclpy / OpenCV / numpy / pyserial 等全部来自 humble 镜像的 apt 包，不用 pip 覆盖（**不要在NUC用pip覆盖ROS的OpenCV/numpy**）。串口伪终端验证所需的 C++ 编译器镜像内已有；脚本本身不安装依赖、不假装通过。日志及合成产物放在被 Git 忽略的`build/offline/`。

1800拍合成链路采用已知几何、0.1px角点噪声、40ms图像延迟及0.8秒遮挡。它检验PnP→坐标→滤波→tick采样→方向→CSV/地图；不代表NUC实时性、真实图像精度或真实云台闭环通过。

## 实际数据流和入口

```text
现有/image_raw → detector → MarkerCorners(ID+图像stamp) → PnP → MarkerObservation
自有Point-LIO Odometry → base_pose_provider → odom/base_link原始底盘位姿
底盘位姿 + 云台IMU + 实测两轴模型 → 同stamp关节估计和base→camera TF
replay_start → 起点插值冻结map→odom
MarkerObservation + 图像时刻完整map←camera TF → TargetState
自有OccupancyGrid → grid_mapper → /map/grid、/map/ray_grid
TargetState + 比赛系底盘 + 地图 → ChassisAdvice
以上状态按/judge/tick采样 → 47.csv、47_map.pgm、47_map.yaml
```

- `vision.launch.py`：检测/PnP，默认ID=-1等待赛前指定，不控制运动。
- `submission.launch.py`：只启动提交器；个人Ubuntu合成实验必须用`/test/`映射和新测试目录。
- `hardware_interface.launch.py`：打开真实串口，仅由现场人员按手册执行；默认不订阅运动指令，adapter转发也关闭。
- `lio_only.launch.py`：只启动已安装的Point-LIO，复用现有雷达输入；先验证CustomMsg类型/IMU单位，无驱动和Nav2。
- `processing.launch.py`：完整实测配置通过检查后，只处理现有数据，提交显式选择；不启动驱动、串口或控制器。配置模板是`config/vehicle_template.yaml`，原样会被拒绝。
- 原`competition.launch.py`、`sim.launch.py`和`mapping.launch.py`保留作历史/仿真参考，**不是当前实机比赛启动说明**。mapping入口消费地图或生成示例图，本身不是雷达SLAM。

地图最短候选为Point-LIO云→pointcloud_to_laserscan→slam_toolbox。真实高度裁剪、TF与地图参考仍需数据；不启动完整Nav2替代这些检查。建图SLAM与比赛原点不能同时给odom两个父节点，具体分阶段做法见手册第九部分。

## 还不能宣布完成的部分

专用检测/PnP的NUC启动和节点内有效观测已有现场日志支持，但额外终端加载自定义消息失败尚需复查。NUC端到端80ms时限、真实PnP/定位/预测误差、实际地图、最终安装外参与时延、机械零位/限位、真实串口有效IMU及云台闭环均待实测。当前控制器的装配假设也必须据实际几何复核；离线通过不等于可直接使能。

每轮开始前运行定位并预热历史；收到真实replay_start后按其stamp冻结，缺历史/缺TF就明确等待，不用最新位姿或单位阵伪造。桥重启或MCU复位后必须重做IMU世界对齐并更新对应epoch。最终输出只追加，不修补/改名已提交记录。

## Git提交范围

这批应提交修改后的`src/`、`42-gimbal_serial/`、`tools/`、`.gitignore`和当前文档，**代码与文档一起提交**。不提交`.venv/、build/、install/、log/`和实验bag/CSV/地图。只保留一份当前实验室手册，历史副本不重新放回仓库。

GitHub Desktop选`rm_task1`→确认Changes包含本批源码与文档→Summary填写“修复赛题一时序、坐标与离线集成”→Commit→Push origin。只有Push后队友才拿得到；当前交付没有代替你执行Git提交或推送。

仅操作选手工作区；不访问主办方保护目录，不向`/judge/*`发布消息。本批未部署、未操作真实硬件。保留源码原版权和许可证声明，未给全部第三方代码重新指定许可证。
