# RoboMaster 赛题一

Ubuntu 22.04 / ROS2 Humble，队号暂用47。基于02d6ef2完成本批离线修复与集成，**尚未通过Humble/NUC或真实机器人验收**。相机已固定在云台上，整套装置还没有安装到底盘；不把模板零值当成安装外参。

## 从哪里开始

- [实验室执行手册](docs/当前任务/实验室执行手册_ABC.md)：开头就是A/B/C现在该做的三项任务，后文包含物理测量、教程章节、命令、预期与失败排查。
- [现场记录表](docs/当前任务/现场记录表_ABC.md)：每次复制填写，空白模板保留。
- [文档入口](docs/README.md)：阶段一数学样例及当前资料位置。

当前先做：B在个人Ubuntu构建并跑20拍/1800拍ROS提交实验；A/C核对已固定相机/云台的原点和轴，并与B完成静态PnP真实测距。不要求现在测尚未安装的整车外参。

## 本批实际完成

| 功能 | 修改与验证 |
|---|---|
| ArUco/PnP | 修复detectMarkers返回值错误；角点消息绑定真实ID与图像stamp；候选PnP、丢失/过期处理；实际OpenCV合成图验证 |
| 定位/时间/预测 | 传感器三维位姿含杠杆臂换到底盘中心；保留源stamp；按图像时刻查询完整TF，滤波留在观测时间；按tick采样当前和100ms预测 |
| 云台几何/控制 | 完整两轴几何正反解、同stamp底盘+IMU派生关节估计；控制限位不再使用惯性yaw充当机械角；控制/搜索默认关闭 |
| 地图/方向 | 保留真实地图尺寸、分辨率和origin yaw；处理旋转地图；输入无效/失联/全阻挡时不凭空给穿墙方向 |
| CSV/评分 | 九列逐拍flush；单写者、重启去重、损坏文件拒绝；正式评分补回底盘40%权重与归一化；缺参考不伪造总分 |
| 串口 | 生产C++解析/串口类经黄金帧、噪声、分片、OS伪终端及ASan/UBSan验证；桥会话/MCU重启使epoch变化 |

当前离线证据见[离线验证结果](docs/当前任务/离线验证结果.json)。可复跑：

```bash
.venv/bin/python -B tools/verify_all_offline.py
```

本地协作仓库不复制本批独立开发副本的`.venv`。如果个人电脑尚无隔离环境，可在个人仓库根目录准备（不要在NUC照做）：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r tools/requirements-offline.txt
.venv/bin/python -B tools/verify_all_offline.py
```

串口伪终端验证还需要已有C++编译器；缺少时脚本会报告失败，不假装通过，不自动安装。脚本本身不安装依赖、不启动ROS或真实设备。个人电脑的隔离环境依赖列在`tools/requirements-offline.txt`；本批环境为Python3.12、numpy2.3.5、OpenCV4.10.0、pytest8.4.2、PyYAML6.0.3。**不要在NUC用pip覆盖ROS的OpenCV/numpy。**日志及合成产物放在被Git忽略的`build/offline/`。

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

Humble消息生成/节点通信/启动、NUC端到端80ms时限、真实PnP/定位/预测误差、实际地图、最终安装外参与时延、机械零位/限位、真实串口及云台闭环均待实测。当前控制器的装配假设也必须据实际几何复核；离线通过不等于可直接使能。

每轮开始前运行定位并预热历史；收到真实replay_start后按其stamp冻结，缺历史/缺TF就明确等待，不用最新位姿或单位阵伪造。桥重启或MCU复位后必须重做IMU世界对齐并更新对应epoch。最终输出只追加，不修补/改名已提交记录。

## Git提交范围

这批应提交修改后的`src/`、`42-gimbal_serial/`、`tools/`、`.gitignore`和当前文档，**代码与文档一起提交**。不提交`.venv/、build/、install/、log/`和实验bag/CSV/地图。只保留一份当前实验室手册，历史副本不重新放回仓库。

GitHub Desktop选`rm_task1`→确认Changes包含本批源码与文档→Summary填写“修复赛题一时序、坐标与离线集成”→Commit→Push origin。只有Push后队友才拿得到；当前交付没有代替你执行Git提交或推送。

仅操作选手工作区；不访问主办方保护目录，不向`/judge/*`发布消息。本批未部署、未操作真实硬件。保留源码原版权和许可证声明，未给全部第三方代码重新指定许可证。
