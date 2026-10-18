# RoboMaster 赛题一

当前已有独立图像跟踪程序：ArUco → 图像偏差 → P/PI/PD/PID → 两轴角速度 → C++ 串口桥。它不等待地图或底盘定位；默认运动关闭。云台通信已确认 ttyUSB0、IMU 轴 [1,1,1]；新相机 K/D 已更新。真实转动效果仍待 NUC 试验。

按[云台跟踪教程](docs/教程/云台跟踪.md)将源码包装到自己的独立 NUC 目录后运行：

```bash
bash "$HOME/ws_championship/src/47-visual-tracking-20261018-r3/run_here.bash" target_id:=0
```

默认只看不动；真实串口模式加 `with_bridge:=true port:=/dev/ttyUSB0`，由人在参数终端开启 motion_enabled。停止在启动终端 Ctrl+C，或设置 motion_enabled=false。先做 3 秒单轴方向试验，再做两轴。参数默认值、调整现象和保存方法见[调参检查清单](docs/教程/上机调参检查清单.md)。

代码保留在 src 与既有 42-gimbal_serial；配置/launch 在对应包内，离线工具在 tools，独立教程在 docs/教程，原理在 docs/原理。没有把代码再复制进文档目录。Point-LIO、目标坐标/预测、点云地图、建议速度和九列提交链已有实现，整车尺寸、真实误差、最终地图与正式 180 秒端到端流程仍待联调。主办方驾驶底盘，我们自行估计底盘位置。

本机完成实际写出器 180 秒/1800 行基线：漏拍 0，读者可见延迟平均 1.73ms、最大 31.62ms；没有 ROS/DDS，也不代表 NUC 验收。点云地图、预测对比与其它离线结果见 docs/验证 的 JSON；原始实验产物不提交 Git。

---

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

本机离线验证环境：个人电脑的隔离 .venv，依赖列在 tools/requirements-offline.txt。公共 NUC 不用 pip 覆盖 ROS 的 OpenCV/numpy，不全量构建，不改 .bashrc，不读取主办方保护目录，不向 /judge/* 发布测试消息。保留现有源码版权及许可证声明。
