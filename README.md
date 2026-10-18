# RoboMaster 赛题一

当前已有独立图像跟踪程序：ArUco → 图像偏差 → P/PI/PD/PID → 两轴角速度 → C++ 串口桥。它不等待地图或底盘定位；默认运动关闭。云台通信已确认 ttyUSB0、IMU 轴 [1,1,1]；新相机 K/D 已更新。真实转动效果仍待 NUC 试验。

按[云台跟踪教程](docs/教程/云台跟踪.md)将源码包装到自己的独立 NUC 目录后运行：

```bash
bash "$HOME/ws_championship/src/47-visual-tracking-20261018-r3/run_here.bash" target_id:=0
```

默认只看不动；真实串口模式加 `with_bridge:=true port:=/dev/ttyUSB0`，由人在参数终端开启 motion_enabled。停止在启动终端 Ctrl+C，或设置 motion_enabled=false。先做 3 秒单轴方向试验，再做两轴。参数默认值、调整现象和保存方法见[调参检查清单](docs/教程/上机调参检查清单.md)。

代码保留在 src 与既有 42-gimbal_serial；配置/launch 在对应包内，离线工具在 tools，独立教程在 docs/教程，原理在 docs/原理。没有把代码再复制进文档目录。Point-LIO、目标坐标/预测、点云地图、建议速度和九列提交链已有实现，整车尺寸、真实误差、最终地图与正式 180 秒端到端流程仍待联调。主办方驾驶底盘，我们自行估计底盘位置。

本机完成实际写出器 180 秒/1800 行基线：漏拍 0，读者可见延迟平均 1.73ms、最大 31.62ms；没有 ROS/DDS，也不代表 NUC 验收。点云地图、预测对比与其它离线结果见 docs/验证 的 JSON；原始实验产物不提交 Git。

本机离线验证环境：个人电脑的隔离 .venv，依赖列在 tools/requirements-offline.txt。公共 NUC 不用 pip 覆盖 ROS 的 OpenCV/numpy，不全量构建，不改 .bashrc，不读取主办方保护目录，不向 /judge/* 发布测试消息。保留现有源码版权及许可证声明。
