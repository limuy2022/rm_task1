# 工具入口

所有验证在根 `Dockerfile` 的 ROS 2 Humble 容器中运行，完整命令见
[测试运行指南](../docs/教程/测试运行指南.md)。

## 常用入口

| 用途 | 入口 |
|---|---|
| 全量离线核验 | `verify_all_offline.py`；标准流程先跑 `pytest src`，再加 `--skip-pytest` |
| 独立集成仿真 | `run_simulation_ci.sh`：`navigation` / `submission` / `gazebo_lio` / `gazebo_vision` |
| CSV 格式检查 / 对照真值评分 | `check_csv.py` / `score_csv.py` |
| 云台闭环产物评分 | `score_bearing_sim.py` |
| 源码包生成 | `make_visual_tracking_bundle.py --output /tmp/47-visual-tracking.zip` |
| 相机标定 / 标定比较 / 参考 PnP | `run_camera_calibration.py` / `compare_calibrations.py` / `run_reference_pnp.bash` |
| 现场只读检查 / 判轴观察 | `inspect_nuc.py` / `watch_gimbal_axes.py` |
| 预测精度 / 写盘时延基线 | `benchmark_prediction.py` / `benchmark_submission_realtime.py` |

## 依赖与专项工具

- `verify_*.py`：独立算法流水线、节点回调、launch、DDS 或退化场景验收，由全量入口编排。
- `aim_sim.py`：可复用的合成图像渲染库。
- `submission_test_source.py`、`observe_submission.py`：隔离域的合成提交源与独立 CSV 观察器；供 DDS 验收和现场教程调用。
- `simulate_navigation.py`、`simulate_gazebo_lio.py`、`simulate_gazebo_vision.py`：不同输入层次的仿真运行器。
- `audit_lio_recording.py`、`prepare_lio_diagnostics.py`、`replay_lio_diagnostics.py`：Point-LIO 失败归因，运行方法见[记录](../docs/记录/Point-LIO稀疏场景失败归因.md)。
- `gazebo/`：镜像构建补丁、C++ 回归和仿真辅助代码。
- `visual_tracking/`：交付源码包中的编译、环境和启动脚本。
- `verify_protocol_offline.cpp`：由同名 Python 入口编译，包含 sanitizer 与 PTY 核验。

## 维护约定

模块级测试放入 `src/<包>/test/`，由 pytest 自动收集；跨模块且依赖 bringup
配置的检查放入 `gimbal_tracker_bringup/test/`。需要独立日志、超时或多进程
运行的验收保留为工具入口。不要按整改日期继续增加模块测试脚本。

2026-10-21 迁移的六个旧入口及其新位置见[迁移表](../docs/教程/测试运行指南.md)。
源码包使用稳定目录 `47-visual-tracking`；实际 Git 基线、未提交状态和逐文件哈希写入 `manifest.json`。
