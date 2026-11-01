# 工具入口

分时比赛：上午 `bash tools/start_mapping.bash` → 另开终端 `bash tools/save_mapping.bash` → 成功后允许关机。下午 `bash tools/start_match.bash` 默认加载上午地图并重定位，裁判给ID后 `bash tools/set_target.bash ID`，确认重定位/跟踪后裁判开赛。默认装车参数为已安装的 `team43_bringup/config/vehicle_field.yaml`。

所有验证在根 `Dockerfile` 的 ROS 2 Humble 容器中运行，完整命令见
[测试运行指南](../docs/教程/测试运行指南.md)。

## 常用入口

| 用途 | 入口 |
|---|---|
| 完整原始数据录制 | `bash tools/record.bash`；可加 `--seconds 900` 或 `--with-camera`，详见[录制与离线加工](../docs/教程/原始数据录制与离线建图.md) |
| 上午15分钟建图 | `start_mapping.bash`；结束前 `save_mapping.bash` 保存到 `maps/current/` |
| 下午实机比赛一键启动 | `bash tools/start_match.bash target_id:=本轮ID`，见[比赛启动与收尾](../docs/教程/比赛启动与收尾.md)；实验室按[统一验证](../docs/教程/实验室统一验证.md)验收 |
| NUC本队Livox驱动构建 | `bash tools/build_livox.bash`；网络配置用`ros2 run team43_bringup configure_livox`生成，见[整体部署](../docs/教程/整体部署.md#雷达驱动由谁启动) |
| 原始录制专项验收 | `verify_record.py --output-dir 新目录`；包含真实DDS、分片、原样读回、播放、相机与锁存epoch，已纳入全量编排 |
| 全量离线核验 | `verify_all_offline.py`；标准流程先跑 `pytest src`，再加 `--skip-pytest` |
| 分时赛程跨会话验证 | 容器内 `python3 -B tools/verify_split_session.py --count 9000 --output-dir build/offline/split_01`；上午实际保存/停机、下午换位重定位并验证15分钟写盘 |
| 实时建图兼容路径演练 | 容器内 `python3 -B tools/verify_match_workflow.py --count 1800 --output-dir build/offline/match_workflow_01`；真实启动脚本、Point-LIO、视觉、控制、CSV、地图与串口PTY，硬件驱动替换为无SDK进程 |
| 独立集成仿真 | `run_simulation_ci.sh`：`navigation` / `submission` / `gazebo_lio` / `gazebo_vision` |
| CSV 格式检查 / 对照真值评分 | `check_csv.py` / `score_csv.py` |
| 云台闭环产物评分 | `score_bearing_sim.py` |
| 相机标定 / 标定比较 / 参考 PnP | `run_camera_calibration.py` / `compare_calibrations.py` / `run_reference_pnp.bash` |
| 未确认参数的现场证据采集与打包 | `collect_field_evidence.py --seconds 15`；在已加载ROS的现场终端运行，详细操作见[现场终端与采样步骤](../docs/教程/未确认参数说明与确认方法.md#附录-b-剩余数据的现场测量与交接)；脚本选项见该文附录A |
| 现场只读检查 / 判轴观察 | `inspect_nuc.py` / `watch_gimbal_axes.py` |
| 预测精度 / 写盘时延基线 | `benchmark_prediction.py` / `benchmark_submission_realtime.py` |

## 依赖与专项工具

- `verify_*.py`：独立算法流水线、节点回调、launch、DDS 或退化场景验收，由全量入口编排。
- `aim_sim.py`：可复用的合成图像渲染库。
- `submission_test_source.py`、`observe_submission.py`：隔离域的合成提交源与独立 CSV 观察器（真实整机轮次用`--live-state`）；供 DDS 验收和现场教程调用。
- `simulate_navigation.py`、`simulate_gazebo_lio.py`、`simulate_gazebo_vision.py`：不同输入层次的仿真运行器。
- Gazebo 场景、模型、插件源码和辅助模块位于 [`src/team43_gazebo`](../src/team43_gazebo/README.md)，验收入口仍在本目录。
- `verify_protocol_offline.cpp`：由同名 Python 入口编译，包含 sanitizer 与 PTY 核验。

## 维护约定

模块级测试放入 `src/<包>/test/`，由 pytest 自动收集；跨模块且依赖 bringup
配置的检查放入 `team43_bringup/test/`。需要独立日志、超时或多进程
运行的验收保留为工具入口。不要按整改日期继续增加模块测试脚本。

模块回归入口见[测试运行指南](../docs/教程/测试运行指南.md#44-模块回归入口)。
