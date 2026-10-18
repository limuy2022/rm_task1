"""pytest 公共设施：路径注入。

本目录的三个测试是 T05「集成联调层」的回归网，直接依赖真 ROS2 Humble
（验证环境见仓库根 Dockerfile；运行前需 source 安装空间以获得
gimbal_interfaces 等生成的消息包）：

| 文件 | 验证对象 | 手法 |
|---|---|---|
| `test_launch_sim.py` | `launch/*.launch.py` 的 `_launch_setup` | 真 launch 模块加载 + LaunchContext 内省生成的 action |
| `test_node_itest.py` | `gimbal_debug/gt_error_monitor_node.py` | 真 rclpy/真消息类型 + 手动定时器注入节点 |
| `test_consistency.py` | 全工程静态一致性 | 纯文本/AST 扫描（entry_points、话题、参数、禁用字面量、墙上时钟） |

设计要点
--------
* **自包含**：所有路径由 `__file__` 推导，不使用绝对路径；临时产物一律用
  pytest 的 `tmp_path`。
* **不假装通过**：环境不满足的项用 `pytest.skip` 明确跳过，不降级、不装桩。
"""

from __future__ import annotations

import os
import sys

# ---------------------------------------------------------------------------
# 路径（全部由 __file__ 推导，无绝对路径）
# ---------------------------------------------------------------------------
TEST_DIR: str = os.path.dirname(os.path.abspath(__file__))
PKG_DIR: str = os.path.abspath(os.path.join(TEST_DIR, '..'))        # src/gimbal_tracker_bringup
SRC_DIR: str = os.path.abspath(os.path.join(PKG_DIR, '..'))         # src
WS_DIR: str = os.path.abspath(os.path.join(SRC_DIR, '..'))          # 工作区根

CONFIG_DIR: str = os.path.join(PKG_DIR, 'config')
LAUNCH_DIR: str = os.path.join(PKG_DIR, 'launch')
RVIZ_DIR: str = os.path.join(PKG_DIR, 'rviz')

VIRTUAL_YAML: str = os.path.join(CONFIG_DIR, 'virtual.yaml')
FIELD_YAML: str = os.path.join(CONFIG_DIR, 'field.yaml')
TRACKER_PARAMS: str = os.path.join(CONFIG_DIR, 'tracker_params.yaml')
RViz_CONFIG: str = os.path.join(RVIZ_DIR, 'gimbal_tracker.rviz')

for _p in (PKG_DIR,
           os.path.join(SRC_DIR, 'gimbal_common'),
           os.path.join(SRC_DIR, 'gimbal_debug')):
    if _p not in sys.path:
        sys.path.insert(0, _p)
