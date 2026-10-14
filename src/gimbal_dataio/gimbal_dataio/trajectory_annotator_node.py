"""trajectory_annotator_node：缓存目标/底盘轨迹，退出时绘制 `map_tracked.png`。

对应 DESIGN §3.11 / §6.11。

订阅（§4.3）：
- `/estimation/target_state`（`gimbal_interfaces/TargetState`）
- `/gimbal/base_pose`（`nav_msgs/Odometry`，frame=map）
- `/map/grid`（`nav_msgs/OccupancyGrid`，一次性，TRANSIENT_LOCAL depth=1）

产物：`outputs/<run_id>/map_tracked.png`，必须标注（DESIGN §6.11）：
目标估计轨迹(蓝)、目标当前(红点)、预测(红×)、底盘轨迹(绿)、底盘当前(绿方块)、
给定点原点(品红十字)。

⚠ 本包依赖 `grid_mapper` 的 `GridMap` / `png_draw`（同一工作区内，已在 package.xml 声明）。

⚠ **待 Ubuntu 24.04 + ROS2 Jazzy 编译验证**：本机为 Windows，无 ROS2，
本文件只做了静态一致性检查，未经 `colcon build` 与运行时验证。
"""

from __future__ import annotations

import os
import sys
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy
    from rclpy.qos import QoSHistoryPolicy
    from rclpy.qos import QoSProfile
    from nav_msgs.msg import Odometry, OccupancyGrid
    from gimbal_interfaces.msg import TargetState
except Exception:  # pragma: no cover - Windows 无 ROS2
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]
    Odometry = None  # type: ignore[assignment]
    OccupancyGrid = None  # type: ignore[assignment]
    TargetState = None  # type: ignore[assignment]

    class _QoSProfile:  # noqa: D401 - 占位
        """ROS 缺失时的占位类。"""

        def __init__(self, **kwargs: Any) -> None:
            self.kwargs = kwargs

    QoSProfile = _QoSProfile  # type: ignore[assignment]
    QoSDurabilityPolicy = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]

try:  # 优先用 T01 的 gimbal_common
    from gimbal_common.time_utils import now
    from gimbal_common.log_utils import get_logger, throttled
except Exception:  # pragma: no cover - Windows 无 ROS2 兜底
    from ._compat import get_logger, now, throttled

from grid_mapper.grid_builder import GridMap
from grid_mapper.png_draw import downsample, draw_tracked

# 默认参数
DEFAULTS: Dict[str, Any] = {
    'csv.out_dir': 'outputs',
    'csv.run_id': 'latest',
    'annotator.max_points': 20000,        # 每条轨迹最多缓存的点数
    'annotator.png_name': 'map_tracked.png',
}

_SENSOR_QOS_DEPTH: int = 10


class TrajectoryAnnotatorNode(Node):  # type: ignore[misc]
    """轨迹缓存与标注图绘制节点。"""

    def __init__(self, name: str = 'trajectory_annotator_node') -> None:
        super().__init__(name)
        self._log = get_logger(self, name)
        self._params: Dict[str, Any] = {}
        for key, val in DEFAULTS.items():
            self._params[key] = self._declare(key, val)

        self._t_start = now()
        self._grid: Optional[GridMap] = None
        self._target_traj: List[Tuple[float, float]] = []
        self._base_traj: List[Tuple[float, float]] = []
        self._target_cur: Optional[Tuple[float, float]] = None
        self._target_pred: Optional[Tuple[float, float]] = None
        self._base_cur: Optional[Tuple[float, float]] = None
        self._max_points = int(self._params['annotator.max_points'])

        sensor_qos = self._sensor_qos()
        self.create_subscription(TargetState, '/estimation/target_state',
                                 self._on_target, sensor_qos)
        self.create_subscription(Odometry, '/gimbal/base_pose',
                                 self._on_base_pose, sensor_qos)
        self.create_subscription(OccupancyGrid, '/map/grid',
                                 self._on_grid, self._latched_qos())
        self._log.info('trajectory_annotator_node 启动')

    # -- ROS 胶水 -------------------------------------------------------------
    def _declare(self, key: str, default: Any) -> Any:
        """声明参数。"""
        try:
            self.declare_parameter(key, default)
            return self.get_parameter(key).value
        except Exception:  # pragma: no cover
            return default

    @staticmethod
    def _sensor_qos() -> Any:
        """SENSOR_DATA 风格 QoS。"""
        if QoSProfile is None:  # pragma: no cover
            return _SENSOR_QOS_DEPTH
        try:
            return QoSProfile(depth=_SENSOR_QOS_DEPTH,
                              history=QoSHistoryPolicy.KEEP_LAST)
        except Exception:  # pragma: no cover
            return QoSProfile(depth=_SENSOR_QOS_DEPTH)

    @staticmethod
    def _latched_qos() -> Any:
        """TRANSIENT_LOCAL + depth=1。"""
        if QoSProfile is None:  # pragma: no cover
            return 1
        try:
            return QoSProfile(depth=1,
                              history=QoSHistoryPolicy.KEEP_LAST,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        except Exception:  # pragma: no cover
            return QoSProfile(depth=1)

    # -- 回调 -----------------------------------------------------------------
    def _on_grid(self, msg: Any) -> None:
        """接收 `/map/grid`（干净三态），作为标注底图。"""
        try:
            info = msg.info
            self._grid = GridMap.from_occupancy_grid(
                int(info.width), int(info.height), float(info.resolution),
                [float(info.origin.position.x), float(info.origin.position.y)],
                list(msg.data))
            self._log.info('已接收 /map/grid: %dx%d @ %.3f'
                           % (int(info.width), int(info.height),
                              float(info.resolution)))
        except Exception as exc:
            throttled(self._log, 'error', '/map/grid 解析失败: %r' % (exc,),
                      'annotator.grid_err', 1.0)

    def _on_target(self, msg: Any) -> None:
        """缓存目标当前/预测位置与轨迹。"""
        try:
            if not bool(msg.valid):
                return
            cx = float(msg.cur_x)
            cy = float(msg.cur_y)
            px = float(msg.pred_x)
            py = float(msg.pred_y)
            if not (np.isfinite(cx) and np.isfinite(cy)):
                return
            self._append(self._target_traj, (cx, cy))
            self._target_cur = (cx, cy)
            if np.isfinite(px) and np.isfinite(py):
                self._target_pred = (px, py)
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'target_state 解析失败: %r' % (exc,),
                      'annotator.target_err', 1.0)

    def _on_base_pose(self, msg: Any) -> None:
        """缓存底盘位置与轨迹。"""
        try:
            p = msg.pose.pose.position
            x = float(p.x)
            y = float(p.y)
            if not (np.isfinite(x) and np.isfinite(y)):
                return
            self._append(self._base_traj, (x, y))
            self._base_cur = (x, y)
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'base_pose 解析失败: %r' % (exc,),
                      'annotator.base_err', 1.0)

    def _append(self, buf: List[Tuple[float, float]],
                pt: Tuple[float, float]) -> None:
        """追加轨迹点，超过上限时抽稀一半。"""
        buf.append(pt)
        if len(buf) > self._max_points:
            del buf[0::2]

    # -- 收尾 -----------------------------------------------------------------
    def shutdown(self) -> None:
        """退出时绘制 `map_tracked.png`（§6.14 第 4 条）。"""
        run_dir = os.path.join(str(self._params['csv.out_dir']),
                               str(self._params['csv.run_id']))
        png_path = os.path.join(run_dir, str(self._params['annotator.png_name']))
        if self._grid is None:
            self._log.warn('未收到 /map/grid，跳过 map_tracked.png')
            return
        try:
            draw_tracked(self._grid,
                         png_path,
                         target_traj=downsample(self._target_traj),
                         target_cur=self._target_cur,
                         target_pred=self._target_pred,
                         base_traj=downsample(self._base_traj),
                         base_cur=self._base_cur,
                         given_point=(0.0, 0.0))
            self._log.info('map_tracked.png 已落盘: %s（target=%d 点, base=%d 点）'
                           % (png_path, len(self._target_traj), len(self._base_traj)))
        except Exception as exc:
            self._log.error('map_tracked.png 绘制失败: %r' % (exc,))
        self._log.info('trajectory_annotator_node 退出，耗时 %.2fs'
                       % (now() - self._t_start))


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    if rclpy is None:  # pragma: no cover
        print('[trajectory_annotator_node] FATAL rclpy 不可用（需要 ROS2 Jazzy）')
        return 1
    rclpy.init(args=args)
    node = TrajectoryAnnotatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
