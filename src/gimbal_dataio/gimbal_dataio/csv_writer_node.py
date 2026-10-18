"""csv_writer_node：20Hz 单调定时器写盘（**严格 8 列**）。

对应 DESIGN §3.11 / §6.12。

订阅（§4.3）：
- `/estimation/target_state`（`gimbal_interfaces/TargetState`）
- `/gimbal/base_pose`（`nav_msgs/Odometry`，frame=map）
- `/chassis/advice`（`gimbal_interfaces/ChassisAdvice`）

产物：
- `outputs/<run_id>/result.csv`（无表头，**恰好 8 列**）
- `outputs/<run_id>/result_with_header.csv`（同数据 + 首行表头）
- `outputs/<run_id>/run_meta.json`

时钟（主理人拍板 1，混合时钟）：
- `header.stamp` 用 ROS 系统时钟；**CSV 时间戳与 dt 一律用 `time_utils.now()`（单调）**
- 定时器周期由 rclpy 提供，但**周期内的 dt 与 timestamp 用单调时钟计算**
"""

from __future__ import annotations

import os
import sys
from typing import Any
from typing import Dict
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from gimbal_interfaces.msg import ChassisAdvice, TargetState

from gimbal_common.time_utils import now
from gimbal_common.log_utils import get_logger, throttled

from .csv_writer import CsvWriter
from .run_meta import RunMeta

# 默认参数（DESIGN §6.13 `csv.*`）
DEFAULTS: Dict[str, Any] = {
    'csv.out_dir': 'outputs',
    'csv.run_id': 'latest',
    'csv.rate_hz': 20.0,
    'csv.csv_extra_cols': False,          # ⚠ 修正 2：默认严格 8 列
    'csv.flush_period_s': 1.0,
    'csv.write_after_first_estimate': True,
}

_SENSOR_QOS_DEPTH: int = 10


class CsvWriterNode(Node):  # type: ignore[misc]
    """20Hz 写盘节点。"""

    def __init__(self, name: str = 'csv_writer_node') -> None:
        super().__init__(name)
        self._log = get_logger(self, name)
        self._params: Dict[str, Any] = {}
        for key, val in DEFAULTS.items():
            self._params[key] = self._declare(key, val)

        self._t_start = now()
        self._t0: Optional[float] = None            # 首个有效估计的单调时刻
        self._writer = CsvWriter(
            extra_cols=bool(self._params['csv.csv_extra_cols']),
            flush_period_s=float(self._params['csv.flush_period_s']),
            logger=self._log)
        self._meta = RunMeta(run_id=str(self._params['csv.run_id']),
                             out_dir=str(self._params['csv.out_dir']),
                             t0_monotonic=self._t_start)

        # 最新数据快照
        self._pred = np.zeros(2, dtype=float)
        self._cur = np.zeros(2, dtype=float)
        self._base = np.zeros(2, dtype=float)
        self._cmd = np.zeros(2, dtype=float)
        self._track_status: int = 0
        self._has_target: bool = False
        self._has_base: bool = False
        self._has_cmd: bool = False
        self._started: bool = not bool(self._params['csv.write_after_first_estimate'])

        run_dir = os.path.join(str(self._params['csv.out_dir']),
                               str(self._params['csv.run_id']))
        ok = self._writer.open(os.path.join(run_dir, 'result.csv'))
        if not ok:
            self._log.error('CSV 文件打开失败，节点将不写盘')
        else:
            self._log.info('CSV 已打开: %s（%d 列，extra_cols=%s）'
                           % (self._writer.path,
                              10 if self._writer.extra_cols else 8,
                              self._writer.extra_cols))

        sensor_qos = self._sensor_qos()
        self.create_subscription(TargetState, '/estimation/target_state',
                                 self._on_target, sensor_qos)
        self.create_subscription(Odometry, '/gimbal/base_pose',
                                 self._on_base_pose, sensor_qos)
        self.create_subscription(ChassisAdvice, '/chassis/advice',
                                 self._on_advice, sensor_qos)

        period = 1.0 / max(float(self._params['csv.rate_hz']), 1e-3)
        self.create_timer(period, self._on_timer)
        self._log.info('csv_writer_node 启动: %.1fHz → %s'
                       % (float(self._params['csv.rate_hz']), run_dir))

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
        try:
            return QoSProfile(depth=_SENSOR_QOS_DEPTH,
                              history=QoSHistoryPolicy.KEEP_LAST)
        except Exception:  # pragma: no cover
            return QoSProfile(depth=_SENSOR_QOS_DEPTH)

    # -- 回调 -----------------------------------------------------------------
    def _on_target(self, msg: Any) -> None:
        """缓存目标状态（pred / cur / track_status）。"""
        try:
            if bool(msg.valid):
                self._pred = np.array([float(msg.pred_x), float(msg.pred_y)],
                                      dtype=float)
                self._cur = np.array([float(msg.cur_x), float(msg.cur_y)],
                                     dtype=float)
                self._track_status = int(msg.track_status)
                self._has_target = True
                if self._t0 is None:
                    self._t0 = now()
                    self._started = True
                    self._log.info('收到首个有效估计，开始写盘（t0=%.3f）' % self._t0)
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'target_state 解析失败: %r' % (exc,),
                      'csv_writer.target_err', 1.0)

    def _on_base_pose(self, msg: Any) -> None:
        """缓存底盘位姿（map 系）。"""
        try:
            p = msg.pose.pose.position
            self._base = np.array([float(p.x), float(p.y)], dtype=float)
            self._has_base = True
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'base_pose 解析失败: %r' % (exc,),
                      'csv_writer.base_err', 1.0)

    def _on_advice(self, msg: Any) -> None:
        """缓存底盘建议速度。"""
        try:
            self._cmd = np.array([float(msg.cmd_vx), float(msg.cmd_vy)],
                                 dtype=float)
            self._has_cmd = True
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'advice 解析失败: %r' % (exc,),
                      'csv_writer.advice_err', 1.0)

    # -- 主循环 ---------------------------------------------------------------
    def _on_timer(self) -> None:
        """20Hz：组一行 8 列并写出。时间戳用单调时钟相对 t0。"""
        if not self._started:
            return
        t_now = now()
        if self._t0 is None:
            self._t0 = t_now
        ts = t_now - self._t0
        self._writer.write_estimate(self._pred, self._cur, self._base, self._cmd,
                                    timestamp=ts, track_status=self._track_status)
        if not (self._has_target and self._has_base and self._has_cmd):
            throttled(self._log, 'warn',
                      '数据未齐(target=%s base=%s cmd=%s)，仍按当前值写盘'
                      % (self._has_target, self._has_base, self._has_cmd),
                      'csv_writer.incomplete', 1.0)

    # -- 收尾 -----------------------------------------------------------------
    def shutdown(self) -> None:
        """flush → close → 写 run_meta.json（§6.14 第 3 条）。"""
        try:
            self._writer.flush()
        finally:
            self._writer.close()
        st = self._writer.stats()
        run_dir = os.path.join(str(self._params['csv.out_dir']),
                               str(self._params['csv.run_id']))
        self._meta.set_params(self._params)
        self._meta.add_artifact('result.csv', os.path.join(run_dir, 'result.csv'))
        self._meta.add_artifact('result_with_header.csv',
                                os.path.join(run_dir, 'result_with_header.csv'))
        self._meta.add_artifact('map_tracked.png',
                                os.path.join(run_dir, 'map_tracked.png'))
        self._meta.set_extra('csv', st)
        self._meta.set_extra('rows_expected',
                             int(round(float(self._params['csv.rate_hz'])
                                       * max(0.0, now() - self._t_start))))
        path = self._meta.write()
        self._log.info('csv_writer_node 退出: rows=%d cols=%d guarded=%d → %s / %s'
                       % (st['rows'], st['n_cols'], st['n_guarded'],
                          st['path'], path))


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    rclpy.init(args=args)
    node = CsvWriterNode()
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
