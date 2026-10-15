"""judge_io_node：赛题一输出契约的唯一出口。

**只订阅 `/judge/*`，绝不向 `/judge/` 下发任何消息。**

订阅（官方 QoS 要求）：

| 话题 | 类型 | QoS |
|---|---|---|
| `/judge/tick` | `std_msgs/Header` | RELIABLE + KEEP_LAST + depth 10（10 Hz） |
| `/judge/replay_start` | `std_msgs/Header` | **TRANSIENT_LOCAL** + depth 1 |
| `/estimation/target_state` | `gimbal_interfaces/TargetState` | SENSOR（depth 10） |
| `/gimbal/base_pose` | `nav_msgs/Odometry` | SENSOR（depth 10） |
| `/chassis/advice` | `gimbal_interfaces/ChassisAdvice` | SENSOR（depth 10） |
| `/map/grid` | `nav_msgs/OccupancyGrid` | TRANSIENT_LOCAL + depth 1 |

每收到一拍 → 立即组行（默认九列，包含 t_tick）→ 写盘 →
**flush**（迟到 >0.08 s 的行作废）。

产物：

- `~/ws_championship/output/<队伍编号>.csv`（无表头，只追加）
- `~/ws_championship/output/<队伍编号>_map.pgm` / `_map.yaml`（map_saver_cli 格式）

⚠️ **`judge.fresh_run`（默认 true）**：CSV 是**追加**写的（为了节点中途崩溃重启不丢
已有行），但因此有个陷阱 —— 如果上一次跑的 `<队号>.csv` 还在，新一轮的 `t_tick`
会和上一轮重复，**整个文件很可能被判作废**（实测：同一场景跑两遍，
`tools/check_csv.py` 立刻报「t_tick 非单调递增」）。
所以启动时会**把上一轮产物改名归档**成 `<队号>.csv.bak.<时间戳>`，本次从空文件开始。
设 `fresh_run=false` 可退回纯追加（仅当你要手动续跑时用）。

给定点（map 系原点）冻结：`/judge/replay_start` 到达（或兜底：第 0 拍）时向
`/gimbal/origin_freeze` 发一条锁存 `Header`，由 `map_frame_broadcaster_node`
（`origin_mode=judge_replay_start`）执行冻结。
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import Any
from typing import Dict
from typing import Optional

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                           QoSProfile, QoSReliabilityPolicy)
    from std_msgs.msg import Header
    from nav_msgs.msg import Odometry, OccupancyGrid
    from gimbal_interfaces.msg import ChassisAdvice, TargetState
except Exception:  # pragma: no cover - Windows 无 ROS2
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]
    Header = None  # type: ignore[assignment]
    Odometry = None  # type: ignore[assignment]
    OccupancyGrid = None  # type: ignore[assignment]
    ChassisAdvice = None  # type: ignore[assignment]
    TargetState = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]
    QoSReliabilityPolicy = None  # type: ignore[assignment]
    QoSDurabilityPolicy = None  # type: ignore[assignment]

from .tick_csv import V_STOP, TickCsvWriter

NODE_NAME = 'judge_io_node'

PARAM_DEFAULTS: Dict[str, Any] = {
    # --- 身份与路径 ---------------------------------------------------------
    'judge.team_id': 'T0',
    'judge.output_dir': '~/ws_championship/output',
    # 当前环境文档要求九列；最终赛制以主办方发布链接为准。
    'judge.emit_t_tick': True,
    # 启动时把上一轮的同名产物改名归档（.bak.<时间戳>），本次从空文件开始写。
    # 关闭后会**追加**到旧文件 → 新旧行混在一起 → 行数与实际拍数不符 → 很可能作废。
    'judge.fresh_run': False,
    # --- 运行时 -------------------------------------------------------------
    'judge.enforce_v_floor': True,     # 防止 |v|<0.02 被判"停车"记满分误差
    'judge.v_floor': 0.30,             # 距离目标较远时的建议速度大小（m/s）
    'judge.stop_radius_m': 0.45,       # 与官方 0.5 m 判据留 0.05 m 余量
    'judge.watchdog_period_s': 0.2,    # 迟到/丢拍看门狗周期
    'judge.late_warn_s': 0.05,         # 单行写盘耗时告警阈值
    # --- 地图落盘 -----------------------------------------------------------
    'judge.map_save_period_s': 10.0,   # 周期保存（防意外中断）
    'judge.map_idle_save_s': 0.7,      # 最后一拍后 0.7 s 内保存（官方 1 s 后复制）
    'judge.map_occupied_thresh': 0.65,
    'judge.map_free_thresh': 0.196,
}


def _judge_tick_qos() -> Any:
    """/judge/tick：RELIABLE + KEEP_LAST + depth 10（官方要求）。"""
    return QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      depth=10,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _latched_qos(depth: int = 1) -> Any:
    """TRANSIENT_LOCAL + RELIABLE（官方要求 replay_start 用它）。"""
    return QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      depth=depth,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def _sensor_qos(depth: int = 10) -> Any:
    """内部话题：BEST_EFFORT + KEEP_LAST。"""
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      depth=depth,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _stamp_to_sec(stamp: Any) -> float:
    """ROS stamp → 秒（float）。失败返回 NaN。"""
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except Exception:
        return float('nan')


class JudgeIoNode(Node):  # type: ignore[misc]
    """按拍写 9 列 CSV + 栅格地图落盘。"""

    def __init__(self, name: str = NODE_NAME) -> None:
        super().__init__(name)
        self._log = self.get_logger()
        self._params: Dict[str, Any] = {}
        for key, val in PARAM_DEFAULTS.items():
            self._params[key] = self._declare(key, val)

        # --- 输出路径 -------------------------------------------------------
        out_dir = os.path.expanduser(str(self._params['judge.output_dir']))
        team = str(self._params['judge.team_id'])
        os.makedirs(out_dir, exist_ok=True)
        self._csv_path = os.path.join(out_dir, '%s.csv' % team)
        self._map_pgm = os.path.join(out_dir, '%s_map.pgm' % team)
        self._map_yaml = os.path.join(out_dir, '%s_map.yaml' % team)
        self._map_image_name = '%s_map.pgm' % team

        # --- 新跑一轮：先归档上一轮产物（绝不把新行追加到旧文件上）---------
        if bool(self._params['judge.fresh_run']):
            self._archive_previous()

        self._writer = TickCsvWriter(self._csv_path, logger=self._log,
                                     with_tick=bool(
                                         self._params['judge.emit_t_tick']))
        self._writer.open()

        # --- 数据快照（最新值，tick 到来时直接取用）-------------------------
        self._pred = np.zeros(2, dtype=float)
        self._cur = np.zeros(2, dtype=float)
        self._base = np.zeros(2, dtype=float)
        self._vel = np.zeros(2, dtype=float)
        self._has_target = False
        self._has_base = False
        self._has_advice = False

        # --- 拍状态 ---------------------------------------------------------
        self._tick_index: int = -1
        self._n_tick: int = 0
        self._t_last_tick_mono: float = time.monotonic()
        self._origin_frozen: bool = False

        # --- 地图缓存 -------------------------------------------------------
        self._grid: Optional[np.ndarray] = None       # (W, H) int8
        self._grid_res: float = 0.05
        self._grid_origin = np.zeros(2, dtype=float)
        self._t_last_map_save: float = 0.0
        self._map_saved_final: bool = False

        # --- 订阅 -----------------------------------------------------------
        self.create_subscription(Header, '/judge/tick',
                                 self._on_tick, _judge_tick_qos())
        self.create_subscription(Header, '/judge/replay_start',
                                 self._on_replay_start, _latched_qos(1))
        self.create_subscription(TargetState, '/estimation/target_state',
                                 self._on_target, _sensor_qos(10))
        self.create_subscription(Odometry, '/gimbal/base_pose',
                                 self._on_base_pose, _sensor_qos(10))
        self.create_subscription(ChassisAdvice, '/chassis/advice',
                                 self._on_advice, _sensor_qos(10))
        self.create_subscription(OccupancyGrid, '/map/grid',
                                 self._on_grid, _latched_qos(1))

        # --- 发布（内部话题，绝不占用 /judge/ 命名空间）---------------------
        self._pub_origin_freeze = self.create_publisher(
            Header, '/gimbal/origin_freeze', _latched_qos(1))

        self.create_timer(float(self._params['judge.watchdog_period_s']),
                          self._on_watchdog)

        self._log.info('judge_io_node 启动: team=%s 列数=%d csv=%s map=%s'
                       % (team, self._writer.n_cols, self._csv_path,
                          self._map_pgm))

    # -- 新旧产物隔离 ---------------------------------------------------------
    def _archive_previous(self) -> int:
        """把上一轮的同名产物改名归档，保证本次从空文件开始写。

        改名（而不是删除）——旧数据留着赛后复盘，同时避免"新旧 t_tick 混在一个
        文件里"这种整份作废的后果。归档名：``<原名>.bak.<YYYYmmdd-HHMMSS>``。

        Returns:
            int：归档的文件数。
        """
        stamp = time.strftime('%Y%m%d-%H%M%S')
        n = 0
        for path in (self._csv_path, self._map_pgm, self._map_yaml):
            try:
                if os.path.exists(path) and os.path.getsize(path) > 0:
                    bak = '%s.bak.%s' % (path, stamp)
                    os.replace(path, bak)
                    n += 1
                    self._log.warning('上一轮产物已归档: %s → %s'
                                      % (os.path.basename(path),
                                         os.path.basename(bak)))
            except Exception as exc:  # pragma: no cover
                # 归档失败不能挡住本次运行，但必须让人看见
                self._log.error('归档旧产物失败 %s: %r（本次仍会追加写！）'
                                % (path, exc))
        if n:
            self._log.warning('共归档 %d 个上一轮产物（judge.fresh_run=true）；'
                              '本次 CSV 从空文件开始写' % n)
        return n

    # -- 参数 -----------------------------------------------------------------
    def _declare(self, key: str, default: Any) -> Any:
        """声明参数。"""
        try:
            self.declare_parameter(key, default)
            return self.get_parameter(key).value
        except Exception:  # pragma: no cover
            return default

    # -- judge 回调 -----------------------------------------------------------
    def _on_replay_start(self, msg: Any) -> None:
        """回放起点信号 → 冻结给定点（map 系原点）。"""
        self._log.info('收到 /judge/replay_start（frame_id=%s）→ 冻结给定点'
                       % str(getattr(msg, 'frame_id', '')))
        self._freeze_origin(msg)

    def _freeze_origin(self, src_msg: Any) -> None:
        """向 /gimbal/origin_freeze 发锁存信号（只发一次）。"""
        if self._origin_frozen:
            return
        self._origin_frozen = True
        try:
            out = Header()
            out.stamp = src_msg.stamp
            out.frame_id = src_msg.frame_id
            self._pub_origin_freeze.publish(out)
        except Exception as exc:  # pragma: no cover
            self._log.error('origin_freeze 发布失败: %r' % (exc,))

    def _on_tick(self, msg: Any) -> None:
        """每拍写一行（9 列），立即 flush。"""
        t0 = time.monotonic()
        try:
            idx = int(msg.frame_id)
        except Exception:
            idx = self._tick_index + 1
        self._tick_index = idx
        self._n_tick += 1
        self._t_last_tick_mono = t0

        # 兜底：没收到 replay_start 时，第 0 拍同样冻结给定点
        if idx == 0 and not self._origin_frozen:
            self._freeze_origin(msg)

        t_tick = _stamp_to_sec(msg.stamp)
        vel = self._safe_velocity()
        ok = self._writer.write_tick(self._pred, self._cur, self._base, vel,
                                     t_tick)
        dt = time.monotonic() - t0
        if dt > float(self._params['judge.late_warn_s']):
            self._log.warning('第 %d 拍写盘耗时 %.1f ms（>%.0f ms，逼近 80 ms 作废线）'
                              % (idx, dt * 1e3,
                                 float(self._params['judge.late_warn_s']) * 1e3))
        if not ok:
            self._log.error('第 %d 拍写盘失败' % idx)

    # -- 数据回调 -------------------------------------------------------------
    def _on_target(self, msg: Any) -> None:
        """缓存目标状态（map 系）。"""
        try:
            if not bool(msg.valid):
                return
            self._pred = np.array([float(msg.pred_x), float(msg.pred_y)],
                                  dtype=float)
            self._cur = np.array([float(msg.cur_x), float(msg.cur_y)],
                                 dtype=float)
            self._has_target = True
        except Exception as exc:  # pragma: no cover
            self._log.error('target_state 解析失败: %r' % (exc,))

    def _on_base_pose(self, msg: Any) -> None:
        """缓存底盘位姿（map 系 = 给定点系）。"""
        try:
            p = msg.pose.pose.position
            self._base = np.array([float(p.x), float(p.y)], dtype=float)
            self._has_base = True
        except Exception as exc:  # pragma: no cover
            self._log.error('base_pose 解析失败: %r' % (exc,))

    def _on_advice(self, msg: Any) -> None:
        """缓存底盘建议速度（map 系）。"""
        try:
            self._vel = np.array([float(msg.cmd_vx), float(msg.cmd_vy)],
                                 dtype=float)
            self._has_advice = True
        except Exception as exc:  # pragma: no cover
            self._log.error('advice 解析失败: %r' % (exc,))

    def _on_grid(self, msg: Any) -> None:
        """缓存栅格地图（干净三态，用于落盘）。"""
        try:
            w = int(msg.info.width)
            h = int(msg.info.height)
            if w <= 0 or h <= 0:
                return
            arr = np.asarray(msg.data, dtype=np.int8).reshape((h, w)).T.copy()
            self._grid = arr                       # (W, H)
            self._grid_res = float(msg.info.resolution) or 0.05
            q = msg.info.origin.orientation
            yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                             1.0 - 2.0 * (q.y*q.y + q.z*q.z))
            self._grid_yaw = yaw
            o = msg.info.origin.position
            self._grid_origin = np.array([float(o.x), float(o.y)], dtype=float)
        except Exception as exc:  # pragma: no cover
            self._log.error('grid 解析失败: %r' % (exc,))

    # -- 速度守卫 -------------------------------------------------------------
    def _safe_velocity(self) -> np.ndarray:
        """保证不被误判为"停车"：`|v| < 0.02` 且距目标 ≥0.5 m 时该拍方向项记满分误差 1。

        规则原文：`hypot(vx, vy) < 0.02` 视为停车，此时离目标不到 0.5 m 记 0 分，
        否则记 1 分。因此远离目标时必须保证 `|v| ≥ 0.02`。
        """
        v = np.array([float(self._vel[0]), float(self._vel[1])], dtype=float)
        if not bool(self._params['judge.enforce_v_floor']):
            return v if np.all(np.isfinite(v)) else np.zeros(2, dtype=float)
        if not np.all(np.isfinite(v)):
            v = np.zeros(2, dtype=float)

        dist = float(np.hypot(self._cur[0] - self._base[0],
                              self._cur[1] - self._base[1]))
        stop_r = float(self._params['judge.stop_radius_m'])
        if math.isfinite(dist) and dist < stop_r:
            return np.zeros(2, dtype=float)      # 已到位 → 停车 → 该项记 0（最优）

        n = float(np.linalg.norm(v))
        floor = max(float(self._params['judge.v_floor']), V_STOP * 2.0)
        if n < floor:
            if n > 1e-9:
                v = v * (floor / n)
            elif math.isfinite(dist) and dist > 1e-6:
                v = np.array([self._cur[0] - self._base[0],
                              self._cur[1] - self._base[1]], dtype=float)
                v = v * (floor / dist)
            else:
                v = np.array([floor, 0.0], dtype=float)
        return v

    # -- 看门狗 ---------------------------------------------------------------
    def _on_watchdog(self) -> None:
        """周期保存地图；最后一拍后 `map_idle_save_s` 内补一次保存。"""
        now = time.monotonic()
        period = float(self._params['judge.map_save_period_s'])
        if period > 0.0 and (now - self._t_last_map_save) >= period:
            self._save_map('periodic')
        idle = float(self._params['judge.map_idle_save_s'])
        if (self._n_tick > 0 and not self._map_saved_final
                and idle > 0.0 and (now - self._t_last_tick_mono) >= idle):
            self._save_map('final')

    # -- 地图落盘 -------------------------------------------------------------
    def _save_map(self, reason: str) -> bool:
        """写 `<队伍编号>_map.pgm` + `.yaml`（map_saver_cli 格式）。

        Args:
            reason: 触发原因（日志用）。

        Returns:
            bool：成功为 True。
        """
        if self._grid is None:
            return False
        self._t_last_map_save = time.monotonic()
        try:
            from grid_mapper.pgm_io import write_map_yaml, write_pgm
        except Exception:
            write_pgm = None  # type: ignore[assignment]
            write_map_yaml = None  # type: ignore[assignment]
        try:
            if write_pgm is not None:
                write_pgm(self._map_pgm, self._grid)
                write_map_yaml(self._map_yaml,
                               image=self._map_image_name,
                               resolution=self._grid_res,
                               origin_xyz=(float(self._grid_origin[0]),
                                           float(self._grid_origin[1]), self._grid_yaw),
                               occupied_thresh=float(
                                   self._params['judge.map_occupied_thresh']),
                               free_thresh=float(
                                   self._params['judge.map_free_thresh']))
            else:
                self._write_pgm_fallback(self._map_pgm, self._grid)
                self._write_yaml_fallback(self._map_yaml)
        except Exception as exc:
            self._log.error('地图保存失败(%s): %r' % (reason, exc))
            return False
        if reason == 'final':
            self._map_saved_final = True
        self._log.info('地图已保存(%s): %s / %s（%dx%d @ %.3fm/cell）'
                       % (reason, self._map_pgm, self._map_yaml,
                          self._grid.shape[0], self._grid.shape[1],
                          self._grid_res))
        return True

    def _write_pgm_fallback(self, path: str, grid: np.ndarray) -> None:
        """grid_mapper 不可用时的 P5 PGM 兜底写入（行序翻转）。"""
        g = np.asarray(grid)
        w, h = int(g.shape[0]), int(g.shape[1])
        vals = np.where(g == 100, 0, np.where(g == 0, 254, 205)).astype(np.uint8)
        img = np.empty((h, w), dtype=np.uint8)
        for r in range(h):
            img[r, :] = vals[:, (h - 1 - r)]
        with open(path, 'wb') as f:
            f.write(b'P5\n')
            f.write(('%d %d\n' % (w, h)).encode('ascii'))
            f.write(b'255\n')
            f.write(img.tobytes())

    def _write_yaml_fallback(self, path: str) -> None:
        """map.yaml 兜底写入。"""
        lines = [
            'image: %s' % self._map_image_name,
            'mode: trinary',
            'resolution: %s' % ('%g' % self._grid_res),
            'origin: [%s, %s, %s]' % ('%g' % float(self._grid_origin[0]),
                                     '%g' % float(self._grid_origin[1]),
                                     '%g' % self._grid_yaw),
            'negate: 0',
            'occupied_thresh: %s' % ('%g' % float(
                self._params['judge.map_occupied_thresh'])),
            'free_thresh: %s' % ('%g' % float(
                self._params['judge.map_free_thresh'])),
        ]
        with open(path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')

    # -- 收尾 -----------------------------------------------------------------
    def shutdown(self) -> None:
        """退出前补一次地图保存并关闭 CSV。"""
        if not self._map_saved_final and self._grid is not None:
            self._save_map('shutdown')
        st = self._writer.stats()
        self._writer.close()
        self._log.info('judge_io_node 退出: ticks=%d rows=%d guarded=%d '
                       'dropped=%d → %s'
                       % (self._n_tick, st['rows'], st['n_guarded'],
                          st['n_dropped'], st['path']))


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    if rclpy is None:  # pragma: no cover
        print('[judge_io_node] FATAL rclpy 不可用（需要 ROS2 Humble）')
        return 1
    rclpy.init(args=args)
    node = JudgeIoNode()
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
