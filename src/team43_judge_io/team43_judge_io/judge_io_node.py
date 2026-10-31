"""judge_io_node：赛题一输出契约的唯一出口。

**只订阅 `/judge/*`，绝不向 `/judge/` 下发任何消息。**

订阅（官方 QoS 要求）：

| 话题 | 类型 | QoS |
|---|---|---|
| `/judge/tick` | `std_msgs/Header` | RELIABLE + KEEP_LAST + depth 10（10 Hz） |
| `/judge/replay_start` | `std_msgs/Header` | **TRANSIENT_LOCAL** + depth 1 |
| `/estimation/target_state` | `team43_interfaces/TargetState` | SENSOR（depth 10） |
| `/gimbal/base_pose` | `nav_msgs/Odometry` | SENSOR（depth 10） |
| `/chassis/advice` | `team43_interfaces/ChassisAdvice` | SENSOR（depth 10） |
| `/map/grid` | `nav_msgs/OccupancyGrid` | TRANSIENT_LOCAL + depth 1 |

每收到一拍 → 立即组行（默认九列，包含 t_tick）→ 写盘 →
**flush**（迟到 >0.08 s 的行作废）。

产物：

- `~/ws_championship/output/<队伍编号>.csv`（无表头，只追加）
- `~/ws_championship/output/<队伍编号>_map.pgm` / `_map.yaml`（map_saver_cli 格式）

CSV只追加，进程重启不得归档/覆盖已提交行；新轮由主办方清理。
旧半行或损坏文件由写出器报错，不修改历史记录。
目标和底盘按源stamp缓存，tick到来时外推到该tick及tick+0.1秒；过期或无效输入不继续
冒充有效状态。建议只有来源有效时才使用，不在缺少绕障解时强造朝目标的速度。

给定点（map 系原点）冻结：`/judge/replay_start` 到达时向
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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, OccupancyGrid
from team43_interfaces.msg import ChassisAdvice, TargetState


from .tick_csv import V_STOP, TickCsvWriter
from .tick_sampler import TickSampler
from team43_common.log_utils import get_logger
from team43_common import frames
from team43_common.node_utils import declare_params, spin_with_finally, shutdown
from team43_common.time_buffer import stamp_seconds
from team43_common.qos_utils import latched_qos, reliable_qos, sensor_qos

NODE_NAME = 'judge_io_node'

PARAM_DEFAULTS: Dict[str, Any] = {
    # --- 身份与路径 ---------------------------------------------------------
    'judge.team_id': '43',
    'judge.output_dir': '~/ws_championship/output',
    # 已确认当前赛题一输出九列。
    'judge.state_timeout_s': 0.2,
    'judge.observation_timeout_s': 0.5,
    # 正式提交只能追加；新轮清理由主办方负责。自行测试使用新目录。
    'judge.fresh_run': False,
    # --- 运行时 -------------------------------------------------------------
    'judge.enforce_v_floor': True,     # 防止 |v|<0.02 被判"停车"记满分误差
    'judge.v_floor': 0.30,             # 距离目标较远时的建议速度大小（m/s）
    'judge.stop_radius_m': 0.45,       # 与官方 0.5 m 判据留 0.05 m 余量
    'judge.watchdog_period_s': 0.2,    # 迟到/丢拍看门狗周期
    'judge.late_warn_s': 0.05,         # tick源stamp到flush完成的端到端告警阈值
    # --- 地图落盘 -----------------------------------------------------------
    'judge.map_save_period_s': 10.0,   # 周期保存（防意外中断）
    'judge.map_idle_save_s': 0.7,      # 最后一拍后 0.7 s 内保存（官方 1 s 后复制）
    'judge.map_occupied_thresh': 0.65,
    'judge.map_free_thresh': 0.196,
}


def _stamp_to_sec(stamp: Any) -> float:
    """ROS stamp → 秒（float）。失败返回 NaN。"""
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (TypeError, ValueError, OverflowError):  # 外部状态数值/时间戳无效时拒绝该输入，由本拍有效期规则处理。
        return float('nan')


class JudgeIoNode(Node):  # type: ignore[misc]
    """按拍写 9 列 CSV + 栅格地图落盘。"""

    def __init__(self, name: str = NODE_NAME) -> None:
        super().__init__(name)
        self._log = get_logger(self, 'judge_io')
        self._params: Dict[str, Any] = declare_params(self, PARAM_DEFAULTS)

        # --- 输出路径 -------------------------------------------------------
        out_dir = os.path.expanduser(str(self._params['judge.output_dir']))
        team = str(self._params['judge.team_id'])
        os.makedirs(out_dir, exist_ok=True)
        self._csv_path = os.path.join(out_dir, '%s.csv' % team)
        self._map_pgm = os.path.join(out_dir, '%s_map.pgm' % team)
        self._map_yaml = os.path.join(out_dir, '%s_map.yaml' % team)
        self._map_image_name = '%s_map.pgm' % team

        # --- 不允许对既有提交做启动归档；测试轮使用新目录 -----------------
        if bool(self._params['judge.fresh_run']):
            raise ValueError('禁止启动时改名既有提交；测试请选择新的output_dir，正式轮由主办方清理')

        self._writer = TickCsvWriter(self._csv_path, logger=self._log)
        if not self._writer.open():
            raise RuntimeError("CSV打不开，拒绝启动；不修改原有提交")

        # --- 按源stamp缓存，tick选择不晚于自身时刻的状态 ------------------
        self._pred = np.zeros(2, dtype=float)
        self._cur = np.zeros(2, dtype=float)
        self._base = np.zeros(2, dtype=float)
        self._vel = np.zeros(2, dtype=float)
        self._has_target = False
        self._has_base = False
        self._has_advice = False
        self._sampler = TickSampler(float(self._params['judge.state_timeout_s']),
                                    float(self._params['judge.observation_timeout_s']))

        # --- 拍状态 ---------------------------------------------------------
        self._tick_index: int = -1
        self._n_tick: int = 0
        self._t_last_tick_mono: float = time.monotonic()
        self._origin_frozen: bool = False
        self._replay_not_before = self.get_clock().now().nanoseconds*1e-9
        self._replay_stamp = None

        # --- 地图缓存 -------------------------------------------------------
        self._grid: Optional[np.ndarray] = None       # (W, H) int8
        self._grid_res: float = 0.05
        self._grid_origin = np.zeros(2, dtype=float)
        self._grid_yaw = 0.0
        self._t_last_map_save: float = 0.0
        self._map_saved_final: bool = False

        # --- 订阅 -----------------------------------------------------------
        # /judge/tick、replay_start 的 QoS 为官方要求（RELIABLE/VOLATILE 与锁存）。
        self.create_subscription(Header, '/judge/tick',
                                 self._on_tick, reliable_qos())
        self.create_subscription(Header, '/judge/replay_start',
                                 self._on_replay_start, latched_qos(1))
        self.create_subscription(TargetState, '/estimation/target_state',
                                 self._on_target, sensor_qos(10))
        self.create_subscription(Odometry, '/gimbal/base_pose',
                                 self._on_base_pose, sensor_qos(10))
        self.create_subscription(ChassisAdvice, '/chassis/advice',
                                 self._on_advice, sensor_qos(10))
        self.create_subscription(OccupancyGrid, '/map/grid',
                                 self._on_grid, latched_qos(1))

        # --- 发布（内部话题，绝不占用 /judge/ 命名空间）---------------------
        self._pub_origin_freeze = self.create_publisher(
            Header, '/gimbal/origin_freeze', latched_qos(1))

        self.create_timer(float(self._params['judge.watchdog_period_s']),
                          self._on_watchdog)

        self._log.info(f'judge_io_node 启动: team={team} 列数=9 '
                       f'csv={self._csv_path} map={self._map_pgm}')

    # -- judge 回调 -----------------------------------------------------------
    def _on_replay_start(self, msg: Any) -> None:
        """回放起点信号 → 冻结给定点（map 系原点）。"""
        self._freeze_origin(msg)

    def _freeze_origin(self, src_msg: Any) -> None:
        """向 /gimbal/origin_freeze 发锁存信号（只发一次）。"""
        stamp = _stamp_to_sec(src_msg.stamp)
        if not math.isfinite(stamp):
            return
        if stamp < self._replay_not_before:
            self._log.warning(f'忽略启动前旧起点stamp={stamp:.6f}；等待本轮新replay_start')
            return
        if self._origin_frozen:
            if abs(stamp-self._replay_stamp) > 1e-6:
                self._log.error('收到不同回放起点；保留本轮提交，请在新轮前重启处理链')
            return
        self._origin_frozen = True
        self._replay_stamp = stamp
        self._log.info(f'接受本轮起点stamp={stamp:.6f}，通知地图冻结给定点')
        out = Header()
        out.stamp = src_msg.stamp
        out.frame_id = src_msg.frame_id
        self._pub_origin_freeze.publish(out)

    def _on_tick(self, msg: Any) -> None:
        """每拍写一行（9 列），立即 flush。"""
        t0 = time.monotonic()
        try:
            idx = int(msg.frame_id)
        except ValueError:
            # Some replay sources leave frame_id empty; the stamp remains authoritative.
            idx = self._tick_index + 1
        self._tick_index = idx
        self._n_tick += 1
        self._t_last_tick_mono = t0
        self._map_saved_final = False  # 节拍恢复后，下一次停拍仍须保存最终地图

        # 第一拍不一定等于回放起点，不能擅自据此改变比赛原点。
        if idx == 0 and not self._origin_frozen:
            self._log.error('尚未收到replay_start；等待正式起点，不能用第一拍替代')

        t_tick = _stamp_to_sec(msg.stamp)
        received = self.get_clock().now().nanoseconds * 1e-9
        queue_age = received - t_tick
        snap = self._sampler.sample(t_tick)
        self._pred, self._cur, self._base, self._vel = (
            snap.prediction, snap.current, snap.base, snap.advice)
        self._has_target, self._has_base, self._has_advice = (
            snap.target_valid, snap.base_valid, snap.advice_valid)
        if not all((self._has_target, self._has_base, self._has_advice)):
            if idx % 20 == 0:
                self._log.warning(f'tick状态缺失/过期：target={self._has_target} '
                                  f'base={self._has_base} advice={self._has_advice}；'
                                  '仍按时写行')
        vel = self._safe_velocity()
        ok = self._writer.write_tick(self._pred, self._cur, self._base, vel,
                                     t_tick)
        dt = time.monotonic() - t0
        completed = self.get_clock().now().nanoseconds * 1e-9
        latency = completed - t_tick
        self._last_tick_timing = {'queue_s': queue_age, 'callback_s': dt,
                                  'stamp_to_flush_s': latency}
        if not math.isfinite(latency) or queue_age < -.03 or completed < received:
            self._log.error(f'第 {idx} 拍时钟异常：tick与本机ROS时间不一致，不能评估提交延迟')
        elif latency > float(self._params['judge.late_warn_s']):
            self._log.warning(f'第 {idx} 拍端到端 {latency*1e3:.1f} ms '
                              f'（接收前 {queue_age*1e3:.1f} ms，回调 {dt*1e3:.1f} ms；'
                              '80 ms作废线，含评分端轮询须留余量）')
        if not ok:
            self._log.error(f'第 {idx} 拍写盘失败')

    # -- 数据回调 -------------------------------------------------------------
    def _on_target(self, msg: Any) -> None:
        try:
            if msg.header.frame_id != 'map': return
            self._sampler.put_target(stamp_seconds(msg.header.stamp),
                [msg.cur_x,msg.cur_y],[msg.vel_x,msg.vel_y],[msg.acc_x,msg.acc_y],
                valid=bool(msg.valid) and int(msg.track_status)!=0,
                observation_age=float(msg.last_obs_age))
        except (TypeError,ValueError) as exc:  # 外部状态数值/时间戳无效时拒绝该输入，由本拍有效期规则处理。
            self._log.error(f'target_state解析失败: {exc!r}')

    def _on_base_pose(self, msg: Any) -> None:
        try:
            if msg.header.frame_id != 'map' or msg.child_frame_id != 'base_link': return
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            quat=np.array([q.w,q.x,q.y,q.z],float)
            v=msg.twist.twist.linear
            if not np.all(np.isfinite(quat)) or np.linalg.norm(quat)<.5:return
            # nav_msgs/Odometry twist is in child/base coordinates.
            velocity=frames.quat_to_mat(quat)@np.array([v.x,v.y,v.z],float)
            self._sampler.put_base(stamp_seconds(msg.header.stamp),[p.x,p.y],velocity[:2])
        except (TypeError,ValueError) as exc:  # 外部状态数值/时间戳无效时拒绝该输入，由本拍有效期规则处理。
            self._log.error(f'base_pose解析失败: {exc!r}')

    def _on_advice(self, msg: Any) -> None:
        try:
            if msg.header.frame_id != 'map': return
            self._sampler.put_advice(stamp_seconds(msg.header.stamp),[msg.cmd_vx,msg.cmd_vy],
                                    valid=bool(msg.valid))
        except (TypeError,ValueError) as exc:  # 外部状态数值/时间戳无效时拒绝该输入，由本拍有效期规则处理。
            self._log.error(f'advice解析失败: {exc!r}')

    def _on_grid(self, msg: Any) -> None:
        """缓存栅格地图（干净三态，用于落盘）。"""
        try:
            if msg.header.frame_id != 'map':
                raise ValueError('提交地图必须已经换到比赛map，不能只改名')
            w, h = int(msg.info.width), int(msg.info.height)
            res = float(msg.info.resolution)
            if w <= 0 or h <= 0 or not math.isfinite(res) or res <= 0:
                raise ValueError('invalid grid dimensions/resolution')
            q=msg.info.origin.orientation
            quat=np.array([q.w,q.x,q.y,q.z],float)
            if not np.all(np.isfinite(quat)) or np.linalg.norm(quat)<.5:
                raise ValueError('invalid map origin orientation')
            R=frames.quat_to_mat(quat)
            if abs(R[2,0])+abs(R[2,1]) > 1e-6:
                raise ValueError('PGM/YAML requires a planar map')
            raw=np.asarray(msg.data)
            if raw.size != w*h or not np.all(np.isin(raw,[-1,0,100])):
                raise ValueError('grid must have exactly width*height trinary cells')
            arr=raw.astype(np.int8).reshape((h,w)).T.copy()
            yaw = math.atan2(float(R[1,0]), float(R[0,0]))
            o = msg.info.origin.position
            origin = np.array([float(o.x), float(o.y)], dtype=float)
            if not np.all(np.isfinite(origin)):
                raise ValueError('invalid map origin translation')
            changed = (getattr(self, '_grid', None) is None
                       or not np.array_equal(self._grid, arr)
                       or self._grid_res != res or self._grid_yaw != yaw
                       or not np.array_equal(self._grid_origin, origin))
            self._grid, self._grid_res = arr, res
            self._grid_yaw, self._grid_origin = yaw, origin
            if changed: self._map_saved_final = False
        except (TypeError, ValueError) as exc:
            # Reject malformed external grid data instead of exporting a fake map.
            self._grid = None
            self._log.error(f'grid 解析失败: {exc!r}')

    # -- 速度守卫 -------------------------------------------------------------
    def _safe_velocity(self) -> np.ndarray:
        """Scale only a fresh nonzero suggestion; do not invent a bypass."""
        if not self._has_advice or not self._has_base or not self._has_target:
            return np.zeros(2)
        v=self._vel.copy()
        if not np.all(np.isfinite(v)): return np.zeros(2)
        if not bool(self._params['judge.enforce_v_floor']):return v
        dist=float(np.linalg.norm(self._cur-self._base))
        if dist<float(self._params['judge.stop_radius_m']):return np.zeros(2)
        norm=float(np.linalg.norm(v))
        floor=max(float(self._params['judge.v_floor']),V_STOP*2.)
        if 1e-9<norm<floor:v*=floor/norm
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
        from team43_grid_mapper.pgm_io import write_map_pair
        try:
            write_map_pair(self._map_pgm, self._map_yaml, self._grid,
                           image=self._map_image_name,
                           resolution=self._grid_res,
                           origin_xyz=(float(self._grid_origin[0]),
                                       float(self._grid_origin[1]), self._grid_yaw),
                           occupied_thresh=float(
                               self._params['judge.map_occupied_thresh']),
                           free_thresh=float(
                               self._params['judge.map_free_thresh']))
        except OSError as exc:
            # Disk/permission failure must leave the last valid submission untouched.
            self._log.error(f'地图保存失败({reason}): {exc!r}')
            return False
        if reason == 'final':
            self._map_saved_final = True
        self._log.info(f'地图已保存({reason}): {self._map_pgm} / {self._map_yaml}'
                       f'（{self._grid.shape[0]}x{self._grid.shape[1]} '
                       f'@ {self._grid_res:.3f}m/cell）')
        return True

    # -- 收尾 -----------------------------------------------------------------
    def shutdown(self) -> None:
        """退出前补一次地图保存并关闭 CSV。"""
        if not self._map_saved_final and self._grid is not None:
            self._save_map('shutdown')
        st = self._writer.stats()
        self._writer.close()
        self._log.info(f'judge_io_node 退出: ticks={self._n_tick} rows={st["rows"]} '
                       f'guarded={st["n_guarded"]} dropped={st["n_dropped"]} '
                       f'→ {st["path"]}')


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    rclpy.init(args=args)
    node = JudgeIoNode()
    try:
        spin_with_finally(node, node.shutdown)
    finally:
        node.destroy_node()
        # 默认信号处理可能已经关闭context，复用幂等关闭入口。
        shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
