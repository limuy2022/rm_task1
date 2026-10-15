"""chassis_advisor_node：订阅目标状态 / 底盘位姿 / ray_grid → 发布 `/chassis/advice`。

对应 DESIGN §3.9 / §6.10。

订阅（§4.3）：
- `/estimation/target_state`（`gimbal_interfaces/TargetState`，100Hz，SENSOR_DATA）
- `/gimbal/base_pose`（`nav_msgs/Odometry`，frame=map，50Hz，SENSOR_DATA）
- `/map/ray_grid`（`nav_msgs/OccupancyGrid`，一次性，TRANSIENT_LOCAL depth=1）
- `/map/grid`（`nav_msgs/OccupancyGrid`，**未膨胀**三态，方向项解析求解用）

发布：
- `/chassis/advice`（`gimbal_interfaces/ChassisAdvice`，50Hz，SENSOR_DATA）

⚠ **赛题一适配（2026-10）**：方向项权重 0.2，标准方向 = "从底盘出发、沿地图可行、
射线最长 1 m、不与不可跨越地形相交、与目标方向夹角最小"的**连续**角度。
旧的 13 方向 12.5° 量化会带来约 3.6° 的角度 RMSE（≈0.004 总分），
改用 `free_dir.FreeDirSolver` 解析求解后可压到 <0.5°。
`chassis.dir_mode='analytic'`（默认）走新路径，`'quantized'` 保留旧行为。
另外：官方判定 `hypot(vx,vy) < 0.02` 为"停车"，此时若距目标 ≥0.5 m 该项记满分误差，
因此除已到位外**必须保证 |v| ≥ `chassis.v_floor`**。

参考系默认 `map`（与 CSV 1~6 列一致）；`cmd_frame=base_link` 时额外左乘 `RotZ(−yaw_base)`（P1）。

时钟（主理人拍板 1，混合时钟）：
- `header.stamp` 用 **ROS 系统时钟** `node.get_clock().now()`（**不替换 node clock**）
- 所有 dt / 超时 / 定时器周期用 `gimbal_common.time_utils.now()`（= `time.monotonic()`）

⚠ **待 Ubuntu 24.04 + ROS2 Jazzy 编译验证**：本机为 Windows，无 ROS2，
本文件只做了静态一致性检查，未经 `colcon build` 与运行时验证。
"""

from __future__ import annotations

import math
import sys
from typing import Any
from typing import Dict
from typing import Optional

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy
    from rclpy.qos import QoSHistoryPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import QoSReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from gimbal_interfaces.msg import ChassisAdvice, TargetState
except Exception:  # pragma: no cover - Windows 无 ROS2
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]
    OccupancyGrid = None  # type: ignore[assignment]
    Odometry = None  # type: ignore[assignment]
    ChassisAdvice = None  # type: ignore[assignment]
    TargetState = None  # type: ignore[assignment]

    class _QoSProfile:  # noqa: D401 - 占位
        """ROS 缺失时的占位类。"""

        def __init__(self, **kwargs: Any) -> None:
            self.kwargs = kwargs

    QoSProfile = _QoSProfile  # type: ignore[assignment]
    QoSDurabilityPolicy = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]
    QoSReliabilityPolicy = None  # type: ignore[assignment]

try:  # 优先用 T01 的 gimbal_common
    from gimbal_common.time_utils import now
    from gimbal_common.log_utils import get_logger, throttled
except Exception:  # pragma: no cover - Windows 无 ROS2 兜底
    from ._compat import get_logger, now, throttled

try:  # rotz2 在 gimbal_common.frames 里（不在 math_utils），单独兜底
    from gimbal_common.frames import rotz2
except Exception:  # pragma: no cover
    try:
        from gimbal_common.math_utils import rotz2
    except Exception:
        from .cost_raycast import rotz2

from .cost_raycast import (DIR_SPAN_DEG, LOOKAHEAD_M, N_DIRS, W_ANG, W_OBS,
                           W_UNK, CostRaycast, RayGridAdapter)
from .free_dir import FREE_DIR_L_M, V_STOP_OFFICIAL, FreeDirSolver
from .velocity_law import (A_MAX, D_SLOW, D_STOP, LOST_DECAY_S, LP_CUTOFF_HZ,
                           V_MAX, VelocityLaw)

# 默认参数（DESIGN §6.13 `chassis.*`）
DEFAULTS: Dict[str, Any] = {
    'chassis.v_max': V_MAX,
    'chassis.d_slow': D_SLOW,
    'chassis.d_stop': D_STOP,
    'chassis.w_ang': W_ANG,
    'chassis.w_obs': W_OBS,
    'chassis.w_unk': W_UNK,
    'chassis.n_dirs': N_DIRS,
    'chassis.dir_span_deg': DIR_SPAN_DEG,
    'chassis.lookahead_m': LOOKAHEAD_M,
    'chassis.a_max': A_MAX,
    'chassis.lp_cutoff_hz': LP_CUTOFF_HZ,
    'chassis.lost_decay_s': LOST_DECAY_S,
    'chassis.cmd_frame': 'map',           # map | base_link（P1）
    'chassis.rate_hz': 50.0,              # 话题表：/chassis/advice 50Hz
    'chassis.enable_avoidance': True,     # false 时跳过射线，直接朝目标
    # --- 赛题一新增 ---------------------------------------------------------
    'chassis.dir_mode': 'analytic',       # analytic（连续）| quantized（旧 13 方向）
    'chassis.dir_lookahead_m': FREE_DIR_L_M,   # 官方固定 1.0 m
    'chassis.dir_block_unknown': False,   # 评分标准方向按真值地图算，未知不算障碍
    'chassis.dir_use_pred': False,        # False=朝目标**当前**位置（cur），True=朝预测位置
    'chassis.v_floor': 0.30,              # 远离目标时的最小建议速度（>0.02 停车线）
    'chassis.stop_radius_m': 0.45,        # 与官方 0.5 m 判据留 0.05 m 余量
}

_SENSOR_QOS_DEPTH: int = 10


class ChassisAdvisorNode(Node):  # type: ignore[misc]
    """底盘建议速度节点。"""

    def __init__(self, name: str = 'chassis_advisor_node') -> None:
        super().__init__(name)
        self._log = get_logger(self, name)
        self._params: Dict[str, Any] = {}
        for key, val in DEFAULTS.items():
            self._params[key] = self._declare(key, val)

        # --- 算法内核（纯 Python） -------------------------------------------
        self._ray = CostRaycast(
            lookahead_m=float(self._params['chassis.lookahead_m']),
            w_ang=float(self._params['chassis.w_ang']),
            w_obs=float(self._params['chassis.w_obs']),
            w_unk=float(self._params['chassis.w_unk']),
            n_dirs=int(self._params['chassis.n_dirs']),
            dir_span_deg=float(self._params['chassis.dir_span_deg']))
        self._law = VelocityLaw(
            v_max=float(self._params['chassis.v_max']),
            d_slow=float(self._params['chassis.d_slow']),
            d_stop=float(self._params['chassis.d_stop']),
            a_max=float(self._params['chassis.a_max']),
            lp_cutoff_hz=float(self._params['chassis.lp_cutoff_hz']),
            lost_decay_s=float(self._params['chassis.lost_decay_s']))
        # 方向项解析求解器（赛题一权重 0.2，最大的可优化项）
        self._free = FreeDirSolver(
            lookahead_m=float(self._params['chassis.dir_lookahead_m']),
            block_unknown=bool(self._params['chassis.dir_block_unknown']))

        # --- 状态 -------------------------------------------------------------
        self._t_last = now()
        self._t_start = self._t_last
        self._has_grid: bool = False
        self._has_base: bool = False
        self._has_target: bool = False
        self._base_xy = np.zeros(2, dtype=float)
        self._base_yaw: float = 0.0
        self._has_clean_grid: bool = False
        self._goal_xy = np.zeros(2, dtype=float)     # 目标**预测**位置
        self._cur_xy = np.zeros(2, dtype=float)      # 目标**当前**位置（方向项基准）
        self._track_status: int = 0
        self._t_lost: Optional[float] = None
        self._dist: float = 0.0
        self._theta: float = 0.0
        self._blocked: bool = False

        # --- 话题 -------------------------------------------------------------
        sensor_qos = self._sensor_qos()
        self.create_subscription(TargetState, '/estimation/target_state',
                                 self._on_target, sensor_qos)
        self.create_subscription(Odometry, '/gimbal/base_pose',
                                 self._on_base_pose, sensor_qos)
        self.create_subscription(OccupancyGrid, '/map/ray_grid',
                                 self._on_ray_grid, self._latched_qos())
        # 未膨胀三态栅格：方向项解析求解用（不可跨越地形 = 占用格 100）
        self.create_subscription(OccupancyGrid, '/map/grid',
                                 self._on_clean_grid, self._latched_qos())
        self._pub = self.create_publisher(ChassisAdvice, '/chassis/advice',
                                          sensor_qos)

        period = 1.0 / max(float(self._params['chassis.rate_hz']), 1e-3)
        self.create_timer(period, self._on_timer)
        self._log.info('chassis_advisor_node 启动: rate=%.1fHz cmd_frame=%s '
                       'v_max=%.3f lookahead=%.2fm'
                       % (float(self._params['chassis.rate_hz']),
                          str(self._params['chassis.cmd_frame']),
                          float(self._params['chassis.v_max']),
                          float(self._params['chassis.lookahead_m'])))

    # -- 参数 / QoS -----------------------------------------------------------
    def _declare(self, key: str, default: Any) -> Any:
        """声明参数（已由 launch 注入时直接取用）。"""
        try:
            self.declare_parameter(key, default)
            return self.get_parameter(key).value
        except Exception:  # pragma: no cover
            return default

    @staticmethod
    def _sensor_qos() -> Any:
        """SENSOR_DATA 风格 QoS：KEEP_LAST depth=10。"""
        if QoSProfile is None:  # pragma: no cover
            return _SENSOR_QOS_DEPTH
        try:
            return QoSProfile(depth=_SENSOR_QOS_DEPTH,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT,
                              history=QoSHistoryPolicy.KEEP_LAST)
        except Exception:  # pragma: no cover
            return QoSProfile(depth=_SENSOR_QOS_DEPTH)

    @staticmethod
    def _latched_qos() -> Any:
        """TRANSIENT_LOCAL + depth=1（锁存，等 grid_mapper 的地图）。"""
        if QoSProfile is None:  # pragma: no cover
            return 1
        try:
            return QoSProfile(depth=1,
                              history=QoSHistoryPolicy.KEEP_LAST,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        except Exception:  # pragma: no cover
            return QoSProfile(depth=1)

    # -- 回调 -----------------------------------------------------------------
    def _on_ray_grid(self, msg: Any) -> None:
        """接收 `/map/ray_grid`（含膨胀），设置避障用栅格。"""
        try:
            info = msg.info
            w = int(info.width)
            h = int(info.height)
            data = np.asarray(msg.data, dtype=np.int8).reshape((h, w))
            arr = np.transpose(data).astype(np.int8)          # (W, H)
            origin = np.array([float(info.origin.position.x),
                               float(info.origin.position.y)], dtype=float)
            self._ray.set_grid(RayGridAdapter(arr, origin, float(info.resolution)))
            self._has_grid = True
            self._log.info('已接收 ray_grid: %dx%d @ %.3f' % (w, h, info.resolution))
        except Exception as exc:
            throttled(self._log, 'error', 'ray_grid 解析失败: %r' % (exc,),
                      'chassis_advisor.ray_grid_err', 1.0)

    def _on_clean_grid(self, msg: Any) -> None:
        """接收 `/map/grid`（**未膨胀**三态），供方向项解析求解使用。"""
        try:
            info = msg.info
            w = int(info.width)
            h = int(info.height)
            data = np.asarray(msg.data, dtype=np.int8).reshape((h, w))
            arr = np.transpose(data).astype(np.int8)          # (W, H)
            origin = np.array([float(info.origin.position.x),
                               float(info.origin.position.y)], dtype=float)
            self._free.set_grid(arr, origin, float(info.resolution))
            self._has_clean_grid = True
            self._log.info('已接收未膨胀 grid: %dx%d @ %.3f（方向项用）'
                           % (w, h, info.resolution))
        except Exception as exc:
            throttled(self._log, 'error', 'grid 解析失败: %r' % (exc,),
                      'chassis_advisor.grid_err', 1.0)

    def _on_base_pose(self, msg: Any) -> None:
        """接收 `/gimbal/base_pose`（map 系）。"""
        try:
            p = msg.pose.pose.position
            self._base_xy = np.array([float(p.x), float(p.y)], dtype=float)
            self._base_yaw = self._yaw_from_quat(msg.pose.pose.orientation)
            self._has_base = True
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'base_pose 解析失败: %r' % (exc,),
                      'chassis_advisor.base_err', 1.0)

    def _on_target(self, msg: Any) -> None:
        """接收 `/estimation/target_state`，取**预测**位置作为目标点 G。"""
        try:
            if not bool(msg.valid):
                self._mark_lost()
                return
            self._goal_xy = np.array([float(msg.pred_x), float(msg.pred_y)],
                                     dtype=float)
            self._cur_xy = np.array([float(msg.cur_x), float(msg.cur_y)],
                                    dtype=float)
            if not (math.isfinite(self._goal_xy[0]) and math.isfinite(self._goal_xy[1])):
                self._mark_lost()
                return
            self._has_target = True
            self._track_status = int(msg.track_status)
            if self._track_status == 0:
                self._mark_lost()
            else:
                self._t_lost = None
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'target_state 解析失败: %r' % (exc,),
                      'chassis_advisor.target_err', 1.0)

    def _mark_lost(self) -> None:
        """记录丢失起始时刻（单调）。"""
        self._track_status = 0
        if self._t_lost is None:
            self._t_lost = now()

    # -- 主循环 ---------------------------------------------------------------
    def _on_timer(self) -> None:
        """50Hz：算一次建议速度并发布。dt 一律用单调时钟。"""
        t_now = now()
        dt = t_now - self._t_last
        self._t_last = t_now
        if dt < 0.0 or not math.isfinite(dt):
            dt = 0.0

        if not (self._has_target and self._has_base):
            self._publish(np.zeros(2, dtype=float), 0.0, 0.0, False, valid=False)
            return

        u, d = VelocityLaw.unit_direction(self._base_xy, self._goal_xy)
        theta = 0.0
        blocked = False
        u_safe = u

        mode = str(self._params['chassis.dir_mode']).lower()
        if mode == 'analytic' and self._has_clean_grid:
            # --- 赛题一方向项：连续解析求解（权重 0.2）----------------------
            # 基准方向取目标**当前**位置（官方"与目标方向"），可切到预测位置。
            ref = self._cur_xy if not bool(
                self._params['chassis.dir_use_pred']) else self._goal_xy
            u_dir, _d_dir = VelocityLaw.unit_direction(self._base_xy, ref)
            if float(np.linalg.norm(u_dir)) > 1e-9:
                u_safe, theta, ok = self._free.solve_unit(self._base_xy, u_dir)
                blocked = not ok
            else:
                u_safe = np.zeros(2, dtype=float)
        elif bool(self._params['chassis.enable_avoidance']) and self._has_grid:
            if float(np.linalg.norm(u)) > 1e-9:
                theta, u_safe, blocked, _hits = self._ray.evaluate(
                    self._base_xy, u, float(self._params['chassis.lookahead_m']))
            else:
                theta, u_safe, blocked, _hits = self._ray.evaluate(
                    self._base_xy, np.array([1.0, 0.0]),
                    float(self._params['chassis.lookahead_m']))
                u_safe = np.zeros(2, dtype=float)

        res = self._law.step(self._base_xy, self._goal_xy, dt, u_safe=u_safe,
                             blocked=blocked, track_status=self._track_status,
                             t_now=t_now, t_lost=self._t_lost)
        cmd = np.asarray(res['cmd'], dtype=float).reshape(2)
        cmd = self._apply_v_floor(cmd)

        # 参考系：base_link 时左乘 RotZ(−yaw_base)（P1）
        if str(self._params['chassis.cmd_frame']).lower() == 'base_link':
            cmd = rotz2(-self._base_yaw) @ cmd

        self._dist = float(res['dist'])
        self._theta = float(theta)
        self._blocked = bool(blocked)
        self._publish(cmd, self._dist, self._theta, self._blocked, valid=True)

    def _apply_v_floor(self, cmd: np.ndarray) -> np.ndarray:
        """速度下限守卫：避免被评分系统判成"停车"而记满分误差。

        官方：`hypot(vx,vy) < 0.02` 视为停车，此时离目标不到 0.5 m 记 0 分，
        **否则记 1 分**（方向项最大误差）。所以除"确实已到位"外，必须让
        `|v| ≥ v_floor`。

        Args:
            cmd: 速度定律输出的原始建议速度（map 系）。

        Returns:
            np.ndarray：修正后的建议速度。
        """
        v = np.asarray(cmd, dtype=float).reshape(2)
        if not np.all(np.isfinite(v)):
            v = np.zeros(2, dtype=float)
        stop_r = float(self._params['chassis.stop_radius_m'])
        dist = float(np.linalg.norm(self._cur_xy - self._base_xy))
        if math.isfinite(dist) and dist < stop_r:
            return np.zeros(2, dtype=float)          # 已到位 → 停车 → 该项记 0
        floor = max(float(self._params['chassis.v_floor']), V_STOP_OFFICIAL * 2.0)
        n = float(np.linalg.norm(v))
        if n >= floor:
            return v
        if n > 1e-9:
            return v * (floor / n)
        u, _d = VelocityLaw.unit_direction(self._base_xy, self._cur_xy)
        if float(np.linalg.norm(u)) > 1e-9:
            return u * floor
        return np.array([floor, 0.0], dtype=float)

    def _publish(self, cmd: np.ndarray, dist: float, theta: float,
                 blocked: bool, valid: bool) -> None:
        """发布 `/chassis/advice`（header.stamp 走 ROS 系统时钟）。"""
        if ChassisAdvice is None:  # pragma: no cover
            return
        msg = ChassisAdvice()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.cmd_vx = float(cmd[0])
        msg.cmd_vy = float(cmd[1])
        msg.dist_to_target = float(dist)
        msg.chosen_theta = float(theta)
        msg.blocked = bool(blocked)
        msg.valid = bool(valid)
        try:
            self._pub.publish(msg)
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'advice 发布失败: %r' % (exc,),
                      'chassis_advisor.pub_err', 1.0)
        if blocked:
            throttled(self._log, 'warn',
                      '360° 全被不可跨越地形挡住（或没有可用栅格），方向项退化为朝目标',
                      'chassis_advisor.blocked', 1.0)

    # -- 工具 -----------------------------------------------------------------
    @staticmethod
    def _yaw_from_quat(q: Any) -> float:
        """从四元数取 yaw（绕 +z），忽略 pitch/roll。"""
        try:
            w = float(q.w)
            x = float(q.x)
            y = float(q.y)
            z = float(q.z)
        except Exception:  # pragma: no cover
            return 0.0
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def shutdown(self) -> None:
        """退出日志摘要。"""
        self._log.info('chassis_advisor_node 退出，耗时 %.2fs，末次 d=%.3f θ=%.1f°'
                       % (now() - self._t_start, self._dist,
                          math.degrees(self._theta)))


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    if rclpy is None:  # pragma: no cover
        print('[chassis_advisor_node] FATAL rclpy 不可用（需要 ROS2 Jazzy）')
        return 1
    rclpy.init(args=args)
    node = ChassisAdvisorNode()
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
