# -*- coding: utf-8 -*-
"""目标状态估计节点 —— DESIGN.md §3.7 / §6.6 / §6.7。

职责：
1. 缓存最新的云台姿态（``/gimbal/state``）、底盘位姿（``/gimbal/base_pose_raw``）、
   ``T_map←world``（``/gimbal/t_map_world``，锁存）；
2. 收到 ``/perception/marker_cam`` 时用 :class:`ObservationBuilder` 做
   optical → world → map 换算 + 噪声旋转；
3. 驱动 :class:`CAKalman`（predict / NIS 门限 / update / 发散保护）；
4. 100Hz 单调定时器发布 ``/estimation/target_state``（含 100ms 外推）。

⏱ 时钟约定（主理人拍板 1，最高优先级）：
* **所有 dt / 超时 / 定时器周期计算** → ``gimbal_common.time_utils.now()``（``time.monotonic()``）；
* ``header.stamp`` 用 ROS 节点时钟 ``self.get_clock().now()``；
* 外部话题的 ``header.stamp`` **只用于日志与丢帧统计，不参与任何运算**。

⚠ 待验证：本文件依赖 ``rclpy`` / ``gimbal_interfaces``，
    需在 **Ubuntu 24.04 + ROS 2 Jazzy** 上执行 ``colcon build`` 后编译/运行验证（Windows 无 ROS2）。
"""

from __future__ import annotations

import math
import sys
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from gimbal_interfaces.msg import GimbalState, MarkerObservation, TargetState

from gimbal_common import frames
from gimbal_common.log_utils import get_logger, throttled
from gimbal_common.time_utils import now

from gimbal_estimation.ca_kalman import (
    DEFAULT_CV_DEGRADE_ENABLE,
    DEFAULT_CV_HOLD_S,
    DEFAULT_INIT_SIGMA_A,
    DEFAULT_INIT_SIGMA_P,
    DEFAULT_INIT_SIGMA_V,
    DEFAULT_JERK_STD,
    DEFAULT_MAX_DT,
    DEFAULT_NIS_GATE,
    DEFAULT_P_MAX_TRACE,
    DEFAULT_PREDICT_DT,
    DEFAULT_REINIT_JUMP_M,
    CAKalman,
)
from gimbal_estimation.observation_builder import (
    DEFAULT_R_MIN_SIGMA,
    ObservationBuilder,
)

# --------------------------------------------------------------------------
# 话题名（DESIGN §4.3，单点定义）
# --------------------------------------------------------------------------
TOPIC_MARKER_CAM = '/perception/marker_cam'
TOPIC_GIMBAL_STATE = '/gimbal/state'
TOPIC_BASE_POSE_RAW = '/gimbal/base_pose_raw'
TOPIC_T_MAP_WORLD = '/gimbal/t_map_world'
TOPIC_TARGET_STATE = '/estimation/target_state'

# track_status 语义（DESIGN §4.2）
TRACK_LOST = 0        # 丢失（age > coast 窗口）
TRACK_OBSERVED = 1    # 观测新鲜（age ≤ fresh 窗口）
TRACK_EXTRAP = 2      # 外推（fresh < age ≤ coast 窗口）

QOS_SENSOR = QoSProfile(
    depth=10,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

QOS_LATCHED = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _rpy_to_mat(rpy) -> np.ndarray:
    """固定轴 XYZ（roll-pitch-yaw）→ 3x3 旋转矩阵。

    优先复用 ``gimbal_common.frames.rpy_to_mat``（若工程师 A 提供），否则本地实现。
    """
    fn = getattr(frames, 'rpy_to_mat', None)
    if callable(fn):
        M = np.asarray(fn(rpy), dtype=np.float64)
        if M.shape == (3, 3):
            return M
    r, p, y = (float(v) for v in np.asarray(rpy, dtype=np.float64).reshape(3))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


class TargetEstimatorNode(Node):
    """目标状态估计节点。"""

    def __init__(self) -> None:
        super().__init__('target_estimator_node')
        self._log = get_logger(self, 'target_estimator')

        # ---------------------------------------------------------- 参数声明
        self.declare_parameter('estimation.jerk_std', DEFAULT_JERK_STD)
        self.declare_parameter('estimation.predict_dt', DEFAULT_PREDICT_DT)
        self.declare_parameter('estimation.max_dt', DEFAULT_MAX_DT)
        self.declare_parameter('estimation.nis_gate', DEFAULT_NIS_GATE)
        self.declare_parameter('estimation.reinit_jump_m', DEFAULT_REINIT_JUMP_M)
        self.declare_parameter('estimation.p_max_trace', DEFAULT_P_MAX_TRACE)
        self.declare_parameter('estimation.init_sigma_p', DEFAULT_INIT_SIGMA_P)
        self.declare_parameter('estimation.init_sigma_v', DEFAULT_INIT_SIGMA_V)
        self.declare_parameter('estimation.init_sigma_a', DEFAULT_INIT_SIGMA_A)
        self.declare_parameter('estimation.rate_hz', 100.0)
        self.declare_parameter('estimation.cv_degrade_enable', DEFAULT_CV_DEGRADE_ENABLE)
        self.declare_parameter('estimation.cv_hold_s', DEFAULT_CV_HOLD_S)
        # track_status 的观测新鲜度阶梯（DESIGN §4.2）：
        #   age ≤ track_fresh_s        → 1（观测新鲜）
        #   age ≤ lost_timeout_s       → 2（外推滑行）
        #   age >  lost_timeout_s      → 0（丢失）
        self.declare_parameter('estimation.track_fresh_s', 0.3)
        self.declare_parameter('estimation.lost_timeout_s', 0.5)
        self.declare_parameter('perception.r_min_sigma', DEFAULT_R_MIN_SIGMA)
        self.declare_parameter('localization.world_odom_xyz_rpy', [0.0] * 6)
        self.declare_parameter('localization.gimbal_mount_xyz_rpy', [0.0] * 6)

        # pitch 符号的唯一入口在 gimbal_common.frames；此处仅把 ROS 参数透传进去，
        # 默认值直接取 frames.PITCH_SIGN，避免本文件出现任何符号字面量。
        _set_pitch_sign = getattr(frames, 'set_pitch_sign', None)
        if callable(_set_pitch_sign):
            self.declare_parameter('gimbal.pitch_sign', float(frames.PITCH_SIGN))
            _set_pitch_sign(float(self.get_parameter('gimbal.pitch_sign').value))

        p = self.get_parameter
        rate_hz = float(p('estimation.rate_hz').value)
        self._rate_hz = rate_hz if rate_hz > 0.0 else 100.0
        self._predict_dt = float(p('estimation.predict_dt').value)
        self._track_fresh_s = float(p('estimation.track_fresh_s').value)
        self._lost_timeout_s = float(p('estimation.lost_timeout_s').value)
        self._r_min_sigma = float(p('perception.r_min_sigma').value)

        # ---------------------------------------------------------- 外部标定
        w_o = [float(v) for v in p('localization.world_odom_xyz_rpy').value]
        self._R_world_odom = _rpy_to_mat(w_o[3:6])
        self._t_world_odom = np.array(w_o[0:3], dtype=np.float64)
        mnt = [float(v) for v in p('localization.gimbal_mount_xyz_rpy').value]
        self._mount_xyz = np.array(mnt[0:3], dtype=np.float64)

        # ---------------------------------------------------------- 算法内核
        self._kf = CAKalman(
            jerk_std=float(p('estimation.jerk_std').value),
            predict_dt=self._predict_dt,
            max_dt=float(p('estimation.max_dt').value),
            nis_gate=float(p('estimation.nis_gate').value),
            reinit_jump_m=float(p('estimation.reinit_jump_m').value),
            p_max_trace=float(p('estimation.p_max_trace').value),
            init_sigma_p=float(p('estimation.init_sigma_p').value),
            init_sigma_v=float(p('estimation.init_sigma_v').value),
            init_sigma_a=float(p('estimation.init_sigma_a').value),
            cv_degrade_enable=bool(p('estimation.cv_degrade_enable').value),
            cv_hold_s=float(p('estimation.cv_hold_s').value),
        )
        self._builder = ObservationBuilder(r_min_sigma=self._r_min_sigma)

        # ---------------------------------------------------------- 缓存状态
        self._gimbal_state: Optional[GimbalState] = None
        self._base_pos_odom = np.zeros(3, dtype=np.float64)
        self._R_odom_base = np.eye(3, dtype=np.float64)
        self._have_base = False
        self._T_map_world: Optional[np.ndarray] = None
        self._last_obs_t: Optional[float] = None
        self._obs_count = 0
        self._marker_msg_count = 0
        self._marker_valid_count = 0
        self._last_result = ''
        self._t_start = now()

        # ---------------------------------------------------------- 话题
        self._pub = self.create_publisher(TargetState, TOPIC_TARGET_STATE, QOS_SENSOR)
        self.create_subscription(MarkerObservation, TOPIC_MARKER_CAM, self.on_marker, QOS_SENSOR)
        self.create_subscription(GimbalState, TOPIC_GIMBAL_STATE, self.on_gimbal_state, QOS_SENSOR)
        self.create_subscription(Odometry, TOPIC_BASE_POSE_RAW, self.on_base_pose, QOS_SENSOR)
        self.create_subscription(TransformStamped, TOPIC_T_MAP_WORLD, self.on_t_map_world, QOS_LATCHED)

        period = 1.0 / self._rate_hz
        self.create_timer(period, self.tick)

        self._log.info(
            'target_estimator_node 启动 | rate=%.1fHz predict_dt=%.3fs jerk_std=%.3f '
            'nis_gate=%.2f max_dt=%.3f r_min_sigma=%.4f',
            self._rate_hz, self._predict_dt, self._kf.jerk_std,
            self._kf.nis_gate, self._kf.max_dt, self._r_min_sigma,
        )

    # ------------------------------------------------------------------ 回调
    def on_gimbal_state(self, msg: GimbalState) -> None:
        """缓存最新云台姿态/陀螺（只存不运算，dt 一律用本地单调时刻）。"""
        try:
            self._gimbal_state = msg
        except Exception as exc:  # pragma: no cover - 防御式
            throttled(self._log, 'error', 'on_gimbal_state 异常: %s' % exc, 'gs_exc', 1.0)

    def on_base_pose(self, msg: Odometry) -> None:
        """缓存底盘位姿（frame=odom），用于计算云台原点。"""
        try:
            pos = msg.pose.pose.position
            if not all(math.isfinite(v) for v in (pos.x, pos.y, pos.z)):
                return
            self._base_pos_odom = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
            o = msg.pose.pose.orientation
            q = np.array([o.w, o.x, o.y, o.z], dtype=np.float64)
            if np.linalg.norm(q) >= 0.5 and np.all(np.isfinite(q)):
                self._R_odom_base = np.asarray(frames.quat_to_mat(q), dtype=np.float64).reshape(3, 3)
            self._have_base = True
        except Exception as exc:  # pragma: no cover - 防御式
            throttled(self._log, 'error', 'on_base_pose 异常: %s' % exc, 'bp_exc', 1.0)

    def on_t_map_world(self, msg: TransformStamped) -> None:
        """接收锁存的 ``T_map←world``（只收一次，之后永不更新）。"""
        try:
            if self._T_map_world is not None:
                return
            tr = msg.transform
            t = np.array([tr.translation.x, tr.translation.y, tr.translation.z], dtype=np.float64)
            q = np.array([tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z], dtype=np.float64)
            R = np.asarray(frames.quat_to_mat(q), dtype=np.float64).reshape(3, 3)
            if not (np.all(np.isfinite(t)) and np.all(np.isfinite(R))):
                return
            T = np.eye(4, dtype=np.float64)
            # 约定：TransformStamped(map → world) 即 T_map←world（world 原点在 map 下的位姿）
            child = str(msg.child_frame_id or '')
            if child == 'map':
                # 反向给出（world → map）：取逆
                R = R.T
                t = -R @ t
            T[0:3, 0:3] = R
            T[0:3, 3] = t
            self._T_map_world = T
            self._log.info('收到 T_map←world：theta=%.4f rad, t=[%.3f, %.3f, %.3f]',
                           math.atan2(R[1, 0], R[0, 0]), t[0], t[1], t[2])
        except Exception as exc:  # pragma: no cover - 防御式
            throttled(self._log, 'error', 'on_t_map_world 异常: %s' % exc, 'tmw_exc', 1.0)

    def on_marker(self, msg: MarkerObservation) -> None:
        """处理一次视觉观测：换算 → predict → NIS 门限 → update。"""
        try:
            self._marker_msg_count += 1
            if not bool(msg.valid):
                return
            if self._gimbal_state is None:
                throttled(self._log, 'warn', '尚无 /gimbal/state，丢弃观测', 'no_state', 1.0)
                return
            if self._T_map_world is None:
                throttled(self._log, 'warn', '尚无 /gimbal/t_map_world，丢弃观测', 'no_tmw', 1.0)
                return

            q = np.asarray(self._gimbal_state.quat, dtype=np.float64).reshape(4)
            if not np.all(np.isfinite(q)) or float(np.linalg.norm(q)) < 0.5:
                throttled(self._log, 'warn', '云台四元数异常，丢弃观测', 'bad_quat', 1.0)
                return

            t_obs = now()                       # ⏱ 单调时钟：dt 的唯一来源
            obs = self._builder.build(
                tvec_opt=(msg.x, msg.y, msg.z),
                cov_opt=list(msg.cov_optical),
                quat=list(self._gimbal_state.quat),
                gimbal_origin_world=self.gimbal_origin_world(),
                T_map_world=self._T_map_world,
            )
            if not obs.valid:
                throttled(self._log, 'warn', '观测换算失败: %s' % obs.reason, 'obs_bad', 1.0)
                return

            self._marker_valid_count += 1
            self._kf.advance(t_obs)             # dt = clamp(t_obs − t_last, 0, max_dt)
            accepted = self._kf.update(obs.pos_map[0:2], obs.R_map)

            if self._kf.last_result != self._last_result:
                if self._kf.last_result in ('reinit', 'init'):
                    self._log.warn('KF %s @ pos=[%.3f, %.3f]（累计 reinit=%d）',
                                   self._kf.last_result, obs.pos_map[0], obs.pos_map[1],
                                   self._kf.reinit_count)
                self._last_result = self._kf.last_result

            if accepted:
                self._last_obs_t = t_obs
                self._obs_count += 1
            elif self._kf.outlier_count % 20 == 0:
                throttled(
                    self._log, 'warn',
                    'NIS=%.2f 超门限 %.2f，已拒绝 %d 次观测'
                    % (self._kf.nis(), self._kf.nis_gate, self._kf.outlier_count),
                    'nis_gate', 1.0,
                )
        except Exception as exc:  # pragma: no cover - 回调异常不得中断回路
            throttled(self._log, 'error', 'on_marker 异常: %s' % exc, 'mk_exc', 1.0)

    # ------------------------------------------------------------------ 计算
    def gimbal_origin_world(self) -> np.ndarray:
        """云台原点在 world 系下的坐标（DESIGN §6.9）。

        ``p_base_world   = T_world←odom ∘ p_base_odom``
        ``p_gimbal_world = p_base_world + R_world←odom · Rz(yaw_base) · mount_xyz``

        真源在 ``gimbal_common.frames.gimbal_origin_world``；不存在时走本地等价实现。
        """
        if not self._have_base:
            return np.zeros(3, dtype=np.float64)
        T_wo = np.eye(4, dtype=np.float64)
        T_wo[0:3, 0:3] = self._R_world_odom
        T_wo[0:3, 3] = self._t_world_odom

        fn = getattr(frames, 'gimbal_origin_world', None)
        if callable(fn):
            out = np.asarray(fn(self._base_pos_odom,
                                float(frames.yaw_of_mat(self._R_odom_base)),
                                self._mount_xyz, T_wo), dtype=np.float64).reshape(3)
        else:  # pragma: no cover - 仅在 gimbal_common 缺失时触发
            p_base_world = self._R_world_odom @ self._base_pos_odom + self._t_world_odom
            out = p_base_world + self._R_world_odom @ self._R_odom_base @ self._mount_xyz
        return out if np.all(np.isfinite(out)) else np.zeros(3, dtype=np.float64)

    def _obs_age(self, t: float) -> float:
        """距上次有效观测的秒数；从未收到则返回 ``inf``。"""
        if self._last_obs_t is None:
            return float('inf')
        return max(0.0, t - self._last_obs_t)

    # ------------------------------------------------------------------ 定时
    def tick(self) -> None:
        """100Hz 单调定时器：predict → 发布 ``TargetState``。"""
        try:
            t = now()                            # ⏱ 单调时钟
            self._kf.advance(t)

            msg = TargetState()
            msg.header.stamp = self.get_clock().now().to_msg()   # ROS 时钟，仅用于 stamp
            msg.header.frame_id = 'map'

            initialized = self._kf.initialized
            cur = self._kf.position
            pred = self._kf.extrapolate(self._predict_dt)
            vel = self._kf.velocity
            acc = self._kf.acceleration
            cov = self._kf.cov_xy

            msg.cur_x = float(cur[0])
            msg.cur_y = float(cur[1])
            msg.pred_x = float(pred[0])
            msg.pred_y = float(pred[1])
            msg.vel_x = float(vel[0])
            msg.vel_y = float(vel[1])
            msg.acc_x = float(acc[0])
            msg.acc_y = float(acc[1])
            msg.cov_xy = [float(cov[0, 0]), float(cov[0, 1]), float(cov[1, 0]), float(cov[1, 1])]

            age = self._obs_age(t)
            if not initialized:
                msg.track_status = TRACK_LOST
            elif age <= self._track_fresh_s:
                msg.track_status = TRACK_OBSERVED
            elif age <= self._lost_timeout_s:
                msg.track_status = TRACK_EXTRAP
            else:
                msg.track_status = TRACK_LOST
            msg.last_obs_age = float(age) if math.isfinite(age) else -1.0
            msg.valid = bool(initialized)

            self._pub.publish(msg)
        except Exception as exc:  # pragma: no cover - 定时器异常不得中断回路
            throttled(self._log, 'error', 'tick 异常: %s' % exc, 'tick_exc', 1.0)

    # ------------------------------------------------------------------ 收尾
    def shutdown_summary(self) -> None:
        """退出时打印一条 INFO 摘要（DESIGN §8.5）。"""
        dur = now() - self._t_start
        self._log.info(
            'target_estimator_node 退出 | 运行 %.1fs, 观测 %d/%d 有效, reinit=%d, outlier=%d, %s',
            dur, self._marker_valid_count, self._marker_msg_count,
            self._kf.reinit_count, self._kf.outlier_count, self._kf.snapshot(),
        )


def main(args=None) -> int:
    """节点入口。"""
    rclpy.init(args=args)
    node = TargetEstimatorNode()
    code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        code = 0
    except Exception as exc:  # pragma: no cover
        node._log.error('target_estimator_node 异常退出: %s' % exc)
        code = 1
    finally:
        try:
            node.shutdown_summary()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return code


if __name__ == '__main__':
    sys.exit(main())
