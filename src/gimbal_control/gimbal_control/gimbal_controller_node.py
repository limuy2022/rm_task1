# -*- coding: utf-8 -*-
"""云台控制节点 —— DESIGN.md §3.8 / §6.8。

100Hz 单调定时器 → 计算 ``(yaw_omega, pitch_omega)`` → 发布 ``/gimbal/cmd_omega``。

订阅：
* ``/estimation/target_state``：KF 输出的 map 系 XY（当前/预测 + 速度）
* ``/gimbal/state``：云台姿态四元数 + 机体系陀螺
* ``/gimbal/t_map_world``：``T_map←world``（锁存，一次性）
* ``/gimbal/base_pose_raw``：底盘位姿（odom 系），用于算云台原点（§6.9）
* ``/perception/marker_cam``：**辅助**，仅取目标的 world 系高度 z（见下方"设计缺口"说明）

⏱ 时钟约定（主理人拍板 1）：dt / 超时 / 定时器周期 → ``gimbal_common.time_utils.now()``
（``time.monotonic()``）；``header.stamp`` → ``self.get_clock().now()``；
外部话题的 ``header.stamp`` **不参与任何运算**。

⚠ 设计缺口与处置（已回报架构师/主理人）：
DESIGN §6.7 的 CA-KF 与 ``TargetState`` 都是 **map 系 2D**（px,py,vx,vy,ax,ay），
但 §6.8 的控制律需要 **3D** 视线向量 ``r = p_t − p_gimbal_world``。
2D 状态无法给出目标高度，若强行令 ``z=0``，目标在 pitch 方向的 ±0.25 rad
（hardware_spec §4）运动会完全无法跟踪，pitch 误差将达 14° 量级（远超 2° 预算）。
处置：XY 仍严格取 KF 输出（cur/pred，由 ``aim_mode`` 决定），
**z 轴**用 ``/perception/marker_cam`` 的 3D 观测经同一套 §6.6 换算得到，
经 :class:`HeightTracker` 做一阶低通 + 有限差分速度估计，丢失期间保持最后有效值。
若后续 ``gimbal_interfaces`` 允许给 ``TargetState`` 增加 ``cur_z``，应改回官方通道。

⚠ 待验证：本文件依赖 ``rclpy`` / ``gimbal_interfaces``，
    需在 **Ubuntu 24.04 + ROS 2 Jazzy** 上 ``colcon build`` 后编译/运行验证（Windows 无 ROS2）。
"""

from __future__ import annotations

import math
import os
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

from gimbal_interfaces.msg import GimbalOmegaCmd, GimbalState, MarkerObservation, TargetState

from gimbal_common import frames
from gimbal_common.log_utils import get_logger, throttled
from gimbal_common.time_utils import now

from gimbal_control.control_law import (
    DEFAULT_FF_GAIN,
    DEFAULT_KD_PITCH,
    DEFAULT_KD_YAW,
    DEFAULT_KP_PITCH,
    DEFAULT_KP_YAW,
    DEFAULT_LIMIT_MARGIN,
    DEFAULT_OMEGA_MAX_PITCH,
    DEFAULT_OMEGA_MAX_YAW,
    DEFAULT_OMEGA_SLEW,
    DEFAULT_PITCH_MAX,
    DEFAULT_PITCH_MIN,
    DEFAULT_YAW_MAX,
    DEFAULT_YAW_MIN,
    ControlLaw,
    current_angles,
    gimbal_origin_world,
    r_gimbal_from_opt,
)
from gimbal_control.search_pattern import (
    DEFAULT_HALF_FOV_RAD,
    DEFAULT_SEARCH_AMP_YAW,
    DEFAULT_SEARCH_FREQ,
    DEFAULT_WIDE_AFTER_S,
    DEFAULT_WIDE_AMP_YAW,
    DEFAULT_WIDE_AMP_PITCH,
    DEFAULT_WIDE_FREQ,
    SearchPattern,
    half_fov_rad_from_intrinsics,
    make_small_search,
    make_wide_search,
)

# --------------------------------------------------------------------------
# 话题名（DESIGN §4.3，单点定义）
# --------------------------------------------------------------------------
TOPIC_TARGET_STATE = '/estimation/target_state'
TOPIC_GIMBAL_STATE = '/gimbal/state'
TOPIC_T_MAP_WORLD = '/gimbal/t_map_world'
TOPIC_BASE_POSE_RAW = '/gimbal/base_pose_raw'
TOPIC_MARKER_CAM = '/perception/marker_cam'
TOPIC_CMD_OMEGA = '/gimbal/cmd_omega'

MODE_TRACK = 'track'
MODE_COAST = 'coast'
MODE_SEARCH = 'search'
MODE_WIDE = 'wide'
MODE_IDLE = 'idle'

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

    优先复用 ``gimbal_common.frames.rpy_to_mat``（若提供），否则本地实现。
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


def _resolve_virtual_yaml(name_or_path: str) -> str:
    """解析 ``virtual.yaml`` 路径（包名相对 → 绝对；缺失时返回原样由调用方兜底）。

    与 ``gimbal_perception.marker_pnp_node`` 完全同一套约定（DESIGN §8.6：
    内参/外参唯一数据源是 ``config/virtual.yaml``）。
    """
    if not name_or_path:
        name_or_path = 'virtual.yaml'
    if os.path.isabs(name_or_path) and os.path.isfile(name_or_path):
        return name_or_path
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('gimbal_tracker_bringup')
        cand = os.path.join(share, 'config', os.path.basename(name_or_path))
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    cand = os.path.join(os.getcwd(), 'src', 'gimbal_tracker_bringup', 'config',
                        os.path.basename(name_or_path))
    return cand if os.path.isfile(cand) else name_or_path


def load_camera_intrinsics(virtual_yaml: str):
    """从 ``virtual.yaml`` 的 ``camera`` 段读 ``(width, fx)``。

    Returns:
        ``(width, fx)``；读取失败或字段非法时返回 ``(0.0, 0.0)``
        （调用方据此回退保守半视场）。
    """
    try:
        from gimbal_common import config_loader as cl
        cfg = cl.load_yaml(_resolve_virtual_yaml(virtual_yaml), required=True)
    except Exception:
        return 0.0, 0.0
    cam = cfg.get('camera', {}) if isinstance(cfg, dict) else {}
    if not isinstance(cam, dict):
        return 0.0, 0.0
    try:
        width = float(cam.get('width', 0.0))
        fx = float(cam.get('fx', 0.0))
    except (TypeError, ValueError):
        return 0.0, 0.0
    if not (math.isfinite(width) and math.isfinite(fx)) or width <= 0.0 or fx <= 0.0:
        return 0.0, 0.0
    return width, fx


class HeightTracker:
    """目标 **world 系高度 z** 的一阶低通 + 速度估计（填补 2D KF 的高度缺口）。

    Args:
        cutoff_hz: 一阶低通截止频率（Hz），默认 8.0（高于 pitch 运动 0.08Hz 两个数量级）。
    """

    def __init__(self, cutoff_hz: float = 8.0) -> None:
        self.cutoff_hz = float(cutoff_hz)
        self.z: Optional[float] = None
        self.vz: float = 0.0
        self._t_last: Optional[float] = None

    def reset(self) -> None:
        """清空高度历史。"""
        self.z = None
        self.vz = 0.0
        self._t_last = None

    def update(self, z_obs: float, t: float) -> None:
        """用一次新的高度观测更新低通状态（t 为单调秒）。"""
        if not math.isfinite(float(z_obs)):
            return
        if self.z is None:
            self.z = float(z_obs)
            self.vz = 0.0
            self._t_last = float(t)
            return
        dt = float(t) - float(self._t_last) if self._t_last is not None else 0.0
        self._t_last = float(t)
        if not math.isfinite(dt) or dt <= 0.0:
            return
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.cutoff_hz * dt)
        alpha = min(max(alpha, 0.0), 1.0)
        vz_raw = (float(z_obs) - self.z) / dt
        self.z = self.z + alpha * (float(z_obs) - self.z)
        self.vz = self.vz + alpha * (vz_raw - self.vz)
        if not math.isfinite(self.z):
            self.z = float(z_obs)
        if not math.isfinite(self.vz):
            self.vz = 0.0

    def height(self, default: float = 0.0) -> float:
        """当前高度（未初始化时返回 ``default``）。"""
        return float(self.z) if self.z is not None else float(default)


class GimbalControllerNode(Node):
    """云台控制节点。"""

    def __init__(self) -> None:
        super().__init__('gimbal_controller_node')
        self._log = get_logger(self, 'gimbal_controller')

        # ---------------------------------------------------------- 参数声明
        d = self.declare_parameter
        d('control.kp_yaw', DEFAULT_KP_YAW)
        d('control.kp_pitch', DEFAULT_KP_PITCH)
        d('control.kd_yaw', DEFAULT_KD_YAW)
        d('control.kd_pitch', DEFAULT_KD_PITCH)
        d('control.ff_gain', DEFAULT_FF_GAIN)
        d('control.omega_max_yaw', DEFAULT_OMEGA_MAX_YAW)
        d('control.omega_max_pitch', DEFAULT_OMEGA_MAX_PITCH)
        d('control.omega_slew', DEFAULT_OMEGA_SLEW)
        d('control.rate_hz', 100.0)
        d('control.lost_timeout_s', 0.3)
        d('control.search_after_s', 0.5)
        d('control.wide_search_after_s', DEFAULT_WIDE_AFTER_S)
        d('control.search_amp_yaw', DEFAULT_SEARCH_AMP_YAW)
        d('control.search_freq', DEFAULT_SEARCH_FREQ)
        d('control.wide_amp_yaw', DEFAULT_WIDE_AMP_YAW)
        d('control.wide_search_freq', DEFAULT_WIDE_FREQ)
        d('control.wide_amp_pitch', DEFAULT_WIDE_AMP_PITCH)
        d('control.aim_mode', 'cur')
        d('control.height_cutoff_hz', 8.0)
        # 水平半视场：**不在参数里写死内参**（DESIGN §8.6）。
        # 优先级：control.camera_fx/width（>0，显式覆盖）→ config.virtual.yaml → 保守兜底。
        d('control.camera_fx', 0.0)
        d('control.camera_width', 0.0)
        d('config.virtual_yaml', 'virtual.yaml')
        d('gimbal.yaw_limits', [DEFAULT_YAW_MIN, DEFAULT_YAW_MAX])
        d('gimbal.pitch_limits', [DEFAULT_PITCH_MIN, DEFAULT_PITCH_MAX])
        d('gimbal.limit_margin', DEFAULT_LIMIT_MARGIN)
        d('localization.world_odom_xyz_rpy', [0.0] * 6)
        d('localization.gimbal_mount_xyz_rpy', [0.0] * 6)

        # pitch 符号的唯一入口在 gimbal_common.frames；此处只透传 ROS 参数，
        # 默认值直接取 frames.PITCH_SIGN，避免本文件出现任何符号字面量。
        _set_pitch_sign = getattr(frames, 'set_pitch_sign', None)
        if callable(_set_pitch_sign):
            d('gimbal.pitch_sign', float(frames.PITCH_SIGN))
            _set_pitch_sign(float(self.get_parameter('gimbal.pitch_sign').value))

        p = self.get_parameter
        rate_hz = float(p('control.rate_hz').value)
        self._rate_hz = rate_hz if rate_hz > 0.0 else 100.0
        self._lost_timeout_s = float(p('control.lost_timeout_s').value)
        self._search_after_s = float(p('control.search_after_s').value)
        self._wide_after_s = float(p('control.wide_search_after_s').value)
        self._aim_mode = str(p('control.aim_mode').value)

        yaw_lim = [float(v) for v in p('gimbal.yaw_limits').value]
        pitch_lim = [float(v) for v in p('gimbal.pitch_limits').value]
        margin = float(p('gimbal.limit_margin').value)

        self._law = ControlLaw(
            kp_yaw=float(p('control.kp_yaw').value),
            kp_pitch=float(p('control.kp_pitch').value),
            kd_yaw=float(p('control.kd_yaw').value),
            kd_pitch=float(p('control.kd_pitch').value),
            ff_gain=float(p('control.ff_gain').value),
            omega_max_yaw=float(p('control.omega_max_yaw').value),
            omega_max_pitch=float(p('control.omega_max_pitch').value),
            omega_slew=float(p('control.omega_slew').value),
            yaw_min=yaw_lim[0], yaw_max=yaw_lim[1],
            pitch_min=pitch_lim[0], pitch_max=pitch_lim[1],
            limit_margin=margin,
        )
        # ---- 水平半视场（搜索幅值上限的来源；DESIGN §8.6 单一数据源）
        self._half_fov_rad = self._resolve_half_fov()

        self._search_small: SearchPattern = make_small_search(
            amp_yaw=float(p('control.search_amp_yaw').value),
            freq=float(p('control.search_freq').value),
            half_fov_rad=self._half_fov_rad,
        )
        self._search_wide: SearchPattern = make_wide_search(
            amp_yaw=float(p('control.wide_amp_yaw').value),
            freq=float(p('control.wide_search_freq').value),
            amp_pitch=float(p('control.wide_amp_pitch').value),
            half_fov_rad=self._half_fov_rad,
        )

        # ---------------------------------------------------------- 外部标定
        w_o = [float(v) for v in p('localization.world_odom_xyz_rpy').value]
        self._R_world_odom = _rpy_to_mat(w_o[3:6])
        self._t_world_odom = np.array(w_o[0:3], dtype=np.float64)
        mnt = [float(v) for v in p('localization.gimbal_mount_xyz_rpy').value]
        self._mount_xyz = np.array(mnt[0:3], dtype=np.float64)

        # ---------------------------------------------------------- 运行时状态
        self._gimbal_state: Optional[GimbalState] = None
        self._target: Optional[TargetState] = None
        self._base_pos_odom = np.zeros(3, dtype=np.float64)
        self._R_odom_base = np.eye(3, dtype=np.float64)
        self._have_base = False
        self._T_map_world: Optional[np.ndarray] = None
        self._height = HeightTracker(cutoff_hz=float(p('control.height_cutoff_hz').value))

        self._t_start = now()
        self._t_last_cmd: Optional[float] = None
        self._t_last_target: Optional[float] = None
        self._search_entry_t: Optional[float] = None
        self._last_mode = MODE_IDLE
        self._have_ever_tracked = False

        # ---------------------------------------------------------- 话题
        self._pub = self.create_publisher(GimbalOmegaCmd, TOPIC_CMD_OMEGA, QOS_SENSOR)
        self.create_subscription(TargetState, TOPIC_TARGET_STATE, self.on_target_state, QOS_SENSOR)
        self.create_subscription(GimbalState, TOPIC_GIMBAL_STATE, self.on_gimbal_state, QOS_SENSOR)
        self.create_subscription(TransformStamped, TOPIC_T_MAP_WORLD, self.on_t_map_world, QOS_LATCHED)
        self.create_subscription(Odometry, TOPIC_BASE_POSE_RAW, self.on_base_pose, QOS_SENSOR)
        self.create_subscription(MarkerObservation, TOPIC_MARKER_CAM, self.on_marker, QOS_SENSOR)

        self.create_timer(1.0 / self._rate_hz, self.tick)

        self._log.info(
            'gimbal_controller_node 启动 | rate=%.1fHz kp=(%.2f,%.2f) kd=(%.2f,%.2f) '
            'ff=%.2f omega_max=(%.2f,%.2f) slew=%.1f aim=%s half_fov=%.4frad '
            'small_search=%s wide_search=%s',
            self._rate_hz, self._law.kp_yaw, self._law.kp_pitch,
            self._law.kd_yaw, self._law.kd_pitch, self._law.ff_gain,
            self._law.omega_max_yaw, self._law.omega_max_pitch,
            self._law.omega_slew, self._aim_mode, self._half_fov_rad,
            self._search_small.snapshot(), self._search_wide.snapshot(),
        )

    # ------------------------------------------------------------------ 内参
    def _resolve_half_fov(self) -> float:
        """解析水平半视场（rad），供搜索幅值上限使用。

        优先级（DESIGN §8.6：内参唯一数据源是 ``config/virtual.yaml``）:

        1. ``control.camera_fx`` + ``control.camera_width`` 均 > 0（显式覆盖，便于调试）
        2. ``config.virtual.yaml`` 的 ``camera.width`` / ``camera.fx``（默认路径）
        3. 两者都拿不到 → :data:`DEFAULT_HALF_FOV_RAD` 保守兜底 + WARN

        Returns:
            水平半视场（rad，正数）。
        """
        p = self.get_parameter
        fx = float(p('control.camera_fx').value)
        width = float(p('control.camera_width').value)
        if fx > 0.0 and width > 0.0:
            hf = half_fov_rad_from_intrinsics(width, fx)
            self._log.info('水平半视场 = %.4f rad（来源：control.camera_fx/width 显式覆盖）', hf)
            return hf

        vpath = str(p('config.virtual_yaml').value)
        w, f = load_camera_intrinsics(vpath)
        if w > 0.0 and f > 0.0:
            hf = half_fov_rad_from_intrinsics(w, f)
            self._log.info('水平半视场 = %.4f rad（来源：%s 的 camera.width/fx）',
                           hf, _resolve_virtual_yaml(vpath))
            return hf

        self._log.warn('未能从 %s 读到相机内参，水平半视场回退保守兜底值 %.4f rad',
                       vpath, DEFAULT_HALF_FOV_RAD)
        return DEFAULT_HALF_FOV_RAD

    # ------------------------------------------------------------------ 回调
    def on_gimbal_state(self, msg: GimbalState) -> None:
        """缓存最新云台姿态与陀螺。"""
        try:
            self._gimbal_state = msg
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'on_gimbal_state 异常: %s' % exc, 'gs_exc', 1.0)

    def on_base_pose(self, msg: Odometry) -> None:
        """缓存底盘位姿（frame=odom）。"""
        try:
            pos = msg.pose.pose.position
            if not all(math.isfinite(v) for v in (pos.x, pos.y, pos.z)):
                return
            self._base_pos_odom = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
            o = msg.pose.pose.orientation
            q = np.array([o.w, o.x, o.y, o.z], dtype=np.float64)
            if np.all(np.isfinite(q)) and float(np.linalg.norm(q)) >= 0.5:
                self._R_odom_base = np.asarray(
                    frames.quat_to_mat(q / float(np.linalg.norm(q))), dtype=np.float64).reshape(3, 3)
            self._have_base = True
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'on_base_pose 异常: %s' % exc, 'bp_exc', 1.0)

    def on_t_map_world(self, msg: TransformStamped) -> None:
        """接收锁存的 ``T_map←world``。"""
        try:
            if self._T_map_world is not None:
                return
            tr = msg.transform
            t = np.array([tr.translation.x, tr.translation.y, tr.translation.z], dtype=np.float64)
            q = np.array([tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z], dtype=np.float64)
            R = np.asarray(frames.quat_to_mat(q), dtype=np.float64).reshape(3, 3)
            if not (np.all(np.isfinite(t)) and np.all(np.isfinite(R))):
                return
            if str(msg.child_frame_id or '') == 'map':
                R = R.T
                t = -R @ t
            T = np.eye(4, dtype=np.float64)
            T[0:3, 0:3] = R
            T[0:3, 3] = t
            self._T_map_world = T
            self._log.info('收到 T_map←world：theta=%.4f rad', math.atan2(R[1, 0], R[0, 0]))
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'on_t_map_world 异常: %s' % exc, 'tmw_exc', 1.0)

    def on_target_state(self, msg: TargetState) -> None:
        """缓存最新的目标状态。"""
        try:
            self._target = msg
            if bool(msg.valid) and int(msg.track_status) != 0:
                self._t_last_target = now()      # ⏱ 单调时钟
                self._have_ever_tracked = True
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'on_target_state 异常: %s' % exc, 'ts_exc', 1.0)

    def on_marker(self, msg: MarkerObservation) -> None:
        """**辅助**：从 3D 观测提取目标的 world 系高度 z（见文件头"设计缺口"）。"""
        try:
            if not bool(msg.valid):
                return
            if self._gimbal_state is None:
                return
            p_opt = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
            if not np.all(np.isfinite(p_opt)):
                return
            q = np.asarray(self._gimbal_state.quat, dtype=np.float64).reshape(4)
            n = float(np.linalg.norm(q))
            if n < 0.5:
                return
            R_wg = np.asarray(frames.quat_to_mat(q / n), dtype=np.float64).reshape(3, 3)
            # optical → gimbal_link → world：只关心 z，用与 §6.6 完全相同的链路
            p_world = self.gimbal_origin_world() + (R_wg @ r_gimbal_from_opt()) @ p_opt
            if not math.isfinite(float(p_world[2])):
                return
            self._height.update(float(p_world[2]), now())
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', 'on_marker 异常: %s' % exc, 'mk_exc', 1.0)

    # ------------------------------------------------------------------ 计算
    def gimbal_origin_world(self) -> np.ndarray:
        """云台原点（world 系），委托 ``gimbal_common.frames``（DESIGN §6.9）。"""
        T_wo = np.eye(4, dtype=np.float64)
        T_wo[0:3, 0:3] = self._R_world_odom
        T_wo[0:3, 3] = self._t_world_odom
        return gimbal_origin_world(
            base_pos_odom=self._base_pos_odom,
            base_yaw=float(frames.yaw_of_mat(self._R_odom_base)),
            mount_xyz=self._mount_xyz,
            T_world_odom=T_wo,
        )

    def _target_age(self, t: float) -> float:
        """距上次有效目标状态的秒数；从未收到则为启动以来的时长。"""
        ref = self._t_last_target
        if ref is None:
            return max(0.0, t - self._t_start)
        return max(0.0, t - ref)

    def _mode_of_age(self, age: float) -> str:
        """按 DESIGN §6.8 丢失阶梯判定工作模式。"""
        if age <= self._lost_timeout_s:
            return MODE_TRACK
        if age <= self._search_after_s:
            return MODE_COAST
        if age <= self._wide_after_s:
            return MODE_SEARCH
        return MODE_WIDE

    def _theta_map_world(self) -> float:
        """``T_map←world`` 的偏航角 θ。"""
        if self._T_map_world is None:
            return 0.0
        R = self._T_map_world[0:3, 0:3]
        return math.atan2(float(R[1, 0]), float(R[0, 0]))

    def _map_xy_to_world_xy(self, xy_map: np.ndarray) -> np.ndarray:
        """map 系 XY → world 系 XY（``RotZ(−θ)·(p_map − t_map)`` 的 XY 部分）。"""
        if self._T_map_world is None:
            return np.asarray(xy_map, dtype=np.float64).reshape(2)
        R = self._T_map_world[0:3, 0:3]
        t = self._T_map_world[0:3, 3]
        # map 与 world 只差绕 z 的定角 ⇒ 取 2x2 分块即可
        Rot2 = R[0:2, 0:2]
        return Rot2.T @ (np.asarray(xy_map, dtype=np.float64).reshape(2) - t[0:2])

    # ------------------------------------------------------------------ 定时
    def tick(self) -> None:
        """100Hz 单调定时器：算 ω → 发布 ``GimbalOmegaCmd``。"""
        try:
            t = now()                                  # ⏱ 单调时钟
            dt = 0.0 if self._t_last_cmd is None else max(0.0, min(t - self._t_last_cmd, 0.5))
            self._t_last_cmd = t

            msg = GimbalOmegaCmd()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gimbal_link'

            if self._gimbal_state is None:
                msg.yaw_omega = 0.0
                msg.pitch_omega = 0.0
                msg.valid = False
                self._pub.publish(msg)
                throttled(self._log, 'warn', '尚无 /gimbal/state，发零指令保活', 'no_state', 1.0)
                return

            age = self._target_age(t)
            mode = self._mode_of_age(age)

            gyro = np.asarray(self._gimbal_state.gyro, dtype=np.float64).reshape(3)
            quat = np.asarray(self._gimbal_state.quat, dtype=np.float64).reshape(4)
            qn = float(np.linalg.norm(quat))
            if not np.all(np.isfinite(quat)) or qn < 0.5:
                throttled(self._log, 'warn', '云台四元数异常，发零指令保活', 'bad_quat', 1.0)
                msg.yaw_omega = 0.0
                msg.pitch_omega = 0.0
                msg.valid = False
                self._pub.publish(msg)
                return
            quat = quat / qn

            # 当前姿态角（软限位衰减需要；pitch 走 frames 单点函数）
            R = np.asarray(frames.quat_to_mat(quat), dtype=np.float64).reshape(3, 3)
            yaw_c, pitch_c = current_angles(R[:, 0])

            if mode in (MODE_TRACK, MODE_COAST) and self._target is not None \
                    and bool(self._target.valid):
                if self._search_entry_t is not None:
                    self._search_entry_t = None
                    self._law.reset()
                    self._log.info('重新捕获目标（age=%.3fs），退出搜索，KF 不重置', age)

                if self._aim_mode == 'pred':
                    xy_map = np.array([self._target.pred_x, self._target.pred_y], dtype=np.float64)
                else:
                    xy_map = np.array([self._target.cur_x, self._target.cur_y], dtype=np.float64)
                v_map = np.array([self._target.vel_x, self._target.vel_y], dtype=np.float64)

                xy_world = self._map_xy_to_world_xy(xy_map)
                theta = self._theta_map_world()
                Rot2 = np.array([[math.cos(theta), -math.sin(theta)],
                                 [math.sin(theta), math.cos(theta)]], dtype=np.float64)
                v_world_xy = Rot2.T @ v_map            # RotZ(−θ)

                p_world = np.array([xy_world[0], xy_world[1], self._height.height()],
                                   dtype=np.float64)
                v_world = np.array([v_world_xy[0], v_world_xy[1], self._height.vz],
                                   dtype=np.float64)
                if not (np.all(np.isfinite(p_world)) and np.all(np.isfinite(v_world))):
                    p_world = np.nan_to_num(p_world, nan=0.0, posinf=0.0, neginf=0.0)
                    v_world = np.nan_to_num(v_world, nan=0.0, posinf=0.0, neginf=0.0)

                out = self._law.compute(
                    target_world=p_world,
                    vel_world=v_world,
                    quat=quat,
                    gyro=gyro,
                    dt=dt,
                    gimbal_origin_world=self.gimbal_origin_world(),
                    mode=mode,
                )
                w_yaw, w_pitch = out.yaw_omega, out.pitch_omega
            elif mode in (MODE_SEARCH, MODE_WIDE) or self._target is None \
                    or not bool(self._target.valid):
                if self._search_entry_t is None:
                    self._search_entry_t = t
                    self._law.reset()
                tau = max(0.0, t - self._search_entry_t)
                pat = self._search_wide if mode == MODE_WIDE else self._search_small
                w_yaw, w_pitch = pat.sample(tau)
                w_yaw, w_pitch = self._law.limit(w_yaw, w_pitch, yaw_c, pitch_c, dt)
                mode = MODE_WIDE if mode == MODE_WIDE else MODE_SEARCH
            else:  # pragma: no cover - 兜底
                w_yaw, w_pitch = self._law.limit(0.0, 0.0, yaw_c, pitch_c, dt)

            if mode != self._last_mode:
                self._log.warn('控制模式切换: %s → %s（age=%.3fs）', self._last_mode, mode, age)
                self._last_mode = mode

            msg.yaw_omega = float(w_yaw)
            msg.pitch_omega = float(w_pitch)
            msg.valid = True
            self._pub.publish(msg)
        except Exception as exc:  # pragma: no cover - 定时器异常不得中断回路
            throttled(self._log, 'error', 'tick 异常: %s' % exc, 'tick_exc', 1.0)
            try:
                safe = GimbalOmegaCmd()
                safe.header.stamp = self.get_clock().now().to_msg()
                safe.yaw_omega = 0.0
                safe.pitch_omega = 0.0
                safe.valid = False
                self._pub.publish(safe)
            except Exception:
                pass

    # ------------------------------------------------------------------ 收尾
    def publish_stop(self) -> None:
        """退出前发一帧零指令（DESIGN §6.14：控制器先停止指令）。"""
        try:
            msg = GimbalOmegaCmd()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.yaw_omega = 0.0
            msg.pitch_omega = 0.0
            msg.valid = False
            self._pub.publish(msg)
        except Exception:
            pass

    def shutdown_summary(self) -> None:
        """退出时打印 INFO 摘要。"""
        self._log.info('gimbal_controller_node 退出 | 运行 %.1fs, %s',
                       now() - self._t_start, self._law.snapshot())


def main(args=None) -> int:
    """节点入口。"""
    rclpy.init(args=args)
    node = GimbalControllerNode()
    code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        code = 0
    except Exception as exc:  # pragma: no cover
        node._log.error('gimbal_controller_node 异常退出: %s' % exc)
        code = 1
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
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
