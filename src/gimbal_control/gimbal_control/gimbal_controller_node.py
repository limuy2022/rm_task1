# -*- coding: utf-8 -*-
"""Guarded world-frame aiming node for Ubuntu 22.04 / ROS 2 Humble.

Disabled by default. A world-alignment flag is an operator assertion after actual
calibration, not an automatic calibration. This legacy world controller assumes
an aligned camera/gimbal x axis and a planar base. No real joint feedback means
no control; IMU world yaw/pitch cannot be used as encoder positions. Search also
requires an explicit opt-in and a previously observed target. The separate pure
BearingControlLaw can be exercised offline without inventing world transforms.
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
from sensor_msgs.msg import JointState

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
TOPIC_JOINT_STATE = '/gimbal/joint_states'

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
        d('control.enabled', False)
        d('control.search_enabled', False)
        d('control.world_geometry_verified', False)
        d('control.input_timeout_s', 0.15)
        d('control.sync_tolerance_s', 0.02)
        d('control.camera_xyz_rpy', [0.0] * 6)
        d('control.joint_yaw_name', 'yaw')
        d('control.joint_pitch_name', 'pitch')
        d('control.kp_yaw', DEFAULT_KP_YAW)
        d('control.kp_pitch', DEFAULT_KP_PITCH)
        d('control.kd_yaw', DEFAULT_KD_YAW)
        d('control.kd_pitch', DEFAULT_KD_PITCH)
        d('control.ff_gain', DEFAULT_FF_GAIN)
        d('control.omega_max_yaw', DEFAULT_OMEGA_MAX_YAW)
        d('control.omega_max_pitch', DEFAULT_OMEGA_MAX_PITCH)
        d('control.omega_slew', DEFAULT_OMEGA_SLEW)
        d('control.rate_hz', 100.0)
        d('control.imu_timeout_s', 0.1)
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
        self._t_last_gimbal: Optional[float] = None
        self._imu_timeout_s = max(0.01, float(p('control.imu_timeout_s').value))
        self._target: Optional[TargetState] = None
        self._joint_angles = None
        self._t_last_joint = None
        self._t_last_base = None
        self._base_source_time = None
        self._base_velocity_world = None
        self._base_omega_world = None
        self._t_last_marker = None
        self._gimbal_source_time = None
        self._camera_xyz_rpy = np.asarray(p('control.camera_xyz_rpy').value, dtype=float).reshape(6)
        self._input_timeout_s = max(0.01, float(p('control.input_timeout_s').value))
        self._sync_tolerance_s = max(0.0, float(p('control.sync_tolerance_s').value))
        self._base_pos_odom = np.zeros(3, dtype=np.float64)
        self._R_odom_base = np.eye(3, dtype=np.float64)
        self._have_base = False
        self._T_map_world: Optional[np.ndarray] = None
        self._height = HeightTracker(cutoff_hz=float(p('control.height_cutoff_hz').value))
        self._marker_id = None

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
        self.create_subscription(JointState, TOPIC_JOINT_STATE, self.on_joint_state, QOS_SENSOR)
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
    def _source_time(self, msg, timeout=None):
        """Reject old/future/zero ROS stamps; reception alone is not freshness."""
        try:
            stamp = msg.header.stamp
            source = float(stamp.sec) + 1e-9 * float(stamp.nanosec)
            ros_now = float(self.get_clock().now().nanoseconds) * 1e-9
            age = ros_now - source
            allowed = self._input_timeout_s if timeout is None else timeout
            if source <= 0.0 or not math.isfinite(source) or not -0.03 <= age <= allowed:
                return None
            return source
        except (AttributeError, TypeError, ValueError):
            return None

    def _source_age(self, source_t):
        return max(0.0, self.get_clock().now().nanoseconds * 1e-9 - source_t)

    def on_joint_state(self, msg: JointState) -> None:
        """Only explicitly named, independently measured mechanical positions."""
        self._joint_angles = None
        self._t_last_joint = None
        try:
            source_t = self._source_time(msg)
            if source_t is None:
                return
            names = list(msg.name)
            yaw_name = str(self.get_parameter('control.joint_yaw_name').value)
            pitch_name = str(self.get_parameter('control.joint_pitch_name').value)
            if names.count(yaw_name) != 1 or names.count(pitch_name) != 1:
                return
            angles = np.array([msg.position[names.index(yaw_name)],
                               msg.position[names.index(pitch_name)]], dtype=float)
            if not np.all(np.isfinite(angles)):
                return
            self._joint_angles = angles
            self._t_last_joint = now() - self._source_age(source_t)
        except (AttributeError, IndexError, TypeError, ValueError):
            return

    def on_gimbal_state(self, msg: GimbalState) -> None:
        self._gimbal_state = None
        self._t_last_gimbal = None
        self._gimbal_source_time = None
        try:
            stamp = self._source_time(msg, self._imu_timeout_s)
            q = np.asarray(msg.quat, dtype=float).reshape(4)
            gyro = np.asarray(msg.gyro, dtype=float).reshape(3)
            if (stamp is None or str(msg.header.frame_id) != 'gimbal_link'
                    or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 0.5
                    or not np.all(np.isfinite(gyro))):
                return
            self._gimbal_state = msg
            self._t_last_gimbal = now() - self._source_age(stamp)
            self._gimbal_source_time = stamp
        except (AttributeError, TypeError, ValueError):
            return

    def on_base_pose(self, msg: Odometry) -> None:
        """Differentiate source-stamped poses; never assume Odometry twist frame."""
        try:
            source_t = self._source_time(msg)
            pos = msg.pose.pose.position
            xyz = np.array([pos.x, pos.y, pos.z], dtype=float)
            o = msg.pose.pose.orientation
            q = np.array([o.w, o.x, o.y, o.z], dtype=float)
            if (source_t is None or str(msg.header.frame_id) != 'odom'
                    or not np.all(np.isfinite(xyz))
                    or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 0.5):
                self._t_last_base = None
                self._base_velocity_world = self._base_omega_world = None
                return
            R = np.asarray(frames.quat_to_mat(q / np.linalg.norm(q)), dtype=float).reshape(3, 3)
            # Keep the planar-base assumption explicit rather than applying a
            # yaw-only formula to a tilted mount/base.
            if np.linalg.norm(R[2, :2]) > 1e-3:
                self._t_last_base = None
                self._base_velocity_world = self._base_omega_world = None
                return
            if self._base_source_time is not None:
                delta_t = source_t - self._base_source_time
                if 1e-4 < delta_t <= self._input_timeout_s:
                    # Differentiating the full origin includes omega x mount.
                    old_origin = self.gimbal_origin_world()
                    old_yaw = float(frames.yaw_of_mat(self._R_odom_base))
                    new_yaw = float(frames.yaw_of_mat(R))
                    self._base_pos_odom, self._R_odom_base = xyz, R
                    self._base_velocity_world = (self.gimbal_origin_world() - old_origin) / delta_t
                    yaw_rate = math.atan2(math.sin(new_yaw - old_yaw),
                                          math.cos(new_yaw - old_yaw)) / delta_t
                    self._base_omega_world = self._R_world_odom @ np.array([0.0, 0.0, yaw_rate])
                else:
                    self._base_velocity_world = self._base_omega_world = None
            self._base_pos_odom, self._R_odom_base = xyz, R
            self._base_source_time = source_t
            self._t_last_base = now() - self._source_age(source_t)
            self._have_base = True
        except (AttributeError, TypeError, ValueError):
            self._t_last_base = None
            self._base_velocity_world = self._base_omega_world = None

    def on_t_map_world(self, msg: TransformStamped) -> None:
        """接收锁存的 ``T_map←world``。"""
        try:
            if self._T_map_world is not None:
                return
            q_test = np.array([msg.transform.rotation.w, msg.transform.rotation.x,
                               msg.transform.rotation.y, msg.transform.rotation.z], dtype=float)
            if not np.all(np.isfinite(q_test)) or np.linalg.norm(q_test) < 0.5:
                return
            tr = msg.transform
            t = np.array([tr.translation.x, tr.translation.y, tr.translation.z], dtype=np.float64)
            q = np.array([tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z], dtype=np.float64)
            R = np.asarray(frames.quat_to_mat(q), dtype=np.float64).reshape(3, 3)
            if (str(msg.header.frame_id), str(msg.child_frame_id)) not in (
                    ('map', 'world'), ('world', 'map')):
                return
            if np.linalg.norm(R[2, :2]) > 1e-3:
                return
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
        """Estimator publication must not renew the last real observation."""
        self._target = None
        try:
            stamp = self._source_time(msg)
            obs_age = float(msg.last_obs_age)
            values = (msg.cur_x, msg.cur_y, msg.pred_x, msg.pred_y, msg.vel_x, msg.vel_y)
            if (stamp is None or str(msg.header.frame_id) != 'map'
                    or not bool(msg.valid) or int(msg.track_status) not in (1, 2)
                    or not all(math.isfinite(float(v)) for v in values)
                    or not math.isfinite(obs_age) or obs_age < 0.0):
                return
            transport_age = max(0.0, self.get_clock().now().nanoseconds * 1e-9 - stamp)
            age = obs_age + transport_age
            self._target = msg
            self._t_last_target = now() - age
            if age <= self._lost_timeout_s:
                self._have_ever_tracked = True
        except (AttributeError, TypeError, ValueError):
            return

    def on_marker(self, msg: MarkerObservation) -> None:
        """**辅助**：从 3D 观测提取目标的 world 系高度 z（见文件头"设计缺口"）。"""
        try:
            self._t_last_marker = None
            marker_time = self._source_time(msg)
            if (not bool(msg.valid) or marker_time is None
                    or str(msg.header.frame_id) != 'camera_optical_frame'):
                self._height.reset()
                return
            marker_id = int(msg.marker_id)
            if self._marker_id != marker_id:
                self._height.reset()
                self._marker_id = marker_id
            if (self._gimbal_state is None or self._gimbal_source_time is None
                    or self._t_last_base is None or now() - self._t_last_base > self._input_timeout_s
                    or abs(marker_time - self._gimbal_source_time) > self._sync_tolerance_s):
                return
            p_opt = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
            if not np.all(np.isfinite(p_opt)) or p_opt[2] <= 0.0:
                return
            q = np.asarray(self._gimbal_state.quat, dtype=np.float64).reshape(4)
            n = float(np.linalg.norm(q))
            if n < 0.5:
                return
            R_wg = np.asarray(frames.quat_to_mat(q / n), dtype=np.float64).reshape(3, 3)
            # optical → gimbal_link → world：只关心 z，用与 §6.6 完全相同的链路
            p_gimbal = self._camera_xyz_rpy[:3] + (
                _rpy_to_mat(self._camera_xyz_rpy[3:]) @ r_gimbal_from_opt()) @ p_opt
            p_world = self.gimbal_origin_world() + R_wg @ p_gimbal
            if not math.isfinite(float(p_world[2])):
                return
            self._height.update(float(p_world[2]), now())
            self._t_last_marker = now() - self._source_age(marker_time)
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
        if (self._target is not None and bool(self._target.valid)
                and math.isfinite(age) and age <= self._lost_timeout_s):
            return MODE_TRACK
        if (not bool(self.get_parameter('control.search_enabled').value)
                or not self._have_ever_tracked or age <= self._search_after_s):
            return MODE_IDLE
        return MODE_SEARCH if age <= self._wide_after_s else MODE_WIDE

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

    def _stop(self, msg) -> None:
        self._law.reset()
        self._search_entry_t = None
        msg.yaw_omega = msg.pitch_omega = 0.0
        msg.valid = False
        self._pub.publish(msg)

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

            # No command becomes valid merely because the node was launched.
            # On every disable/invalid-input stop discard slew history, so a
            # later enable cannot resume a stale nonzero output.
            if (not bool(self.get_parameter('control.enabled').value)
                    or not bool(self.get_parameter('control.world_geometry_verified').value)
                    or self._joint_angles is None or self._t_last_joint is None
                    or t - self._t_last_joint > self._input_timeout_s
                    or self._T_map_world is None or self._t_last_base is None
                    or t - self._t_last_base > self._input_timeout_s
                    or self._base_velocity_world is None or self._base_omega_world is None
                    or not np.all(np.isfinite(self._camera_xyz_rpy))
                    or np.linalg.norm(self._camera_xyz_rpy[3:]) > 1e-6
                    or np.linalg.norm(self._R_world_odom[2, :2]) > 1e-3):
                self._stop(msg)
                return

            if (self._gimbal_state is None or self._t_last_gimbal is None
                    or t - self._t_last_gimbal > self._imu_timeout_s):
                msg.yaw_omega = 0.0
                msg.pitch_omega = 0.0
                msg.valid = False
                self._stop(msg)
                throttled(self._log, 'warn', '云台姿态缺失或过期，输出无效零指令', 'no_state', 1.0)
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
                self._stop(msg)
                return
            quat = quat / qn

            # World attitude is used for geometry only; limits use JointState.
            R = np.asarray(frames.quat_to_mat(quat), dtype=np.float64).reshape(3, 3)
            yaw_c, pitch_c = self._joint_angles

            if mode in (MODE_TRACK, MODE_COAST) and self._target is not None \
                    and bool(self._target.valid):
                if self._search_entry_t is not None:
                    self._search_entry_t = None
                    self._law.reset()
                    self._log.info('重新捕获目标（age=%.3fs），退出搜索，KF 不重置', age)

                if (self._t_last_marker is None or t - self._t_last_marker > self._input_timeout_s
                        or self._height.z is None):
                    self._stop(msg)
                    return
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
                    self._stop(msg)
                    return

                out = self._law.compute(
                    target_world=p_world,
                    vel_world=v_world,
                    quat=quat,
                    gyro=gyro,
                    dt=dt,
                    # Aim from the CAMERA centre. The IMU-camera lever arm
                    # also contributes omega x r to origin translation.
                    gimbal_origin_world=self.gimbal_origin_world() + R @ self._camera_xyz_rpy[:3],
                    mode=mode,
                    joint_angles=self._joint_angles,
                    origin_velocity_world=(self._base_velocity_world
                                           + R @ np.cross(gyro, self._camera_xyz_rpy[:3])),
                    base_angular_velocity_world=self._base_omega_world,
                )
                if not out.valid:
                    self._stop(msg)
                    return
                w_yaw, w_pitch = out.yaw_omega, out.pitch_omega
            elif mode in (MODE_SEARCH, MODE_WIDE):
                if self._half_fov_rad <= 0.0:
                    self._stop(msg)
                    return
                if self._search_entry_t is None:
                    self._search_entry_t = t
                    self._law.reset()
                tau = max(0.0, t - self._search_entry_t)
                pat = self._search_wide if mode == MODE_WIDE else self._search_small
                w_yaw, w_pitch = pat.sample(tau)
                w_yaw, w_pitch = self._law.limit(w_yaw, w_pitch, yaw_c, pitch_c, dt)
                mode = MODE_WIDE if mode == MODE_WIDE else MODE_SEARCH
            else:
                self._stop(msg)
                return

            if mode != self._last_mode:
                self._log.warn('控制模式切换: %s → %s（age=%.3fs）', self._last_mode, mode, age)
                self._last_mode = mode

            msg.yaw_omega = float(w_yaw)
            msg.pitch_omega = float(w_pitch)
            msg.valid = True
            self._pub.publish(msg)
        except Exception as exc:  # pragma: no cover - 定时器异常不得中断回路
            self._law.reset()
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
