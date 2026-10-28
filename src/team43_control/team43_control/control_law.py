# -*- coding: utf-8 -*-
"""Pure world-frame aiming and camera-bearing control, with no ROS or hardware IO.

World aiming requires an aligned world frame, a planar base, known mechanical
joint positions and a gimbal x-axis that is the calibrated camera optical axis.
Never use world IMU yaw/pitch as mechanical limits. Feedforward uses relative
translation and removes base rotation before emitting joint-rate commands.
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional, Sequence, Tuple

import numpy as np

from team43_common import frames
from team43_common.math_utils import clamp, wrap_pi

# --------------------------------------------------------------------------
# 默认参数（DESIGN §6.13 control.* / gimbal.* / hardware_spec.md）
# --------------------------------------------------------------------------
DEFAULT_KP_YAW = 6.0
DEFAULT_KP_PITCH = 6.0
DEFAULT_KD_YAW = 0.5
DEFAULT_KD_PITCH = 0.5
DEFAULT_FF_GAIN = 1.0
DEFAULT_OMEGA_MAX_YAW = 2.0
DEFAULT_OMEGA_MAX_PITCH = 2.0
DEFAULT_OMEGA_SLEW = 15.0          # rad/s^2
DEFAULT_YAW_MIN = -2.8
DEFAULT_YAW_MAX = 2.8
DEFAULT_PITCH_MIN = -0.5           # 协议域（低头为正）
DEFAULT_PITCH_MAX = 0.35           # 协议域（低头为正）
DEFAULT_LIMIT_MARGIN = 0.20
DEFAULT_COS_PITCH_FLOOR = 0.1      # cos(pitch_c) 下限，防除零

EPS_R2 = 1e-9                      # ‖r‖² 的防零分母


class ControlOutput(NamedTuple):
    """一次控制律计算的完整输出（同时是一个二元组以上的具名 tuple）。

    Attributes:
        yaw_omega: yaw 关节角速度指令（协议域 rad/s）。
        pitch_omega: pitch 关节角速度指令（协议域 rad/s，低头为正）。
        valid: 指令是否有效（输入齐全且数值有限）。
        mode: 工作模式字符串（``track``/``coast``/``search``/``wide``/``idle``）。
        e_yaw: yaw 误差（rad）。
        e_pitch: pitch 误差（rad，协议域）。
        ff_yaw: yaw 前馈分量（rad/s）。
        ff_pitch: pitch 前馈分量（rad/s，协议域）。
        yaw_c: 云台当前 yaw（rad）。
        pitch_c: 云台当前 pitch（rad，协议域）。
    """

    yaw_omega: float = 0.0
    pitch_omega: float = 0.0
    valid: bool = False
    mode: str = 'idle'
    e_yaw: float = 0.0
    e_pitch: float = 0.0
    ff_yaw: float = 0.0
    ff_pitch: float = 0.0
    yaw_c: float = 0.0
    pitch_c: float = 0.0


# --------------------------------------------------------------------------
# 纯函数构件（可单独单测）
# --------------------------------------------------------------------------
def line_of_sight(target_world: Sequence[float],
                  gimbal_origin_world: Sequence[float]) -> np.ndarray:
    """Step 1：``r = p_t − p_gimbal_world``（world 系）。"""
    p_t = np.asarray(target_world, dtype=np.float64).reshape(3)
    p_g = np.asarray(gimbal_origin_world, dtype=np.float64).reshape(3)
    r = p_t - p_g
    return r if np.all(np.isfinite(r)) else np.zeros(3, dtype=np.float64)


def attitude_axes(R: Sequence[Sequence[float]]) -> Tuple[np.ndarray, np.ndarray]:
    """Step 1：取机体 x 轴（光轴）与 y 轴在 world 下的表示。

    Args:
        R: ``R_world←gimbal``（列 = 机体系基向量在 world 下的表示）。

    Returns:
        ``(f, y_body)``，各 shape (3,)。
    """
    M = np.asarray(R, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(M)):
        M = np.eye(3, dtype=np.float64)
    return M[:, 0].copy(), M[:, 1].copy()


def target_angles(r: np.ndarray) -> Tuple[float, float]:
    """Step 2：由视线向量求目标 yaw / pitch（pitch 为**协议域**）。

    ``yaw_t = atan2(r.y, r.x)``
    ``pitch_t = frames.to_protocol_pitch(atan2(r.z, hypot(r.x, r.y)))``
    """
    rv = np.asarray(r, dtype=np.float64).reshape(3)
    yaw_t = math.atan2(float(rv[1]), float(rv[0]))
    horiz = math.hypot(float(rv[0]), float(rv[1]))
    pitch_head_up = math.atan2(float(rv[2]), horiz)
    pitch_t = float(frames.to_protocol_pitch(pitch_head_up))
    return float(yaw_t), pitch_t


def current_angles(f: np.ndarray) -> Tuple[float, float]:
    """Step 2：由光轴向量求云台当前 yaw / pitch（pitch 为**协议域**）。"""
    fv = np.asarray(f, dtype=np.float64).reshape(3)
    yaw_c = math.atan2(float(fv[1]), float(fv[0]))
    horiz = math.hypot(float(fv[0]), float(fv[1]))
    pitch_head_up = math.atan2(float(fv[2]), horiz)
    pitch_c = float(frames.to_protocol_pitch(pitch_head_up))
    return float(yaw_c), pitch_c


def feedforward(r: np.ndarray, vel_world: Sequence[float]) -> Tuple[float, float]:
    """Exact yaw/elevation rate of a sight line, in protocol pitch convention.

    vel_world is target minus aiming-origin translational velocity.
    Singular/invalid sight lines give zero feedforward; callers
    must reject the undefined aiming geometry, as ControlLaw.compute does.
    """
    rv = np.asarray(r, dtype=np.float64).reshape(3)
    vv = np.asarray(vel_world, dtype=np.float64).reshape(3)
    if not (np.all(np.isfinite(rv)) and np.all(np.isfinite(vv))):
        return 0.0, 0.0
    # Exact derivatives of yaw and protocol pitch. cross(r,v)/|r|² is an
    # angular velocity vector, whose z component is not yaw_dot off the horizon.
    h2 = float(rv[0] ** 2 + rv[1] ** 2)
    h = math.sqrt(h2)
    r2 = float(rv @ rv)
    if h < 1e-6 or r2 < EPS_R2:
        return 0.0, 0.0
    ff_yaw = float((rv[0] * vv[1] - rv[1] * vv[0]) / h2)
    h_dot = float((rv[0] * vv[0] + rv[1] * vv[1]) / h)
    pitch_up_dot = float((h * vv[2] - rv[2] * h_dot) / r2)
    ff_pitch = float(frames.to_protocol_pitch(pitch_up_dot))
    return finite_guard(ff_yaw), finite_guard(ff_pitch)


def imu_feedback(gyro: Sequence[float], pitch_c: float) -> Tuple[float, float]:
    """Step 5：从机体系陀螺估计**世界系姿态**角速度（平面底盘、无 roll）（协议域）。

    ``ω_imu_yaw = gyro.z / max(cos(pitch_c), 0.1)``
    ``ω_imu_pitch = gyro.y``

    协议自检（DESIGN §9.3 T-C2 / hardware_spec §2）：
    下发正的 ``pitch_omega`` → 机体 ``gy > 0``；下发正的 ``yaw_omega`` → ``gz > 0``。
    因此这里的映射必须与之一致（pitch 取 ``gyro.y``，yaw 取 ``gyro.z``）。
    """
    gv = np.asarray(gyro, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(gv)):
        return 0.0, 0.0
    cos_p = math.cos(float(pitch_c))
    if not math.isfinite(cos_p):
        cos_p = DEFAULT_COS_PITCH_FLOOR
    denom = max(cos_p, DEFAULT_COS_PITCH_FLOOR)
    w_yaw = float(gv[2]) / denom
    w_pitch = float(gv[1])
    return (w_yaw if math.isfinite(w_yaw) else 0.0,
            w_pitch if math.isfinite(w_pitch) else 0.0)


def slew_limit(omega: float, omega_prev: float, dt: float,
               slew: float = DEFAULT_OMEGA_SLEW) -> float:
    """Step 6-1：斜坡限幅 ``|ω(k) − ω(k−1)| ≤ slew · dt``。"""
    if dt <= 0.0 or slew <= 0.0:
        return float(omega)
    delta = float(omega) - float(omega_prev)
    max_delta = float(slew) * float(dt)
    if delta > max_delta:
        return float(omega_prev) + max_delta
    if delta < -max_delta:
        return float(omega_prev) - max_delta
    return float(omega)


def soft_limit_factor(angle: float, omega: float, lo: float, hi: float,
                      margin: float = DEFAULT_LIMIT_MARGIN) -> float:
    """Step 6-3：软限位衰减系数。

    当角速度把云台继续往限位方向推、且已进入减速带时，按剩余距离线性衰减。

    Returns:
        ``[0, 1]`` 的衰减系数；不需要衰减时返回 1.0。
    """
    if not all(math.isfinite(float(v)) for v in (angle, omega, lo, hi, margin)) or lo >= hi:
        return 0.0
    if omega > 0.0 and angle >= hi:
        return 0.0
    if omega < 0.0 and angle <= lo:
        return 0.0
    if margin <= 0.0:
        return 1.0
    if omega > 0.0 and angle > hi - margin:
        return max(0.0, (hi - angle) / margin)
    if omega < 0.0 and angle < lo + margin:
        return max(0.0, (angle - lo) / margin)
    return 1.0


def finite_guard(v: float) -> float:
    """Step 6-4：非有限值 → 0.0。"""
    return float(v) if math.isfinite(float(v)) else 0.0


# --------------------------------------------------------------------------
# 控制律主体
# --------------------------------------------------------------------------
class ControlLaw:
    """前馈 + PD 云台控制律（DESIGN §6.8）。

    内部保存上一次输出的角速度，用于 Step 6 的 slew 限幅。
    """

    def __init__(
        self,
        kp_yaw: float = DEFAULT_KP_YAW,
        kp_pitch: float = DEFAULT_KP_PITCH,
        kd_yaw: float = DEFAULT_KD_YAW,
        kd_pitch: float = DEFAULT_KD_PITCH,
        ff_gain: float = DEFAULT_FF_GAIN,
        omega_max_yaw: float = DEFAULT_OMEGA_MAX_YAW,
        omega_max_pitch: float = DEFAULT_OMEGA_MAX_PITCH,
        omega_slew: float = DEFAULT_OMEGA_SLEW,
        yaw_min: float = DEFAULT_YAW_MIN,
        yaw_max: float = DEFAULT_YAW_MAX,
        pitch_min: float = DEFAULT_PITCH_MIN,
        pitch_max: float = DEFAULT_PITCH_MAX,
        limit_margin: float = DEFAULT_LIMIT_MARGIN,
    ) -> None:
        self.kp_yaw = float(kp_yaw)
        self.kp_pitch = float(kp_pitch)
        self.kd_yaw = float(kd_yaw)
        self.kd_pitch = float(kd_pitch)
        self.ff_gain = float(ff_gain)
        self.omega_max_yaw = float(omega_max_yaw)
        self.omega_max_pitch = float(omega_max_pitch)
        self.omega_slew = float(omega_slew)
        self.yaw_min = float(yaw_min)
        self.yaw_max = float(yaw_max)
        self.pitch_min = float(pitch_min)
        self.pitch_max = float(pitch_max)
        self.limit_margin = float(limit_margin)
        scalars = (self.kp_yaw, self.kp_pitch, self.kd_yaw, self.kd_pitch,
                   self.ff_gain, self.omega_max_yaw, self.omega_max_pitch,
                   self.omega_slew, self.yaw_min, self.yaw_max,
                   self.pitch_min, self.pitch_max, self.limit_margin)
        if (not all(math.isfinite(v) for v in scalars)
                or any(v < 0.0 for v in scalars[:7]) or self.omega_slew <= 0.0
                or self.yaw_min >= self.yaw_max or self.pitch_min >= self.pitch_max
                or self.limit_margin < 0.0):
            raise ValueError('control gains, rate limits and mechanical limits must be finite and ordered')

        self._omega_yaw_prev = 0.0
        self._omega_pitch_prev = 0.0

    # ------------------------------------------------------------------ 状态
    def reset(self) -> None:
        """清空 slew 历史（切回跟踪 / 重新捕获时调用不影响限位安全）。"""
        self._omega_yaw_prev = 0.0
        self._omega_pitch_prev = 0.0

    @property
    def omega_prev(self) -> Tuple[float, float]:
        """上一次输出的 ``(yaw_omega, pitch_omega)``。"""
        return (self._omega_yaw_prev, self._omega_pitch_prev)

    # ------------------------------------------------------------------ 主计算
    def compute(
        self,
        target_world: Sequence[float],
        vel_world: Sequence[float],
        quat: Sequence[float],
        gyro: Sequence[float],
        dt: float = 0.01,
        gimbal_origin_world: Optional[Sequence[float]] = None,
        mode: str = 'track',
        *,
        joint_angles: Optional[Sequence[float]] = None,
        origin_velocity_world: Optional[Sequence[float]] = None,
        base_angular_velocity_world: Optional[Sequence[float]] = None,
    ) -> ControlOutput:
        """执行一次完整的六步控制律（含限幅与保护）。

        Args:
            target_world: 目标在 **world** 系下的位置，shape (3,)。
            vel_world: 目标在 **world** 系下的速度，shape (3,)（map 系速度需先反旋）。
            quat: ``(w,x,y,z)`` 表示 R_world←gimbal，不能当成关节角。
            gyro: 机体系陀螺 ``(gx, gy, gz)``，rad/s。
            dt: 距上次指令的间隔（秒，**单调时钟**），用于 slew 限幅。
            gimbal_origin_world: 云台原点（world 系）；``None`` 视为原点。
            mode: 工作模式标签，原样填入输出。
            joint_angles: 真实机械 yaw/pitch 角；缺失或非法则输出无效零指令。
            origin_velocity_world: 云台原点平移速度，须与目标速度在同一世界系。
            base_angular_velocity_world: 底盘角速度；已验证平面底盘模型下扣除底盘旋转。
                后两项省略只适用于明确静止的台架。

        Returns:
            :class:`ControlOutput`。
        """
        # Mechanical positions must come from encoders or an independently
        # verified joint-state source. An IMU world yaw is never a joint angle.
        try:
            target = np.asarray(target_world, dtype=float).reshape(3)
            velocity = np.asarray(vel_world, dtype=float).reshape(3)
            origin = np.zeros(3) if gimbal_origin_world is None else np.asarray(
                gimbal_origin_world, dtype=float).reshape(3)
            origin_v = np.zeros(3) if origin_velocity_world is None else np.asarray(
                origin_velocity_world, dtype=float).reshape(3)
            base_w = np.zeros(3) if base_angular_velocity_world is None else np.asarray(
                base_angular_velocity_world, dtype=float).reshape(3)
            joints = np.asarray(joint_angles, dtype=float).reshape(2)
            q = np.asarray(quat, dtype=float).reshape(4)
            gv = np.asarray(gyro, dtype=float).reshape(3)
            values = (target, velocity, origin, origin_v, base_w, joints, q, gv)
            if not all(np.all(np.isfinite(v)) for v in values):
                raise ValueError('non-finite input')
            if not math.isfinite(float(dt)) or dt <= 0.0 or np.linalg.norm(q) < 0.5:
                raise ValueError('invalid interval or attitude')
            r = target - origin
            if math.hypot(float(r[0]), float(r[1])) < 1e-6:
                raise ValueError('undefined yaw')
        except (TypeError, ValueError, OverflowError):
            # 外部目标/姿态/关节数值非法时清斜坡，明确输出 invalid 零指令。
            self.reset()
            return ControlOutput(mode=mode)

        R = np.asarray(frames.quat_to_mat(q / np.linalg.norm(q)), dtype=float).reshape(3, 3)
        f, y_body = attitude_axes(R)
        yaw_t, pitch_t = target_angles(r)
        yaw_c, pitch_c = current_angles(f)
        # These Euler rates require the explicitly verified planar-base model.
        if abs(float(R[2, 1])) > 1e-3 or abs(math.cos(pitch_c)) < 0.2:
            self.reset()
            return ControlOutput(mode=mode)
        e_yaw = float(wrap_pi(yaw_t - yaw_c))
        e_pitch = float(pitch_t - pitch_c)
        ff_yaw, ff_pitch = feedforward(r, velocity - origin_v)
        measured_yaw, measured_pitch = imu_feedback(gv, pitch_c)
        # The drive accepts joint rates, whereas the gyro measures total world
        # motion (base + joints). Subtract base rates, and damp the rate error
        # around the desired sight-line rate rather than damping feedforward.
        w_yaw = (self.ff_gain * ff_yaw + self.kp_yaw * e_yaw
                 - self.kd_yaw * (measured_yaw - ff_yaw) - float(base_w[2]))
        w_pitch = (self.ff_gain * ff_pitch + self.kp_pitch * e_pitch
                   - self.kd_pitch * (measured_pitch - ff_pitch) - float(base_w @ y_body))
        w_yaw, w_pitch = self.limit(w_yaw, w_pitch, joints[0], joints[1], dt)
        return ControlOutput(w_yaw, w_pitch, True, mode, e_yaw, e_pitch,
                             ff_yaw, ff_pitch, yaw_c, pitch_c)

    # ------------------------------------------------------------------ 限幅
    def limit(self, omega_yaw: float, omega_pitch: float,
              yaw_c: float, pitch_c: float, dt: float) -> Tuple[float, float]:
        """Step 6 的完整限幅链（顺序**不可调换**）。

        1. slew 限幅
        2. 幅值限幅 ±omega_max
        3. 软限位衰减
        4. 有限性守卫

        Returns:
            ``(omega_yaw, omega_pitch)``。
        """
        # 此处yaw_c/pitch_c是实测机械关节位置，不是IMU惯性系的朝向。
        values = (omega_yaw, omega_pitch, yaw_c, pitch_c, dt)
        if not all(math.isfinite(float(v)) for v in values) or dt <= 0.0:
            self.reset()
            return 0.0, 0.0
        d = float(dt)

        # 1) slew
        w_yaw = slew_limit(omega_yaw, self._omega_yaw_prev, d, self.omega_slew)
        w_pitch = slew_limit(omega_pitch, self._omega_pitch_prev, d, self.omega_slew)

        # 2) 幅值限幅
        w_yaw = float(clamp(w_yaw, -self.omega_max_yaw, self.omega_max_yaw))
        w_pitch = float(clamp(w_pitch, -self.omega_max_pitch, self.omega_max_pitch))

        # 3) 软限位衰减
        w_yaw *= soft_limit_factor(yaw_c, w_yaw, self.yaw_min, self.yaw_max, self.limit_margin)
        w_pitch *= soft_limit_factor(pitch_c, w_pitch, self.pitch_min, self.pitch_max,
                                     self.limit_margin)

        # 4) 有限性守卫
        w_yaw = finite_guard(w_yaw)
        w_pitch = finite_guard(w_pitch)

        self._omega_yaw_prev = w_yaw
        self._omega_pitch_prev = w_pitch
        return w_yaw, w_pitch

    def snapshot(self) -> dict:
        """导出当前增益与限位配置（日志/调试用）。"""
        return {
            'kp_yaw': self.kp_yaw, 'kp_pitch': self.kp_pitch,
            'kd_yaw': self.kd_yaw, 'kd_pitch': self.kd_pitch,
            'ff_gain': self.ff_gain,
            'omega_max': [self.omega_max_yaw, self.omega_max_pitch],
            'omega_slew': self.omega_slew,
            'yaw_limits': [self.yaw_min, self.yaw_max],
            'pitch_limits': [self.pitch_min, self.pitch_max],
            'limit_margin': self.limit_margin,
            'omega_prev': [self._omega_yaw_prev, self._omega_pitch_prev],
        }


class BearingControlLaw(ControlLaw):
    """Camera-bearing aiming without a map, LIO or absolute IMU heading.

    This is a pure calculation entry, not a ROS/serial actuator. It uses the
    verified R_gimbal←optical and two mechanical joint axes. For a yaw-then-pitch
    head, yaw axis in gimbal coordinates is (-sin(pitch), 0, cos(pitch)); pitch
    axis is (0,1,0). The Jacobian maps joint rates to optical-axis motion.

    Base angular velocity, when available, is expressed in gimbal coordinates.
    target_relative_velocity_gimbal is target minus CAMERA-origin translational
    velocity, expressed in the current gimbal basis, BEFORE rotational transport.
    Do not feed a finite-differenced rotating-camera coordinate into that field.
    Omitted velocities explicitly model a static bench, not a moving robot.
    """

    def compute_bearing(self, marker_optical, *, joint_angles=None,
                        R_gimbal_optical=None, geometry_verified=False,
                        base_omega_gimbal=(0.0, 0.0, 0.0),
                        target_relative_velocity_gimbal=(0.0, 0.0, 0.0),
                        dt=0.01, observation_age=0.0, max_age=0.15,
                        enabled=False):
        try:
            point = np.asarray(marker_optical, dtype=float).reshape(3)
            joints = np.asarray(joint_angles, dtype=float).reshape(2)
            rotation = np.asarray(R_gimbal_optical, dtype=float).reshape(3, 3)
            base_w = np.asarray(base_omega_gimbal, dtype=float).reshape(3)
            rel_v = np.asarray(target_relative_velocity_gimbal, dtype=float).reshape(3)
            if not enabled or not geometry_verified:
                raise ValueError('disabled or unverified extrinsics')
            if not all(np.all(np.isfinite(v)) for v in (point, joints, rotation, base_w, rel_v)):
                raise ValueError('non-finite input')
            if (not math.isfinite(float(observation_age)) or not math.isfinite(float(max_age))
                    or max_age <= 0.0 or not 0.0 <= observation_age <= max_age
                    or not math.isfinite(float(dt)) or dt <= 0.0 or point[2] <= 1e-6):
                raise ValueError('stale observation or invalid depth')
            if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                    or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6)):
                raise ValueError('extrinsic rotation is not SO(3)')
            ray = rotation @ point
            distance = float(np.linalg.norm(ray))
            desired = ray / distance
            forward = rotation[:, 2]
            yaw_axis = np.array([-math.sin(joints[1]), 0.0, math.cos(joints[1])])
            pitch_axis = np.array([0.0, 1.0, 0.0])
            jac = np.column_stack((np.cross(yaw_axis, forward), np.cross(pitch_axis, forward)))
            if float(np.linalg.svd(jac, compute_uv=False)[-1]) < 0.2:
                raise ValueError('near singular joint geometry')
            error = desired - float(desired @ forward) * forward
            error_rates = np.linalg.lstsq(jac, error, rcond=None)[0]
            target_ray_rate = (rel_v - desired * float(desired @ rel_v)) / distance
            ff_rates = np.linalg.lstsq(jac, target_ray_rate - np.cross(base_w, forward),
                                     rcond=None)[0]
            raw = np.array([self.kp_yaw, self.kp_pitch]) * error_rates + self.ff_gain * ff_rates
            yaw, pitch = self.limit(raw[0], raw[1], joints[0], joints[1], dt)
            return ControlOutput(yaw, pitch, True, 'bearing', float(error_rates[0]),
                                 float(error_rates[1]), float(ff_rates[0]),
                                 float(ff_rates[1]))
        except (TypeError, ValueError, OverflowError, np.linalg.LinAlgError):
            # 观测无效或几何求解奇异时必须清斜坡，不能保留上一拍角速度。
            self.reset()
            return ControlOutput(mode='bearing')


# --------------------------------------------------------------------------
# 云台原点（DESIGN §6.9）—— 真源在 team43_common.frames，此处只做薄封装
# --------------------------------------------------------------------------
def gimbal_origin_world(base_pos_odom: Sequence[float],
                        base_yaw: float,
                        mount_xyz: Sequence[float],
                        T_world_odom: Optional[Sequence[Sequence[float]]] = None) -> np.ndarray:
    """云台原点在 world 系下的坐标（DESIGN §6.9，estimation 与 control 共用）。

    ``p_base_world   = T_world←odom ∘ p_base_odom``
    ``p_gimbal_world = p_base_world + R_world←odom · Rz(yaw_base) · mount_xyz``

    委托 ``team43_common.frames.gimbal_origin_world``（单一真源）。

    Args:
        base_pos_odom: 底盘在 odom 系的 (x, y) 或 (x, y, z)。
        base_yaw: 底盘朝向（rad）。
        mount_xyz: 云台安装位置在 base_link 系的 (x, y, z)。
        T_world_odom: (4,4) ``T_world←odom``；``None`` 表示单位阵（仿真默认）。

    Returns:
        云台原点在 world 系下的坐标，shape (3,)。
    """
    return np.asarray(frames.gimbal_origin_world(
        base_pos_odom, float(base_yaw), mount_xyz, T_world_odom),
        dtype=np.float64).reshape(3)


def r_gimbal_from_opt() -> np.ndarray:
    """标准 optical → gimbal_link 旋转由 common 单一定义。"""
    # common 的历史函数名相反；其文档/矩阵明确 R_opt_from_link 才是 G←O。
    return np.asarray(frames.R_opt_from_link(), dtype=np.float64).reshape(3, 3)
