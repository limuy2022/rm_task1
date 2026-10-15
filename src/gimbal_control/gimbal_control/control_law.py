# -*- coding: utf-8 -*-
"""云台控制律 —— DESIGN.md §6.8 的参考实现（纯 Python + numpy，不依赖 ROS）。

六步（**顺序不可调换**）：

1. 视线与光轴：``r = p_t − p_gimbal_world``；``f = R[:,0]``；``y_body = R[:,1]``
2. 目标/当前姿态分解（先 yaw 后 pitch，无 roll，**pitch 一律走
   ``frames.to_protocol_pitch``**）
3. 误差：``e_yaw = wrap_pi(yaw_t − yaw_c)``；``e_pitch = pitch_t − pitch_c``
4. 前馈：``ω_vec = cross(r, v_t)/(‖r‖²+1e-9)``；
   ``ω_ff_yaw = ω_vec·ẑ``；``ω_ff_pitch = ω_vec·y_body``
5. 反馈（PD + IMU 阻尼）：
   ``ω_yaw   = ff_gain·ω_ff_yaw   + kp_yaw·e_yaw   − kd_yaw·ω_imu_yaw``
   ``ω_pitch = ff_gain·ω_ff_pitch + kp_pitch·e_pitch − kd_pitch·ω_imu_pitch``
   其中 ``ω_imu_yaw = gyro.z / max(cos(pitch_c), 0.1)``；``ω_imu_pitch = gyro.y``
6. 限幅与保护：slew → 幅值 clamp → 软限位衰减 → 有限性守卫

⚠ pitch 符号（主理人拍板 3 / DESIGN 修正 1）：本文件**不得出现任何 pitch 符号字面量**，
所有"抬头为正 → 低头为正"的换算必须调用 ``gimbal_common.frames.to_protocol_pitch``。
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional, Sequence, Tuple

import numpy as np

from gimbal_common import frames
from gimbal_common.math_utils import clamp, wrap_pi

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


def feedforward(r: np.ndarray, vel_world: Sequence[float],
                y_body: Sequence[float]) -> Tuple[float, float]:
    """Step 4：视线角速度前馈。

    ``ω_vec = cross(r, v_t) / (‖r‖² + 1e-9)``
    ``ω_ff_yaw = ω_vec·[0,0,1]``；``ω_ff_pitch = ω_vec·y_body``

    自检（DESIGN §6.8）：``r=(d,0,0)``、``v=(0,0,vz)``、``y_body=(0,1,0)``
    → ``ω_vec=(0, −vz/d, 0)`` → ``ω_ff_pitch = −vz/d < 0``（目标上升应抬头，
    协议域 pitch 减小，负号正确）。

    Returns:
        ``(ff_yaw, ff_pitch)``，均为有限值。
    """
    rv = np.asarray(r, dtype=np.float64).reshape(3)
    vv = np.asarray(vel_world, dtype=np.float64).reshape(3)
    yv = np.asarray(y_body, dtype=np.float64).reshape(3)
    if not (np.all(np.isfinite(rv)) and np.all(np.isfinite(vv)) and np.all(np.isfinite(yv))):
        return 0.0, 0.0
    denom = float(rv @ rv) + EPS_R2
    omega_vec = np.cross(rv, vv) / denom
    ff_yaw = float(omega_vec[2])                 # ω_vec · [0,0,1]
    ff_pitch = float(omega_vec @ yv)             # 绕机体 +y → 协议域（低头为正）
    if not math.isfinite(ff_yaw):
        ff_yaw = 0.0
    if not math.isfinite(ff_pitch):
        ff_pitch = 0.0
    return ff_yaw, ff_pitch


def imu_feedback(gyro: Sequence[float], pitch_c: float) -> Tuple[float, float]:
    """Step 5：从机体系陀螺反解**关节**角速度（协议域）。

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
    ) -> ControlOutput:
        """执行一次完整的六步控制律（含限幅与保护）。

        Args:
            target_world: 目标在 **world** 系下的位置，shape (3,)。
            vel_world: 目标在 **world** 系下的速度，shape (3,)（map 系速度需先反旋）。
            quat: 云台姿态四元数 ``(w, x, y, z)``，world → gimbal_link。
            gyro: 机体系陀螺 ``(gx, gy, gz)``，rad/s。
            dt: 距上次指令的间隔（秒，**单调时钟**），用于 slew 限幅。
            gimbal_origin_world: 云台原点（world 系）；``None`` 视为原点。
            mode: 工作模式标签，原样填入输出。

        Returns:
            :class:`ControlOutput`。
        """
        # ---- Step 1 -------------------------------------------------------
        origin = (np.zeros(3, dtype=np.float64) if gimbal_origin_world is None
                  else np.asarray(gimbal_origin_world, dtype=np.float64).reshape(3))
        r = line_of_sight(target_world, origin)

        q = np.asarray(quat, dtype=np.float64).reshape(4)
        if not np.all(np.isfinite(q)) or float(np.linalg.norm(q)) < 0.5:
            return ControlOutput(0.0, 0.0, False, mode)
        R = np.asarray(frames.quat_to_mat(q / float(np.linalg.norm(q))),
                       dtype=np.float64).reshape(3, 3)
        f, y_body = attitude_axes(R)

        gv = np.asarray(gyro, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(gv)):
            gv = np.zeros(3, dtype=np.float64)

        # ---- Step 2 -------------------------------------------------------
        yaw_t, pitch_t = target_angles(r)
        yaw_c, pitch_c = current_angles(f)

        # ---- Step 3 -------------------------------------------------------
        e_yaw = float(wrap_pi(yaw_t - yaw_c))
        e_pitch = float(pitch_t - pitch_c)
        e_yaw = finite_guard(e_yaw)
        e_pitch = finite_guard(e_pitch)

        # ---- Step 4 -------------------------------------------------------
        ff_yaw, ff_pitch = feedforward(r, vel_world if vel_world is not None
                                       else np.zeros(3), y_body)

        # ---- Step 5 -------------------------------------------------------
        w_imu_yaw, w_imu_pitch = imu_feedback(gv, pitch_c)
        w_yaw = self.ff_gain * ff_yaw + self.kp_yaw * e_yaw - self.kd_yaw * w_imu_yaw
        w_pitch = self.ff_gain * ff_pitch + self.kp_pitch * e_pitch - self.kd_pitch * w_imu_pitch

        # ---- Step 6 -------------------------------------------------------
        w_yaw, w_pitch = self.limit(w_yaw, w_pitch, yaw_c, pitch_c, dt)

        return ControlOutput(
            yaw_omega=w_yaw,
            pitch_omega=w_pitch,
            valid=True,
            mode=mode,
            e_yaw=e_yaw,
            e_pitch=e_pitch,
            ff_yaw=ff_yaw,
            ff_pitch=ff_pitch,
            yaw_c=yaw_c,
            pitch_c=pitch_c,
        )

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
        d = float(dt) if math.isfinite(float(dt)) else 0.0
        d = max(d, 0.0)

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


# --------------------------------------------------------------------------
# 云台原点（DESIGN §6.9）—— 真源在 gimbal_common.frames，此处只做薄封装
# --------------------------------------------------------------------------
def gimbal_origin_world(base_pos_odom: Sequence[float],
                        base_yaw: float,
                        mount_xyz: Sequence[float],
                        T_world_odom: Optional[Sequence[Sequence[float]]] = None) -> np.ndarray:
    """云台原点在 world 系下的坐标（DESIGN §6.9，estimation 与 control 共用）。

    ``p_base_world   = T_world←odom ∘ p_base_odom``
    ``p_gimbal_world = p_base_world + R_world←odom · Rz(yaw_base) · mount_xyz``

    优先委托 ``gimbal_common.frames.gimbal_origin_world``（单一真源）；
    该函数不存在时使用本地等价实现。

    Args:
        base_pos_odom: 底盘在 odom 系的 (x, y) 或 (x, y, z)。
        base_yaw: 底盘朝向（rad）。
        mount_xyz: 云台安装位置在 base_link 系的 (x, y, z)。
        T_world_odom: (4,4) ``T_world←odom``；``None`` 表示单位阵（仿真默认）。

    Returns:
        云台原点在 world 系下的坐标，shape (3,)。
    """
    fn = getattr(frames, 'gimbal_origin_world', None)
    if callable(fn):
        out = np.asarray(fn(base_pos_odom, float(base_yaw), mount_xyz, T_world_odom),
                         dtype=np.float64).reshape(3)
        return out if np.all(np.isfinite(out)) else np.zeros(3, dtype=np.float64)

    p = np.asarray(base_pos_odom, dtype=np.float64).reshape(-1)
    p3 = np.array([p[0], p[1], p[2] if p.size > 2 else 0.0], dtype=np.float64)
    T_wo = (np.eye(4, dtype=np.float64) if T_world_odom is None
            else np.asarray(T_world_odom, dtype=np.float64).reshape(4, 4))
    R2 = np.asarray(frames.rotz2(float(base_yaw)), dtype=np.float64).reshape(2, 2)
    m = np.asarray(mount_xyz, dtype=np.float64).reshape(3)
    off_body = np.array([R2[0, 0] * m[0] + R2[0, 1] * m[1],
                         R2[1, 0] * m[0] + R2[1, 1] * m[1],
                         m[2]], dtype=np.float64)
    out = T_wo[0:3, 0:3] @ p3 + T_wo[0:3, 3] + T_wo[0:3, 0:3] @ off_body
    return out if np.all(np.isfinite(out)) else np.zeros(3, dtype=np.float64)


def r_gimbal_from_opt() -> np.ndarray:
    """取得 ``R_gimbal←opt``（optical 系 → gimbal_link 系）。

    与 estimation 侧同一套**物理方向自检**逻辑：optical 的 z（向前）必须映射到
    gimbal_link 的 x（向前），即 ``R[:, 2] == (1, 0, 0)``；否则转置纠正。
    这样无论 ``frames.R_link_from_opt`` / ``frames.R_opt_from_link`` 的命名如何，
    都能拿到方向正确的矩阵。

    Returns:
        shape (3, 3) 的旋转矩阵 ``R_gimbal←opt``。
    """
    forward = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    candidates = []
    fn_link_from_opt = getattr(frames, 'R_link_from_opt', None)
    if callable(fn_link_from_opt):
        candidates.append(np.asarray(fn_link_from_opt(), dtype=np.float64).reshape(3, 3))
    fn_opt_from_link = getattr(frames, 'R_opt_from_link', None)
    if callable(fn_opt_from_link):
        candidates.append(np.asarray(fn_opt_from_link(), dtype=np.float64).reshape(3, 3).T)
    for R in candidates:
        if np.allclose(R[:, 2], forward, atol=1e-6) and np.allclose(R @ R.T, np.eye(3), atol=1e-6):
            return R
    return np.array([[0.0, 0.0, 1.0],
                     [-1.0, 0.0, 0.0],
                     [0.0, -1.0, 0.0]], dtype=np.float64)
