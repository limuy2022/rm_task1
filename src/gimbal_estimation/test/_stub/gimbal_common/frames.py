# -*- coding: utf-8 -*-
"""``gimbal_common.frames`` 测试桩（仅用于无 ROS 环境的单测）。

实现严格对齐 DESIGN.md §2.3 / §3.3 的契约；**PITCH_SIGN 只在此处定义一次**。
"""

import math

import numpy as np

#: 协议域 pitch = PITCH_SIGN × atan2(z_up, 水平距离)；-1 表示"低头为正"
PITCH_SIGN = -1.0

#: R_gimbal←opt：optical(x右,y下,z前) → gimbal_link(x前,y左,z上)
_R_GIMBAL_FROM_OPT = np.array(
    [[0.0, 0.0, 1.0],
     [-1.0, 0.0, 0.0],
     [0.0, -1.0, 0.0]], dtype=np.float64,
)


def set_pitch_sign(v: float) -> None:
    """由 ROS 参数 ``gimbal.pitch_sign`` 初始化（默认 -1.0）。"""
    global PITCH_SIGN
    PITCH_SIGN = float(v)


def to_protocol_pitch(pitch_head_up: float) -> float:
    """抬头为正 → 协议域（低头为正）。"""
    return PITCH_SIGN * float(pitch_head_up)


def from_protocol_pitch(pitch_proto: float) -> float:
    """协议域（低头为正）→ 抬头为正。"""
    return PITCH_SIGN * float(pitch_proto)


def quat_to_mat(q_wxyz) -> np.ndarray:
    """四元数 (w,x,y,z) → 3x3 旋转矩阵（列 = 机体系基向量在父系下的表示）。"""
    w, x, y, z = (float(v) for v in np.asarray(q_wxyz, dtype=np.float64).reshape(4))
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n <= 0.0:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
         [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
         [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]],
        dtype=np.float64,
    )


def decompose_yaw_pitch(R) -> tuple:
    """先 yaw 后 pitch 分解（无 roll）；pitch 返回**协议域**（低头为正）。"""
    M = np.asarray(R, dtype=np.float64).reshape(3, 3)
    f = M[:, 0]
    yaw = math.atan2(float(f[1]), float(f[0]))
    pitch = to_protocol_pitch(math.atan2(float(f[2]), math.hypot(float(f[0]), float(f[1]))))
    return float(yaw), float(pitch)


def R_opt_from_link() -> np.ndarray:
    """``R_gimbal←opt``（gimbal_link ← camera_optical_frame）。"""
    return _R_GIMBAL_FROM_OPT.copy()


def rotz2(theta: float) -> np.ndarray:
    """2D 绕 z 旋转 ``[[cos, -sin], [sin, cos]]``。"""
    c, s = math.cos(float(theta)), math.sin(float(theta))
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def compose_T(R, t) -> np.ndarray:
    """由 3x3 旋转 + 3 平移组装 4x4 齐次矩阵。"""
    T = np.eye(4, dtype=np.float64)
    T[0:3, 0:3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[0:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T
