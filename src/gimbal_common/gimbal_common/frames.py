"""坐标系、姿态与 pitch 符号的单一真源。

硬规则（DESIGN §2.3）：

1. ``PITCH_SIGN`` 是**全工程唯一**的 pitch 符号定义处，任何其它文件不得出现 pitch 符号字面量。
2. 所有"抬头为正"的 pitch 必须先经 :func:`to_protocol_pitch` 才能进入协议域；反向一律
   :func:`from_protocol_pitch`。
3. 云台**没有 roll 关节**，姿态一律按**先 yaw 后 pitch** 分解：``R = Rz(yaw) · Ry(pitch_proto)``。
   （绕 ``+y`` 的正向旋转把机体 x 轴压向 -z，即"低头为正"，与协议域一致。）

本模块只依赖 numpy，不依赖 ROS，可在无 ROS 环境下单测。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

__all__ = [
    'PITCH_SIGN',
    'set_pitch_sign',
    'get_pitch_sign',
    'to_protocol_pitch',
    'from_protocol_pitch',
    'quat_to_mat',
    'quat_from_mat',
    'compose_T',
    'invert_T',
    'rotz',
    'rotz2',
    'rpy_to_mat',
    'R_opt_from_link',
    'R_link_from_opt',
    'decompose_yaw_pitch',
    'compose_yaw_pitch',
    'yaw_of_mat',
    'freeze_T_map_odom',
    'gimbal_origin_world',
    'Pose2D',
]

# --------------------------------------------------------------------------------------
# ⚠ 修正 1：pitch 符号的单一真源
# --------------------------------------------------------------------------------------
# 协议/关节域：绕机体 +y 转，低头为正。
# atan2(z, 水平距离) 给出的是"抬头为正"，两者差一个负号。
PITCH_SIGN: float = -1.0

# 相机光学系相对相机/云台系的固定外参：camera_link -> camera_optical_frame
# rpy = (-pi/2, 0, -pi/2)，按 ROS 约定 R = Rz(yaw)·Ry(pitch)·Rx(roll)。
_OPT_RPY: Tuple[float, float, float] = (-math.pi / 2.0, 0.0, -math.pi / 2.0)

# R_gimbal←opt（把 optical 系向量变换到 gimbal_link / camera_link 系）。
# 由 rpy = (-pi/2, 0, -pi/2) 推出（TF 约定：TransformStamped 的旋转是 parent←child）：
#   R_cam←opt = Rz(-pi/2)·Ry(0)·Rx(-pi/2) = [[0,0,1],[-1,0,0],[0,-1,0]]
# virtual.yaml 中 camera_link 与 gimbal_link 外参为全 0，故 R_gimbal←opt = R_cam←opt。
_R_LINK_FROM_OPT: np.ndarray = np.array(
    [[0.0, 0.0, 1.0],
     [-1.0, 0.0, 0.0],
     [0.0, -1.0, 0.0]], dtype=np.float64)


def set_pitch_sign(v: float) -> None:
    """由 ROS 参数 ``gimbal.pitch_sign`` 初始化 pitch 符号（默认 -1.0）。

    Args:
        v: 符号，必须是 +1.0 或 -1.0。
    """
    global PITCH_SIGN
    v = float(v)
    if abs(v) != 1.0:
        raise ValueError(f'pitch_sign 只能是 +1.0 或 -1.0，收到 {v!r}')
    PITCH_SIGN = v


def get_pitch_sign() -> float:
    """返回当前生效的 pitch 符号。"""
    return PITCH_SIGN


def to_protocol_pitch(pitch_head_up: float) -> float:
    """把"抬头为正"的 pitch 转换为协议域（低头为正）。"""
    return PITCH_SIGN * float(pitch_head_up)


def from_protocol_pitch(pitch_proto: float) -> float:
    """把协议域（低头为正）的 pitch 转换回"抬头为正"。"""
    return PITCH_SIGN * float(pitch_proto)


# --------------------------------------------------------------------------------------
# 姿态基本工具
# --------------------------------------------------------------------------------------
def quat_to_mat(q_wxyz: Sequence[float]) -> np.ndarray:
    """四元数 (w,x,y,z) → 3×3 旋转矩阵。

    Args:
        q_wxyz: 长度为 4 的四元数，顺序 w,x,y,z。

    Returns:
        np.ndarray: (3,3) 旋转矩阵。输入范数为 0 时返回单位阵。
    """
    q = np.asarray(q_wxyz, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n <= 0.0 or not np.isfinite(n):
        return np.eye(3, dtype=np.float64)
    w, x, y, z = (q / n).tolist()
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def quat_from_mat(R: np.ndarray) -> np.ndarray:
    """3×3 旋转矩阵 → 四元数 (w,x,y,z)（Shepperd 方法，数值稳定）。

    Args:
        R: (3,3) 旋转矩阵。

    Returns:
        np.ndarray: (4,) 归一化四元数，顺序 w,x,y,z。
    """
    Rm = np.asarray(R, dtype=np.float64).reshape(3, 3)
    tr = float(Rm[0, 0] + Rm[1, 1] + Rm[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (Rm[2, 1] - Rm[1, 2]) / s
        y = (Rm[0, 2] - Rm[2, 0]) / s
        z = (Rm[1, 0] - Rm[0, 1]) / s
    elif Rm[0, 0] > Rm[1, 1] and Rm[0, 0] > Rm[2, 2]:
        s = math.sqrt(1.0 + Rm[0, 0] - Rm[1, 1] - Rm[2, 2]) * 2.0
        w = (Rm[2, 1] - Rm[1, 2]) / s
        x = 0.25 * s
        y = (Rm[0, 1] + Rm[1, 0]) / s
        z = (Rm[0, 2] + Rm[2, 0]) / s
    elif Rm[1, 1] > Rm[2, 2]:
        s = math.sqrt(1.0 + Rm[1, 1] - Rm[0, 0] - Rm[2, 2]) * 2.0
        w = (Rm[0, 2] - Rm[2, 0]) / s
        x = (Rm[0, 1] + Rm[1, 0]) / s
        y = 0.25 * s
        z = (Rm[1, 2] + Rm[2, 1]) / s
    else:
        s = math.sqrt(1.0 + Rm[2, 2] - Rm[0, 0] - Rm[1, 1]) * 2.0
        w = (Rm[1, 0] - Rm[0, 1]) / s
        x = (Rm[0, 2] + Rm[2, 0]) / s
        y = (Rm[1, 2] + Rm[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    n = float(np.linalg.norm(q))
    return q / n if n > 0.0 else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def rotz(theta: float) -> np.ndarray:
    """三维绕 z 旋转矩阵 (3,3)：``Rz(θ)``。"""
    c = math.cos(float(theta))
    s = math.sin(float(theta))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def rotz2(theta: float) -> np.ndarray:
    """二维绕 z 旋转矩阵 (2,2)：``[[c,-s],[s,c]]``。"""
    c = math.cos(float(theta))
    s = math.sin(float(theta))
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def rpy_to_mat(rpy: Sequence[float]) -> np.ndarray:
    """``[roll, pitch, yaw]`` → (3,3) 旋转矩阵，``R = Rz(yaw)·Ry(pitch)·Rx(roll)``。

    与 ROS ``static_transform_publisher`` 的 rpy 参数约定一致（固定轴、先 roll 后 pitch
    再 yaw）。长度不足 6 的输入按 ``[roll, pitch, yaw]`` 解释；长度 ≥ 6 时取
    ``[3:6]``（兼容 ``[x,y,z,roll,pitch,yaw]`` 位姿向量）。
    """
    v = [float(x) for x in list(rpy)]
    if len(v) >= 6:
        v = v[3:6]
    while len(v) < 3:
        v.append(0.0)
    return rotz(v[2]) @ np.array(
        [[math.cos(v[1]), 0.0, math.sin(v[1])],
         [0.0, 1.0, 0.0],
         [-math.sin(v[1]), 0.0, math.cos(v[1])]], dtype=np.float64) @ np.array(
        [[1.0, 0.0, 0.0],
         [0.0, math.cos(v[0]), -math.sin(v[0])],
         [0.0, math.sin(v[0]), math.cos(v[0])]], dtype=np.float64)


def compose_T(R: np.ndarray, t: Sequence[float]) -> np.ndarray:
    """由 (3,3) 旋转与 (3,) 平移组装 (4,4) 齐次变换。"""
    T = np.eye(4, dtype=np.float64)
    T[0:3, 0:3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[0:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_T(T: np.ndarray) -> np.ndarray:
    """求 (4,4) 齐次变换的逆（刚体专用，比通用求逆更快更稳）。"""
    Tm = np.asarray(T, dtype=np.float64).reshape(4, 4)
    R = Tm[0:3, 0:3]
    t = Tm[0:3, 3]
    out = np.eye(4, dtype=np.float64)
    out[0:3, 0:3] = R.T
    out[0:3, 3] = -R.T @ t
    return out


def R_opt_from_link() -> np.ndarray:
    """返回 ``R_gimbal←opt``：把 ``camera_optical_frame`` 系向量变换到 ``gimbal_link`` 系。

    由固定外参 rpy = (-pi/2, 0, -pi/2) 推出::

        R_gimbal←opt = [[ 0,  0,  1],
                        [-1,  0,  0],
                        [ 0, -1,  0]]

    校验（列 = optical 基向量在 gimbal 系下的表示）：

    * optical z（前）→ 列 2 = (1,0,0) = gimbal x（前）✓
    * optical x（右）→ 列 0 = (0,-1,0) = gimbal -y（右）✓
    * optical y（下）→ 列 1 = (0,0,-1) = gimbal -z（下）✓

    Returns:
        np.ndarray: (3,3) 旋转矩阵的一份拷贝。
    """
    return _R_LINK_FROM_OPT.copy()


def R_link_from_opt() -> np.ndarray:
    """返回 ``R_opt←gimbal``（:func:`R_opt_from_link` 的逆/转置）。"""
    return _R_LINK_FROM_OPT.T.copy()


def compose_yaw_pitch(yaw: float, pitch_proto: float) -> np.ndarray:
    """先 yaw 后 pitch 组装姿态矩阵：``R = Rz(yaw) · Ry(pitch_proto)``。

    Args:
        yaw: 协议域 yaw（绕世界 +z，逆时针为正）。
        pitch_proto: 协议域 pitch（绕机体 +y，低头为正）。

    Returns:
        np.ndarray: (3,3) 旋转矩阵。
    """
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    cp, sp = math.cos(float(pitch_proto)), math.sin(float(pitch_proto))
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    return Rz @ Ry


def decompose_yaw_pitch(R: np.ndarray) -> Tuple[float, float]:
    """把姿态矩阵分解为协议域 ``(yaw, pitch)``（先 yaw 后 pitch，无 roll）。

    与 :func:`compose_yaw_pitch` 互逆：``R = Rz(yaw)·Ry(pitch)``，其中
    ``R[:,0] = (cy·cp, sy·cp, -sp)``。

    Args:
        R: (3,3) 旋转矩阵（world←gimbal）。

    Returns:
        tuple: ``(yaw, pitch)``，均为协议域弧度（pitch 低头为正）。
    """
    Rm = np.asarray(R, dtype=np.float64).reshape(3, 3)
    yaw = math.atan2(float(Rm[1, 0]), float(Rm[0, 0]))
    pitch = math.atan2(-float(Rm[2, 0]), float(Rm[2, 2]))
    return yaw, pitch


def yaw_of_mat(R: np.ndarray) -> float:
    """取姿态矩阵的 yaw（等价于 ``atan2(R[1,0], R[0,0])``）。"""
    Rm = np.asarray(R, dtype=np.float64).reshape(3, 3)
    return math.atan2(float(Rm[1, 0]), float(Rm[0, 0]))


# --------------------------------------------------------------------------------------
# map 系冻结与云台原点（DESIGN §2.1 / §5.4 / §6.9）
# --------------------------------------------------------------------------------------
def freeze_T_map_odom(x: float, y: float, yaw: float) -> np.ndarray:
    """由 t0 时刻底盘位姿冻结 ``T_map←odom``（DESIGN §2.1）。

    ``T_map←odom = Translate(−P_base_odom(t0)) ∘ RotZ(−yaw_base_odom(t0))``

    Args:
        x: t0 时刻底盘在 odom 系的 x（m）。
        y: t0 时刻底盘在 odom 系的 y（m）。
        yaw: t0 时刻底盘在 odom 系的朝向（rad）。

    Returns:
        np.ndarray: (4,4) 齐次变换 ``T_map←odom``（z 分量保持 0）。
    """
    R2 = rotz2(-float(yaw))
    p = np.array([float(x), float(y)], dtype=np.float64)
    t2 = -R2 @ p
    T = np.eye(4, dtype=np.float64)
    T[0, 0] = R2[0, 0]
    T[0, 1] = R2[0, 1]
    T[1, 0] = R2[1, 0]
    T[1, 1] = R2[1, 1]
    T[0, 3] = t2[0]
    T[1, 3] = t2[1]
    return T


def gimbal_origin_world(base_pos_odom: Sequence[float],
                        base_yaw: float,
                        mount_xyz: Sequence[float],
                        T_world_odom: np.ndarray = None) -> np.ndarray:
    """计算云台原点在 world 系的坐标（DESIGN §6.9，estimation 与 control 共用）。

    ``p_base_world   = T_world←odom ∘ p_base_odom``
    ``p_gimbal_world = p_base_world + R_world←odom · Rz(yaw_base) · mount_xyz``

    Args:
        base_pos_odom: 底盘在 odom 系的 (x, y) 或 (x, y, z)。
        base_yaw: 底盘朝向（rad）。
        mount_xyz: 云台安装位置在 base_link 系的 (x, y, z)。
        T_world_odom: (4,4) ``T_world←odom``，None 表示单位阵（仿真默认）。

    Returns:
        np.ndarray: (3,) 云台原点在 world 系的坐标。
    """
    p = np.asarray(base_pos_odom, dtype=np.float64).reshape(-1)
    p3 = np.array([p[0], p[1], p[2] if p.size > 2 else 0.0], dtype=np.float64)
    if T_world_odom is None:
        T_wo = np.eye(4, dtype=np.float64)
    else:
        T_wo = np.asarray(T_world_odom, dtype=np.float64).reshape(4, 4)
    p_base_world = T_wo[0:3, 0:3] @ p3 + T_wo[0:3, 3]
    R2 = rotz2(float(base_yaw))
    m = np.asarray(mount_xyz, dtype=np.float64).reshape(3)
    off_body = np.array([R2[0, 0] * m[0] + R2[0, 1] * m[1],
                         R2[1, 0] * m[0] + R2[1, 1] * m[1],
                         m[2]], dtype=np.float64)
    return p_base_world + T_wo[0:3, 0:3] @ off_body


@dataclass
class Pose2D:
    """二维位姿纯数据类（DESIGN §5.4），不依赖 ROS。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    valid: bool = True
    t_mono: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        """返回 ``(x, y, yaw)``。"""
        return (self.x, self.y, self.yaw)
