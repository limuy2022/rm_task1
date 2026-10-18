# -*- coding: utf-8 -*-
"""观测换算：``camera_optical_frame → gimbal_link → world → map``（DESIGN.md §6.6）。

纯 Python + numpy 实现，**不依赖 ROS / rclpy**，可在无 ROS 环境下直接 import 并单测。
唯一依赖的跨包模块是 ``gimbal_common.frames``（提供四元数→矩阵、``R_gimbal←opt``、
``rotz2`` 以及 pitch 符号单点函数）。

换算链（严格照抄 §6.6）::

    p_world = p_gimbal_world + R(q_world←gimbal) · R_gimbal←opt · p_opt
    p_map   = T_map←world · p_world
    R_w     = R(q_world←gimbal) · R_gimbal←opt
    Σ_w     = R_w · Σ_opt · R_wᵀ
    θ       = yaw(T_map←world)
    R_map   = RotZ(θ) · Σ_w[0:2,0:2] · RotZ(θ)ᵀ + diag(σ_min², σ_min²)

其中 ``Σ_opt = diag(σ_lat², σ_lat², σ_depth²)`` 由 ``gimbal_perception`` 的噪声模型给出
（通过 ``MarkerObservation.cov_optical``，3x3 行主序）。

⚠ 本文件**不得出现任何 pitch 符号字面量**；pitch 相关换算一律走
``gimbal_common.frames``（全工程唯一真源）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from gimbal_common import frames

#: 观测噪声对角下限（防奇异），DESIGN §6.13 ``perception.r_min_sigma``
DEFAULT_R_MIN_SIGMA = 0.002

#: 当 perception 未提供 cov_optical 时的退化协方差（单位 m）
FALLBACK_K_LAT = 0.0     # >0 时按 σ_lat = k_lat · z 构造
FALLBACK_K_DEPTH = 0.0   # >0 时按 σ_depth = k_depth · z² 构造


def _r_gimbal_from_opt() -> np.ndarray:
    """取得 ``R_gimbal←opt``（把 optical 系向量转到 gimbal_link 系）。

    取自单一真源 ``gimbal_common.frames.R_opt_from_link()``（历史命名方向相反：
    该函数实际返回 R_gimbal←opt，见其自身 docstring 的轴向量校验）。

    Returns:
        shape (3, 3) 的旋转矩阵 ``R_gimbal←opt``。
    """
    return np.asarray(frames.R_opt_from_link(), dtype=np.float64).reshape(3, 3)


def _rotz2(theta: float) -> np.ndarray:
    """2D 绕 z 旋转矩阵 ``[[cos, -sin], [sin, cos]]``。"""
    return np.asarray(frames.rotz2(theta), dtype=np.float64).reshape(2, 2)


def yaw_of_T(T: Sequence[Sequence[float]]) -> float:
    """从 4x4 变换矩阵取出绕 z 的偏航角（弧度）。

    Args:
        T: 4x4 齐次变换矩阵。

    Returns:
        ``atan2(T[1,0], T[0,0])``；输入非有限时报错。
    """
    M = np.asarray(T, dtype=np.float64).reshape(4, 4)
    th = math.atan2(float(M[1, 0]), float(M[0, 0]))
    if not math.isfinite(th):
        raise ValueError('transform yaw must be finite')
    return th


def as_T(transform_like) -> np.ndarray:
    """Require the caller's actual finite rigid transform; never substitute identity."""
    M = np.asarray(transform_like, dtype=np.float64).reshape(4, 4)
    R = M[:3, :3]
    if (not np.all(np.isfinite(M)) or not np.allclose(M[3], [0, 0, 0, 1])
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(R), 1., atol=1e-6)):
        raise ValueError('map/world transform must be finite and rigid')
    return M


@dataclass
class Observation:
    """一次视觉观测换算后的结果。

    Attributes:
        valid: 是否可用于 KF 更新。
        reason: 无效原因（有效时为 ``''``）。
        pos_map: 目标在 map 系下的位置，shape (3,)。
        pos_world: 目标在 world 系下的位置，shape (3,)（控制律/调试用）。
        R_map: map 系 XY 的观测噪声 2x2。
        depth: optical 系深度（= ``p_opt[2]``），米。
        theta_map_world: ``T_map←world`` 的偏航角，弧度。
    """

    valid: bool = False
    reason: str = ''
    pos_map: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    pos_world: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    R_map: np.ndarray = field(default_factory=lambda: np.zeros((2, 2), dtype=np.float64))
    depth: float = 0.0
    theta_map_world: float = 0.0


class ObservationBuilder:
    """把 ``camera_optical_frame`` 下的 PnP 结果换算成 map 系观测 + 噪声。"""

    def __init__(
        self,
        r_min_sigma: float = DEFAULT_R_MIN_SIGMA,
        fallback_k_lat: float = FALLBACK_K_LAT,
        fallback_k_depth: float = FALLBACK_K_DEPTH,
    ) -> None:
        self.r_min_sigma = float(r_min_sigma)
        self.fallback_k_lat = float(fallback_k_lat)
        self.fallback_k_depth = float(fallback_k_depth)
        self._r_gopt = _r_gimbal_from_opt()
        self._r_gopt_inv = self._r_gopt.T

    # ------------------------------------------------------------------ 工具
    @property
    def R_gimbal_from_opt(self) -> np.ndarray:
        """``R_gimbal←opt``（只读副本）。"""
        return self._r_gopt.copy()

    def rot_opt_to_world(self, quat_wxyz: Sequence[float]) -> np.ndarray:
        """``R_world←opt = R_world←gimbal · R_gimbal←opt``。

        Args:
            quat_wxyz: 四元数 ``(w, x, y, z)``，world → gimbal_link（``GimbalState.quat``）。

        Returns:
            shape (3, 3) 的旋转矩阵；无效四元数报错，不伪造单位姿态。
        """
        q = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
        n = float(np.linalg.norm(q))
        if not np.all(np.isfinite(q)) or n < 0.5:
            raise ValueError('orientation quaternion must be finite with norm >= 0.5')
        Rwg = np.asarray(frames.quat_to_mat(q / n), dtype=np.float64).reshape(3, 3)
        return Rwg @ self._r_gopt

    @staticmethod
    def cov_from_flat(cov_optical: Optional[Sequence[float]]) -> Optional[np.ndarray]:
        """把 ``MarkerObservation.cov_optical``（9 元素行主序）还原成 3x3。

        Returns:
            shape (3, 3)；输入缺失或含非有限值时返回 ``None``。
        """
        if cov_optical is None:
            return None
        C = np.asarray(cov_optical, dtype=np.float64)
        if C.size < 9:
            return None
        C = C.reshape(3, 3)
        if not np.all(np.isfinite(C)):
            return None
        return C

    def _fallback_cov(self, depth: float) -> np.ndarray:
        """perception 未给出协方差时的退化构造。"""
        if self.fallback_k_lat > 0.0 or self.fallback_k_depth > 0.0:
            z = max(float(depth), 0.0)
            s_lat = self.fallback_k_lat * z
            s_depth = self.fallback_k_depth * z * z
        else:
            s_lat = self.r_min_sigma
            s_depth = self.r_min_sigma
        s_lat = max(s_lat, self.r_min_sigma)
        s_depth = max(s_depth, self.r_min_sigma)
        return np.diag([s_lat * s_lat, s_lat * s_lat, s_depth * s_depth])

    def build_from_transform(self, tvec_opt, cov_opt, T_map_camera) -> Observation:
        """Use the complete TF map<-camera_optical at the image stamp.

        No identity fallback for absent/invalid calibration. Translation includes
        the optical centre and all rotating lever arms supplied by the TF chain.
        """
        bad = Observation(valid=False, reason='invalid optical transform')
        try:
            p = np.asarray(tvec_opt, float).reshape(3)
            T = np.asarray(T_map_camera, float).reshape(4, 4)
            R = T[:3, :3]
            if (not np.all(np.isfinite(p)) or p[2] <= 0 or not np.all(np.isfinite(T))
                    or not np.allclose(T[3], [0,0,0,1], atol=1e-7)
                    or not np.allclose(R.T@R, np.eye(3), atol=1e-6)
                    or not np.isclose(np.linalg.det(R), 1., atol=1e-6)):
                return bad
            C = self.cov_from_flat(cov_opt)
            if C is None: C = self._fallback_cov(float(p[2]))
            C = (C+C.T)/2
            if np.linalg.eigvalsh(C).min() < -1e-10:
                bad.reason='negative observation covariance'; return bad
            result=R@p+T[:3,3]
            covariance=(R@C@R.T)[:2,:2]+np.eye(2)*self.r_min_sigma**2
            return Observation(valid=True, reason='', pos_map=result,
                pos_world=result.copy(), R_map=covariance, depth=float(p[2]))
        except (TypeError, ValueError, np.linalg.LinAlgError):
            # Invalid external coordinates/covariance are rejected, never replaced by a pose.
            return bad

    # ------------------------------------------------------------------ 主入口
    def build(
        self,
        tvec_opt: Sequence[float],
        cov_opt: Optional[Sequence[float]],
        quat: Sequence[float],
        gimbal_origin_world: Sequence[float],
        T_map_world,
    ) -> Observation:
        """执行 §6.6 的完整换算。

        Args:
            tvec_opt: 目标在 ``camera_optical_frame`` 下的坐标 ``[x, y, z]``（米）。
            cov_opt: ``Σ_opt``：3x3 或 9 元素行主序；``None`` 时走退化构造。
            quat: 云台姿态四元数 ``(w, x, y, z)``，world → gimbal_link。
            gimbal_origin_world: 云台原点在 world 系下的坐标，shape (3,)。
            T_map_world: 4x4 的 ``T_map←world``。

        Returns:
            :class:`Observation`。``valid=False`` 时调用方应丢弃该观测。
        """
        bad = Observation(valid=False, reason='invalid input')

        p_opt = np.asarray(tvec_opt, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(p_opt)):
            bad.reason = 'tvec_opt 非有限'
            return bad

        origin = np.zeros(3, dtype=np.float64)
        if gimbal_origin_world is not None:
            o = np.asarray(gimbal_origin_world, dtype=np.float64).reshape(3)
            if not np.all(np.isfinite(o)):
                bad.reason = 'gimbal_origin_world 非有限'
                return bad
            origin = o

        T = as_T(T_map_world)
        R_mw = T[0:3, 0:3]
        t_mw = T[0:3, 3]

        R_w = self.rot_opt_to_world(quat)          # R_world←opt
        p_world = origin + R_w @ p_opt
        p_map = R_mw @ p_world + t_mw
        if not np.all(np.isfinite(p_map)):
            bad.reason = 'p_map 非有限'
            return bad

        depth = float(p_opt[2])

        C_opt = self.cov_from_flat(cov_opt)
        if C_opt is None:
            C_opt = self._fallback_cov(depth)

        # Σ_w = R_w Σ_opt R_wᵀ → 取 XY 分块 → 旋到 map
        sigma_w = R_w @ C_opt @ R_w.T
        M = sigma_w[0:2, 0:2]
        theta = yaw_of_T(T)
        Rot = _rotz2(theta)
        R_map = Rot @ M @ Rot.T
        if not np.all(np.isfinite(R_map)):
            bad.reason = 'observation covariance 非有限'
            return bad
        R_map = 0.5 * (R_map + R_map.T)            # 强制对称

        # 防奇异：加 σ_min² 的对角
        smin2 = float(self.r_min_sigma) ** 2
        R_map = R_map + np.diag([smin2, smin2])

        # 再把深度方向带来的各向异性"抬平"到不低于 σ_min
        eig_min = float(np.linalg.eigvalsh(R_map).min())
        if not math.isfinite(eig_min) or eig_min <= 0.0:
            R_map = R_map + np.diag([smin2, smin2])

        return Observation(
            valid=True,
            reason='',
            pos_map=p_map,
            pos_world=p_world,
            R_map=R_map,
            depth=depth,
            theta_map_world=theta,
        )


def map_from_world(p_world: Sequence[float], T_map_world) -> np.ndarray:
    """``p_map = T_map←world · p_world``（3D）。"""
    T = as_T(T_map_world)
    p = np.asarray(p_world, dtype=np.float64).reshape(3)
    return T[0:3, 0:3] @ p + T[0:3, 3]


def world_from_map(p_map: Sequence[float], T_map_world) -> np.ndarray:
    """``p_world = (T_map←world)⁻¹ · p_map``（3D）。"""
    T = as_T(T_map_world)
    p = np.asarray(p_map, dtype=np.float64).reshape(3)
    R = T[0:3, 0:3]
    return R.T @ (p - T[0:3, 3])


def world_vec_from_map_vec(v_map_xy: Sequence[float], T_map_world) -> np.ndarray:
    """把 map 系的 **XY 平面向量**（速度/加速度）反旋到 world 系，返回 3D。

    ``v_world_xy = RotZ(−θ) · v_map_xy``，z 分量取 0（KF 只有 2D 速度）。
    """
    T = as_T(T_map_world)
    theta = yaw_of_T(T)
    Rot = _rotz2(theta)
    v_xy = np.asarray(v_map_xy, dtype=np.float64).reshape(2)
    v_world_xy = Rot.T @ v_xy                       # RotZ(−θ) = RotZ(θ)ᵀ
    return np.array([v_world_xy[0], v_world_xy[1], 0.0], dtype=np.float64)
