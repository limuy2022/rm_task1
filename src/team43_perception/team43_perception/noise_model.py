"""深度相关各向异性观测噪声（DESIGN §6.6，关键）。

建模推导：marker 在图像上的宽度 ``w = s·fx / z``

* 横向（图像平面）：``σ_lat ≈ σ_px · z / fx`` → 与 ``z`` 成正比
* 深度：``∂w/∂z = −s·fx/z²`` → ``σ_depth ≈ σ_px · z² / (s·fx)`` → 与 ``z²`` 成正比

代入 ``s=0.16``、virtual.yaml 的 ``fx``、``σ_px=0.5``、``z=2``：
``σ_lat = 1.8 mm``，``σ_depth = 2.26 cm``（与官方文档「0.5px → 约 −1.9cm」量级一致）。

实现（自动推导 + 下限保护）::

    k_lat_auto   = corner_noise_px / fx
    k_depth_auto = corner_noise_px / (marker_size_m * fx)
    k_lat   = max(k_lat_auto,   k_lat_floor   = 5.0e-4)
    k_depth = max(k_depth_auto, k_depth_floor = 4.0e-3)
    σ_lat   = k_lat   * z_c
    σ_depth = k_depth * z_c²
    Σ_opt = diag(σ_lat², σ_lat², σ_depth²)      # camera_optical_frame 下

纯 numpy，不依赖 ROS，可在无 ROS 环境下单测。
"""

from __future__ import annotations

import numpy as np


class NoiseModel:
    """深度相关各向异性观测噪声模型。"""

    def __init__(self,
                 fx: float = 0.0,
                 marker_size_m: float = 0.16,
                 corner_noise_px: float = 0.5,
                 k_lat_floor: float = 5.0e-4,
                 k_depth_floor: float = 4.0e-3,
                 force_k_lat: float = 0.0,
                 force_k_depth: float = 0.0,
                 r_min_sigma: float = 0.002) -> None:
        """构造噪声模型。

        Args:
            fx: 相机焦距（像素），**必须**来自 ``virtual.yaml``（无默认值，禁止手抄）。
            marker_size_m: marker 物理边长（m）。
            corner_noise_px: 角点像素噪声标准差（保守默认 0.5，不是 0）。
            k_lat_floor: 横向系数下限。
            k_depth_floor: 深度系数下限。
            force_k_lat: >0 时直接覆盖自动推导值。
            force_k_depth: >0 时直接覆盖自动推导值。
            r_min_sigma: 协方差防奇异的最小标准差（m）。
        """
        self.marker_size_m: float = float(marker_size_m)
        self.corner_noise_px: float = float(corner_noise_px)
        self.k_lat_floor: float = float(k_lat_floor)
        self.k_depth_floor: float = float(k_depth_floor)
        self.r_min_sigma: float = float(r_min_sigma)
        self._fx: float = 0.0
        self._force_k_lat: float = float(force_k_lat) if force_k_lat and force_k_lat > 0 else 0.0
        self._force_k_depth: float = float(force_k_depth) if force_k_depth and force_k_depth > 0 else 0.0
        self._k_lat: float = 0.0
        self._k_depth: float = 0.0
        self._k_lat_auto: float = 0.0
        self._k_depth_auto: float = 0.0
        self.fx = float(fx)                 # 走 setter，顺便完成系数推导

    # ------------------------------------------------------------------ 系数
    @property
    def fx(self) -> float:
        """相机焦距（像素）；setter 会连同系数一起重算。"""
        return self._fx

    @fx.setter
    def fx(self, value: float) -> None:
        self._fx = float(value)
        k_lat_auto = self.corner_noise_px / self._fx if self._fx > 0.0 else 0.0
        k_depth_auto = (self.corner_noise_px / (self.marker_size_m * self._fx)
                        if (self._fx > 0.0 and self.marker_size_m > 0.0) else 0.0)
        self._k_lat_auto = float(k_lat_auto)
        self._k_depth_auto = float(k_depth_auto)
        self._k_lat = (self._force_k_lat if self._force_k_lat > 0.0
                       else max(k_lat_auto, self.k_lat_floor))
        self._k_depth = (self._force_k_depth if self._force_k_depth > 0.0
                         else max(k_depth_auto, self.k_depth_floor))

    @property
    def k_lat(self) -> float:
        """横向噪声系数（σ_lat = k_lat · z）。"""
        return self._k_lat

    @property
    def k_depth(self) -> float:
        """深度噪声系数（σ_depth = k_depth · z²）。"""
        return self._k_depth

    @property
    def k_lat_auto(self) -> float:
        """自动推导出的横向系数（未取下限前）。"""
        return self._k_lat_auto

    @property
    def k_depth_auto(self) -> float:
        """自动推导出的深度系数（未取下限前）。"""
        return self._k_depth_auto

    # ------------------------------------------------------------------ 标准差
    def sigma_lat(self, z_c: float) -> float:
        """横向标准差（m），与深度成正比。

        注意：DESIGN §6.6 的 ``σ_min = r_min_sigma`` 防奇异项只加在 **map 系的 R_map** 上
        （由 estimation 负责），这里保持纯 ``k_lat · z``，避免污染 optical 系协方差。
        """
        return self._k_lat * abs(float(z_c))

    def sigma_depth(self, z_c: float) -> float:
        """深度标准差（m），与深度平方成正比。"""
        return self._k_depth * float(z_c) * float(z_c)

    def build_sigma(self, z_c: float) -> tuple:
        """返回 ``(σ_lat, σ_depth)``。"""
        return (self.sigma_lat(z_c), self.sigma_depth(z_c))

    # ------------------------------------------------------------------ 协方差
    def cov_optical(self, z_c: float) -> np.ndarray:
        """``Σ_opt = diag(σ_lat², σ_lat², σ_depth²)``（camera_optical_frame 下）。

        Args:
            z_c: 光心深度（m）。

        Returns:
            np.ndarray: (3,3) 对角协方差。
        """
        sl = self.sigma_lat(z_c)
        sd = self.sigma_depth(z_c)
        # 仅做数值防奇异（避免 fx 未设置时协方差全零导致下游矩阵奇异）
        var_min = min(self.r_min_sigma, 1e-12) ** 2
        return np.diag([max(sl * sl, var_min), max(sl * sl, var_min),
                        max(sd * sd, var_min)]).astype(np.float64)

    def cov_optical_flat(self, z_c: float) -> list:
        """``cov_optical`` 的 9 元素行主序列表（便于填 msg）。"""
        return [float(v) for v in self.cov_optical(z_c).reshape(9)]
