"""ArUco 角点 → PnP 位姿（DESIGN §6.5）。

物点顺序 **TL → TR → BR → BL**（marker 局部坐标系：x 右 / **y 上** / z 出平面，即 ArUco 标准约定；注意它与 ``camera_optical_frame`` 的「y 下」**相反**，翻转的是 rvec 而非 tvec）::

    s = marker_size_m (默认 0.16)
    OBJ = [[-s/2, +s/2, 0],   # 0 左上 (TL)
           [+s/2, +s/2, 0],   # 1 右上 (TR)
           [+s/2, -s/2, 0],   # 2 右下 (BR)
           [-s/2, -s/2, 0]]   # 3 左下 (BL)

* 主解算器 ``cv2.SOLVEPNP_IPPE_SQUARE``（要求上述点序），比较其双解并精修；
  近正视退化、异常结果或旧API不可用时回退 ``SOLVEPNP_ITERATIVE``。
* 点序是码自身的规范角点顺序；码旋转后不得重新按图像左上角排序。
* 重投影 RMS > ``reproj_max_px``(2.0) → 拒绝（视为本帧无观测，不报错）。
* 深度 ``z_c ∈ [depth_min, depth_max]`` = [0.05, 30.0]，否则拒绝。

纯 numpy + cv2，不依赖 ROS，可在无 ROS 环境下单测。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

import cv2


def marker_object_points(marker_size_m: float) -> np.ndarray:
    """生成 4 个物点（TL→TR→BR→BL），形状 (4,3)。"""
    s = float(marker_size_m)
    if not math.isfinite(s) or s <= 0.0:
        raise ValueError('marker_size_m must be finite and positive')
    return np.array([[-s / 2.0, +s / 2.0, 0.0],
                     [+s / 2.0, +s / 2.0, 0.0],
                     [+s / 2.0, -s / 2.0, 0.0],
                     [-s / 2.0, -s / 2.0, 0.0]], dtype=np.float64)


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """固定轴 RPY → (3,3) 旋转矩阵，``R = Rz(yaw)·Ry(pitch)·Rx(roll)``。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return Rz @ Ry @ Rx


@dataclass
class PnpResult:
    """一次 PnP 求解的结果。"""

    ok: bool = False
    reason: str = ''
    rvec: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    tvec_opt: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    reproj_err_px: float = float('inf')

    @property
    def depth(self) -> float:
        """光心深度 ``z_c``（camera_optical_frame 下的 z）。"""
        return float(self.tvec_opt[2])


class PnpSolver:
    """带校验的 PnP 解算器（状态中保存上一帧解，供 ITERATIVE 作初值）。"""

    def __init__(self,
                 fx: float = 0.0,
                 fy: float = 0.0,
                 cx: float = 0.0,
                 cy: float = 0.0,
                 marker_size_m: float = 0.16,
                 dist: Optional[Sequence[float]] = None,
                 reproj_max_px: float = 2.0,
                 depth_min: float = 0.05,
                 depth_max: float = 30.0) -> None:
        """构造解算器。

        内参不设默认值：**必须**来自 ``virtual.yaml``（唯一数据源），
        推荐用 :meth:`from_camera_config` 构造，避免二次手抄内参。

        Args:
            fx/fy/cx/cy: 相机内参（来自 virtual.yaml）。
            marker_size_m: marker 物理边长（m）。
            dist: 5 参数畸变；None 表示全 0。
            reproj_max_px: 重投影 RMS 上限（px）。
            depth_min/depth_max: 合法深度区间（m）。
        """
        self.marker_size_m: float = float(marker_size_m)
        self.reproj_max_px: float = float(reproj_max_px)
        self.depth_min: float = float(depth_min)
        self.depth_max: float = float(depth_max)
        if not (math.isfinite(self.reproj_max_px) and self.reproj_max_px > 0.0
                and 0.0 < self.depth_min < self.depth_max
                and math.isfinite(self.depth_max)):
            raise ValueError('invalid reprojection/depth limits')
        self._obj: np.ndarray = marker_object_points(self.marker_size_m)
        self._K: np.ndarray = np.eye(3, dtype=np.float64)
        self._D: np.ndarray = np.zeros(5, dtype=np.float64)
        self._intrinsics_ready = False
        if fx > 0.0 and fy > 0.0:
            self.set_intrinsics(fx, fy, cx, cy, dist)
        self._rvec_prev: Optional[np.ndarray] = None
        self._tvec_prev: Optional[np.ndarray] = None

    @classmethod
    def from_camera_config(cls, cam_cfg: dict, **kwargs) -> 'PnpSolver':
        """由 ``virtual.yaml`` 的 ``camera`` 段构造解算器（**唯一数据源**）。

        Args:
            cam_cfg: ``{'fx':..., 'fy':..., 'cx':..., 'cy':..., 'distortion': [...]}``。
            **kwargs: 透传给构造函数（``marker_size_m`` / ``reproj_max_px`` 等）。

        Returns:
            PnpSolver: 已装载内参的解算器。

        Raises:
            ValueError: 内参缺失或非法。
        """
        fx = float(cam_cfg.get('fx', 0.0))
        fy = float(cam_cfg.get('fy', 0.0))
        cx = float(cam_cfg.get('cx', 0.0))
        cy = float(cam_cfg.get('cy', 0.0))
        if not (fx > 0.0 and fy > 0.0):
            raise ValueError(f'virtual.yaml 的 camera.fx/fy 非法：fx={fx} fy={fy}')
        return cls(fx=fx, fy=fy, cx=cx, cy=cy,
                   dist=cam_cfg.get('distortion', None), **kwargs)

    # ------------------------------------------------------------------ 配置
    def set_intrinsics(self, fx: float, fy: float, cx: float, cy: float,
                       dist: Optional[Sequence[float]] = None) -> None:
        """更新内参与畸变（**必须**来自 virtual.yaml，禁止手抄）。"""
        if not all(math.isfinite(float(v)) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
            raise ValueError('camera intrinsics must be finite with positive fx/fy')
        d = np.zeros(5, dtype=np.float64) if dist is None else np.asarray(dist, dtype=np.float64).reshape(-1)
        if d.size == 0:
            d = np.zeros(5, dtype=np.float64)
        if d.size not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(d)):
            raise ValueError('invalid OpenCV distortion coefficients')
        self._K = np.array([[float(fx), 0.0, float(cx)],
                            [0.0, float(fy), float(cy)],
                            [0.0, 0.0, 1.0]], dtype=np.float64)
        self._D = d.copy()
        self._intrinsics_ready = True
        self.reset_guess()

    @property
    def K(self) -> np.ndarray:
        """内参矩阵 (3,3)。"""
        return self._K.copy()

    @property
    def D(self) -> np.ndarray:
        """OpenCV畸变系数；保留配置中全部受支持的分量。"""
        return self._D.copy()

    @property
    def fx(self) -> float:
        """fx。"""
        return float(self._K[0, 0])

    def reset_guess(self) -> None:
        """清空上一帧初值（目标丢失后重新捕获时调用）。"""
        self._rvec_prev = None
        self._tvec_prev = None

    # ------------------------------------------------------------------ 求解
    def reprojection_error(self, rvec: np.ndarray, tvec: np.ndarray,
                           corners_uv: np.ndarray) -> float:
        """重投影 RMS 误差（像素）。"""
        proj, _ = cv2.projectPoints(self._obj, rvec, tvec, self._K, self._D)
        proj = proj.reshape(-1, 2)
        obs = np.asarray(corners_uv, dtype=np.float64).reshape(-1, 2)
        diff = proj - obs
        return float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))

    def solve(self, corners_uv: Sequence[Sequence[float]]) -> PnpResult:
        """解算 marker 在 ``camera_optical_frame`` 下的位姿。

        Args:
            corners_uv: (4,2) 像素角点，顺序 TL→TR→BR→BL。

        Returns:
            PnpResult: ``ok=False`` 时 ``reason`` 说明拒绝原因（均属正常路径，不报错）。
        """
        try:
            obs = np.asarray(corners_uv, dtype=np.float64).reshape(-1, 2)
        except (TypeError, ValueError):
            # 外部角点形状非法是无观测，不能送入 OpenCV。
            return PnpResult(ok=False, reason='corner_shape')
        if obs.shape[0] != 4:
            return PnpResult(ok=False, reason='corner_count')
        if not np.all(np.isfinite(obs)):
            return PnpResult(ok=False, reason='nonfinite_corners')
        if not self._intrinsics_ready:
            return PnpResult(ok=False, reason='intrinsics_missing')
        # Four finite values alone are insufficient: reject repeated, crossed,
        # concave or almost collinear corners before asking a planar solver.
        with np.errstate(over='ignore'):
            contour = np.asarray(obs, dtype=np.float32).reshape(4, 1, 2)
        if (not np.all(np.isfinite(contour)) or not cv2.isContourConvex(contour)
                or abs(cv2.contourArea(contour)) < 1.0):
            return PnpResult(ok=False, reason='degenerate_corners')

        candidates = []
        try:
            result = cv2.solvePnPGeneric(self._obj, obs, self._K, self._D,
                                         flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if result[0]:
                candidates.extend(zip(result[1], result[2]))
        except cv2.error:
            # IPPE 对部分平面观测会拒绝求解；保留实际 ITERATIVE 候选比较。
            pass

        evaluated = []
        for r, t in candidates:
            result = self._evaluate(r, t, obs)
            if result is not None:
                evaluated.append(result)
            # LM fixes the near-fronto-parallel IPPE degeneracy present in older
            # OpenCV builds. Keep the original candidate if refinement worsens it.
            try:
                rr, tt = cv2.solvePnPRefineLM(self._obj, obs, self._K, self._D,
                                             np.asarray(r).copy(), np.asarray(t).copy())
                refined = self._evaluate(rr, tt, obs)
                if refined is not None:
                    evaluated.append(refined)
            except cv2.error:
                # 观测可能无法完成 LM 精化，仍可评估本次未精化的候选。
                pass

        # Compare a fresh planar ITERATIVE initialization even if an IPPE branch
        # is below the rejection threshold: fronto-parallel IPPE failures can
        # have a plausible subpixel error while still choosing the wrong pose.
        # Four points make this small fallback cheaper than accepting biased data.
        try:
            ok, r, t = cv2.solvePnP(self._obj, obs, self._K, self._D,
                                   flags=cv2.SOLVEPNP_ITERATIVE)
            if ok:
                result = self._evaluate(r, t, obs)
                if result is not None:
                    evaluated.append(result)
        except cv2.error:
            # 某帧退化观测可令 ITERATIVE 求解失败；仅保留其他有效候选。
            pass
        if not evaluated:
            return PnpResult(ok=False, reason='solve_failed')
        best = min(evaluated, key=lambda x: x.reproj_err_px)
        rvec, tvec, err_px = best.rvec, best.tvec_opt, best.reproj_err_px
        if err_px > self.reproj_max_px:
            return PnpResult(ok=False, reason=f'reproj_{err_px:.2f}px',
                             rvec=rvec, tvec_opt=tvec, reproj_err_px=err_px)

        z_c = float(tvec[2])
        if not (self.depth_min <= z_c <= self.depth_max):
            return PnpResult(ok=False, reason=f'depth_{z_c:.3f}m',
                             rvec=rvec, tvec_opt=tvec, reproj_err_px=err_px)

        self._rvec_prev = rvec.copy()
        self._tvec_prev = tvec.copy()
        return PnpResult(ok=True, reason='', rvec=rvec, tvec_opt=tvec, reproj_err_px=err_px)

    def _evaluate(self, rvec, tvec, obs) -> Optional[PnpResult]:
        """Only physically forward-facing, finite candidate points can be ranked."""
        try:
            r = np.asarray(rvec, dtype=np.float64).reshape(3)
            t = np.asarray(tvec, dtype=np.float64).reshape(3)
            if not (np.all(np.isfinite(r)) and np.all(np.isfinite(t))):
                return None
            rotation, _ = cv2.Rodrigues(r)
            if np.any((self._obj @ rotation.T + t)[:, 2] <= 0.0):
                return None
            error = self.reprojection_error(r, t, obs)
            if not math.isfinite(error):
                return None
            return PnpResult(rvec=r.copy(), tvec_opt=t.copy(), reproj_err_px=error)
        except cv2.error:
            # 候选投影不成立时不能参与重投影误差排名。
            return None
