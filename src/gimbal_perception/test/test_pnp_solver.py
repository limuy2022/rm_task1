"""PnP 解算器 / 噪声模型 / marker 适配层单测（DESIGN §6.4-§6.6）。

纯 Python（只依赖 numpy + cv2，**不 import rclpy**）。运行::

    python -m pytest src/gimbal_perception/test/test_pnp_solver.py -q

内参一律从 ``gimbal_tracker_bringup/config/virtual.yaml`` 读取（唯一数据源），
本文件不二次手抄任何内参字面量。
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.abspath(os.path.join(_TEST_DIR, '..'))
_SRC_DIR = os.path.abspath(os.path.join(_PKG_DIR, '..'))
_WS_DIR = os.path.abspath(os.path.join(_SRC_DIR, '..'))
sys.path.insert(0, _PKG_DIR)
sys.path.insert(0, os.path.join(_SRC_DIR, 'gimbal_common'))

from gimbal_common import config_loader as CL  # noqa: E402
from gimbal_perception.marker_source import (  # noqa: E402
    RawMarker, SingleTopicMarkerSource, make_marker_source,
)
from gimbal_perception.noise_model import NoiseModel  # noqa: E402
from gimbal_perception.pnp_solver import (  # noqa: E402
    PnpSolver, marker_object_points, rpy_to_R,
)

VIRTUAL_YAML = os.path.join(_WS_DIR, 'src', 'gimbal_tracker_bringup', 'config', 'virtual.yaml')
CAM = CL.load_yaml(VIRTUAL_YAML, required=True).get('camera', {})
FX = float(CAM['fx'])
FY = float(CAM['fy'])
CX = float(CAM['cx'])
CY = float(CAM['cy'])
MARKER = 0.16


def _solver(**kwargs) -> PnpSolver:
    return PnpSolver.from_camera_config(CAM, marker_size_m=MARKER, **kwargs)


def _project(obj_pts: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """使用配置中的畸变，与待测解算器一致。"""
    import cv2
    K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], dtype=np.float64)
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K,
                                np.asarray(CAM.get('distortion', np.zeros(5))))
    return proj.reshape(-1, 2)


class _P:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y


class _FakeMarkerCorners:
    def __init__(self, pts, marker_id=0, frame: str = 'camera_optical_frame') -> None:
        self.corners = [_P(float(x), float(y)) for (x, y) in pts]
        self.marker_id = marker_id
        self.valid = True
        self.header = type('H', (), {'frame_id': frame, 'stamp': None})()


# --------------------------------------------------------------------------------------
# 物点顺序
# --------------------------------------------------------------------------------------
def test_object_points_order_is_TL_TR_BR_BL():
    obj = marker_object_points(MARKER)
    s = MARKER / 2.0
    assert np.allclose(obj[0], [-s, +s, 0.0])     # 码局部TL：x右、y上
    assert np.allclose(obj[1], [+s, +s, 0.0])     # TR
    assert np.allclose(obj[2], [+s, -s, 0.0])     # BR
    assert np.allclose(obj[3], [-s, -s, 0.0])     # BL


def test_object_points_form_a_square_of_expected_image_width():
    """正对 2m 时，4 个物点投影成一个边长 ≈ s·fx/z ≈ 44px 的正方形（官方文档量级）。"""
    obj = marker_object_points(MARKER)
    uv = _project(obj, np.zeros(3), np.array([0.0, 0.0, 2.0]))
    assert uv.shape == (4, 2)
    # 对角线长度 = s*sqrt(2)*fx/z
    d01 = float(np.linalg.norm(uv[1] - uv[0]))
    d12 = float(np.linalg.norm(uv[2] - uv[1]))
    d23 = float(np.linalg.norm(uv[3] - uv[2]))
    d30 = float(np.linalg.norm(uv[0] - uv[3]))
    expect_edge = MARKER * FX / 2.0
    for d in (d01, d12, d23, d30):
        assert d == pytest.approx(expect_edge, abs=0.5)
    # 不锁定旧教学相机44px：当前内参与畸变来自上述配置。


# --------------------------------------------------------------------------------------
# 正解回归
# --------------------------------------------------------------------------------------
def test_solve_recovers_known_pose_frontal():
    solver = _solver()
    obj = marker_object_points(MARKER)
    tvec_true = np.array([0.05, -0.03, 2.0])
    uv = _project(obj, np.zeros(3), tvec_true)
    res = solver.solve(uv)
    assert res.ok, res.reason
    assert np.allclose(res.tvec_opt, tvec_true, atol=1e-3)
    assert res.depth == pytest.approx(2.0, abs=1e-3)
    assert res.reproj_err_px < 1e-3


def test_solve_recovers_known_pose_rotated():
    solver = _solver()
    obj = marker_object_points(MARKER)
    R = rpy_to_R(0.15, -0.25, 0.4)
    import cv2
    rvec_true, _ = cv2.Rodrigues(R)
    rvec_true = rvec_true.reshape(3)
    tvec_true = np.array([-0.2, 0.1, 1.4])
    uv = _project(obj, rvec_true, tvec_true)
    res = solver.solve(uv)
    assert res.ok, res.reason
    assert np.allclose(res.tvec_opt, tvec_true, atol=2e-3)
    assert res.reproj_err_px < 0.05
    # 姿态也应还原（旋转向量可能差一个等价表示，用矩阵比较）
    rvec_est = res.rvec
    R_est, _ = cv2.Rodrigues(rvec_est.reshape(3, 1))
    assert np.allclose(R_est, R, atol=5e-3)


def test_solve_at_two_meters_matches_sim_expectation():
    """仿真典型工况：正对 2m，深度应 ≈2.0 ± 0.05（DESIGN T02 验收点）。"""
    solver = _solver()
    obj = marker_object_points(MARKER)
    uv = _project(obj, np.zeros(3), np.array([0.0, 0.0, 2.0]))
    res = solver.solve(uv)
    assert res.ok
    assert res.depth == pytest.approx(2.0, abs=0.05)


# --------------------------------------------------------------------------------------
# 拒绝路径（均属"本帧无观测"，不报错）
# --------------------------------------------------------------------------------------
def test_reject_on_large_reprojection_error():
    solver = _solver(reproj_max_px=2.0)
    obj = marker_object_points(MARKER)
    uv = _project(obj, np.zeros(3), np.array([0.0, 0.0, 2.0]))
    uv_noisy = uv.copy()
    uv_noisy[0] += 25.0          # 把一个角点拽远 → 重投影必然超标
    res = solver.solve(uv_noisy)
    assert not res.ok
    assert res.reason.startswith('reproj') or res.reason.startswith('depth')


def test_reject_out_of_range_depth():
    solver = _solver(depth_min=0.05, depth_max=30.0)
    obj = marker_object_points(MARKER)
    uv = _project(obj, np.zeros(3), np.array([0.0, 0.0, 0.02]))     # 太近
    assert not solver.solve(uv).ok
    uv_far = _project(obj, np.zeros(3), np.array([0.0, 0.0, 40.0]))  # 太远
    res = solver.solve(uv_far)
    assert not res.ok
    assert res.reason.startswith('depth')


def test_reject_wrong_corner_count_or_nonfinite():
    solver = _solver()
    assert not solver.solve(np.zeros((3, 2))).ok
    bad = np.zeros((4, 2))
    bad[2, 0] = float('nan')
    res = solver.solve(bad)
    assert not res.ok
    assert res.reason == 'nonfinite_corners'


def test_solver_recovers_after_reset_guess():
    solver = _solver()
    obj = marker_object_points(MARKER)
    uv = _project(obj, np.zeros(3), np.array([0.0, 0.0, 2.0]))
    assert solver.solve(uv).ok
    solver.reset_guess()
    assert solver.solve(uv).ok


def test_intrinsics_must_come_from_virtual_yaml():
    with pytest.raises(ValueError):
        PnpSolver.from_camera_config({'fx': 0.0, 'fy': 0.0, 'cx': 0.0, 'cy': 0.0})


# --------------------------------------------------------------------------------------
# 噪声模型（DESIGN §6.6）
# --------------------------------------------------------------------------------------
def test_noise_sigma_scaling():
    nm = NoiseModel(fx=FX, marker_size_m=MARKER, corner_noise_px=0.5)
    expected_lat = max(0.5 / FX, 5.0e-4)
    expected_depth = max(0.5 / (MARKER * FX), 4.0e-3)
    assert nm.k_lat == pytest.approx(expected_lat, rel=1e-9)
    assert nm.k_depth == pytest.approx(expected_depth, rel=1e-9)
    assert nm.sigma_lat(2.0) == pytest.approx(expected_lat * 2.0, rel=1e-6)
    assert nm.sigma_depth(2.0) == pytest.approx(expected_depth * 4.0, rel=1e-6)
    # 深度噪声比横向大一个数量级
    assert nm.sigma_depth(2.0) / nm.sigma_lat(2.0) > 8.0
    # 平方/线性缩放
    assert nm.sigma_depth(4.0) == pytest.approx(4.0 * nm.sigma_depth(2.0), rel=1e-6)
    assert nm.sigma_lat(4.0) == pytest.approx(2.0 * nm.sigma_lat(2.0), rel=1e-6)


def test_noise_floor_protection():
    nm = NoiseModel(fx=FX, marker_size_m=MARKER, corner_noise_px=0.0)
    assert nm.k_lat == pytest.approx(5.0e-4)     # 走到下限
    assert nm.k_depth == pytest.approx(4.0e-3)
    assert nm.sigma_lat(2.0) == pytest.approx(5.0e-4 * 2.0)     # 1.0 mm
    assert nm.sigma_depth(2.0) == pytest.approx(4.0e-3 * 4.0)   # 1.6 cm


def test_noise_force_override():
    nm = NoiseModel(fx=FX, marker_size_m=MARKER, corner_noise_px=0.5,
                    force_k_lat=1.0e-3, force_k_depth=1.0e-2)
    assert nm.k_lat == pytest.approx(1.0e-3)
    assert nm.k_depth == pytest.approx(1.0e-2)


def test_noise_cov_is_diagonal_and_finite():
    nm = NoiseModel(fx=FX, marker_size_m=MARKER)
    C = nm.cov_optical(2.0)
    assert C.shape == (3, 3)
    assert np.allclose(C, np.diag(np.diag(C)))
    assert np.all(np.isfinite(C))
    sl, sd = nm.build_sigma(2.0)
    assert C[0, 0] == pytest.approx(sl * sl)
    assert C[1, 1] == pytest.approx(sl * sl)
    assert C[2, 2] == pytest.approx(sd * sd)
    assert len(nm.cov_optical_flat(2.0)) == 9
    # 深度为 0 时退化为数值下限，不允许出现 NaN / 负数
    C0 = nm.cov_optical(0.0)
    assert np.all(np.isfinite(C0))
    assert np.all(np.diag(C0) >= 0.0)


def test_noise_fx_setter_recomputes():
    nm = NoiseModel(fx=FX, marker_size_m=MARKER)
    nm.fx = 2.0 * FX
    assert nm.k_lat == pytest.approx(max(0.5 / (2.0 * FX), 5.0e-4), rel=1e-9)
    nm.fx = FX
    assert nm.k_lat == pytest.approx(max(0.5 / FX, 5.0e-4), rel=1e-9)


# --------------------------------------------------------------------------------------
# marker 适配层（DESIGN §6.4）
# --------------------------------------------------------------------------------------
def test_single_topic_source_accepts_unique_observation_when_filter_off():
    src = SingleTopicMarkerSource(target_digit=0, id_filter_mode='off')
    poly = _FakeMarkerCorners([(100.0, 100.0), (140.0, 100.0), (140.0, 140.0), (100.0, 140.0)], marker_id=7)
    m = src.on_polygon(poly, t_mono=1.0)
    assert m is not None
    assert m.marker_id == 7
    assert m.n_corners == 4
    assert m.frame_id == 'camera_optical_frame'
    assert src.poll() == [m]


def test_single_topic_source_filters_by_id_when_filter_on():
    src = SingleTopicMarkerSource(target_digit=3, id_filter_mode='on')
    poly = _FakeMarkerCorners([(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)], marker_id=5)
    assert src.on_polygon(poly, t_mono=0.0) is None
    poly.marker_id = 3
    assert src.on_polygon(poly, t_mono=0.1) is not None
    assert src.stats['filtered'] == 1


def test_single_topic_source_treats_empty_or_bad_length_as_no_observation():
    src = SingleTopicMarkerSource()
    assert src.on_polygon(_FakeMarkerCorners([]), t_mono=0.0) is None
    assert src.on_polygon(_FakeMarkerCorners([(1.0, 1.0)] * 3), t_mono=0.0) is None
    assert src.poll() == []
    assert src.stats['seen'] == 0       # 不算"看到"，也不报错


def test_single_topic_source_rejects_nonfinite():
    src = SingleTopicMarkerSource()
    poly = _FakeMarkerCorners([(float('nan'), 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)])
    assert src.on_polygon(poly, t_mono=0.0) is None


def test_marker_source_factory():
    assert isinstance(make_marker_source('single_topic', target_digit=2),
                      SingleTopicMarkerSource)
    with pytest.raises(ValueError):
        make_marker_source('nope')
    # P1 源可实例化，但 poll 明确报未实现
    with pytest.raises(NotImplementedError):
        make_marker_source('multi_topic').poll()


def test_raw_marker_defaults():
    m = RawMarker()
    assert m.n_corners == 4
    assert m.valid


# --------------------------------------------------------------------------------------
# 端到端：合成角点 -> PnP -> 噪声
# --------------------------------------------------------------------------------------
def test_end_to_end_marker_at_two_meters_off_axis():
    solver = _solver()
    nm = NoiseModel(fx=FX, marker_size_m=MARKER, corner_noise_px=0.5)
    obj = marker_object_points(MARKER)
    tvec_true = np.array([0.35, -0.25, 2.0])
    uv = _project(obj, np.zeros(3), tvec_true)
    res = solver.solve(uv)
    assert res.ok
    assert res.depth == pytest.approx(2.0, abs=0.02)
    C = nm.cov_optical(res.depth)
    assert np.all(np.isfinite(C))
    assert C[2, 2] > C[0, 0]      # 深度不确定度远大于横向
