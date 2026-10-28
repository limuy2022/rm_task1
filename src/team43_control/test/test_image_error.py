"""`visual_servo_node.image_error()` 的几何穷举回归（真 ROS2 Humble）。

`image_error` 是链 B（图像 PID）的误差源头：4 角点质心 → `undistortPoints`
去畸变归一化 → ``yaw = -atan2(x, 1)``、``pitch = atan2(y, hypot(1, x))``
（图像右 → yaw 误差负；图像下 → pitch 误差正）。

三层校验（内参一律读 `camera_team43.yaml`，不手抄字面量）：

1. **网格穷举**：对称角点的质心 = 中心像素，误差必须**精确**等于对中心
   像素做 `undistortPoints` 的解析公式——覆盖全图（含强畸变边缘）与四象限。
2. **PnP 互检**：`projectPoints(K, D)` 真投影的透视角点 → 误差应接近
   目标方位角 ``(-atan2(tx, tz), atan2(ty, hypot(tz, tx)))``；差值暴露
   "undistort(质心) vs 质心(undistort)" 的系统偏差，按离轴度放宽。
3. **符号与非法输入**：右负/下正的符号约定跨象限成立；NaN/交叉/越界
   角点必须 ValueError。
"""

from __future__ import annotations

import math
import os
import sys

import cv2
import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import conftest  # noqa: F401,E402  触发 sys.path 注入

pytest.importorskip('rclpy')

from team43_control.visual_servo_node import image_error  # noqa: E402

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CAMERA = yaml.safe_load(open(os.path.join(
    _SRC, 'team43_bringup', 'config', 'camera_team43.yaml'),
    encoding='utf-8').read())['camera']
K = np.array([[CAMERA['fx'], 0., CAMERA['cx']],
              [0., CAMERA['fy'], CAMERA['cy']], [0., 0., 1.]])
D = np.asarray(CAMERA['distortion'], dtype=float)
W, H = int(CAMERA['width']), int(CAMERA['height'])


def _analytic_error(pixel_xy):
    """对单个像素做 undistort 后的解析误差（与实现公式逐项对应）。"""
    pt = np.array([[pixel_xy]], dtype=np.float32)
    xn, yn = cv2.undistortPoints(pt, K, D).reshape(2)
    return (-math.atan2(xn, 1.0), math.atan2(yn, math.hypot(1.0, xn)))


def _square_corners(cx, cy, half):
    return [cx - half, cy - half, cx + half, cy - half,
            cx + half, cy + half, cx - half, cy + half]


# ---------------------------------------------------------------------------
# 1. 网格穷举：对称角点 → 精确等于中心像素的解析公式
# ---------------------------------------------------------------------------
def test_symmetric_grid_matches_undistorted_center_exactly():
    """全图网格（中心/边缘/四角）× 对称方块 → 误差精确等于解析公式。

    对称角点质心严格等于中心像素，故该等式不受畸变影响、应严格成立
    （浮点容差 1e-9 只吸收 undistort 两次调用的数值差）。
    """
    half = 12.0
    xs = np.linspace(half + 2, W - half - 2, 9)
    ys = np.linspace(half + 2, H - half - 2, 7)
    checked = 0
    for cx in xs:
        for cy in ys:
            (yaw, pitch), _off = image_error(_square_corners(cx, cy, half), CAMERA)
            yaw_ref, pitch_ref = _analytic_error((cx, cy))
            # abs=1e-6：实现内 float32 undistort 与参考 float64 的数值差
            assert yaw == pytest.approx(yaw_ref, abs=1e-6), (cx, cy, 'yaw')
            assert pitch == pytest.approx(pitch_ref, abs=1e-6), (cx, cy, 'pitch')
            checked += 1
    assert checked >= 60, '网格必须覆盖全图（含畸变最强的边缘）'


def test_grid_covers_all_quadrants_with_distortion_edge():
    """四象限 + 最靠近图像边缘的一圈内都必须可用且符号正确。"""
    half = 10.0
    cx0, cy0 = float(CAMERA['cx']), float(CAMERA['cy'])
    for cx, cy in ((cx0 - 400, cy0 - 300), (cx0 + 400, cy0 - 300),
                   (cx0 + 400, cy0 + 300), (cx0 - 400, cy0 + 300),
                   (half + 3, half + 3), (W - half - 3, H - half - 3)):
        (yaw, pitch), (ox, oy) = image_error(_square_corners(cx, cy, half), CAMERA)
        expect_yaw_negative = cx > cx0
        assert (yaw < 0) == expect_yaw_negative, (cx, cy, yaw)
        assert (pitch > 0) == (cy > cy0), (cx, cy, pitch)
        assert ox == pytest.approx(cx - cx0, abs=1e-6)
        assert oy == pytest.approx(cy - cy0, abs=1e-6)


# ---------------------------------------------------------------------------
# 2. PnP 互检：真投影视角落到方位角附近（容差吸收透视不对称）
# ---------------------------------------------------------------------------
def _project_marker(tvec, rvec):
    s = 0.10
    obj = np.array([[-s / 2, s / 2, 0.], [s / 2, s / 2, 0.],
                    [s / 2, -s / 2, 0.], [-s / 2, -s / 2, 0.]])
    pts, _ = cv2.projectPoints(obj, np.asarray(rvec, dtype=float),
                               np.asarray(tvec, dtype=float), K, D)
    return pts.reshape(4, 2)


@pytest.mark.parametrize('tvec,tol_deg', [
    ((0., 0., 2.0), 0.05),
    ((0., 0., 1.0), 0.05),
    ((0.20, -0.12, 2.0), 0.6),
    ((-0.30, 0.18, 1.5), 1.0),
    ((0.05, 0.03, 3.0), 0.1),
])
def test_perspective_corners_match_bearing_direction(tvec, tol_deg):
    """真投影（含畸变）的角点 → 误差 ≈ 目标方位角；离轴越远透视不对称越大。"""
    rvec, _ = cv2.Rodrigues(np.eye(3))     # 正对相机（无旋转）
    corners = _project_marker(tvec, rvec).reshape(-1).tolist()
    (yaw, pitch), _off = image_error(corners, CAMERA)
    tx, ty, tz = tvec
    yaw_ref = -math.atan2(tx, tz)
    pitch_ref = math.atan2(ty, math.hypot(tz, tx))
    assert math.degrees(abs(yaw - yaw_ref)) < tol_deg, (tvec, 'yaw')
    assert math.degrees(abs(pitch - pitch_ref)) < tol_deg, (tvec, 'pitch')


def test_centered_marker_is_essentially_zero_error():
    """光轴上的目标（任意深度）→ 误差接近 0。"""
    for tz in (0.8, 2.0, 4.0):
        corners = _project_marker((0., 0., tz), np.zeros(3)).reshape(-1).tolist()
        (yaw, pitch), _off = image_error(corners, CAMERA)
        assert abs(math.degrees(yaw)) < 0.01 and abs(math.degrees(pitch)) < 0.01


# ---------------------------------------------------------------------------
# 3. 符号约定与非法输入
# ---------------------------------------------------------------------------
def test_sign_conventions():
    """图像右 → yaw 负（向右转）；图像下 → pitch 正（向下转）。"""
    cx, cy, half = float(CAMERA['cx']), float(CAMERA['cy']), 15.0
    (yaw_r, _p), _ = image_error(_square_corners(cx + 120, cy, half), CAMERA)
    (yaw_l, _p), _ = image_error(_square_corners(cx - 120, cy, half), CAMERA)
    (yaw_c, pitch_d), _ = image_error(_square_corners(cx, cy + 90, half), CAMERA)
    (yaw_c2, pitch_u), _ = image_error(_square_corners(cx, cy - 90, half), CAMERA)
    assert yaw_r < 0 < yaw_l
    assert pitch_d > 0 > pitch_u
    # 中心处 ~3e-7 的残差来自 float32 undistort 的数值精度
    assert abs(yaw_c) < 1e-5 and abs(yaw_c2) < 1e-5


def test_invalid_corners_raise():
    cx, cy, half = float(CAMERA['cx']), float(CAMERA['cy']), 20.0
    nan_corners = _square_corners(cx, cy, half)
    nan_corners[0] = float('nan')
    crossed = [cx - half, cy - half, cx + half, cy + half,   # TL,BR,TR,BL → 自交蝴蝶结
               cx + half, cy - half, cx - half, cy + half]
    out_of_bounds = _square_corners(2.0, 2.0, half)          # 角点为负
    for bad in (nan_corners, crossed, out_of_bounds):
        with pytest.raises(ValueError):
            image_error(bad, CAMERA)
