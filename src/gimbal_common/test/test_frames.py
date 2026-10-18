"""frames / math_utils / time_utils 单测（含 pitch 符号自检 T-C2）。

纯 Python，不依赖 ROS。运行::

    python -m pytest src/gimbal_common/test/test_frames.py -q
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from gimbal_common import frames as F
from gimbal_common import math_utils as M
from gimbal_common import time_utils as T


# --------------------------------------------------------------------------------------
# ⚠ pitch 符号（DESIGN §2.3 / 自检 T-C2）
# --------------------------------------------------------------------------------------
def test_pitch_sign_single_source():
    assert F.PITCH_SIGN == -1.0
    assert F.to_protocol_pitch(0.1) == pytest.approx(-0.1)
    assert F.from_protocol_pitch(-0.1) == pytest.approx(0.1)
    assert F.from_protocol_pitch(F.to_protocol_pitch(0.37)) == pytest.approx(0.37)


def test_pitch_sign_head_up_maps_to_negative_protocol():
    """抬头（z>0）在协议域必须是负的（低头为正）。"""
    horiz = 2.0
    z_up = 0.5
    head_up = math.atan2(z_up, horiz)
    assert head_up > 0.0
    assert F.to_protocol_pitch(head_up) < 0.0


def test_pitch_sign_flip_roundtrip():
    old = F.get_pitch_sign()
    try:
        F.set_pitch_sign(1.0)
        assert F.to_protocol_pitch(0.1) == pytest.approx(0.1)
        F.set_pitch_sign(-1.0)
        assert F.to_protocol_pitch(0.1) == pytest.approx(-0.1)
    finally:
        F.set_pitch_sign(old)
    with pytest.raises(ValueError):
        F.set_pitch_sign(0.5)


# --------------------------------------------------------------------------------------
# 四元数 / 姿态分解
# --------------------------------------------------------------------------------------
def test_quat_to_mat_identity():
    R = F.quat_to_mat([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(R, np.eye(3))


def test_quat_to_mat_roundtrip():
    rng = np.random.default_rng(7)
    for _ in range(20):
        q = rng.normal(size=4)
        R = F.quat_to_mat(q)
        q2 = F.quat_from_mat(R)
        R2 = F.quat_to_mat(q2)
        assert np.allclose(R, R2, atol=1e-9)


def test_decompose_yaw_pitch_roundtrip():
    for yaw in (-2.5, -0.3, 0.0, 0.7, 2.8):
        for pitch in (-0.5, -0.1, 0.0, 0.2, 0.35):
            R = F.compose_yaw_pitch(yaw, pitch)
            y2, p2 = F.decompose_yaw_pitch(R)
            assert y2 == pytest.approx(yaw, abs=1e-9)
            assert p2 == pytest.approx(pitch, abs=1e-9)


def test_decompose_yaw_pitch_matches_control_law_definition():
    """DESIGN §6.8：yaw_c = atan2(f.y, f.x)；pitch_c = to_protocol_pitch(atan2(f.z, |f_xy|))。"""
    yaw, pitch = 0.6, -0.2
    R = F.compose_yaw_pitch(yaw, pitch)
    f = R[:, 0]
    yaw_c = math.atan2(f[1], f[0])
    pitch_c = F.to_protocol_pitch(math.atan2(f[2], math.hypot(f[0], f[1])))
    assert yaw_c == pytest.approx(yaw, abs=1e-9)
    assert pitch_c == pytest.approx(pitch, abs=1e-9)


def test_positive_protocol_pitch_is_nose_down():
    """协议域 pitch>0（低头）→ 机体 x 轴指向下方（world z 分量为负）。"""
    R = F.compose_yaw_pitch(0.0, 0.2)
    f = R[:, 0]
    assert f[2] < 0.0
    assert math.atan2(f[2], math.hypot(f[0], f[1])) < 0.0


def test_no_roll_in_decomposition():
    """云台没有 roll：分解后重新组合必须还原原矩阵（对无 roll 的姿态）。"""
    rng = np.random.default_rng(11)
    for _ in range(10):
        yaw = float(rng.uniform(-2.8, 2.8))
        pitch = float(rng.uniform(-0.5, 0.35))
        R = F.compose_yaw_pitch(yaw, pitch)
        assert np.allclose(F.compose_yaw_pitch(*F.decompose_yaw_pitch(R)), R, atol=1e-9)


# --------------------------------------------------------------------------------------
# R_opt_from_link
# --------------------------------------------------------------------------------------
def test_R_opt_from_link_maps_axes():
    R = F.R_opt_from_link()
    assert R.shape == (3, 3)
    # optical z（前）→ gimbal x（前）
    assert np.allclose(R @ np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))
    # optical x（右）→ gimbal -y（右）
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), np.array([0.0, -1.0, 0.0]))
    # optical y（下）→ gimbal -z（下）
    assert np.allclose(R @ np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, -1.0]))


def test_R_opt_from_link_is_inverse_of_R_link_from_opt():
    A = F.R_opt_from_link()
    B = F.R_link_from_opt()
    assert np.allclose(A @ B, np.eye(3))
    assert np.allclose(A, B.T)
    assert np.allclose(np.linalg.det(A), 1.0)


# --------------------------------------------------------------------------------------
# compose_T / invert_T / rotz2 / freeze_T_map_odom / gimbal_origin_world
# --------------------------------------------------------------------------------------
def test_compose_T_and_invert_T():
    R = F.compose_yaw_pitch(0.4, -0.1)
    t = np.array([1.0, -2.0, 0.5])
    T = F.compose_T(R, t)
    assert T.shape == (4, 4)
    assert np.allclose(T[0:3, 0:3], R)
    assert np.allclose(T[0:3, 3], t)
    assert np.allclose(T @ F.invert_T(T), np.eye(4), atol=1e-12)


def test_rotz2():
    R = F.rotz2(math.pi / 2.0)
    assert np.allclose(R @ np.array([1.0, 0.0]), np.array([0.0, 1.0]))


def test_rotz3_matches_rotz2():
    R3 = F.rotz(0.7)
    R2 = F.rotz2(0.7)
    assert np.allclose(R3[0:2, 0:2], R2)
    assert np.allclose(R3[2, :], np.array([0.0, 0.0, 1.0]))


def test_rpy_to_mat_accepts_3_and_6_element_vectors():
    rpy3 = [0.1, -0.2, 0.3]
    pose6 = [1.0, 2.0, 3.0, 0.1, -0.2, 0.3]
    assert np.allclose(F.rpy_to_mat(rpy3), F.rpy_to_mat(pose6))
    R = F.rpy_to_mat(rpy3)
    assert np.allclose(np.linalg.det(R), 1.0, atol=1e-12)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)


def test_rpy_to_mat_zero_is_identity():
    assert np.allclose(F.rpy_to_mat([0.0, 0.0, 0.0]), np.eye(3))
    assert np.allclose(F.rpy_to_mat([]), np.eye(3))


def test_rpy_to_mat_yaw_only_matches_rotz():
    assert np.allclose(F.rpy_to_mat([0.0, 0.0, 0.4]), F.rotz(0.4))


def test_freeze_T_map_odom_puts_base_at_origin():
    x, y, yaw = 1.5, -0.75, 0.6
    T = F.freeze_T_map_odom(x, y, yaw)
    p = np.array([x, y, 0.0, 1.0])
    assert np.allclose((T @ p)[:2], np.zeros(2), atol=1e-12)
    # 冻结后 map 的 X 轴 = t0 底盘朝向
    dir_map = T[0:2, 0:2] @ np.array([math.cos(yaw), math.sin(yaw)])
    assert np.allclose(dir_map, np.array([1.0, 0.0]), atol=1e-12)
    # 刚体性质
    assert np.allclose(np.linalg.det(T[0:2, 0:2]), 1.0)
    assert np.allclose(T[3, :], np.array([0.0, 0.0, 0.0, 1.0]))


def test_freeze_T_map_odom_zero_yaw_translation_only():
    T = F.freeze_T_map_odom(3.0, 4.0, 0.0)
    assert np.allclose((T @ np.array([3.0, 4.0, 0.0, 1.0]))[:3], np.zeros(3))
    assert np.allclose((T @ np.array([4.0, 4.0, 0.0, 1.0]))[:2], np.array([1.0, 0.0]))


def test_gimbal_origin_world_identity_in_sim():
    p = F.gimbal_origin_world((0.0, 0.0), 0.0, (0.0, 0.0, 0.0))
    assert np.allclose(p, np.zeros(3))


def test_gimbal_origin_world_with_mount_and_rotation():
    # 底盘原点在 (1,0,0)，朝向 +90°，云台安装在 base_link 前方 0.2m
    p = F.gimbal_origin_world((1.0, 0.0), math.pi / 2.0, (0.2, 0.0, 0.0))
    assert np.allclose(p, np.array([1.0, 0.2, 0.0]), atol=1e-12)
    # 带 T_world←odom 平移 (0,0,0.5)
    T_wo = F.compose_T(np.eye(3), np.array([0.0, 0.0, 0.5]))
    p2 = F.gimbal_origin_world((1.0, 0.0), math.pi / 2.0, (0.2, 0.0, 0.0), T_wo)
    assert np.allclose(p2, np.array([1.0, 0.2, 0.5]), atol=1e-12)


# --------------------------------------------------------------------------------------
# math_utils
# --------------------------------------------------------------------------------------
def test_wrap_pi():
    assert M.wrap_pi(0.0) == pytest.approx(0.0)
    assert M.wrap_pi(3.0 * math.pi) == pytest.approx(math.pi, abs=1e-9) or \
        abs(M.wrap_pi(3.0 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert M.wrap_pi(-1.5 * math.pi) == pytest.approx(0.5 * math.pi, abs=1e-9)
    assert -math.pi <= M.wrap_pi(12.345) <= math.pi
    assert M.wrap_pi(float('nan')) == 0.0


def test_clamp():
    assert M.clamp(5.0, 0.0, 1.0) == 1.0
    assert M.clamp(-5.0, 0.0, 1.0) == 0.0
    assert M.clamp(0.3, 0.0, 1.0) == pytest.approx(0.3)


def test_lowpass_monotonic_towards_target():
    v = 0.0
    for _ in range(200):
        v = M.lowpass(v, 1.0, 0.01, 5.0)
    assert v == pytest.approx(1.0, abs=1e-3)
    assert M.lowpass(0.5, float('nan'), 0.01, 5.0) == pytest.approx(0.5)


def test_accel_limit():
    d = np.array([10.0, 0.0])
    out = M.accel_limit(d, 0.1, 1.0)
    assert np.linalg.norm(out) == pytest.approx(0.1)
    assert out[0] > 0.0 and abs(out[1]) < 1e-12
    assert np.allclose(M.accel_limit(np.array([0.01, 0.0]), 0.1, 1.0), np.array([0.01, 0.0]))
    assert np.allclose(M.accel_limit(np.array([float('nan'), 0.0]), 0.1, 1.0), np.zeros(2))


def test_ray_cells_axis_aligned():
    cells = M.ray_cells((0.05, 0.05), (1.0, 0.0), 0.5, 0.1)
    assert cells[0] == (0, 0)
    assert (1, 0) in cells and (2, 0) in cells
    assert all(j == 0 for (_, j) in cells)


def test_ray_cells_diagonal_is_supercover():
    """沿 45° 穿过格点时，两侧单元都要覆盖（不漏检）。"""
    res = 0.1
    cells = M.ray_cells((res * 0.5, res * 0.5), (1.0, 1.0), res * 3.0, res)
    for (i, j) in cells:
        assert abs(i - j) <= 1
    assert (1, 1) in cells and (2, 2) in cells


def test_ray_cells_degenerate_inputs():
    assert M.ray_cells((0.0, 0.0), (0.0, 0.0), 1.0, 0.1) == []
    assert M.ray_cells((0.0, 0.0), (1.0, 0.0), 0.0, 0.1) == []
    assert M.ray_cells((0.0, 0.0), (1.0, 0.0), 1.0, 0.0) == []


# --------------------------------------------------------------------------------------
# time_utils
# --------------------------------------------------------------------------------------
def test_ticker_due():
    tk = T.MonotonicTicker(0.01, auto_start=False)
    assert tk.due(t=100.0) is True          # 首次立即触发（相位初始化）
    assert tk.due(t=100.005) is False
    assert tk.due(t=100.011) is True


def test_rate_limiter():
    rl = T.RateLimiter(1.0)
    assert rl.allow(t=0.0) is True
    assert rl.allow(t=0.5) is False
    assert rl.allow(t=1.0) is True


def test_timeout_hold():
    to = T.Timeout(0.2)
    assert to.update(True, t=0.0) is False
    assert to.update(True, t=0.1) is False
    assert to.update(True, t=0.25) is True
    assert to.update(False, t=0.3) is False
    assert to.update(True, t=0.4) is False


def test_monotonic_now_is_monotonic():
    a = T.now()
    b = T.now()
    assert b >= a
