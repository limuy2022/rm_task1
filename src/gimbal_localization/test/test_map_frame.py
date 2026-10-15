"""map 系冻结与底盘位姿源单测（DESIGN §2.1 / §5.4 / §6.9）。

纯 Python（**不 import rclpy**）。运行::

    python -m pytest src/gimbal_localization/test/test_map_frame.py -q
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

_PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
_SRC_DIR = os.path.abspath(os.path.join(_PKG_DIR, '..'))
sys.path.insert(0, _PKG_DIR)
sys.path.insert(0, os.path.join(_SRC_DIR, 'gimbal_common'))   # gimbal_common 源码目录

from gimbal_common import frames as FR  # noqa: E402
from gimbal_localization.base_pose_source import (  # noqa: E402
    MockSource, OdomTopicSource, T_map_world_from_odom, make_base_pose_source, quat_yaw,
)


def _apply2(T: np.ndarray, x: float, y: float) -> np.ndarray:
    return (T @ np.array([x, y, 0.0, 1.0]))[:2]


# --------------------------------------------------------------------------------------
# t0 冻结
# --------------------------------------------------------------------------------------
def test_freeze_puts_t0_base_at_map_origin():
    x0, y0, yaw0 = 1.25, -0.75, 0.4
    T = T_map_world_from_odom(x0, y0, yaw0)
    assert np.allclose(_apply2(T, x0, y0), np.zeros(2), atol=1e-12)


def test_frozen_origin_does_not_drift():
    """t0 冻结后矩阵恒定：后续底盘运动不影响 map 原点。"""
    x0, y0, yaw0 = 2.0, 1.0, -0.3
    T0 = T_map_world_from_odom(x0, y0, yaw0)
    # 之后底盘移动到 (3.0, 1.0)：map 系应反映真实位移，原点不跟着漂移
    p_map = _apply2(T0, 3.0, 1.0)
    R = FR.rotz2(-yaw0)
    expect = R @ np.array([3.0 - x0, 1.0 - y0])
    assert np.allclose(p_map, expect, atol=1e-12)
    # 原点仍在 t0 位置：反查 map (0,0) → odom (x0,y0)
    assert np.allclose(_apply2(T0, x0, y0), np.zeros(2), atol=1e-12)
    assert np.allclose(T0, T_map_world_from_odom(x0, y0, yaw0), atol=1e-15)


def test_freeze_yaw_alignment():
    x0, y0, yaw0 = 0.0, 0.0, math.pi / 2.0
    T = T_map_world_from_odom(x0, y0, yaw0)
    # odom 系中沿 t0 朝向前进 1m → map 系 +x
    ahead = (math.cos(yaw0), math.sin(yaw0))
    assert np.allclose(_apply2(T, ahead[0], ahead[1]), np.array([1.0, 0.0]), atol=1e-12)


def test_freeze_with_nonidentity_world_odom():
    T_wo = FR.compose_T(np.eye(3), np.array([0.5, -0.25, 0.1]))
    T = T_map_world_from_odom(1.0, 1.0, 0.0, T_world_odom=T_wo)
    # world 系中底盘在 (1+0.5, 1-0.25) = (1.5, 0.75)
    assert np.allclose(_apply2(T, 1.5, 0.75), np.zeros(2), atol=1e-12)


def test_frozen_transform_is_rigid():
    T = T_map_world_from_odom(-3.0, 2.0, 1.1)
    assert np.allclose(np.linalg.det(T[0:3, 0:3]), 1.0, atol=1e-12)
    assert np.allclose(T[3, :], np.array([0.0, 0.0, 0.0, 1.0]))


# --------------------------------------------------------------------------------------
# MockSource
# --------------------------------------------------------------------------------------
def test_mock_static():
    src = MockSource(motion='static', origin_xy=(0.5, -0.5), yaw0=0.7)
    for t in (0.0, 1.0, 10.0):
        p = src.poll(t)
        assert (p.x, p.y, p.yaw) == pytest.approx((0.5, -0.5, 0.7))
        assert p.vx == pytest.approx(0.0) and p.vy == pytest.approx(0.0)
        assert p.valid


def test_mock_sine_matches_formula_and_derivative():
    src = MockSource(motion='sine', origin_xy=(0.0, 0.0), amp_xy=(1.0, 0.5), freq=0.1)
    src.set_t0(0.0)
    for tau in (0.0, 0.7, 2.3, 5.0):
        p = src.poll(tau)
        assert p.x == pytest.approx(1.0 * math.sin(2.0 * math.pi * 0.1 * tau), abs=1e-12)
        assert p.y == pytest.approx(0.5 * math.sin(math.pi * 0.1 * tau), abs=1e-12)
        assert p.yaw == pytest.approx(0.0)
        # 解析速度 vs 数值差分
        h = 1e-4
        pn = src.poll(tau + h)
        assert p.vx == pytest.approx((pn.x - p.x) / h, abs=1e-4)
        assert p.vy == pytest.approx((pn.y - p.y) / h, abs=1e-4)


def test_mock_circle_matches_formula_and_derivative():
    src = MockSource(motion='circle', origin_xy=(1.0, 1.0), radius=1.5, freq=0.1, yaw0=0.2)
    src.set_t0(0.0)
    for tau in (0.0, 1.1, 3.7):
        ph = 2.0 * math.pi * 0.1 * tau
        p = src.poll(tau)
        assert p.x == pytest.approx(1.0 + 1.5 * math.cos(ph), abs=1e-12)
        assert p.y == pytest.approx(1.0 + 1.5 * math.sin(ph), abs=1e-12)
        assert p.yaw == pytest.approx(0.2 + ph, abs=1e-12)
        h = 1e-4
        pn = src.poll(tau + h)
        assert p.vx == pytest.approx((pn.x - p.x) / h, abs=1e-4)
        assert p.vy == pytest.approx((pn.y - p.y) / h, abs=1e-4)


def test_mock_t0_defaults_to_first_poll():
    src = MockSource(motion='sine', amp_xy=(1.0, 0.0), freq=0.125)
    p0 = src.poll(100.0)
    assert p0.x == pytest.approx(0.0)          # τ=0
    p1 = src.poll(101.0)                        # τ=1 → sin(2π·0.125)=sin(π/4)
    assert p1.x == pytest.approx(math.sin(math.pi / 4.0), abs=1e-9)


def test_mock_invalid_motion():
    with pytest.raises(ValueError):
        MockSource(motion='teleport')


# --------------------------------------------------------------------------------------
# OdomTopicSource
# --------------------------------------------------------------------------------------
class _FakeQuat:
    def __init__(self, yaw: float) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = math.sin(0.5 * yaw)
        self.w = math.cos(0.5 * yaw)


class _FakeOdom:
    def __init__(self, x: float, y: float, yaw: float, vx: float = 0.0, vy: float = 0.0,
                 frame: str = 'odom', child: str = 'base_link') -> None:
        self.pose = type('P', (), {})()
        self.pose.pose = type('PP', (), {})()
        self.pose.pose.position = type('Pos', (), {'x': x, 'y': y, 'z': 0.0})()
        self.pose.pose.orientation = _FakeQuat(yaw)
        self.twist = type('T', (), {})()
        self.twist.twist = type('TT', (), {})()
        self.twist.twist.linear = type('L', (), {'x': vx, 'y': vy, 'z': 0.0})()
        self.header = type('H', (), {'frame_id': frame, 'stamp': None})()
        self.child_frame_id = child


def test_odom_source_poll_and_timeout():
    src = OdomTopicSource(timeout_s=0.5)
    assert src.poll(0.0).valid is False
    src.on_odom(_FakeOdom(1.0, 2.0, 0.5, vx=0.1, vy=-0.2), t_mono=10.0)
    p = src.poll(10.1)
    assert p.valid
    assert (p.x, p.y) == pytest.approx((1.0, 2.0))
    assert p.yaw == pytest.approx(0.5, abs=1e-9)
    assert (p.vx, p.vy) == pytest.approx((0.1, -0.2))
    assert src.frame_id == 'odom' and src.child_frame_id == 'base_link'
    # 超时失效
    assert src.poll(11.0).valid is False


def test_odom_source_rejects_malformed_msg():
    src = OdomTopicSource()
    with pytest.raises(TypeError):
        src.on_odom(object(), t_mono=0.0)


def test_quat_yaw():
    assert quat_yaw(0.0, 0.0, math.sin(0.3), math.cos(0.3)) == pytest.approx(0.6, abs=1e-9)


# --------------------------------------------------------------------------------------
# 工厂
# --------------------------------------------------------------------------------------
def test_factory_selects_source():
    assert isinstance(make_base_pose_source({'base_pose_source': 'mock'}), MockSource)
    assert isinstance(make_base_pose_source({'base_pose_source': 'odom'}), OdomTopicSource)
    with pytest.raises(ValueError):
        make_base_pose_source({'base_pose_source': 'magic'})


def test_factory_passes_mock_params():
    src = make_base_pose_source({
        'base_pose_source': 'mock',
        'mock_motion': 'circle',
        'mock_origin_xy': (1.0, 2.0),
        'mock_radius': 0.8,
        'mock_freq': 0.25,
    })
    src.set_t0(0.0)
    p = src.poll(0.0)
    assert p.x == pytest.approx(1.0 + 0.8)
    assert p.y == pytest.approx(2.0)
