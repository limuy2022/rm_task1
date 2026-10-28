"""``team43_debug.metrics`` 单测（DESIGN §3.12 / PRD §6.1 / hardware_spec §5）。

纯 numpy + `team43_common.frames`（本身就是纯 numpy），**不 import rclpy**，可直接跑::

    python -m pytest src/team43_debug/test -q

相机内参一律从 ``team43_bringup/config/virtual.yaml`` 读取（单一数据源），
本文件**不二次手抄**任何内参 / 外参字面量（DESIGN §8.6）。
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
sys.path.insert(0, os.path.join(_SRC_DIR, 'team43_common'))

from team43_common import config_loader as CL  # noqa: E402
from team43_common import frames as FR  # noqa: E402
from team43_debug import metrics as M  # noqa: E402

VIRTUAL_YAML = os.path.join(_WS_DIR, 'src', 'team43_bringup',
                            'config', 'virtual.yaml')
_CAM = CL.load_yaml(VIRTUAL_YAML, required=True).get('camera', {})
HALF_FOV_RAD = math.atan((float(_CAM['width']) * 0.5) / float(_CAM['fx']))

P_GIMBAL = np.zeros(3, dtype=np.float64)


def _quat(yaw: float = 0.0, pitch_proto: float = 0.0) -> np.ndarray:
    """由协议域 ``(yaw, pitch)`` 造 ``GimbalState.quat``（world→gimbal_link）。"""
    return FR.quat_from_mat(FR.compose_yaw_pitch(yaw, pitch_proto))


# ===========================================================================
# 1. 光轴与角度误差
# ===========================================================================
def test_optical_axis_matches_frames_single_source():
    """光轴 = R_world←gimbal · R_gimbal←opt · [0,0,1]；零姿态时 = gimbal +x。"""
    axis = M.optical_axis_world(_quat())
    assert axis is not None
    assert np.allclose(axis, np.array([1.0, 0.0, 0.0]), atol=1e-12)
    # R_gimbal←opt · (0,0,1) 恰好是 gimbal +x（由 frames 的单一真源保证）
    assert np.allclose(FR.R_opt_from_link() @ np.array([0.0, 0.0, 1.0]),
                       np.array([1.0, 0.0, 0.0]), atol=1e-12)
    # 显式注入等价
    axis2 = M.optical_axis_world(_quat(), FR.R_opt_from_link())
    assert np.allclose(axis, axis2)


def test_optical_axis_rotates_with_yaw():
    """yaw = +90° → 光轴指向 world +y（绕 +z 逆时针为正）。"""
    axis = M.optical_axis_world(_quat(yaw=math.pi / 2.0))
    assert axis is not None
    assert np.allclose(axis, np.array([0.0, 1.0, 0.0]), atol=1e-9)


def test_optical_axis_pitch_positive_looks_down():
    """协议域 pitch 为正 = 低头 → 光轴 z 分量为负。"""
    p = 0.3
    axis = M.optical_axis_world(_quat(pitch_proto=p))
    assert axis is not None
    assert axis[2] == pytest.approx(-math.sin(p), abs=1e-9)
    assert axis[0] == pytest.approx(math.cos(p), abs=1e-9)


def test_angle_error_zero_when_on_axis():
    """目标正好在光轴上 → e_ang = 0°。"""
    for yaw, pitch in ((0.0, 0.0), (0.7, 0.25), (-1.1, -0.3)):
        q = _quat(yaw, pitch)
        axis = M.optical_axis_world(q)
        target = P_GIMBAL + axis * 2.0            # 2m 处、正好沿光轴
        assert M.angle_error_deg(target, P_GIMBAL, q) == pytest.approx(0.0, abs=1e-6)


def test_angle_error_orthogonal_is_90deg():
    """目标与光轴垂直 → 90°。"""
    q = _quat()
    assert M.angle_error_deg(np.array([0.0, 2.0, 0.0]), P_GIMBAL, q) == \
        pytest.approx(90.0, abs=1e-9)
    assert M.angle_error_deg(np.array([0.0, 0.0, 2.0]), P_GIMBAL, q) == \
        pytest.approx(90.0, abs=1e-9)


def test_angle_error_known_offset_in_yaw():
    """目标相对光轴偏 10° → e_ang = 10°。"""
    q = _quat()
    off = math.radians(10.0)
    target = np.array([2.0 * math.cos(off), 2.0 * math.sin(off), 0.0])
    assert M.angle_error_deg(target, P_GIMBAL, q) == pytest.approx(10.0, abs=1e-6)


def test_angle_error_uses_gimbal_origin_offsets():
    """云台原点非零时：误差按「目标相对云台原点的方向」算，与纯距离无关。"""
    q = _quat(yaw=math.pi / 2.0)                       # 光轴指向 +y
    origin = np.array([10.0, 0.0, 0.0])
    # 目标在云台正 +y 方向 2m → 0°
    assert M.angle_error_deg(origin + np.array([0.0, 2.0, 0.0]), origin, q) == \
        pytest.approx(0.0, abs=1e-6)
    # 目标在 +x 方向 5m → 90°
    assert M.angle_error_deg(origin + np.array([5.0, 0.0, 0.0]), origin, q) == \
        pytest.approx(90.0, abs=1e-6)


def test_angle_error_degenerate_returns_nan():
    """真值=云台原点（零方向）或非法四元数 → NaN，不抛异常。"""
    assert math.isnan(M.angle_error_deg(P_GIMBAL, P_GIMBAL, _quat()))
    assert math.isnan(M.angle_error_deg(np.array([np.nan, 0.0, 0.0]), P_GIMBAL, _quat()))


def test_optical_axis_accepts_2d_and_3d():
    """2 维输入补 0 处理，不抛异常。"""
    assert M.optical_axis_world([1.0, 0.0, 0.0, 0.0]) is not None
    assert M.target_direction([1.0, 1.0], [0.0, 0.0]) is not None


# ===========================================================================
# 2. 统计函数
# ===========================================================================
def test_rms_max_mean_known_values():
    """RMS/max/mean 与手算一致；非有限值被剔除。"""
    vals = [1.0, 2.0, 3.0, 4.0]
    assert M.rms_deg(vals) == pytest.approx(math.sqrt(30.0 / 4.0))
    assert M.max_deg(vals) == pytest.approx(4.0)
    assert M.mean_deg(vals) == pytest.approx(2.5)
    assert M.rms_deg([float('nan'), 3.0, 4.0]) == pytest.approx(math.sqrt(12.5))
    assert math.isnan(M.rms_deg([]))
    assert math.isnan(M.max_deg([float('nan')]))
    assert math.isnan(M.mean_deg([]))


def test_swing_wraps_around_pi():
    """跨 ±π 的方位角序列：解缠后 170° → 190°(= −170°)，摆幅 20°。"""
    b = [math.radians(170.0), math.radians(-170.0), math.radians(175.0)]
    assert M.swing_rad(b) == pytest.approx(math.radians(20.0), abs=1e-12)
    assert M.swing_deg(b) == pytest.approx(20.0, abs=1e-9)
    # 未解缠时 max−min 会得到 345°（175 − (−170)），是错的
    naive = math.degrees(max(b) - min(b))
    assert naive == pytest.approx(345.0, abs=1e-9)
    assert M.swing_deg([0.1]) == pytest.approx(0.0)
    assert M.swing_deg([]) == pytest.approx(0.0)


def test_bearing_azimuth():
    """方位角 = atan2(dy, dx)。"""
    assert M.bearing_azimuth_rad([2.0, 0.0, 0.0], [0.0, 0.0, 0.0]) == pytest.approx(0.0)
    assert M.bearing_azimuth_rad([0.0, 2.0, 0.0], [0.0, 0.0, 0.0]) == \
        pytest.approx(math.pi / 2.0)


# ===========================================================================
# 3. 收敛时间
# ===========================================================================
def test_convergence_time_detects_first_hold():
    """0~0.4s 大误差，之后都在 2° 内 → 收敛时刻 = 0.4s。"""
    t = [round(0.01 * k, 4) for k in range(1, 401)]          # 0.01 .. 4.00
    e = [5.0 if x < 0.4 else 0.5 for x in t]
    tc, ok = M.convergence_time(t, e, window_s=2.0, tol_deg=2.0)
    assert ok is True
    assert tc == pytest.approx(0.4, abs=0.02)


def test_convergence_time_not_converged():
    """误差始终超容差 → 未收敛。"""
    t = [0.01 * k for k in range(1, 401)]
    e = [9.0] * len(t)
    tc, ok = M.convergence_time(t, e, window_s=2.0, tol_deg=2.0)
    assert ok is False and tc is None


def test_convergence_time_insufficient_tail():
    """数据总长不足「保持窗口」→ 无法判定，返回 False（不误报收敛）。"""
    t = [0.0, 0.1, 0.2]
    e = [0.1, 0.1, 0.1]
    tc, ok = M.convergence_time(t, e, window_s=2.0, tol_deg=2.0)
    assert ok is False and tc is None


def test_convergence_time_returns_earliest_not_latest():
    """中途短暂超标（仍在窗口内已满足）→ 返回最早的满足点。"""
    # 0~0.2 超差，0.3 起全部 0.5°，窗口 1.0s
    t = [round(0.1 * k, 4) for k in range(0, 40)]
    e = [5.0 if x < 0.3 else 0.5 for x in t]
    tc, ok = M.convergence_time(t, e, window_s=1.0, tol_deg=2.0)
    assert ok is True
    assert tc == pytest.approx(0.3, abs=0.02)


def test_convergence_time_empty():
    """空输入不抛异常。"""
    assert M.convergence_time([], []) == (None, False)


# ===========================================================================
# 4. evaluate 主入口
# ===========================================================================
def _samples_from(t: list, e: list, bearings: list = None,
                  est: list = None, status: list = None):
    return M.samples_from_arrays(t, e, bearings, est, status)


def test_evaluate_pass_case():
    """收敛后误差很小 → PASS，且 RMS 只统计 t_rel ≥ 2.0s 的段。"""
    n = 800
    t = [0.02 * k for k in range(n)]                     # 16s @50Hz
    e = [8.0 if x < 1.0 else 0.8 for x in t]             # 1.0s 收敛，之后 0.8°
    res = M.evaluate(_samples_from(t, e), M.EvalThresholds(post_convergence_skip_s=2.0))
    assert res.valid is True
    assert res.converged is True
    assert res.convergence_time_s == pytest.approx(1.0, abs=0.05)
    assert res.rms_deg == pytest.approx(0.8, abs=1e-9)   # 前 2s 的超差段被排除
    assert res.max_deg == pytest.approx(0.8, abs=1e-9)
    assert res.passed is True
    assert res.n_samples == n
    assert res.n_used == sum(1 for x in t if x >= 2.0 - 1e-12)


def test_evaluate_rms_fails_when_window_has_outliers():
    """收敛段之内仍有 >2° 的样本（t_rel ≥ 2s）→ RMS 超限 → FAIL。"""
    t = [0.02 * k for k in range(800)]
    e = [1.0] * len(t)
    for k in range(200, 800, 7):                          # t_rel ≥ 4s 起周期性毛刺
        e[k] = 6.0
    res = M.evaluate(_samples_from(t, e))
    assert res.max_deg == pytest.approx(6.0)
    assert res.rms_deg > 2.0
    assert res.passed is False


def test_evaluate_invalid_on_too_few_samples():
    """样本数 < min_samples → invalid，且 reason 说明。"""
    t = [0.02 * k for k in range(10)]
    e = [0.1] * 10
    res = M.evaluate(_samples_from(t, e), M.EvalThresholds(min_samples=50))
    assert res.valid is False
    assert '样本数' in res.reason
    assert res.passed is False


def test_evaluate_require_swing_invalid():
    """require_swing=True 且摆幅 < 半视场 → invalid（hardware_spec §5）。"""
    n = 800
    t = [0.02 * k for k in range(n)]
    e = [0.5] * n
    tiny = [math.radians(2.0)] * n                        # 摆幅 2° ≪ 半视场
    th = M.EvalThresholds(require_swing=True, half_fov_rad=HALF_FOV_RAD)
    res = M.evaluate(_samples_from(t, e, tiny), th)
    assert res.valid is False
    assert '摆幅' in res.reason
    assert res.swing_deg == pytest.approx(0.0, abs=1e-9)

    # 摆幅超过半视场 → valid 恢复
    big = [math.radians(-40.0), math.radians(30.0)] * (n // 2)
    res2 = M.evaluate(_samples_from(t, e, big), th)
    assert res2.valid is True
    assert res2.swing_deg > math.degrees(HALF_FOV_RAD)


def test_evaluate_no_samples():
    """无样本 → valid False 且 reason 明确，不抛异常。"""
    res = M.evaluate([])
    assert res.valid is False and res.n_samples == 0
    assert '无有效' in res.reason
    assert res.as_dict()['rms_deg'] is None


def test_evaluate_short_data_falls_back_to_full_window():
    """数据比跳过段还短 → RMS 退化为全段统计，并在 reason 中说明。"""
    t = [0.0, 0.5, 1.0]
    e = [1.0, 1.0, 1.0]
    res = M.evaluate(_samples_from(t, e),
                     M.EvalThresholds(min_samples=1, post_convergence_skip_s=2.0))
    assert res.n_used == 3
    assert res.rms_deg == pytest.approx(1.0)
    assert '退化' in res.reason


def test_evaluate_est_err_and_track_status_hist():
    """估计误差与 track_status 直方图被正确统计。"""
    n = 100
    t = [0.02 * k for k in range(n)]
    e = [0.5] * n
    est = [0.1] * 60 + [0.3] * 40
    st = [1] * 70 + [2] * 20 + [0] * 10
    res = M.evaluate(_samples_from(t, e, None, est, st),
                     M.EvalThresholds(min_samples=1, post_convergence_skip_s=0.0))
    assert res.est_err_mean_m == pytest.approx(0.18, abs=1e-9)
    assert res.est_err_max_m == pytest.approx(0.3, abs=1e-9)
    assert res.track_status_hist == {1: 70, 2: 20, 0: 10}


def test_bucket_rms_segments():
    """分时段统计：每段样本数与 RMS 正确。"""
    t = [0.5 * k for k in range(20)]                      # 0 .. 9.5s
    e = list(t)                                           # e_ang 数值 = 时刻（便于断言）
    buckets = M.bucket_rms(_samples_from(t, e), bucket_s=5.0)
    assert len(buckets) == 2
    assert buckets[0]['n'] == 10 and buckets[0]['t_start'] == pytest.approx(0.0)
    assert buckets[1]['n'] == 10 and buckets[1]['t_start'] == pytest.approx(5.0)
    assert buckets[0]['max_deg'] == pytest.approx(4.5)


# ===========================================================================
# 5. 报告渲染
# ===========================================================================
def test_format_report_contains_key_fields():
    """报告含结论表、判据口径、PASS/FAIL 与元信息。"""
    t = [0.02 * k for k in range(800)]
    e = [8.0 if x < 1.0 else 0.8 for x in t]
    samples = _samples_from(t, e, [math.radians(x) for x in range(800)])
    th = M.EvalThresholds()
    res = M.evaluate(samples, th)
    md = M.format_report_md(res, th, meta={'run_id': 'latest', 'gt_topic': 'X'},
                            buckets=M.bucket_rms(samples, 5.0), notes=['note-1'])
    assert 'eval_report.md' in md
    assert '角度 RMS' in md and '角度 max' in md and '收敛时间' in md
    assert 'PASS' in md
    assert 'R_world←gimbal' in md
    assert '| run_id | latest |' in md
    assert 'note-1' in md
    assert '分时段 RMS' in md


def test_format_report_fail_marks_cross():
    """未达标时报告出现 ❌ 且汇总为 FAIL。"""
    t = [0.02 * k for k in range(800)]
    e = [9.0] * 800
    res = M.evaluate(_samples_from(t, e))
    md = M.format_report_md(res, M.EvalThresholds())
    assert 'FAIL' in md
    assert '❌' in md                       # RMS / max / 收敛三项都不达标
    assert md.count('❌') >= 3
    assert 'n/a' in md                      # 收敛时间未判定 → n/a


def test_format_report_handles_nan_fields():
    """空结果渲染不抛异常，非有限值显示 n/a。"""
    md = M.format_report_md(M.evaluate([]), M.EvalThresholds())
    assert 'n/a' in md
    assert 'FAIL' in md


# ===========================================================================
# 6. 与独立实现交叉验证
# ===========================================================================
def test_angle_error_cross_checked_with_explicit_math():
    """用显式公式独立复算 e_ang，与 metrics 一致（避免"自证"）。"""
    rng = np.random.default_rng(20261010)
    for _ in range(200):
        yaw = float(rng.uniform(-2.0, 2.0))
        pitch = float(rng.uniform(-0.5, 0.5))
        R_wg = FR.compose_yaw_pitch(yaw, pitch)
        R_go = FR.R_opt_from_link()
        axis = R_wg @ R_go @ np.array([0.0, 0.0, 1.0])
        axis = axis / np.linalg.norm(axis)
        tgt = rng.normal(size=3)
        d = tgt / np.linalg.norm(tgt)
        expect = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(axis, d))))))
        got = M.angle_error_deg(tgt, np.zeros(3), FR.quat_from_mat(R_wg))
        assert got == pytest.approx(expect, abs=1e-9)


def test_samples_helpers_roundtrip():
    """samples_from_arrays / values_from_samples 往返一致，缺省字段填 NaN/-1。"""
    s = M.samples_from_arrays([1.0, 2.0], [0.5, 0.6])
    assert len(s) == 2
    assert math.isnan(s[0].bearing_rad) and s[0].track_status == -1
    ts, es = M.values_from_samples(s)
    assert ts == [1.0, 2.0] and es == [0.5, 0.6]
    # 非有限值被剔除
    s2 = M.samples_from_arrays([1.0, float('nan')], [0.5, 0.6])
    assert M.values_from_samples(s2) == ([1.0], [0.5])


def test_eval_result_as_dict_json_safe():
    """as_dict 结果可 JSON 序列化，NaN 转 None。"""
    import json
    res = M.evaluate(_samples_from([0.0, 0.1], [1.0, 1.0]),
                     M.EvalThresholds(min_samples=1))
    d = res.as_dict()
    assert d['n_samples'] == 2
    assert json.dumps(d, allow_nan=False)
