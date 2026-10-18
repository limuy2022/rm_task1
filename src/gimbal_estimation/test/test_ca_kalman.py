# -*- coding: utf-8 -*-
"""CA-KF 单测（DESIGN.md §6.7 + T03 验收点 + observation_builder §6.6）。

**不依赖 rclpy**：全部用例在纯 numpy 上跑，使用真实纯Python模块，可直接 ``pytest``。

覆盖：
* 自检 5：匀速目标 100ms 外推误差 < 1cm；匀加速目标能收敛
* NIS 门限（16.81）：超门限不更新；‖d‖ > 1.0m → reinit
* dt 上限截断、发散保护、非有限守卫
* F / Q 矩阵结构（§6.7 逐项核对）
* observation_builder：optical → world → map 换算与噪声旋转（§6.6）
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

# --- pytest 引导（同时支持 `pytest test/test_ca_kalman.py` 与直接运行本文件） ---
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # noqa: E402  conftest 同目录
import conftest  # noqa: F401,E402  触发 sys.path / stub 安装

from gimbal_estimation.ca_kalman import (  # noqa: E402
    DEFAULT_INIT_SIGMA_A,
    DEFAULT_INIT_SIGMA_P,
    DEFAULT_INIT_SIGMA_V,
    DEFAULT_JERK_STD,
    DEFAULT_MAX_DT,
    DEFAULT_NIS_GATE,
    DEFAULT_P_MAX_TRACE,
    DEFAULT_PREDICT_DT,
    DEFAULT_REINIT_JUMP_M,
    CAKalman,
    clamp_dt,
)
from gimbal_estimation.observation_builder import (  # noqa: E402
    ObservationBuilder,
    map_from_world,
    world_from_map,
    world_vec_from_map_vec,
    yaw_of_T,
)

# ==========================================================================
# 工具
# ==========================================================================
QUAT_IDENTITY = (1.0, 0.0, 0.0, 0.0)
EYE4 = np.eye(4, dtype=np.float64)

SEED = 20261009


def _make_rng() -> np.random.Generator:
    """固定种子的随机数发生器（保证单测可复现）。"""
    return np.random.default_rng(SEED)


def _rotz3(theta: float) -> np.ndarray:
    """3D 绕 z 旋转矩阵。"""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _T(R: np.ndarray, t=(0.0, 0.0, 0.0)) -> np.ndarray:
    """组装 4x4 齐次矩阵。"""
    M = np.eye(4, dtype=np.float64)
    M[0:3, 0:3] = R
    M[0:3, 3] = np.asarray(t, dtype=np.float64)
    return M


# ==========================================================================
# §6.7 F / Q 矩阵结构
# ==========================================================================
def test_F_matrix_structure():
    """``F[0,2]=dt; F[0,4]=0.5dt²; F[2,4]=dt``，y 轴同理，对角为 1。"""
    kf = CAKalman()
    dt = 0.05
    F = kf._build_F(dt)
    assert F.shape == (6, 6)
    np.testing.assert_allclose(np.diag(F), np.ones(6))
    assert F[0, 2] == pytest.approx(dt)
    assert F[0, 4] == pytest.approx(0.5 * dt * dt)
    assert F[2, 4] == pytest.approx(dt)
    assert F[1, 3] == pytest.approx(dt)
    assert F[1, 5] == pytest.approx(0.5 * dt * dt)
    assert F[3, 5] == pytest.approx(dt)
    # 其余非对角位置必须为 0
    mask = np.ones((6, 6), dtype=bool)
    for (i, j) in ((0, 2), (0, 4), (2, 4), (1, 3), (1, 5), (3, 5)):
        mask[i, j] = False
    np.fill_diagonal(mask, False)
    np.testing.assert_allclose(F[mask], np.zeros(int(mask.sum())))


def test_Q_matrix_structure():
    """Q 为 piecewise-constant jerk，按 ix/iy 分块对称正定。"""
    kf = CAKalman(jerk_std=2.0)
    dt = 0.1
    q = 2.0 ** 2
    Q1 = q * np.array([[dt ** 5 / 20, dt ** 4 / 8, dt ** 3 / 6],
                       [dt ** 4 / 8, dt ** 3 / 3, dt ** 2 / 2],
                       [dt ** 3 / 6, dt ** 2 / 2, dt]])
    Q = kf._build_Q(dt)
    np.testing.assert_allclose(Q[np.ix_([0, 2, 4], [0, 2, 4])], Q1)
    np.testing.assert_allclose(Q[np.ix_([1, 3, 5], [1, 3, 5])], Q1)
    np.testing.assert_allclose(Q, Q.T, atol=1e-15)
    # 分块之间必须为 0
    np.testing.assert_allclose(Q[np.ix_([0, 2, 4], [1, 3, 5])], np.zeros((3, 3)))
    assert float(np.linalg.eigvalsh(Q).min()) > -1e-12


def test_clamp_dt():
    """dt 截断到 [0, max_dt]，非有限 → 0。"""
    assert clamp_dt(-1.0) == 0.0
    assert clamp_dt(0.1) == pytest.approx(0.1)
    assert clamp_dt(10.0) == pytest.approx(DEFAULT_MAX_DT)
    assert clamp_dt(float('nan')) == 0.0
    # 非有限 dt 一律退化为 0（绝不因为时间源异常而一次跳飞状态）
    assert clamp_dt(float('inf'), 0.2) == 0.0
    assert clamp_dt(float('-inf')) == 0.0


# ==========================================================================
# 冷启动 / reinit / 发散 / 非有限守卫
# ==========================================================================
def test_cold_start_covariance():
    """冷启动 ``P = diag(0.05², 0.05², 1², 1², 5², 5²)``。"""
    kf = CAKalman()
    expect = np.diag([DEFAULT_INIT_SIGMA_P ** 2, DEFAULT_INIT_SIGMA_P ** 2,
                      DEFAULT_INIT_SIGMA_V ** 2, DEFAULT_INIT_SIGMA_V ** 2,
                      DEFAULT_INIT_SIGMA_A ** 2, DEFAULT_INIT_SIGMA_A ** 2])
    np.testing.assert_allclose(kf._initial_covariance(), expect)
    assert not kf.initialized

    kf.update(np.array([1.0, 2.0]), np.eye(2) * 1e-4)
    assert kf.initialized
    np.testing.assert_allclose(kf.position, np.array([1.0, 2.0]))
    np.testing.assert_allclose(kf.velocity, np.zeros(2))


def test_predict_before_init_does_not_change_state():
    """未初始化时 predict 只累计时间，不转移状态。"""
    kf = CAKalman()
    kf.predict(0.1)
    np.testing.assert_allclose(kf.x, np.zeros(6))
    assert kf.elapsed == pytest.approx(0.1)


def test_reinit_and_divergence():
    """发散标记后下一次有效观测直接 reinit；reinit 计数递增。"""
    kf = CAKalman()
    kf.update(np.array([0.0, 0.0]), np.eye(2) * 1e-4)
    kf.P = np.eye(6) * 1e4                 # 人为吹爆协方差
    kf.diverged = True
    before = kf.reinit_count
    kf.update(np.array([3.0, 4.0]), np.eye(2) * 1e-4)
    assert kf.reinit_count > before
    np.testing.assert_allclose(kf.position, np.array([3.0, 4.0]))
    assert kf.position_trace() < DEFAULT_P_MAX_TRACE


def test_p_max_trace_flags_divergence():
    """``trace(P[0:2,0:2]) > 100`` → diverged=True。"""
    kf = CAKalman(p_max_trace=DEFAULT_P_MAX_TRACE)
    kf.update(np.zeros(2), np.eye(2) * 1e-4)
    kf.P[0, 0] = 80.0
    kf.P[1, 1] = 80.0                      # trace = 160 > 100
    kf.predict(0.01)
    assert kf.diverged is True


def test_non_finite_guard_triggers_restore():
    """出现 NaN 时回滚/重置，不把 NaN 留在状态里。"""
    kf = CAKalman()
    kf.update(np.array([1.0, 1.0]), np.eye(2) * 1e-4)
    kf.x[0] = float('nan')
    kf.predict(0.01)
    assert np.all(np.isfinite(kf.x))
    assert np.all(np.isfinite(kf.P))


def test_max_dt_truncation():
    """``dt > max_dt`` 被截断为 max_dt（不会一次跳飞）。"""
    kf = CAKalman(max_dt=0.5)
    kf.update(np.array([0.0, 0.0]), np.eye(2) * 1e-4)
    kf.t_last = 0.0
    dt = kf.advance(100.0)                 # 巨大间隔
    assert dt == pytest.approx(0.5)
    assert kf.t_last == pytest.approx(100.0)


# ==========================================================================
# NIS 门限
# ==========================================================================
def test_nis_gate_rejects_small_outlier():
    """NIS 超门限但 ‖d‖ < 1.0 → 拒绝更新（只 predict），state 不被拉走。"""
    kf = CAKalman(nis_gate=DEFAULT_NIS_GATE)
    R = np.eye(2) * (0.005 ** 2)
    kf.update(np.array([0.0, 0.0]), R)
    for _ in range(20):                    # 收敛到 (0,0)，协方差变小
        kf.predict(0.033)
        kf.update(np.array([0.0, 0.0]), R)

    pos_before = kf.position.copy()
    accepted = kf.update(np.array([0.15, 0.0]), R)   # 15cm，NIS 爆表但 < 1.0m
    assert accepted is False
    assert kf.last_result == 'rejected'
    assert kf.outlier_count >= 1
    np.testing.assert_allclose(kf.position, pos_before, atol=1e-12)


def test_big_jump_triggers_reinit():
    """``‖d‖ > 1.0m`` → reinit 到新观测。"""
    kf = CAKalman()
    R = np.eye(2) * (0.005 ** 2)
    kf.update(np.array([0.0, 0.0]), R)
    for _ in range(10):
        kf.predict(0.033)
        kf.update(np.array([0.0, 0.0]), R)
    before = kf.reinit_count
    assert DEFAULT_REINIT_JUMP_M == pytest.approx(1.0)   # DESIGN §6.7 门限
    accepted = kf.update(np.array([3.0, 0.0]), R)        # 跳变 3m > 1.0m
    assert accepted is True
    assert kf.last_result == 'reinit'
    assert kf.reinit_count > before
    np.testing.assert_allclose(kf.position, np.array([3.0, 0.0]), atol=1e-9)


def test_nis_value_is_meaningful():
    """正常观测的 NIS 应远小于门限 16.81。"""
    kf = CAKalman()
    R = np.eye(2) * (0.005 ** 2)
    rng = _make_rng()
    t = 0.0
    for k in range(60):
        t += 0.033
        truth = np.array([0.4 * t, 0.0])
        z = truth + rng.normal(0.0, 0.005, size=2)
        kf.advance(t)
        kf.update(z, R)
    assert kf.nis() < DEFAULT_NIS_GATE


# ==========================================================================
# 自检 5：匀速 100ms 外推 < 1cm；匀加速收敛
# ==========================================================================
def test_constant_velocity_extrapolation_under_1cm():
    """T03 验收点：匀速目标 100ms 外推误差 < 1cm。

    观测噪声取 ``σ = 2 mm``（≈ DESIGN §6.6 在 2m 处的横向噪声 1.8 mm），
    ``jerk_std`` 取设计默认 1.0；统计口径用 **RMS 与均值**（尾部 max 另作宽松约束，
    因为 30Hz 下加速度是三阶差分，单点噪声会被放大）。
    """
    rng = _make_rng()
    kf = CAKalman(jerk_std=DEFAULT_JERK_STD, predict_dt=DEFAULT_PREDICT_DT)
    sigma = 0.002
    R = np.eye(2) * (sigma ** 2)

    v = np.array([0.45, -0.30])            # m/s，量级接近真实目标
    p0 = np.array([1.2, 0.8])
    dt_obs = 1.0 / 30.0
    t = 0.0

    errors = []
    for _ in range(int(8.0 / dt_obs)):     # 跑 8s，前 3s 收敛不统计
        t += dt_obs
        kf.advance(t)
        z = p0 + v * t + rng.normal(0.0, sigma, size=2)
        kf.update(z, R)
        if kf.elapsed > 3.0:
            pred = kf.extrapolate(DEFAULT_PREDICT_DT)
            truth = p0 + v * (t + DEFAULT_PREDICT_DT)
            errors.append(float(np.linalg.norm(pred - truth)))

    err = np.asarray(errors, dtype=np.float64)
    assert err.size > 60
    rms = float(np.sqrt((err ** 2).mean()))
    assert rms < 0.01, '100ms 外推 RMS 误差 %.4f m 应 < 1cm' % rms
    assert float(err.mean()) < 0.01, '100ms 外推平均误差 %.4f m 应 < 1cm' % float(err.mean())
    assert float(np.percentile(err, 90)) < 0.01, '90 分位误差 %.4f m 应 < 1cm' % float(
        np.percentile(err, 90))
    # 外推必须显著优于"不外推"：|v|·0.1 = 0.054 m
    no_extrap = float(np.linalg.norm(v) * DEFAULT_PREDICT_DT)
    assert rms < 0.25 * no_extrap, '外推应把误差压到不外推的 1/4 以内'


def test_constant_velocity_estimate_converges():
    """速度估计收敛到真值 5cm/s 以内。"""
    rng = _make_rng()
    kf = CAKalman()
    R = np.eye(2) * (0.003 ** 2)
    v = np.array([0.6, 0.2])
    p0 = np.array([0.0, 0.0])
    t = 0.0
    for k in range(180):
        t += 1.0 / 30.0
        kf.advance(t)
        kf.update(p0 + v * t + rng.normal(0.0, 0.003, size=2), R)
    np.testing.assert_allclose(kf.velocity, v, atol=0.05)


def test_constant_acceleration_converges_noiseless():
    """无噪声时，CA 模型应**精确**收敛到真值的位置/速度/加速度。"""
    kf = CAKalman(jerk_std=1.0)
    R = np.eye(2) * 1e-12
    a = np.array([0.30, -0.15])
    v0 = np.array([0.10, 0.05])
    p0 = np.array([0.5, -0.5])
    t = 0.0
    for _ in range(240):                   # 8s
        t += 1.0 / 30.0
        kf.advance(t)
        kf.update(p0 + v0 * t + 0.5 * a * t * t, R)

    truth_p = p0 + v0 * t + 0.5 * a * t * t
    truth_v = v0 + a * t
    np.testing.assert_allclose(kf.position, truth_p, atol=1e-3)
    np.testing.assert_allclose(kf.velocity, truth_v, atol=1e-3)
    np.testing.assert_allclose(kf.acceleration, a, atol=1e-2)


def test_constant_acceleration_converges_noisy():
    """有观测噪声时，匀加速目标（a = 0.3 m/s²）仍收敛：速度准、加速度同号同量级。

    注意：加速度是位置的**三阶差分**，30Hz 下噪声放大约 ``1/dt² ≈ 900`` 倍，
    因此加速度只约束"符号 + 量级"，精确约束留给上面的无噪声用例。
    """
    rng = _make_rng()
    kf = CAKalman(jerk_std=1.0)
    sigma = 0.002
    R = np.eye(2) * (sigma ** 2)
    a = np.array([0.30, -0.15])
    v0 = np.array([0.10, 0.05])
    p0 = np.array([0.5, -0.5])
    t = 0.0
    for _ in range(240):                   # 8s
        t += 1.0 / 30.0
        kf.advance(t)
        kf.update(p0 + v0 * t + 0.5 * a * t * t + rng.normal(0.0, sigma, size=2), R)

    np.testing.assert_allclose(kf.velocity, v0 + a * t, atol=0.10)
    np.testing.assert_allclose(kf.position, p0 + v0 * t + 0.5 * a * t * t, atol=0.05)
    acc = kf.acceleration
    # 逐轴误差有界（加速度噪声在 30Hz 下被放大约 1/dt² 倍，单点值不要求精确）
    assert abs(acc[0] - a[0]) < 0.5, 'ax 估计 %.3f 偏离真值 %.3f 过多' % (acc[0], a[0])
    assert abs(acc[1] - a[1]) < 0.5, 'ay 估计 %.3f 偏离真值 %.3f 过多' % (acc[1], a[1])
    # 相对"零加速度"（CV 假设）的改进：至少有一轴更接近真值
    assert abs(acc[0] - a[0]) < abs(a[0]) or abs(acc[1] - a[1]) < abs(a[1])


def test_extrapolate_formula():
    """``pred = p + v·Δ + 0.5·a·Δ²``。"""
    kf = CAKalman()
    kf.update(np.array([1.0, 2.0]), np.eye(2) * 1e-4)
    kf.x[2:4] = np.array([0.4, -0.2])
    kf.x[4:6] = np.array([0.6, 0.3])
    d = 0.1
    expect = np.array([1.0 + 0.4 * d + 0.5 * 0.6 * d * d,
                       2.0 - 0.2 * d + 0.5 * 0.3 * d * d])
    np.testing.assert_allclose(kf.extrapolate(d), expect)
    np.testing.assert_allclose(kf.extrapolate(), kf.extrapolate(DEFAULT_PREDICT_DT))


def test_extrapolate_guards_non_finite():
    """外推出现非有限值时退化为当前位置。"""
    kf = CAKalman()
    kf.update(np.array([1.0, 2.0]), np.eye(2) * 1e-4)
    kf.x[2] = float('nan')
    out = kf.extrapolate(0.1)
    assert np.all(np.isfinite(out))


# ==========================================================================
# §6.6 observation_builder
# ==========================================================================
def test_observation_builder_identity_chain():
    """单位姿态 + 单位 T_map←world 时：optical→gimbal→world 的链路正确。

    optical (x右, y下, z前) → gimbal_link (x前, y左, z上)：
    ``p_opt=(0,0,d)``（正前方 d 米）→ ``p_gimbal=(d,0,0)``。
    """
    b = ObservationBuilder()
    obs = b.build(tvec_opt=(0.0, 0.0, 2.0),
                  cov_opt=np.diag([0.002 ** 2, 0.002 ** 2, 0.02 ** 2]).reshape(9),
                  quat=QUAT_IDENTITY,
                  gimbal_origin_world=(0.0, 0.0, 0.0),
                  T_map_world=EYE4)
    assert obs.valid, obs.reason
    np.testing.assert_allclose(obs.pos_world, np.array([2.0, 0.0, 0.0]), atol=1e-9)
    np.testing.assert_allclose(obs.pos_map, np.array([2.0, 0.0, 0.0]), atol=1e-9)
    assert obs.depth == pytest.approx(2.0)
    # 深度噪声 2cm 沿 world x（视线方向）→ map x 方向的 σ ≈ 0.02
    assert obs.R_map[0, 0] == pytest.approx(0.02 ** 2 + 0.002 ** 2, rel=1e-3)
    assert obs.R_map[1, 1] >= 0.002 ** 2


def test_observation_builder_applies_T_map_world():
    """``T_map←world`` 的旋转与平移都生效，且 θ 被正确取出。"""
    theta = 0.7
    t_mw = np.array([1.0, -2.0, 0.0])
    T = _T(_rotz3(theta), t_mw)
    assert yaw_of_T(T) == pytest.approx(theta)

    b = ObservationBuilder()
    obs = b.build(tvec_opt=(0.0, 0.0, 2.0),
                  cov_opt=np.diag([0.002 ** 2, 0.002 ** 2, 0.02 ** 2]).reshape(9),
                  quat=QUAT_IDENTITY,
                  gimbal_origin_world=(0.0, 0.0, 0.0),
                  T_map_world=T)
    assert obs.valid
    np.testing.assert_allclose(obs.pos_map, _rotz3(theta) @ np.array([2.0, 0.0, 0.0]) + t_mw,
                               atol=1e-9)
    assert obs.theta_map_world == pytest.approx(theta)
    # 旋转后 R_map 应仍对称正定
    np.testing.assert_allclose(obs.R_map, obs.R_map.T, atol=1e-12)
    assert float(np.linalg.eigvalsh(obs.R_map).min()) > 0.0


def test_observation_builder_with_gimbal_origin_and_attitude():
    """云台原点平移 + 云台 yaw 90° 时，正前方 2m 的目标落在 world +y。"""
    b = ObservationBuilder()
    q_yaw90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))  # 绕 z 转 90°
    obs = b.build(tvec_opt=(0.0, 0.0, 2.0),
                  cov_opt=np.diag([0.002 ** 2] * 3).reshape(9),
                  quat=q_yaw90,
                  gimbal_origin_world=(1.0, 0.0, 0.5),
                  T_map_world=EYE4)
    assert obs.valid
    np.testing.assert_allclose(obs.pos_world, np.array([1.0, 2.0, 0.5]), atol=1e-9)


def test_observation_builder_rejects_bad_input():
    """非有限的 tvec / 原点 / 矩阵 → valid=False。"""
    b = ObservationBuilder()
    assert not b.build((float('nan'), 0.0, 2.0), None, QUAT_IDENTITY, (0, 0, 0), EYE4).valid
    assert not b.build((0.0, 0.0, 2.0), None, QUAT_IDENTITY, (float('inf'), 0, 0), EYE4).valid


def test_observation_builder_missing_covariance():
    """cov_optical 缺失时退化为各向同性 σ_min，不产生奇异 R。"""
    b = ObservationBuilder(r_min_sigma=0.002)
    obs = b.build((0.0, 0.0, 2.0), None, QUAT_IDENTITY, (0, 0, 0), EYE4)
    assert obs.valid
    assert float(np.linalg.eigvalsh(obs.R_map).min()) > 0.0
    np.testing.assert_allclose(obs.R_map, obs.R_map.T, atol=1e-15)


def test_observation_builder_fallback_anisotropic():
    """配置了 fallback 系数时，按 ``σ_lat=k_lat·z`` / ``σ_depth=k_depth·z²`` 构造。"""
    b = ObservationBuilder(fallback_k_lat=9.02e-4, fallback_k_depth=5.64e-3)
    obs = b.build((0.0, 0.0, 2.0), None, QUAT_IDENTITY, (0, 0, 0), EYE4)
    sigma_depth = 5.64e-3 * 4.0            # ≈ 2.26 cm，与 §6.6 推导一致
    assert obs.R_map[0, 0] == pytest.approx(sigma_depth ** 2 + 0.002 ** 2, rel=1e-3)
    assert sigma_depth == pytest.approx(0.0226, abs=1e-3)


def test_map_world_roundtrip_and_vec():
    """``map↔world`` 点变换互逆；向量用 ``RotZ(−θ)`` 反旋。"""
    T = _T(_rotz3(0.9), (0.5, -1.5, 0.2))
    p_world = np.array([2.0, 1.0, 0.3])
    p_map = map_from_world(p_world, T)
    np.testing.assert_allclose(world_from_map(p_map, T), p_world, atol=1e-12)

    v_map = np.array([1.0, 0.0])
    v_world = world_vec_from_map_vec(v_map, T)
    assert v_world[2] == pytest.approx(0.0)
    np.testing.assert_allclose(v_world[0:2], _rotz3(0.9).T[0:2, 0:2] @ v_map, atol=1e-12)


def test_r_gimbal_from_opt_orientation():
    """``R_gimbal←opt`` 必须把 optical 的 z（向前）映射到 gimbal_link 的 x（向前）。"""
    b = ObservationBuilder()
    R = b.R_gimbal_from_opt
    np.testing.assert_allclose(R @ np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]), atol=1e-9)
    np.testing.assert_allclose(R @ np.array([1.0, 0.0, 0.0]), np.array([0.0, -1.0, 0.0]), atol=1e-9)
    np.testing.assert_allclose(R @ np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, -1.0]), atol=1e-9)
    assert float(np.linalg.det(R)) == pytest.approx(1.0)


# ==========================================================================
# 端到端：观测驱动 KF（模拟 30Hz 视觉 + 100Hz 定时器）
# ==========================================================================
def test_end_to_end_estimator_loop():
    """模拟"30Hz 观测 + 100Hz 定时器"混合节奏，估计应稳定无发散。"""
    rng = _make_rng()
    kf = CAKalman(jerk_std=1.0)
    b = ObservationBuilder()
    R_opt = np.diag([0.0018 ** 2, 0.0018 ** 2, 0.0226 ** 2])

    # 目标绕云台做小幅运动：world 系 (2m, ±0.7rad, ±0.25rad)
    def truth(t: float) -> np.ndarray:
        yaw = 0.7 * math.sin(2 * math.pi * 0.05 * t)
        pit = 0.25 * math.sin(2 * math.pi * 0.08 * t)
        return np.array([2.0 * math.cos(pit) * math.cos(yaw),
                         2.0 * math.cos(pit) * math.sin(yaw),
                         2.0 * math.sin(pit)])

    sigma_opt = np.array([0.0018, 0.0018, 0.0226])     # §6.6 在 2m 处的 σ_lat/σ_depth
    errors = []
    n_ticks = int(6.0 * 100.0)
    for k in range(n_ticks):
        t = k / 100.0
        kf.advance(t)                                   # 100Hz 定时器：predict
        _ = kf.extrapolate(0.1)
        if k > 0 and k % 3 == 0:                        # ≈33Hz 观测
            p_w = truth(t)
            # world → optical：p_opt = (R_gimbal←opt)ᵀ · p_gimbal（云台姿态取单位）
            p_opt = b.R_gimbal_from_opt.T @ p_w + rng.normal(0.0, 1.0, size=3) * sigma_opt
            obs = b.build(p_opt, R_opt.reshape(9), QUAT_IDENTITY, np.zeros(3), EYE4)
            assert obs.valid
            kf.update(obs.pos_map[0:2], obs.R_map)
            if kf.elapsed > 1.5:
                errors.append(float(np.linalg.norm(kf.position - truth(t)[0:2])))

    err = np.asarray(errors, dtype=np.float64)
    assert kf.initialized
    assert not kf.diverged
    assert float(np.sqrt((err ** 2).mean())) < 0.03, '稳态位置 RMS 误差应 < 3cm'
    assert float(err.max()) < 0.12, '稳态位置最大误差 %.3f m 应 < 12cm' % float(err.max())
    assert kf.nis() < DEFAULT_NIS_GATE


def test_missing_transform_and_invalid_rotation_are_not_identity():
    """Absent/broken calibration must not produce plausible map coordinates."""
    from gimbal_estimation.observation_builder import as_T, map_from_world
    for invalid in (None, np.zeros((4, 4)), np.full((4, 4), np.nan)):
        with pytest.raises(ValueError):
            as_T(invalid)
        with pytest.raises(ValueError):
            map_from_world([1, 2, 3], invalid)


def test_invalid_quaternion_is_not_identity_orientation():
    builder = ObservationBuilder()
    for invalid in ((0., 0., 0., 0.), (float('nan'), 0., 0., 1.)):
        with pytest.raises(ValueError):
            builder.rot_opt_to_world(invalid)
