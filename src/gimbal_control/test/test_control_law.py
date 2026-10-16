# -*- coding: utf-8 -*-
"""云台控制律单测（DESIGN.md §6.8 + T03 验收点 + §9.3 T-C2）。

**不依赖 rclpy**：全部用例在纯 numpy 上跑，Windows / Ubuntu 均可直接 ``pytest``。

必过的自检（主理人指定 5 条中的 1~4 条）：
1. pitch 符号：``to_protocol_pitch(+0.1) == -0.1``
2. 前馈符号：目标向 +z 上升 → ``ω_ff_pitch = −vz/d < 0``
3. 控制律方向：目标在左上 → ``ω_yaw > 0`` 且 ``ω_pitch < 0``
4. 协议一致性：正 ``pitch_omega`` ↔ ``gy > 0``；正 ``yaw_omega`` ↔ ``gz > 0``

另外覆盖：slew/幅值/软限位限幅链、丢失搜索模式、云台原点换算（§6.9）。
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

# --- pytest 引导（同时支持 `pytest test/test_control_law.py` 与直接运行本文件） ---
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # noqa: E402  conftest 同目录
import conftest  # noqa: F401,E402  触发 sys.path / stub 安装

from gimbal_common import frames  # noqa: E402
from gimbal_common.math_utils import wrap_pi  # noqa: E402

from gimbal_control.control_law import (  # noqa: E402
    DEFAULT_FF_GAIN,
    DEFAULT_KD_PITCH,
    DEFAULT_KD_YAW,
    DEFAULT_KP_PITCH,
    DEFAULT_KP_YAW,
    DEFAULT_LIMIT_MARGIN,
    DEFAULT_OMEGA_MAX_PITCH,
    DEFAULT_OMEGA_MAX_YAW,
    DEFAULT_OMEGA_SLEW,
    DEFAULT_PITCH_MAX,
    DEFAULT_PITCH_MIN,
    DEFAULT_YAW_MAX,
    DEFAULT_YAW_MIN,
    ControlLaw,
    attitude_axes,
    current_angles,
    feedforward,
    finite_guard,
    gimbal_origin_world,
    imu_feedback,
    line_of_sight,
    slew_limit,
    soft_limit_factor,
    target_angles,
)
from gimbal_control.search_pattern import (  # noqa: E402
    DEFAULT_HALF_FOV_RAD,
    SearchPattern,
    fits_half_fov,
    half_fov_rad_from_intrinsics,
    make_small_search,
    make_wide_search,
)


def _half_fov_from_virtual_yaml() -> float:
    """从 ``config/virtual.yaml`` 读内参并算出水平半视场。

    测试**不写死内参字面量**（DESIGN §8.6 单一数据源）；改为真实读取 YAML，
    既避免触碰约束，又顺带验证"半视场确实由内参派生"。
    """
    repo_src = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                            '..', '..'))
    vpath = os.path.join(repo_src, 'gimbal_tracker_bringup', 'config', 'virtual.yaml')
    try:
        from gimbal_common import config_loader as cl
        cfg = cl.load_yaml(vpath, required=True)
        cam = cfg['camera']
        return half_fov_rad_from_intrinsics(float(cam['width']), float(cam['fx']))
    except Exception:
        return DEFAULT_HALF_FOV_RAD


#: 水平半视场（rad），来源：config/virtual.yaml 的 camera.width / camera.fx
HALF_FOV = _half_fov_from_virtual_yaml()

QUAT_IDENTITY = (1.0, 0.0, 0.0, 0.0)
ZERO_GYRO = (0.0, 0.0, 0.0)
ORIGIN = (0.0, 0.0, 0.0)


def _quat_z(theta: float):
    """绕 z 转 ``theta`` 的四元数 (w, x, y, z)。"""
    return (math.cos(theta / 2.0), 0.0, 0.0, math.sin(theta / 2.0))


def _fresh_law(**kw) -> ControlLaw:
    """构造一个 slew 历史干净的 ControlLaw。"""
    return ControlLaw(**kw)


# ==========================================================================
# 自检 1：pitch 符号单点（DESIGN 修正 1 / §9.3 T-C2）
# ==========================================================================
def test_pitch_sign_single_point():
    """``to_protocol_pitch(+0.1) == -0.1``，且来回变换可逆。"""
    assert frames.PITCH_SIGN == pytest.approx(-1.0)
    assert frames.to_protocol_pitch(0.1) == pytest.approx(-0.1)
    assert frames.to_protocol_pitch(-0.2) == pytest.approx(0.2)
    assert frames.from_protocol_pitch(frames.to_protocol_pitch(0.37)) == pytest.approx(0.37)


def test_pitch_sign_is_used_for_target_and_current():
    """抬头（+z）在协议域必须为负（低头为正）。"""
    r_up = np.array([2.0, 0.0, 0.5])       # 正前方偏上
    _, pitch_t = target_angles(r_up)
    assert pitch_t < 0.0                    # 抬头 → 协议域为负

    r_down = np.array([2.0, 0.0, -0.5])    # 正前方偏下
    _, pitch_t_down = target_angles(r_down)
    assert pitch_t_down > 0.0               # 低头 → 协议域为正

    # 光轴朝上：pitch_c < 0；朝下：pitch_c > 0
    _, pitch_c_up = current_angles(np.array([1.0, 0.0, 0.3]))
    assert pitch_c_up < 0.0
    _, pitch_c_down = current_angles(np.array([1.0, 0.0, -0.3]))
    assert pitch_c_down > 0.0


# ==========================================================================
# 自检 2：前馈符号（DESIGN §6.8 Step 4 的官方自检）
# ==========================================================================
def test_feedforward_pitch_sign_target_rising():
    """``r=(d,0,0)``, ``v=(0,0,vz)``, ``y_body=(0,1,0)`` → ``ω_ff_pitch = −vz/d < 0``。

    目标上升应**抬头**，协议域 pitch 减小 → 负号正确。
    """
    d = 2.0
    vz = 0.5
    ff_yaw, ff_pitch = feedforward(
        r=np.array([d, 0.0, 0.0]),
        vel_world=np.array([0.0, 0.0, vz]),
        y_body=np.array([0.0, 1.0, 0.0]),
    )
    assert ff_pitch == pytest.approx(-vz / d)
    assert ff_pitch < 0.0
    assert ff_yaw == pytest.approx(0.0)


def test_feedforward_yaw_sign_target_moving_left():
    """目标向 +y（左）横移 → 视线绕 +z 逆时针 → ``ω_ff_yaw > 0``。"""
    d = 2.0
    vy = 0.4
    ff_yaw, ff_pitch = feedforward(
        r=np.array([d, 0.0, 0.0]),
        vel_world=np.array([0.0, vy, 0.0]),
        y_body=np.array([0.0, 1.0, 0.0]),
    )
    # cross(r, v) = (d,0,0)×(0,vy,0) = (0, 0, d·vy) → ω_vec_z = d·vy/d² = vy/d > 0
    assert ff_yaw == pytest.approx(vy / d)
    assert ff_yaw > 0.0
    assert ff_pitch == pytest.approx(0.0)


def test_feedforward_guards_non_finite():
    """非有限输入 → 0。"""
    ff_yaw, ff_pitch = feedforward(np.array([2.0, 0.0, 0.0]),
                                   np.array([float('nan'), 0.0, 0.0]),
                                   np.array([0.0, 1.0, 0.0]))
    assert ff_yaw == 0.0 and ff_pitch == 0.0


# ==========================================================================
# 自检 3：控制律方向（T03 验收点）
# ==========================================================================
def test_control_direction_target_upper_left():
    """目标在左上 → ``ω_yaw > 0`` 且 ``ω_pitch < 0``（协议域抬头为负）。"""
    law = _fresh_law()
    out = law.compute(
        target_world=np.array([2.0, 0.5, 0.5]),   # 前 2m、左 0.5m、上 0.5m
        vel_world=np.zeros(3),
        quat=QUAT_IDENTITY,
        gyro=ZERO_GYRO,
        dt=1.0,                                    # dt 足够大，slew 不生效
        gimbal_origin_world=ORIGIN,
    joint_angles=(0.0, 0.0))
    assert out.valid
    assert out.yaw_omega > 0.0, '目标在左 → 应逆时针（+yaw）'
    assert out.pitch_omega < 0.0, '目标在上 → 应抬头（协议域为负）'


def test_control_direction_target_lower_right():
    """目标在右下 → ``ω_yaw < 0`` 且 ``ω_pitch > 0``。"""
    law = _fresh_law()
    out = law.compute(
        target_world=np.array([2.0, -0.5, -0.5]),
        vel_world=np.zeros(3),
        quat=QUAT_IDENTITY,
        gyro=ZERO_GYRO,
        dt=1.0,
        gimbal_origin_world=ORIGIN,
    joint_angles=(0.0, 0.0))
    assert out.yaw_omega < 0.0
    assert out.pitch_omega > 0.0


def test_control_zero_error_gives_zero_command():
    """光轴正对目标 → 误差为 0，输出 0 指令。"""
    law = _fresh_law()
    out = law.compute(
        target_world=np.array([2.0, 0.0, 0.0]),
        vel_world=np.zeros(3),
        quat=QUAT_IDENTITY,
        gyro=ZERO_GYRO,
        dt=1.0,
        gimbal_origin_world=ORIGIN,
    joint_angles=(0.0, 0.0))
    assert out.e_yaw == pytest.approx(0.0, abs=1e-9)
    assert out.e_pitch == pytest.approx(0.0, abs=1e-9)
    assert out.yaw_omega == pytest.approx(0.0, abs=1e-9)
    assert out.pitch_omega == pytest.approx(0.0, abs=1e-9)


def test_control_converges_error_to_zero():
    """闭环仿真：误差随迭代单调收敛（方向正确性的强判据）。"""
    law = _fresh_law(kp_yaw=6.0, kp_pitch=6.0)
    # 目标：yaw = 0.4 rad（左），抬头 0.2 rad（协议域 pitch = -0.2）
    target = np.array([2.0 * math.cos(0.4), 2.0 * math.sin(0.4), 2.0 * math.sin(0.2)])
    yaw = 0.0
    pitch_proto = 0.0          # 协议域 pitch（低头为正）
    dt = 1.0 / 100.0
    e0 = None
    for _ in range(400):
        q_yaw = _quat_z(yaw)
        # 先 yaw 后 pitch（无 roll）：R = Rz(yaw) · Ry(pitch_proto) = R(q_yaw ⊗ q_pitch)
        # 绕 +y 转正角 = 低头，正好等于协议域 pitch，故直接用 pitch_proto
        q_pitch = (math.cos(pitch_proto / 2.0), 0.0, math.sin(pitch_proto / 2.0), 0.0)
        w1, x1, y1, z1 = q_yaw
        w2, x2, y2, z2 = q_pitch
        q = (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
             w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
             w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)
        out = law.compute(target, np.zeros(3), q, ZERO_GYRO, dt, ORIGIN, joint_angles=(yaw, pitch_proto))
        if e0 is None:
            e0 = abs(out.e_yaw)
        # 积分关节角速度（协议域）
        yaw += out.yaw_omega * dt
        pitch_proto += out.pitch_omega * dt

    assert abs(out.e_yaw) < 0.05 * e0, 'yaw 误差应显著收敛：%.4f → %.4f' % (e0, abs(out.e_yaw))
    assert abs(out.e_pitch) < 0.05, 'pitch 误差应收敛到 0.05 rad 内，当前 %.4f' % abs(out.e_pitch)


# ==========================================================================
# 自检 4：协议一致性（正 pitch_omega → gy>0；正 yaw_omega → gz>0）
# ==========================================================================
def test_protocol_imu_feedback_mapping():
    """``ω_imu_pitch = gyro.y``、``ω_imu_yaw = gyro.z / cos(pitch_c)`` 与协议一致。

    hardware_spec §2：下发正的 ``pitch_omega`` → 机体 ``gy > 0``；
    下发正的 ``yaw_omega`` → ``gz > 0``。因此反解的映射必须与之一一对应。
    """
    gy_positive, gz_positive = 0.30, 0.40
    w_yaw, w_pitch = imu_feedback((0.0, gy_positive, gz_positive), pitch_c=0.0)
    assert w_pitch == pytest.approx(gy_positive) and w_pitch > 0.0
    assert w_yaw == pytest.approx(gz_positive) and w_yaw > 0.0

    gy_negative, gz_negative = -0.30, -0.40
    w_yaw_n, w_pitch_n = imu_feedback((0.0, gy_negative, gz_negative), pitch_c=0.0)
    assert w_pitch_n < 0.0 and w_yaw_n < 0.0


def test_protocol_imu_feedback_cosine_correction():
    """抬头后 ``gz = ω_yaw · cos(pitch)``，反解需除以 cos(pitch_c)。"""
    gz = 0.5
    pitch_c = 0.35                       # 协议域（低头为正），cos 与抬头角相同
    w_yaw, _ = imu_feedback((0.0, 0.0, gz), pitch_c)
    assert w_yaw == pytest.approx(gz / math.cos(pitch_c))
    assert w_yaw > gz                    # 抬头后关节角速度大于机体 z 轴分量


def test_protocol_imu_feedback_cosine_floor():
    """cos(pitch_c) 下限 0.1 防除零。"""
    w_yaw, _ = imu_feedback((0.0, 0.0, 0.5), pitch_c=math.pi / 2.0)
    assert w_yaw == pytest.approx(0.5 / 0.1)
    assert math.isfinite(w_yaw)


def test_damping_opposes_motion():
    """阻尼项符号：已有同向角速度时指令被压低（``− kd·ω_imu``）。"""
    law = _fresh_law(kp_yaw=6.0, kd_yaw=0.5)
    target = np.array([2.0, 0.5, 0.0])
    out_static = law.compute(target, np.zeros(3), QUAT_IDENTITY, (0.0, 0.0, 0.0),
                             dt=1.0, gimbal_origin_world=ORIGIN, joint_angles=(0.0, 0.0))
    law.reset()
    out_moving = law.compute(target, np.zeros(3), QUAT_IDENTITY, (0.0, 0.0, 1.0),
                             dt=1.0, gimbal_origin_world=ORIGIN, joint_angles=(0.0, 0.0))
    assert out_moving.yaw_omega < out_static.yaw_omega


# ==========================================================================
# 前馈增益与主合成公式
# ==========================================================================
def test_combined_formula_matches_design():
    """``ω = ff_gain·ω_ff + kp·e − kd·ω_imu`` 逐项核对（限幅前）。"""
    law = _fresh_law(kp_yaw=6.0, kd_yaw=0.5, kp_pitch=6.0, kd_pitch=0.5, ff_gain=1.0)
    r = np.array([2.0, 0.3, 0.2])
    v = np.array([0.1, 0.4, -0.2])
    R = np.asarray(frames.quat_to_mat(QUAT_IDENTITY), dtype=np.float64).reshape(3, 3)
    f, y_body = attitude_axes(R)

    yaw_t, pitch_t = target_angles(r)
    yaw_c, pitch_c = current_angles(f)
    e_yaw = wrap_pi(yaw_t - yaw_c)
    e_pitch = pitch_t - pitch_c
    ff_yaw, ff_pitch = feedforward(r, v, y_body)
    w_imu_yaw, w_imu_pitch = imu_feedback((0.0, 0.1, 0.2), pitch_c)

    out = law.compute(r, v, QUAT_IDENTITY, (0.0, 0.1, 0.2), dt=1.0, gimbal_origin_world=ORIGIN, joint_angles=(0.0, 0.0))

    expect_yaw = DEFAULT_FF_GAIN * ff_yaw + DEFAULT_KP_YAW * e_yaw - DEFAULT_KD_YAW * (w_imu_yaw - ff_yaw)
    expect_pitch = (DEFAULT_FF_GAIN * ff_pitch + DEFAULT_KP_PITCH * e_pitch
                    - DEFAULT_KD_PITCH * (w_imu_pitch - ff_pitch))
    assert out.yaw_omega == pytest.approx(expect_yaw, abs=1e-9)
    assert out.pitch_omega == pytest.approx(expect_pitch, abs=1e-9)
    assert out.ff_pitch == pytest.approx(ff_pitch, abs=1e-12)


def test_line_of_sight_with_gimbal_origin():
    """``r = p_t − p_gimbal_world``。"""
    r = line_of_sight((3.0, 1.0, 0.5), (1.0, 1.0, 0.5))
    np.testing.assert_allclose(r, np.array([2.0, 0.0, 0.0]))


def test_invalid_quat_returns_zero():
    """四元数范数 < 0.5 或含 NaN → 无效指令 0。"""
    law = _fresh_law()
    out = law.compute(np.array([2.0, 0.0, 0.0]), np.zeros(3), (0.1, 0.1, 0.1, 0.1),
                      ZERO_GYRO, dt=0.01, gimbal_origin_world=ORIGIN, joint_angles=(0.0, 0.0))
    assert out.valid is False
    assert out.yaw_omega == 0.0 and out.pitch_omega == 0.0


# ==========================================================================
# Step 6：限幅链（顺序不可换）
# ==========================================================================
def test_slew_limit():
    """``|ω(k) − ω(k−1)| ≤ slew·dt``。"""
    assert DEFAULT_OMEGA_SLEW == pytest.approx(15.0)                # DESIGN §6.13
    assert slew_limit(2.0, 0.0, 0.01, DEFAULT_OMEGA_SLEW) == pytest.approx(0.15)
    assert slew_limit(-2.0, 0.0, 0.01, DEFAULT_OMEGA_SLEW) == pytest.approx(-0.15)
    assert slew_limit(0.05, 0.0, 0.01, DEFAULT_OMEGA_SLEW) == pytest.approx(0.05)
    assert slew_limit(2.0, 0.0, 1.0, DEFAULT_OMEGA_SLEW) == pytest.approx(2.0)  # dt 大时不限制


def test_magnitude_clamp():
    """幅值限幅 ±2.0。"""
    law = _fresh_law()
    w_yaw, w_pitch = law.limit(10.0, -10.0, 0.0, 0.0, dt=1.0)
    assert w_yaw == pytest.approx(DEFAULT_OMEGA_MAX_YAW)
    assert w_pitch == pytest.approx(-DEFAULT_OMEGA_MAX_PITCH)


def test_soft_limit_attenuation():
    """进入减速带后按剩余距离线性衰减；到限位处衰减为 0。"""
    margin = DEFAULT_LIMIT_MARGIN
    # yaw 接近上限且继续正向推 → 衰减
    assert soft_limit_factor(DEFAULT_YAW_MAX - 0.05, 1.0,
                             DEFAULT_YAW_MIN, DEFAULT_YAW_MAX, margin) == pytest.approx(0.25)
    assert soft_limit_factor(DEFAULT_YAW_MAX, 1.0,
                             DEFAULT_YAW_MIN, DEFAULT_YAW_MAX, margin) == pytest.approx(0.0)
    # 反向远离限位 → 不衰减
    assert soft_limit_factor(0.0, 1.0, DEFAULT_YAW_MIN, DEFAULT_YAW_MAX, margin) == pytest.approx(1.0)
    # pitch（协议域 -0.5 .. 0.35）
    assert soft_limit_factor(DEFAULT_PITCH_MAX - 0.04, 1.0,
                             DEFAULT_PITCH_MIN, DEFAULT_PITCH_MAX, margin) == pytest.approx(0.2)
    assert soft_limit_factor(DEFAULT_PITCH_MIN + 0.04, -1.0,
                             DEFAULT_PITCH_MIN, DEFAULT_PITCH_MAX, margin) == pytest.approx(0.2)


def test_limit_chain_order_slew_then_clamp_then_soft():
    """限幅链顺序：先 slew、再 clamp、再软限位；结果不超过 2.0 且方向保持。"""
    law = _fresh_law()
    law._omega_yaw_prev = 0.0
    w_yaw, _ = law.limit(100.0, 0.0, yaw_c=DEFAULT_YAW_MAX, pitch_c=0.0, dt=0.01)
    # slew 先切到 0.15 → clamp 不生效 → 软限位（yaw_c 已到上限）衰减为 0
    assert abs(w_yaw) <= DEFAULT_OMEGA_MAX_YAW + 1e-12
    assert w_yaw == pytest.approx(0.0)


def test_finite_guard():
    """非有限 → 0。"""
    assert finite_guard(float('nan')) == 0.0
    assert finite_guard(float('inf')) == 0.0
    assert finite_guard(1.5) == 1.5


def test_limit_guards_non_finite():
    """limit 输入 NaN 时输出仍为有限值。"""
    law = _fresh_law()
    w_yaw, w_pitch = law.limit(float('nan'), float('nan'), 0.0, 0.0, dt=0.01)
    assert math.isfinite(w_yaw) and math.isfinite(w_pitch)


# ==========================================================================
# 搜索模式（DESIGN §6.8 丢失与搜索）
# ==========================================================================
def test_half_fov_derives_from_intrinsics():
    """半视场由内参计算 ``atan(width/2/fx)``，且非法输入回退保守兜底值。"""
    # atan(width/2 / fx)：width = 2·fx 时恰好是 π/4
    assert half_fov_rad_from_intrinsics(640.0, 320.0) == pytest.approx(math.pi / 4.0)
    assert half_fov_rad_from_intrinsics(640.0, 640.0) == pytest.approx(math.atan(0.5))
    for bad in ((0.0, 640.0), (640.0, 0.0), (-1.0, 640.0), (float('nan'), 640.0),
                (float('inf'), 640.0)):
        assert half_fov_rad_from_intrinsics(*bad) == pytest.approx(DEFAULT_HALF_FOV_RAD)
    # 真实内参（由 virtual.yaml 读取）应落在合理区间
    assert 0.3 < HALF_FOV < 0.8
    assert DEFAULT_HALF_FOV_RAD == 0.0  # 未知内参时禁止猜测非零搜索幅值


def test_small_search_amplitude_within_half_fov():
    """小幅搜索 ``A=0.25 ≤ 0.5 × 半视场``（半视场由调用方注入，非硬编码）。"""
    pat = make_small_search(half_fov_rad=HALF_FOV)
    assert pat.freq == pytest.approx(0.5)
    assert pat.half_fov_rad == pytest.approx(HALF_FOV)
    assert fits_half_fov(pat.amp_yaw, HALF_FOV)
    assert pat.amp_yaw <= 0.5 * HALF_FOV + 1e-12
    assert pat.within_half_fov()


def test_small_search_clamps_amplitude_to_half_fov():
    """注入更小半视场时，超限幅值被截断到 ``0.5 × half_fov`` 并留下标记。"""
    tiny = 0.2
    pat = make_small_search(amp_yaw=0.9, half_fov_rad=tiny)
    assert pat.clamped_to_half_fov is True
    assert pat.amp_yaw_unclamped == pytest.approx(0.9)
    assert pat.amp_yaw == pytest.approx(0.5 * tiny)
    assert pat.within_half_fov()


def test_small_search_default_half_fov_is_conservative_fallback():
    """未注入半视场时使用保守兜底值，且仍满足 ``≤ 0.5 × half_fov``。"""
    pat = make_small_search(amp_yaw=0.25)
    assert pat.half_fov_rad == pytest.approx(DEFAULT_HALF_FOV_RAD)
    assert pat.amp_yaw <= 0.5 * DEFAULT_HALF_FOV_RAD + 1e-12
    assert pat.within_half_fov()


def test_half_fov_invalid_injection_falls_back():
    """非法注入值（None/0/负/NaN）一律回退保守兜底值。"""
    for bad in (None, 0.0, -1.0, float('nan'), float('inf')):
        pat = SearchPattern(amp_yaw=0.25, freq=0.5, half_fov_rad=bad)
        assert pat.half_fov_rad == pytest.approx(DEFAULT_HALF_FOV_RAD)


def test_wide_search_not_clamped_by_half_fov():
    """全范围扫描**故意**扫出视场外，不受半视场上限截断。"""
    pat = make_wide_search(half_fov_rad=HALF_FOV)
    assert pat.clamped_to_half_fov is False
    assert pat.amp_yaw == pytest.approx(1.4)
    assert not pat.within_half_fov()          # 本就该超出
    assert pat.half_fov_rad == pytest.approx(HALF_FOV)


def test_small_search_pitch_is_zero():
    """小幅搜索 pitch 保持 0。"""
    pat = make_small_search(half_fov_rad=HALF_FOV)
    for tau in (0.0, 0.3, 1.1, 2.7):
        w_yaw, w_pitch = pat.sample(tau)
        assert w_pitch == pytest.approx(0.0)
        assert abs(w_yaw) <= pat.amp_yaw * 2 * math.pi * pat.freq + 1e-12


def test_wide_search_parameters():
    """全范围扫描：A=1.4, f=0.25, pitch 0.2 慢扫。"""
    pat = make_wide_search(half_fov_rad=HALF_FOV)
    assert pat.amp_yaw == pytest.approx(1.4)
    assert pat.freq == pytest.approx(0.25)
    assert pat.amp_pitch == pytest.approx(0.2)
    w_yaw, w_pitch = pat.sample(0.0)
    assert w_yaw == pytest.approx(1.4 * 2 * math.pi * 0.25)
    assert w_pitch == pytest.approx(0.2 * 2 * math.pi * 0.25)


def test_search_is_sinusoidal_derivative():
    """``ω_yaw(τ) = A·2πf·cos(2πf τ)``：τ=0 峰值，τ=1/(4f) 过零。"""
    pat = SearchPattern(amp_yaw=0.25, freq=0.5)
    peak = 0.25 * 2 * math.pi * 0.5
    assert pat.sample(0.0)[0] == pytest.approx(peak)
    assert pat.sample(1.0 / (4.0 * 0.5))[0] == pytest.approx(0.0, abs=1e-12)
    assert pat.sample(1.0 / (2.0 * 0.5))[0] == pytest.approx(-peak)


def test_search_sample_negative_time_is_safe():
    """负 τ / NaN 不产生异常。"""
    pat = make_small_search()
    w_yaw, w_pitch = pat.sample(-1.0)
    assert math.isfinite(w_yaw) and math.isfinite(w_pitch)
    w_yaw, w_pitch = pat.sample(float('nan'))
    assert math.isfinite(w_yaw) and math.isfinite(w_pitch)


def test_search_output_passes_through_limit():
    """搜索输出同样经过 slew + 幅值 + 软限位链。"""
    law = _fresh_law()
    pat = make_wide_search()
    w_yaw, w_pitch = pat.sample(0.0)          # 峰值 1.4·2π·0.25 ≈ 2.199 > 2.0
    out_yaw, out_pitch = law.limit(w_yaw, w_pitch, yaw_c=0.0, pitch_c=0.0, dt=0.01)
    assert abs(out_yaw) <= DEFAULT_OMEGA_MAX_YAW + 1e-12
    assert abs(out_pitch) <= DEFAULT_OMEGA_MAX_PITCH + 1e-12


# ==========================================================================
# 云台原点换算（DESIGN §6.9）
# ==========================================================================
def _T_world_odom_from_rpy_t(R, t) -> np.ndarray:
    """组装 (4,4) ``T_world←odom``。"""
    T = np.eye(4, dtype=np.float64)
    T[0:3, 0:3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[0:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def test_gimbal_origin_identity():
    """仿真默认（world←odom=I、底盘在原点的静止 mock、mount=0）→ 原点为 0。"""
    p = gimbal_origin_world(np.zeros(3), 0.0, (0.0, 0.0, 0.0))
    np.testing.assert_allclose(p, np.zeros(3))


def test_gimbal_origin_with_mount_offset():
    """底盘 yaw=90°、mount=(0.2,0,0.1) → 原点在 world (0, 0.2, 0.1)。"""
    p = gimbal_origin_world(np.zeros(3), math.pi / 2.0, (0.2, 0.0, 0.1))
    np.testing.assert_allclose(p, np.array([0.0, 0.2, 0.1]), atol=1e-12)


def test_gimbal_origin_with_world_odom():
    """``T_world←odom`` 的旋转平移都生效。"""
    R_wo = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    T_wo = _T_world_odom_from_rpy_t(R_wo, (0.0, 0.0, 0.5))
    p = gimbal_origin_world(np.array([1.0, 0.0, 0.0]), 0.0, (0.0, 0.0, 0.0), T_wo)
    np.testing.assert_allclose(p, np.array([0.0, 1.0, 0.5]), atol=1e-12)


def test_gimbal_origin_matches_frames_single_source():
    """与 ``gimbal_common.frames.gimbal_origin_world`` 结果一致（单一真源）。"""
    fn = getattr(frames, 'gimbal_origin_world', None)
    if not callable(fn):  # pragma: no cover - 桩环境跳过
        pytest.skip('gimbal_common.frames.gimbal_origin_world 不可用')
    T_wo = _T_world_odom_from_rpy_t(np.eye(3), (0.3, -0.2, 0.1))
    a = gimbal_origin_world(np.array([1.0, 2.0, 0.0]), 0.6, (0.1, 0.05, 0.2), T_wo)
    b = np.asarray(fn(np.array([1.0, 2.0, 0.0]), 0.6, (0.1, 0.05, 0.2), T_wo), dtype=np.float64)
    np.testing.assert_allclose(a, b.reshape(3), atol=1e-12)


# ==========================================================================
# 默认参数与文档一致性（DESIGN §6.13）
# ==========================================================================
def test_defaults_match_design():
    """默认增益/限位与 DESIGN §6.13 / hardware_spec §3 一致。"""
    law = _fresh_law()
    assert law.kp_yaw == pytest.approx(6.0)
    assert law.kp_pitch == pytest.approx(6.0)
    assert law.kd_yaw == pytest.approx(0.5)
    assert law.kd_pitch == pytest.approx(0.5)
    assert law.ff_gain == pytest.approx(1.0)
    assert law.omega_max_yaw == pytest.approx(2.0)
    assert law.omega_max_pitch == pytest.approx(2.0)
    assert law.omega_slew == pytest.approx(15.0)
    assert (law.yaw_min, law.yaw_max) == pytest.approx((-2.8, 2.8))
    assert (law.pitch_min, law.pitch_max) == pytest.approx((-0.5, 0.35))
    assert law.limit_margin == pytest.approx(0.20)


def test_snapshot_is_serialisable():
    """snapshot 只含有限标量/列表，便于打日志。"""
    law = _fresh_law()
    snap = law.snapshot()
    assert set(snap.keys()) >= {'kp_yaw', 'kd_yaw', 'ff_gain', 'yaw_limits', 'pitch_limits'}
    assert all(math.isfinite(v) for v in (snap['kp_yaw'], snap['kd_yaw'], snap['ff_gain']))
