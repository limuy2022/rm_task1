# -*- coding: utf-8 -*-
"""BearingControlLaw 单测（审计 P0：控制ROS接入前先验证纯函数层）。

覆盖四类必过点：
1. 门控：enabled / geometry_verified / 观测龄 / 深度 / 关节角 / SO(3) 外参，
   任一不满足 → invalid 零输出且清空 slew 历史；
2. 符号：marker 偏右 → yaw_omega < 0（右转为负）；marker 偏下 → pitch_omega > 0
   （协议域低头为正）；与 world 版 ControlLaw 的方向约定一致；
3. 几何：误差率 = lstsq(雅可比, 视线偏差)；pitch→±90° 雅可比奇异拒绝；
   前馈（目标下移 / 底盘左转）符号正确；
4. 限幅链与闭环收敛：slew 从零起步、软限位衰减、invalid 后斜坡历史清零；
   一阶云台模型下闭环收敛到静态目标。

**不依赖 rclpy**：全部在 numpy 上跑，使用本仓库真实 common。
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

# --- pytest 引导（与 test_control_law.py 相同） ---
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # noqa: E402
import conftest  # noqa: F401,E402

from team43_control.control_law import (  # noqa: E402
    BearingControlLaw,
    r_gimbal_from_opt,
)

#: 标准 ``R_gimbal←opt``（optical z 向前 → gimbal x；与真实 common 定义一致）
R_GO = np.array([[0.0, 0.0, 1.0],
                 [-1.0, 0.0, 0.0],
                 [0.0, -1.0, 0.0]])
JOINTS_ZERO = (0.0, 0.0)


def _law(**kw) -> BearingControlLaw:
    kw.setdefault('ff_gain', 0.0)          # 符号/限幅用例默认关前馈
    return BearingControlLaw(**kw)


def _bearing(law, point, joints=JOINTS_ZERO, **kw):
    kw.setdefault('R_gimbal_optical', R_GO)
    kw.setdefault('geometry_verified', True)
    kw.setdefault('enabled', True)
    return law.compute_bearing(point, joint_angles=joints, **kw)


# ==========================================================================
# 1. 门控：任一不满足 → invalid 零输出
# ==========================================================================
def test_gate_disabled():
    out = _bearing(_law(), (0.0, 0.0, 2.0), enabled=False)
    assert not out.valid and out.mode == 'bearing'
    assert out.yaw_omega == 0.0 and out.pitch_omega == 0.0


def test_gate_geometry_unverified():
    out = _bearing(_law(), (0.0, 0.0, 2.0), geometry_verified=False)
    assert not out.valid


def test_gate_stale_observation():
    law = _law()
    out = _bearing(law, (0.0, 0.0, 2.0), observation_age=0.151, max_age=0.15)
    assert not out.valid
    out = _bearing(law, (0.0, 0.0, 2.0), observation_age=-0.01, max_age=0.15)
    assert not out.valid
    # 边界：age == max_age 与 age == 0 都必须可用
    assert _bearing(law, (0.0, 0.0, 2.0), observation_age=0.15, max_age=0.15).valid
    assert _bearing(law, (0.0, 0.0, 2.0), observation_age=0.0, max_age=0.15).valid


def test_gate_invalid_depth():
    law = _law()
    for point in ((0.0, 0.0, 0.0), (0.0, 0.0, -2.0), (0.1, 0.1, 1e-7)):
        assert not _bearing(law, point).valid, point


def test_gate_missing_or_nonfinite_joints():
    law = _law()
    assert not _bearing(law, (0.0, 0.0, 2.0), joints=None).valid
    assert not _bearing(law, (0.0, 0.0, 2.0),
                        joints=(float('nan'), 0.0)).valid


def test_gate_nonfinite_marker():
    law = _law()
    assert not _bearing(law, (float('nan'), 0.0, 2.0)).valid
    assert not _bearing(law, (float('inf'), 0.0, 2.0)).valid


def test_gate_extrinsic_not_so3():
    law = _law()
    bad = np.eye(3) * 2.0                                  # 非正交
    assert not _bearing(law, (0.0, 0.0, 2.0), R_gimbal_optical=bad).valid
    mirror = np.diag([1.0, -1.0, 1.0])                     # det = -1
    assert not _bearing(law, (0.0, 0.0, 2.0), R_gimbal_optical=mirror).valid


def test_invalid_call_resets_slew_history():
    """invalid 调用后 slew 历史必须清零，下一次有效输出从 0 重新斜坡。"""
    law = _law(kp_yaw=20.0, kp_pitch=20.0, omega_slew=15.0)
    # 先积累一段非零输出
    out = _bearing(law, (-1.0, 0.0, 2.0))                  # marker 偏左 → 需求正 yaw
    assert 0.0 < out.yaw_omega <= 15.0 * 0.01
    out2 = _bearing(law, (-1.0, 0.0, 2.0))
    assert out2.yaw_omega > out.yaw_omega                  # 斜坡继续爬升
    # 观测过期 → invalid 且 reset
    out3 = _bearing(law, (-1.0, 0.0, 2.0), observation_age=0.5, max_age=0.15)
    assert not out3.valid
    # 重新有效：从 0 起步（若未 reset，会从 out2 的值继续斜坡）
    out4 = _bearing(law, (-1.0, 0.0, 2.0))
    assert out4.yaw_omega == pytest.approx(out.yaw_omega)


# ==========================================================================
# 2. 符号约定（与 world 版及协议一致）
# ==========================================================================
def test_sign_marker_right_turns_right():
    """marker 在光学系右侧（x>0）→ yaw_omega < 0（右转，绕 +z 顺时针）。"""
    law = _law(kp_yaw=6.0)
    out = _bearing(law, (0.5, 0.0, 2.0))
    assert out.valid
    assert out.yaw_omega < 0.0
    assert out.pitch_omega == pytest.approx(0.0, abs=1e-9)
    assert out.e_yaw < 0.0


def test_sign_marker_left_turns_left():
    law = _law(kp_yaw=6.0)
    out = _bearing(law, (-0.5, 0.0, 2.0))
    assert out.valid and out.yaw_omega > 0.0


def test_sign_marker_below_pitches_down():
    """marker 在光学系下方（y>0，optical y 向下）→ pitch_omega > 0（低头为正）。"""
    law = _law(kp_pitch=6.0)
    out = _bearing(law, (0.0, 0.5, 2.0))
    assert out.valid
    assert out.pitch_omega > 0.0
    assert out.yaw_omega == pytest.approx(0.0, abs=1e-9)
    assert out.e_pitch > 0.0


def test_sign_marker_above_pitches_up():
    law = _law(kp_pitch=6.0)
    out = _bearing(law, (0.0, -0.5, 2.0))
    assert out.valid and out.pitch_omega < 0.0


def test_centered_marker_gives_zero_command():
    law = _law(kp_yaw=6.0, kp_pitch=6.0)
    out = _bearing(law, (0.0, 0.0, 2.0))
    assert out.valid
    assert out.yaw_omega == pytest.approx(0.0, abs=1e-12)
    assert out.pitch_omega == pytest.approx(0.0, abs=1e-12)


# ==========================================================================
# 3. 几何：误差率、奇异、前馈
# ==========================================================================
def test_error_rates_match_explicit_lstsq():
    """e_yaw/e_pitch 必须等于 lstsq(雅可比, 视线偏差) 的解析结果。"""
    law = _law(kp_yaw=1.0, kp_pitch=1.0)
    point = (0.3, -0.2, 2.0)
    out = _bearing(law, point)
    # 手工重建控制律内部几何
    p = np.asarray(point, dtype=float)
    ray = R_GO @ p
    desired = ray / np.linalg.norm(ray)
    forward = R_GO[:, 2]
    yaw_axis = np.array([0.0, 0.0, 1.0])
    pitch_axis = np.array([0.0, 1.0, 0.0])
    jac = np.column_stack((np.cross(yaw_axis, forward), np.cross(pitch_axis, forward)))
    error = desired - float(desired @ forward) * forward
    rates = np.linalg.lstsq(jac, error, rcond=None)[0]
    assert out.e_yaw == pytest.approx(float(rates[0]))
    assert out.e_pitch == pytest.approx(float(rates[1]))
    assert out.yaw_omega == pytest.approx(float(rates[0]))    # kp=1 且无前馈
    assert out.pitch_omega == pytest.approx(float(rates[1]))


def test_pitched_yaw_axis_enters_jacobian():
    """pitch≠0 时 yaw 轴倾斜（-sinθ,0,cosθ）：同样的横向偏差需求不同的 yaw 率。"""
    law = _law(kp_yaw=1.0)
    point = (0.5, 0.0, 2.0)
    out0 = _bearing(law, point, joints=(0.0, 0.0))
    out_p = _bearing(law, point, joints=(0.0, 0.6))          # pitch 低头 0.6 rad
    # |yaw 轴 × forward| = cos(pitch) → e_yaw 放大 1/cos(pitch)
    assert out_p.e_yaw == pytest.approx(out0.e_yaw / math.cos(0.6), rel=1e-9)


def test_near_singular_pitch_rejected():
    """|pitch| → 90° 时 yaw 轴与光轴平行，最小奇异值 = |cosθ| < 0.2 必须拒绝。"""
    law = _law()
    assert _bearing(law, (0.5, 0.0, 2.0), joints=(0.0, 1.2)).valid      # cos1.2≈0.36 > 0.2
    assert not _bearing(law, (0.5, 0.0, 2.0), joints=(0.0, 1.4)).valid  # cos1.4≈0.17 < 0.2
    assert not _bearing(law, (0.5, 0.0, 2.0), joints=(0.0, -1.4)).valid


def test_feedforward_target_descending():
    """目标竖直下移（gimbal 系 -z）→ 前馈 pitch > 0（低头跟踪）。"""
    law = BearingControlLaw(kp_yaw=0.0, kp_pitch=0.0, ff_gain=1.0)
    d, vz = 2.0, 0.5
    out = _bearing(law, (0.0, 0.0, d),
                   target_relative_velocity_gimbal=(0.0, 0.0, -vz))
    assert out.valid
    assert out.ff_pitch == pytest.approx(vz / d)
    assert out.ff_pitch > 0.0
    assert out.ff_yaw == pytest.approx(0.0, abs=1e-12)


def test_feedforward_target_moving_left():
    """目标向 gimbal +y（左）横移 → 前馈 yaw > 0。"""
    law = BearingControlLaw(kp_yaw=0.0, kp_pitch=0.0, ff_gain=1.0)
    d, vy = 2.0, 0.4
    out = _bearing(law, (0.0, 0.0, d),
                   target_relative_velocity_gimbal=(0.0, vy, 0.0))
    assert out.ff_yaw == pytest.approx(vy / d)
    assert out.ff_yaw > 0.0


def test_feedforward_base_turning_left_compensates_right():
    """底盘左转（base_w=+z）时为保持指向静态目标，前馈 yaw < 0（反向右转）。

    雅可比两侧都是 rad/s：底盘旋转项 ``ω_base × forward`` 不除以距离，
    期望 ``ff_yaw = -w``（与目标平移项的 /d 不同）。
    """
    law = BearingControlLaw(kp_yaw=0.0, kp_pitch=0.0, ff_gain=1.0)
    w = 0.3
    out = _bearing(law, (0.0, 0.0, 2.0), base_omega_gimbal=(0.0, 0.0, w))
    assert out.ff_yaw == pytest.approx(-w)
    assert out.ff_yaw < 0.0


def test_r_gimbal_from_opt_convention():
    """r_gimbal_from_opt 与 R_GO 同约定（optical z→gimbal x）。"""
    R = r_gimbal_from_opt()
    assert np.allclose(R, R_GO) or np.allclose(R @ R.T, np.eye(3), atol=1e-6)
    assert np.allclose(R[:, 2], (1.0, 0.0, 0.0), atol=1e-6)


# ==========================================================================
# 4. 限幅链 + 闭环收敛
# ==========================================================================
def test_slew_from_rest_then_clamp():
    """需求 20 rad/s：首拍被 slew 压到 15·dt，随后被 ±omega_max 截住。"""
    law = _law(kp_yaw=100.0, kp_pitch=0.0, omega_max_yaw=1.0, omega_slew=15.0)
    dt = 0.01
    o1 = _bearing(law, (-2.0, 0.0, 2.0), dt=dt)             # 偏左 → 需求正 yaw
    assert o1.yaw_omega == pytest.approx(15.0 * dt)
    o2 = _bearing(law, (-2.0, 0.0, 2.0), dt=dt)
    assert o2.yaw_omega == pytest.approx(2 * 15.0 * dt)
    for _ in range(40):                                     # 爬到幅值限
        out = _bearing(law, (-2.0, 0.0, 2.0), dt=dt)
    assert out.yaw_omega == pytest.approx(1.0)              # omega_max_yaw


def test_soft_limit_attenuates_outward_rate():
    """yaw 已在减速带内且需求向外（+方向）→ 输出按剩余距离线性衰减；压线时为 0。"""
    law = _law(kp_yaw=100.0, omega_max_yaw=2.0, omega_slew=15.0,
               yaw_min=-2.8, yaw_max=2.8, limit_margin=0.2)
    dt = 0.01
    # joints yaw=2.79：factor=(2.8-2.79)/0.2=0.05；首拍 slew 0.15 再乘 0.05
    out = _bearing(law, (-2.0, 0.0, 2.0), joints=(2.79, 0.0), dt=dt)
    assert 0.0 < out.yaw_omega <= 0.15 * 0.05 + 1e-12
    # joints yaw=2.80（压线）且需求 + → 衰减为 0
    law2 = _law(kp_yaw=100.0, omega_max_yaw=2.0, omega_slew=15.0,
                yaw_min=-2.8, yaw_max=2.8, limit_margin=0.2)
    out2 = _bearing(law2, (-2.0, 0.0, 2.0), joints=(2.8, 0.0), dt=dt)
    assert out2.yaw_omega == 0.0
    # 反方向（向限位内走）不受衰减
    law3 = _law(kp_yaw=100.0, omega_max_yaw=2.0, omega_slew=15.0,
                yaw_min=-2.8, yaw_max=2.8, limit_margin=0.2)
    out3 = _bearing(law3, (2.0, 0.0, 2.0), joints=(2.8, 0.0), dt=dt)   # 偏右 → 需求负
    assert out3.yaw_omega < 0.0


def test_closed_loop_converges_to_static_target():
    """一阶云台模型（关节按指令角速度积分）+ 静态目标 → 闭环收敛、误差→0。

    世界系目标方位 yaw_t=0.5 rad（左）、仰角 +0.2 rad（抬头 → 协议域 pitch=-0.2）；
    由 R_world←gimbal = Rz(ψ)·Ry(θ) 与 R_GO 反算光学系坐标喂给控制律。
    """
    law = _law(kp_yaw=6.0, kp_pitch=6.0, omega_max_yaw=2.0, omega_max_pitch=2.0)

    def rot_z(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def rot_y(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

    d, yaw_t, elev = 3.0, 0.5, 0.2
    target_world = d * np.array([math.cos(elev) * math.cos(yaw_t),
                                 math.cos(elev) * math.sin(yaw_t),
                                 math.sin(elev)])
    psi, theta = 0.0, 0.0                                  # 关节初值
    dt = 0.01
    last = None
    for _ in range(600):                                   # 6 s
        marker_gimbal = (rot_z(psi) @ rot_y(theta)).T @ target_world
        marker_optical = R_GO.T @ marker_gimbal
        last = _bearing(law, marker_optical, joints=(psi, theta), dt=dt)
        assert last.valid, '闭环过程中不应出现 invalid'
        psi += last.yaw_omega * dt
        theta += last.pitch_omega * dt
    assert math.hypot(last.e_yaw, last.e_pitch) < 1e-3      # 误差收敛
    assert abs(psi - yaw_t) < 5e-3
    assert abs(theta - (-elev)) < 5e-3                      # 协议域：抬头为负
    assert abs(last.yaw_omega) < 1e-2 and abs(last.pitch_omega) < 1e-2
