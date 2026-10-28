# -*- coding: utf-8 -*-
"""BearingSupervisor 单测：闭环决策核心的门控、停止原因与 epoch 语义。

时间全在单调秒域（节点回调负责把源 stamp 折算进来），因此无需任何 ROS 桩。
"""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # noqa: E402
import conftest  # noqa: F401,E402

from team43_control.bearing_supervisor import BearingSupervisor  # noqa: E402
from team43_control.control_law import (  # noqa: E402
    BearingControlLaw,
    r_gimbal_from_opt,
)


def _sup(**kw):
    kw.setdefault('max_age_s', 0.15)
    kw.setdefault('joint_timeout_s', 0.2)
    law = kw.pop('law', None) or BearingControlLaw(kp_yaw=6.0, kp_pitch=6.0, ff_gain=0.0)
    return BearingSupervisor(law, **kw)


def _arm(sup, t=100.0):
    sup.enabled = True
    sup.geometry_verified = True


def _feed_valid_inputs(sup, t=100.0, point=(-0.3, 0.0, 2.0)):
    """喂一份新鲜有效的观测 + 关节角（marker 偏左 → 需求正 yaw）。"""
    assert sup.on_marker(point[0], point[1], point[2], True, t - 0.02, marker_id=3)
    assert sup.on_joints(0.0, 0.0, t - 0.01)


# ==========================================================================
# 构造校验
# ==========================================================================
def test_constructor_rejects_bad_timeouts():
    law = BearingControlLaw()
    with pytest.raises(ValueError):
        BearingSupervisor(law, max_age_s=0.0)
    with pytest.raises(ValueError):
        BearingSupervisor(law, joint_timeout_s=-0.1)
    with pytest.raises(ValueError):
        BearingSupervisor(law, default_dt_s=0.0)


# ==========================================================================
# 停止原因逐项（按 tick 内的判定顺序）
# ==========================================================================
def test_stop_reasons_in_gate_order():
    sup = _sup()
    t = 100.0
    assert sup.tick(t).reason == 'disabled'          # enabled=False
    sup.enabled = True
    assert sup.tick(t).reason == 'unverified'        # geometry_verified=False
    sup.geometry_verified = True
    assert sup.tick(t).reason == 'no-marker'         # 从未收到观测

    sup.on_marker(0.0, 0.0, 2.0, True, t - 0.02)
    assert sup.tick(t).reason == 'no-joints'         # 有关节超时前的观测但无关节角

    sup2 = _sup()
    _arm(sup2)
    sup2.on_marker(0.0, 0.0, 2.0, True, t - 0.5)
    assert sup2.tick(t).reason == 'stale-marker'     # 观测龄 0.5 > max_age 0.15

    sup3 = _sup()
    _arm(sup3)
    sup3.on_marker(0.0, 0.0, 2.0, True, t - 0.02)
    sup3.on_joints(0.0, 0.0, t - 0.3)
    assert sup3.tick(t).reason == 'stale-joints'     # 关节龄 0.3 > 0.2

    sup4 = _sup(camera_rpy=(0.0, 0.01, 0.0))
    _arm(sup4)
    _feed_valid_inputs(sup4, t)
    assert sup4.tick(t).reason == 'camera-rpy'       # 相机旋转外参非零

    sup5 = _sup()
    _arm(sup5)
    sup5.fault = True                                # 模拟 epoch 复位后的待上膛状态
    _feed_valid_inputs(sup5, t)
    assert sup5.tick(t).reason == 'epoch-reset'      # fault 未清


def test_valid_track_decision():
    sup = _sup()
    _arm(sup)
    _feed_valid_inputs(sup, point=(-0.3, 0.0, 2.0))
    out = sup.tick(100.0)
    assert out.valid and out.reason == 'track'
    assert out.yaw_omega > 0.0                       # 偏左 → 正 yaw
    assert out.pitch_omega == pytest.approx(0.0, abs=1e-9)
    assert out.marker_age_s == pytest.approx(0.02)


def test_invalid_marker_clears_previous_observation():
    sup = _sup()
    _arm(sup)
    _feed_valid_inputs(sup)
    assert sup.tick(100.0).valid
    sup.on_marker(0.0, 0.0, 0.0, False, 100.02)      # 感知节点丢失时发 invalid 空观测
    out = sup.tick(100.05)
    assert not out.valid and out.reason == 'no-marker'


def test_nonfinite_or_bad_depth_marker_rejected():
    sup = _sup()
    _arm(sup)
    assert not sup.on_marker(float('nan'), 0.0, 2.0, True, 99.98)
    assert not sup.on_marker(0.0, 0.0, -1.0, True, 99.98)
    assert not sup.on_marker(0.0, 0.0, 2.0, True, None)   # 无源时刻
    assert sup.tick(100.0).reason == 'no-marker'


def test_bad_joints_rejected():
    sup = _sup()
    _arm(sup)
    sup.on_marker(0.0, 0.0, 2.0, True, 99.98)
    assert not sup.on_joints(float('nan'), 0.0, 99.99)
    assert not sup.on_joints(0.0, float('inf'), 99.99)
    assert not sup.on_joints(0.0, 0.0, None)
    assert sup.tick(100.0).reason == 'no-joints'


# ==========================================================================
# epoch：复位 → fault → 上升沿 rearm
# ==========================================================================
def test_epoch_change_faults_until_rearm():
    sup = _sup()
    _arm(sup)
    assert sup.on_epoch(7) is False                  # 首个值只锁存
    _feed_valid_inputs(sup)
    assert sup.tick(100.0).valid
    assert sup.on_epoch(8) is True                   # 复位 → fault
    out = sup.tick(100.05)
    assert not out.valid and out.reason == 'epoch-reset'
    # fault 后即使观测/关节都新鲜也不输出
    _feed_valid_inputs(sup, t=100.1)
    assert sup.tick(100.1).reason == 'epoch-reset'
    # rearm：清 fault，重新锁存新 epoch
    sup.rearm()
    assert sup.on_epoch(8) is False
    _feed_valid_inputs(sup, t=100.2)
    assert sup.tick(100.2).valid


def test_epoch_reset_drops_marker_but_keeps_joints_semantics():
    sup = _sup()
    _arm(sup)
    sup.on_epoch(1)
    _feed_valid_inputs(sup)
    sup.on_epoch(2)
    # 复位把观测一并清空（IMU 世界对齐失效，旧观测不可信）
    assert sup.tick(100.0).reason == 'epoch-reset'


def test_rearm_relatches_latest_epoch_without_new_message():
    """桥的 epoch 是 TRANSIENT_LOCAL：复位值只到达一次。

    rearm 必须直接以最新 epoch 重新武装——若清锁存值等下一条消息，
    下一次 MCU 复位会被误当作首次锁存而不报 fault。
    """
    sup = _sup()
    _arm(sup)
    sup.on_epoch(5)
    _feed_valid_inputs(sup)
    assert sup.tick(100.0).valid
    sup.on_epoch(6)                                   # 复位 → fault
    assert sup.tick(100.05).reason == 'epoch-reset'
    sup.rearm()                                       # 操作员重新上膛
    _feed_valid_inputs(sup, t=100.1)                  # 不再有新的 epoch 消息
    assert sup.tick(100.1).valid
    sup.on_epoch(7)                                   # 再复位必须仍然报 fault
    assert sup.tick(100.15).reason == 'epoch-reset'


# ==========================================================================
# 停止时清 slew 历史 + 指令与控制律一致
# ==========================================================================
def test_stop_resets_law_slew_history():
    sup = _sup()
    _arm(sup)
    _feed_valid_inputs(sup, point=(-1.0, 0.0, 2.0))
    first = sup.tick(100.0)
    assert 0.0 < first.yaw_omega <= 15.0 * 0.02      # 首拍受 slew 限制
    sup.on_marker(0.0, 0.0, 0.0, False, 100.01)      # 丢失 → 停止
    assert not sup.tick(100.05).valid
    assert sup.law.omega_prev == (0.0, 0.0)          # 斜坡历史已清
    _feed_valid_inputs(sup, point=(-1.0, 0.0, 2.0), t=100.07)
    again = sup.tick(100.07)
    assert again.yaw_omega == pytest.approx(first.yaw_omega)   # 从零重新起步（dt=0.02）


def test_decision_matches_direct_compute_bearing():
    """决策输出必须与直接调用控制律一致（同观测龄/dt/关节角）。

    用一份**全新的同参 law** 做对照：共用 sup.law 会让两次调用的 slew
    历史互相污染。
    """
    sup = _sup()
    _arm(sup)
    _feed_valid_inputs(sup, point=(0.2, -0.15, 2.0))
    out = sup.tick(100.0)
    fresh_law = BearingControlLaw(kp_yaw=6.0, kp_pitch=6.0, ff_gain=0.0)
    direct = fresh_law.compute_bearing(
        (0.2, -0.15, 2.0), joint_angles=(0.0, 0.0),
        R_gimbal_optical=r_gimbal_from_opt(),
        geometry_verified=True, enabled=True, dt=0.02,
        observation_age=0.02, max_age=0.15)
    assert direct.valid
    assert out.yaw_omega == pytest.approx(direct.yaw_omega)
    assert out.pitch_omega == pytest.approx(direct.pitch_omega)
