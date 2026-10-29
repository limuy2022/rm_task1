"""链路状态机单测（DESIGN §6.3 使能/保活/故障/恢复时序）。

纯 Python（**不 import rclpy**），用假时钟驱动。运行::

    python -m pytest src/team43_gimbal_io/test/test_link_state_machine.py -q
"""

from __future__ import annotations

import os
import sys

import pytest

_PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
_SRC_DIR = os.path.abspath(os.path.join(_PKG_DIR, '..'))
sys.path.insert(0, _PKG_DIR)
sys.path.insert(0, os.path.join(_SRC_DIR, 'team43_common'))   # team43_common 源码目录

from team43_gimbal_io.link_state_machine import (  # noqa: E402
    ACTIVE, FAULT_SILENT, IDLE, SHUTDOWN, LinkStateMachine,
)


class FakeClock:
    """可推进的假单调时钟。"""

    def __init__(self, t: float = 0.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += float(dt)
        return self.t


def _make(clock: FakeClock, **kwargs) -> LinkStateMachine:
    defaults = dict(fault_silence_s=0.6, cmd_period_s=0.01)
    defaults.update(kwargs)
    return LinkStateMachine(clock=clock, **defaults)


# --------------------------------------------------------------------------------------
# 使能时序
# --------------------------------------------------------------------------------------
def test_idle_sends_nothing_before_first_imu():
    clk = FakeClock()
    lsm = _make(clk)
    assert lsm.state == IDLE
    assert lsm.imu_seen is False
    assert lsm.enabled is False
    # 首帧 0x01 前：无论推进多久都不发帧
    for _ in range(100):
        clk.advance(0.01)
        assert lsm.cmd_due() is False
    assert lsm.state == IDLE


def test_first_imu_arms_link():
    clk = FakeClock()
    lsm = _make(clk)
    assert lsm.on_imu_frame() is True      # IDLE -> ACTIVE 迁移
    assert lsm.state == ACTIVE
    assert lsm.imu_seen is True
    assert lsm.cmd_due() is True           # 立即可发一帧
    assert lsm.cmd_due() is False          # 同一周期内不再发
    clk.advance(0.01)
    assert lsm.cmd_due() is True


def test_repeated_imu_does_not_retransition():
    clk = FakeClock()
    lsm = _make(clk)
    assert lsm.on_imu_frame() is True
    assert lsm.on_imu_frame() is False
    assert lsm.on_imu_frame() is False


def test_keepalive_zero_frames_are_legal_and_continuous():
    """ACTIVE 时即使没有指令也要持续发（含 (0,0) 保活），周期 10ms。"""
    clk = FakeClock()
    lsm = _make(clk)
    lsm.on_imu_frame()
    n = 0
    for _ in range(50):          # 0.5s
        clk.advance(0.01)
        if lsm.cmd_due():
            n += 1
    assert n == 50


# --------------------------------------------------------------------------------------
# 故障与恢复
# --------------------------------------------------------------------------------------
def test_command_schedule_keeps_phase_under_timer_jitter():
    """相邻回调抖动不能导致持续漏拍；长时间停顿也不补发积压指令。"""
    clk = FakeClock()
    lsm = _make(clk, cmd_period_s=0.02)
    lsm.on_imu_frame()
    start = clk()
    assert lsm.cmd_due(start)
    for i in range(1, 101):
        t = start + i * 0.02 + (0.0002 if i % 2 else 0.0001)
        assert lsm.cmd_due(t), f'第 {i} 拍被回调抖动丢弃'
        assert not lsm.cmd_due(t)
    assert lsm.cmd_due(start + 10.001)
    assert not lsm.cmd_due(start + 10.001)
    assert lsm.cmd_due(start + 10.0201)


def test_fault_silence_then_recover_after_0_6s():
    clk = FakeClock()
    lsm = _make(clk)
    lsm.on_imu_frame()
    assert lsm.on_fault('test') is True
    assert lsm.state == FAULT_SILENT
    assert lsm.cmd_due() is False

    clk.advance(0.59)
    assert lsm.tick() is False
    assert lsm.state == FAULT_SILENT

    clk.advance(0.02)            # 累计 0.61s > 0.6
    assert lsm.tick() is True
    assert lsm.state == ACTIVE
    assert lsm.cmd_due() is True


def test_fault_reason_recorded():
    clk = FakeClock()
    lsm = _make(clk)
    lsm.on_imu_frame()
    lsm.on_fault('runaway')
    assert lsm.n_faults == 1
    assert lsm.last_fault_reason == 'runaway'
    assert lsm.fault_log()[0][1] == 'runaway'


def test_fault_while_idle_does_not_disable_further():
    clk = FakeClock()
    lsm = _make(clk)
    # IDLE 下的故障仍记为故障，但不进入 FAULT_SILENT（本来就没使能）
    assert lsm.on_fault('weird') is False
    assert lsm.state == IDLE
    assert lsm.n_faults == 1


# --------------------------------------------------------------------------------------
# 故障判据
# --------------------------------------------------------------------------------------
def test_yaw_out_of_range_triggers_fault():
    clk = FakeClock()
    lsm = _make(clk, yaw_limits=(-2.8, 2.8), limit_fault_margin=0.10)
    lsm.on_imu_frame()
    assert lsm.check_limits(2.85, 0.0) is False       # 在 margin 内
    assert lsm.check_limits(2.95, 0.0) is True        # > 2.8 + 0.10
    assert lsm.state == FAULT_SILENT


def test_pitch_out_of_range_triggers_fault():
    clk = FakeClock()
    lsm = _make(clk, pitch_limits=(-0.5, 0.35), limit_fault_margin=0.10)
    lsm.on_imu_frame()
    assert lsm.check_limits(0.0, 0.40) is False
    assert lsm.check_limits(0.0, 0.50) is True
    assert lsm.state == FAULT_SILENT


def test_runaway_requires_hold_time():
    clk = FakeClock()
    lsm = _make(clk, runaway_factor=1.5, err_thresh=0.5, runaway_hold_s=0.2)
    lsm.on_imu_frame()
    # 短时超限不判故障
    assert lsm.check_runaway(3.0, 0.0) is False
    clk.advance(0.1)
    assert lsm.check_runaway(3.0, 0.0) is False
    clk.advance(0.15)            # 累计 0.25s > 0.2
    assert lsm.check_runaway(3.0, 0.0) is True
    assert lsm.state == FAULT_SILENT


def test_runaway_interrupted_resets_timer():
    clk = FakeClock()
    lsm = _make(clk, runaway_hold_s=0.2)
    lsm.on_imu_frame()
    lsm.check_runaway(3.0, 0.0)
    clk.advance(0.1)
    lsm.check_runaway(0.1, 0.0)   # 条件中断 → 清零
    clk.advance(0.3)
    assert lsm.check_runaway(3.0, 0.0) is False
    assert lsm.state == ACTIVE


def test_runaway_threshold_scales_with_cmd():
    clk = FakeClock()
    lsm = _make(clk, runaway_factor=1.5, err_thresh=0.5, runaway_hold_s=0.0)
    lsm.on_imu_frame()
    # 门限 = 1.5*2.0 + 0.5 = 3.5
    assert lsm.check_runaway(3.4, 2.0) is False
    assert lsm.check_runaway(3.6, 2.0) is False    # 起始计时
    assert lsm.check_runaway(3.6, 2.0) is True     # hold=0 → 立即判故障


def test_bad_frames_burst_triggers_fault():
    clk = FakeClock()
    lsm = _make(clk, bad_frame_window_s=1.0, bad_frame_limit=50)
    lsm.on_imu_frame()
    for _ in range(50):
        assert lsm.on_bad_frame() is False
    assert lsm.on_bad_frame() is True       # 第 51 帧
    assert lsm.state == FAULT_SILENT


def test_bad_frames_spread_over_time_no_fault():
    clk = FakeClock(100.0)
    lsm = _make(clk, bad_frame_window_s=1.0, bad_frame_limit=50)
    lsm.on_imu_frame()
    for _ in range(100):                     # 20Hz 校验错 → 窗口内约 20 < 50，不判故障
        assert lsm.on_bad_frame() is False
        clk.advance(0.05)
    assert lsm.bad_frames_in_window() <= 21
    assert lsm.state == ACTIVE


def test_write_failure_is_a_fault():
    clk = FakeClock()
    lsm = _make(clk)
    lsm.on_imu_frame()
    assert lsm.on_write_failure() is True
    assert lsm.last_fault_reason == 'serial_write_failed'


# --------------------------------------------------------------------------------------
# 退出
# --------------------------------------------------------------------------------------
def test_shutdown_sends_exactly_n_zero_frames():
    clk = FakeClock()
    lsm = _make(clk)
    lsm.on_imu_frame()
    assert lsm.request_shutdown(zero_frames=5) == 5
    assert lsm.state == SHUTDOWN
    assert lsm.zero_frames_left == 5
    assert lsm.cmd_due() is False           # SHUTDOWN 不再走正常下发
    taken = 0
    while lsm.take_zero_frame():
        taken += 1
    assert taken == 5
    assert lsm.take_zero_frame() is False


def test_shutdown_from_fault_silent_is_allowed():
    clk = FakeClock()
    lsm = _make(clk)
    lsm.on_imu_frame()
    lsm.on_fault('x')
    assert lsm.request_shutdown(zero_frames=3) == 3
    assert lsm.state == SHUTDOWN


def test_state_id_encoding_matches_msg():
    assert LinkStateMachine(clock=FakeClock()).state_id == 0
    lsm = _make(FakeClock())
    lsm.on_imu_frame()
    assert lsm.state_id == 1
    lsm.on_fault('x')
    assert lsm.state_id == 2
    lsm.request_shutdown(1)
    assert lsm.state_id == 3
