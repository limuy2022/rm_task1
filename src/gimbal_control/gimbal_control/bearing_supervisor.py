# -*- coding: utf-8 -*-
"""相机方位闭环的无 ROS 决策核心：新鲜观测 + 机械关节角 + epoch → 指令。

与 ``gimbal_serial_bridge.command_gate`` 同一模式：节点壳只做参数/话题搬运，
每拍可测的决策全部在这里。时间统一用**单调秒**——节点回调把源 stamp 折算到
该域（``t_msg = now() - (ros_now - source_t)``），与旧控制器
``_t_last_joint = now() - _source_age(...)`` 的约定一致。

安全语义（与旧控制器 / CommandGate 对齐）：
- ``enabled``、``geometry_verified`` 是操作员断言，默认 False；
- 丢失/过期/无效观测、关节角缺失或过期、epoch 复位、相机旋转外参非零
  → 输出 invalid 零指令并清空 slew 历史（下游靠 CommandGate 与固件看门狗停转）；
- epoch 复位（MCU 重启）后 world 对齐失效，必须由操作员重新上膛
  （``rearm()``，节点在 enabled 关→开的上升沿调用），与适配器一致。
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional, Tuple

import numpy as np

from .control_law import BearingControlLaw, r_gimbal_from_opt


class BearingDecision(NamedTuple):
    """一次 tick 的决策结果。"""

    yaw_omega: float = 0.0
    pitch_omega: float = 0.0
    valid: bool = False
    reason: str = 'idle'
    marker_age_s: float = float('nan')


class BearingSupervisor:
    """每拍决策：把最新观测/关节角/门控状态汇成一条 ``GimbalOmegaCmd`` 语义。"""

    def __init__(self, law: BearingControlLaw, *,
                 max_age_s: float = 0.15,
                 joint_timeout_s: float = 0.2,
                 camera_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 max_camera_rpy_norm: float = 1e-6,
                 default_dt_s: float = 0.02) -> None:
        self.law = law
        self.max_age_s = float(max_age_s)
        self.joint_timeout_s = float(joint_timeout_s)
        self.camera_rpy = np.asarray(camera_rpy, dtype=float).reshape(3)
        self.max_camera_rpy_norm = float(max_camera_rpy_norm)
        self.default_dt_s = float(default_dt_s)
        if (self.max_age_s <= 0.0 or self.joint_timeout_s <= 0.0
                or self.default_dt_s <= 0.0 or self.max_camera_rpy_norm < 0.0):
            raise ValueError('max_age_s/joint_timeout_s/default_dt_s 须为正，'
                             'max_camera_rpy_norm 须非负')
        # 操作员门控（节点每拍从 ROS 参数刷新）
        self.enabled = False
        self.geometry_verified = False
        self.fault = False                     # epoch 复位，等待 rearm
        # epoch 语义：桥的 epoch 是 TRANSIENT_LOCAL，每个值只到达一次。
        # 因此始终记录最新值；fault 判定用「最新值 != 上膛时锁存的值」。
        self._latest_epoch: Optional[int] = None
        self._armed_epoch: Optional[int] = None
        self._marker: Optional[Tuple[float, float, float]] = None
        self._marker_t: Optional[float] = None
        self._joints: Optional[Tuple[float, float]] = None
        self._joints_t: Optional[float] = None
        self._last_tick_t: Optional[float] = None
        self.last_marker_id: Optional[int] = None

    # ------------------------------------------------------------------ 上膛
    def rearm(self) -> None:
        """epoch 复位后的重新上膛：清 fault，以**当前最新 epoch** 重新武装。

        不能清锁存值等下一条消息——桥只在复位时重发 epoch，清了就永远
        比不上下一个复位了。"""
        self.fault = False
        self._armed_epoch = self._latest_epoch
        self.law.reset()

    # ------------------------------------------------------------------ 输入
    def on_marker(self, x: float, y: float, z: float, valid: bool,
                  t_msg: Optional[float],
                  marker_id: Optional[int] = None) -> bool:
        """最新光学系观测。无效观测即丢失：清空，让 tick 停止输出。"""
        self._marker = None
        self._marker_t = None
        self.last_marker_id = marker_id
        if not valid or t_msg is None:
            return False
        p = (float(x), float(y), float(z))
        if not all(math.isfinite(v) for v in p) or p[2] <= 0.0:
            return False
        self._marker = p
        self._marker_t = float(t_msg)
        return True

    def on_joints(self, yaw: float, pitch: float,
                  t_msg: Optional[float]) -> bool:
        """最新机械关节角（yaw, pitch；pitch 为协议域，低头为正）。"""
        self._joints = None
        self._joints_t = None
        if t_msg is None:
            return False
        a = (float(yaw), float(pitch))
        if not all(math.isfinite(v) for v in a):
            return False
        self._joints = a
        self._joints_t = float(t_msg)
        return True

    def on_epoch(self, value: int) -> bool:
        """复位 epoch。最新值恒更新；与上膛值不同 → fault（需 rearm）。"""
        v = int(value)
        self._latest_epoch = v
        if self._armed_epoch is None:
            self._armed_epoch = v
            return False
        if v != self._armed_epoch:
            self.fault = True
            self._marker = None
            self._marker_t = None
            return True
        return False

    # ------------------------------------------------------------------ 决策
    def marker_age(self, t: float) -> float:
        if self._marker_t is None:
            return float('nan')
        return max(0.0, float(t) - self._marker_t)

    def tick(self, t: float) -> BearingDecision:
        """一拍决策。任何门控不满足都输出 invalid 零并清 slew 历史。"""
        dt = self.default_dt_s if self._last_tick_t is None else t - self._last_tick_t
        self._last_tick_t = t
        if not self.enabled:
            return self._stop(t, 'disabled')
        if not self.geometry_verified:
            return self._stop(t, 'unverified')
        if self.fault:
            return self._stop(t, 'epoch-reset')
        if (not np.all(np.isfinite(self.camera_rpy))
                or float(np.linalg.norm(self.camera_rpy)) > self.max_camera_rpy_norm):
            return self._stop(t, 'camera-rpy')
        if self._marker is None or self._marker_t is None:
            return self._stop(t, 'no-marker')
        age = self.marker_age(t)
        if not math.isfinite(age) or age > self.max_age_s:
            return self._stop(t, 'stale-marker')
        if self._joints is None or self._joints_t is None:
            return self._stop(t, 'no-joints')
        if float(t) - self._joints_t > self.joint_timeout_s:
            return self._stop(t, 'stale-joints')
        dt = min(max(dt, 1e-4), 0.5)
        out = self.law.compute_bearing(
            self._marker, joint_angles=self._joints,
            R_gimbal_optical=r_gimbal_from_opt(),
            geometry_verified=True, enabled=True,
            dt=dt, observation_age=age, max_age=self.max_age_s)
        if not out.valid:
            return self._stop(t, 'law-invalid')
        return BearingDecision(float(out.yaw_omega), float(out.pitch_omega),
                               True, 'track', age)

    def _stop(self, t: float, reason: str) -> BearingDecision:
        self.law.reset()
        return BearingDecision(0.0, 0.0, False, reason, self.marker_age(t))
