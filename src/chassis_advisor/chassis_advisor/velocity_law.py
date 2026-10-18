"""距离速度律 + 一阶低通 + 加速度限幅 + 丢失衰减（DESIGN §6.10 Step 1 / Step 3）。

纯 Python（不依赖 rclpy），可直接单测。

**Step 1 — 期望幅值**（`G` = 目标**预测**位置，`B` = 底盘位置，`e = G − B`，`d = ‖e‖`）

```
u     = e / d                                   (d < 1e-6 时 u = 0)
k_v   = v_max / (d_slow − d_stop)
v_des = clamp(k_v * (d − d_stop), 0.0, v_max)
```

**Step 3 — 平滑与限幅**

```
cmd_raw = v_des * u_safe                                  # map 系
α = 1 − exp(−2π · lp_cutoff_hz · Δt)                      # 一阶低通，默认 5Hz
cmd_lp = cmd_lp_prev + α * (cmd_raw − cmd_lp_prev)
Δ = cmd_lp − cmd_out_prev；若 ‖Δ‖ > a_max*Δt → Δ *= (a_max*Δt)/‖Δ‖
cmd_out = cmd_out_prev + Δ
```

**丢失衰减**：`track_status == 0` 时
```
cmd_out *= max(0.0, 1.0 − (t_now − t_lost) / lost_decay_s)     # lost_decay_s = 2.0
```

默认 `v_max=0.5 m/s`、`d_slow=1.0 m`、`d_stop=0.15 m`、`a_max=1.0 m/s²`、`lp_cutoff_hz=5.0`。
"""

from __future__ import annotations

import math
from typing import Any
from typing import Dict
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np

from gimbal_common.math_utils import accel_limit, clamp, lowpass

from gimbal_common.time_utils import now


def _accel_limit_apply(target: np.ndarray, prev: np.ndarray, dt: float,
                       a_max: float) -> np.ndarray:
    """把 `prev` 平滑到 `target`，限制 ‖Δ‖ ≤ a_max·dt。

    直接调用 ``gimbal_common.math_utils.accel_limit(delta, dt, a_max)``
    （3 参增量式，单一真源；``a_max<=0`` 或 ``dt<=0`` 表示不限幅）。
    """
    tgt = np.asarray(target, dtype=float).reshape(2)
    prv = np.asarray(prev, dtype=float).reshape(2)
    d = accel_limit(tgt - prv, dt, a_max)
    return prv + np.asarray(d, dtype=float).reshape(2)

__all__ = [
    'V_MAX',
    'D_SLOW',
    'D_STOP',
    'A_MAX',
    'LP_CUTOFF_HZ',
    'LOST_DECAY_S',
    'VelocityLaw',
]

# 默认参数（DESIGN §6.13 `chassis.*`）
V_MAX: float = 0.5
D_SLOW: float = 1.0
D_STOP: float = 0.15
A_MAX: float = 1.0
LP_CUTOFF_HZ: float = 5.0
LOST_DECAY_S: float = 2.0

_EPS_D: float = 1e-6


class VelocityLaw:
    """底盘建议速度的"平滑与限幅"环节。

    Attributes:
        v_max: 最大速度 m/s。
        d_slow: 开始减速的距离 m（在该距离处达到 v_max）。
        d_stop: 停止带距离 m（小于该距离速度为 0）。
        a_max: 最大加速度 m/s²。
        lp_cutoff_hz: 一阶低通截止频率 Hz。
        lost_decay_s: 丢失后速度线性衰减到 0 的时长 s。
    """

    def __init__(self,
                 v_max: float = V_MAX,
                 d_slow: float = D_SLOW,
                 d_stop: float = D_STOP,
                 a_max: float = A_MAX,
                 lp_cutoff_hz: float = LP_CUTOFF_HZ,
                 lost_decay_s: float = LOST_DECAY_S) -> None:
        self.v_max = float(v_max)
        self.d_slow = float(d_slow)
        self.d_stop = float(d_stop)
        self.a_max = float(a_max)
        self.lp_cutoff_hz = float(lp_cutoff_hz)
        self.lost_decay_s = float(lost_decay_s)

        self._cmd_lp = np.zeros(2, dtype=float)     # 低通状态
        self._cmd_out = np.zeros(2, dtype=float)    # 上一次输出

    # -- 状态 -----------------------------------------------------------------
    def reset(self, cmd: Optional[Sequence[float]] = None) -> None:
        """重置内部状态（滤波器与上一次输出）。"""
        if cmd is None:
            self._cmd_lp = np.zeros(2, dtype=float)
            self._cmd_out = np.zeros(2, dtype=float)
        else:
            v = np.asarray(cmd, dtype=float).reshape(2)
            self._cmd_lp = v.copy()
            self._cmd_out = v.copy()

    @property
    def cmd_lp(self) -> np.ndarray:
        """当前低通后的速度（m/s）。"""
        return self._cmd_lp.copy()

    @property
    def cmd_out(self) -> np.ndarray:
        """上一次输出的速度（m/s）。"""
        return self._cmd_out.copy()

    # -- Step 1：距离速度律 ---------------------------------------------------
    def desired_speed(self, d: float) -> float:
        """按距离给出期望速度幅值 `v_des`。

        Args:
            d: 底盘到目标**预测**位置的距离（m）。

        Returns:
            float：期望速度幅值，范围 [0, v_max]。
        """
        dd = float(d)
        if not math.isfinite(dd):
            return 0.0
        denom = self.d_slow - self.d_stop
        if denom <= 1e-9:
            denom = 1e-9
        k_v = self.v_max / denom
        return float(clamp(k_v * (dd - self.d_stop), 0.0, self.v_max))

    @staticmethod
    def unit_direction(base_xy: Sequence[float],
                       goal_xy: Sequence[float]) -> Tuple[np.ndarray, float]:
        """`B → G` 的单位方向与距离。距离 < 1e-6 时方向为零向量。

        Args:
            base_xy: 底盘位置 (x, y)。
            goal_xy: 目标预测位置 (x, y)。

        Returns:
            tuple：(u (2,) ndarray, d float)
        """
        b = np.asarray(base_xy, dtype=float).reshape(2)
        g = np.asarray(goal_xy, dtype=float).reshape(2)
        e = g - b
        d = float(np.linalg.norm(e))
        if (not math.isfinite(d)) or d < _EPS_D:
            return np.zeros(2, dtype=float), (d if math.isfinite(d) else 0.0)
        return e / d, d

    # -- Step 3：平滑与限幅 ---------------------------------------------------
    def compute(self, v_des: float, u_safe: Sequence[float], dt: float,
                last: Sequence[float]) -> np.ndarray:
        """低通 + 加速度限幅，返回本次输出速度（map 系，m/s）。

        Args:
            v_des: 期望速度幅值（m/s）。
            u_safe: 避障后的单位方向 (2,)。
            dt: 距上次的单调时间差（s），≤0 时退化为直接输出低通结果。
            last: 上一次输出速度 (2,)。

        Returns:
            np.ndarray：(2,) 本次输出速度。
        """
        u = np.asarray(u_safe, dtype=float).reshape(2)
        if not np.all(np.isfinite(u)):
            u = np.zeros(2, dtype=float)
        vd = float(v_des) if math.isfinite(float(v_des)) else 0.0
        raw = u * vd
        if not np.all(np.isfinite(raw)):
            raw = np.zeros(2, dtype=float)

        dt = float(dt)
        if not math.isfinite(dt) or dt < 0.0:
            dt = 0.0

        # 一阶低通：α = 1 − exp(−2π·f_c·Δt)
        lp = np.zeros(2, dtype=float)
        for k in range(2):
            lp[k] = lowpass(float(self._cmd_lp[k]), float(raw[k]), dt,
                            self.lp_cutoff_hz)
        self._cmd_lp = lp

        prev = np.asarray(last, dtype=float).reshape(2)
        if not np.all(np.isfinite(prev)):
            prev = np.zeros(2, dtype=float)
        out = _accel_limit_apply(lp, prev, dt, self.a_max)
        if not np.all(np.isfinite(out)):  # pragma: no cover - 数值兜底
            out = prev
        self._cmd_out = np.asarray(out, dtype=float).reshape(2)
        return self._cmd_out.copy()

    # -- 丢失衰减 -------------------------------------------------------------
    def lost_gain(self, t_now: float, t_lost: Optional[float]) -> float:
        """丢失衰减系数 `max(0, 1 − (t_now − t_lost)/lost_decay_s)`。

        Args:
            t_now: 当前单调时刻（s）。
            t_lost: 目标丢失的单调时刻（s）；None 表示未丢失。

        Returns:
            float：[0, 1] 的衰减系数。
        """
        if t_lost is None:
            return 1.0
        if self.lost_decay_s <= 1e-9:
            return 0.0
        return float(max(0.0, 1.0 - (float(t_now) - float(t_lost))
                         / self.lost_decay_s))

    def apply_lost_decay(self, cmd: Sequence[float], t_now: float,
                         t_lost: Optional[float]) -> np.ndarray:
        """对速度施加丢失衰减。"""
        g = self.lost_gain(t_now, t_lost)
        return np.asarray(cmd, dtype=float).reshape(2) * g

    # -- 一站式的完整计算 -----------------------------------------------------
    def step(self,
             base_xy: Sequence[float],
             goal_xy: Sequence[float],
             dt: float,
             u_safe: Optional[Sequence[float]] = None,
             blocked: bool = False,
             track_status: int = 1,
             t_now: Optional[float] = None,
             t_lost: Optional[float] = None) -> Dict[str, Any]:
        """一次算出底盘建议速度的**全部**诊断量。

        Args:
            base_xy: 底盘位置 (x, y)（map 系）。
            goal_xy: 目标**预测**位置 (x, y)（map 系）。
            dt: 单调时间差（s）。
            u_safe: 避障后的单位方向；None 时直接用 `B→G` 方向。
            blocked: 13 个方向全被占用格挡住时为 True（此时 v_des = 0）。
            track_status: 0=丢失 / 1=有观测 / 2=外推。
            t_now: 当前单调时刻（默认取 `now()`）。
            t_lost: 丢失起始单调时刻（track_status==0 时必填）。

        Returns:
            dict：{'cmd': (2,) ndarray, 'v_des': float, 'dist': float,
                   'u': (2,) ndarray, 'u_safe': (2,) ndarray,
                   'blocked': bool, 'lost_gain': float}
        """
        u, d = self.unit_direction(base_xy, goal_xy)
        v_des = 0.0 if blocked else self.desired_speed(d)
        us = np.asarray(u_safe, dtype=float).reshape(2) if u_safe is not None else u
        if not np.all(np.isfinite(us)):
            us = np.zeros(2, dtype=float)
        cmd = self.compute(v_des, us, dt, self._cmd_out)

        gain = 1.0
        if int(track_status) == 0:
            tt = float(t_now) if t_now is not None else now()
            tl = float(t_lost) if t_lost is not None else tt
            gain = self.lost_gain(tt, tl)
            cmd = cmd * gain
            self._cmd_out = cmd.copy()
        return {
            'cmd': cmd,
            'v_des': float(v_des),
            'dist': float(d),
            'u': u,
            'u_safe': us,
            'blocked': bool(blocked),
            'lost_gain': float(gain),
        }
