"""单调时钟唯一入口。

硬约束（DESIGN §8.2 + 主理人拍板 1「混合时钟」）：

* 所有 **dt、超时、预测、定时器周期计算、CSV 时间戳** 一律走 ``now()``（``time.monotonic()``）。
* ``header.stamp`` / TF 时间戳走 ROS 系统时钟（``node.get_clock().now()``），保证 RViz / rosbag
  与外部节点同域可显示。
* **禁止**用 ``msg.header.stamp`` 或 ``node.get_clock().now()`` 推算目标运动。

本模块不依赖 ROS，可在无 ROS 环境下单测。
"""

from __future__ import annotations

import time
from typing import Optional


def now() -> float:
    """返回单调时钟秒数（``time.monotonic()``）。

    Returns:
        float: 单调递增的秒数，不受系统时钟跳变影响。
    """
    return time.monotonic()


# 周期比较容差：吸收浮点累加误差（0.01 累加 50 次后差值可能是 0.009999999999999998），
# 否则 100Hz 回路会出现"该发没发"的丢拍。
_PERIOD_EPS_S: float = 1e-9


class MonotonicTicker:
    """基于单调时钟的周期判定器。

    典型用法::

        ticker = MonotonicTicker(1.0 / 100.0)
        ...
        if ticker.due():
            do_something()

    若某次回调严重滞后（例如被阻塞），不会补偿式地连发多次，而是直接把相位重置到当前时刻，
    避免 100Hz 回路里出现"补发风暴"。
    """

    def __init__(self, period_s: float, auto_start: bool = True) -> None:
        """构造周期判定器。

        Args:
            period_s: 判定周期（秒）。必须为正数。
            auto_start: True 时以当前单调时刻作为起点。
        """
        period_s = float(period_s)
        if not (period_s > 0.0):
            raise ValueError(f'period_s 必须为正数，收到 {period_s!r}')
        self._period_s: float = period_s
        self._last_s: Optional[float] = now() if auto_start else None

    @property
    def period_s(self) -> float:
        """当前周期（秒）。"""
        return self._period_s

    @period_s.setter
    def period_s(self, value: float) -> None:
        value = float(value)
        if not (value > 0.0):
            raise ValueError(f'period_s 必须为正数，收到 {value!r}')
        self._period_s = value

    def reset(self, t: Optional[float] = None) -> None:
        """重置相位起点。

        Args:
            t: 起点时刻；None 表示当前单调时刻。
        """
        self._last_s = now() if t is None else float(t)

    def elapsed(self, t: Optional[float] = None) -> float:
        """距上次触发经过的秒数（未启动时为 0.0）。"""
        if self._last_s is None:
            return 0.0
        t = now() if t is None else float(t)
        return max(0.0, t - self._last_s)

    def due(self, t: Optional[float] = None) -> bool:
        """判断是否到达下一次触发时刻；到达则推进相位并返回 True。

        Args:
            t: 参考时刻；None 表示取当前单调时刻。

        Returns:
            bool: 是否应当触发。
        """
        t = now() if t is None else float(t)
        if self._last_s is None:
            self._last_s = t
            return True
        if (t - self._last_s) < (self._period_s - _PERIOD_EPS_S):
            return False
        # 直接对齐到当前时刻：不补偿式补发，避免滞后后的突发风暴。
        self._last_s = t
        return True


class RateLimiter:
    """基于单调时钟的节流器（最小间隔限流）。"""

    def __init__(self, min_interval_s: float) -> None:
        """构造节流器。

        Args:
            min_interval_s: 两次放行之间的最小间隔（秒）。<=0 表示每次都放行。
        """
        self._min_interval_s: float = float(min_interval_s)
        self._last_s: Optional[float] = None

    def reset(self, t: Optional[float] = None) -> None:
        """清空历史，下一次调用必然放行。"""
        self._last_s = None if t is None else float(t)

    def allow(self, t: Optional[float] = None) -> bool:
        """判断本次是否放行；放行则记录时刻。

        Args:
            t: 参考时刻；None 表示取当前单调时刻。

        Returns:
            bool: 是否放行。
        """
        t = now() if t is None else float(t)
        if self._last_s is None or (t - self._last_s) >= self._min_interval_s:
            self._last_s = t
            return True
        return False


class Timeout:
    """单调时钟超时判定器：持续满足条件超过 ``hold_s`` 才置位。"""

    def __init__(self, hold_s: float) -> None:
        """构造超时判定器。

        Args:
            hold_s: 需要持续满足的秒数。
        """
        self._hold_s: float = float(hold_s)
        self._since_s: Optional[float] = None

    @property
    def active(self) -> bool:
        """当前是否处于计时中。"""
        return self._since_s is not None

    def update(self, condition: bool, t: Optional[float] = None) -> bool:
        """喂入一次条件判定。

        Args:
            condition: 本时刻条件是否成立。
            t: 参考时刻；None 表示取当前单调时刻。

        Returns:
            bool: 条件已持续成立超过 ``hold_s`` 则为 True。
        """
        t = now() if t is None else float(t)
        if not condition:
            self._since_s = None
            return False
        if self._since_s is None:
            self._since_s = t
            return self._hold_s <= 0.0
        return (t - self._since_s) >= self._hold_s

    def reset(self) -> None:
        """清空计时。"""
        self._since_s = None
