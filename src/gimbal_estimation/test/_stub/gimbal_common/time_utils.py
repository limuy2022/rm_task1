# -*- coding: utf-8 -*-
"""``gimbal_common.time_utils`` 测试桩（仅用于无 ROS 环境的单测）。"""

import time


def now() -> float:
    """全工程唯一时间入口：单调时钟（秒）。"""
    return time.monotonic()


class MonotonicTicker:
    """按固定周期触发的单调节拍器。"""

    def __init__(self, period_s: float) -> None:
        self.period_s = float(period_s)
        self._t_last = None

    def due(self, t: float) -> bool:
        """到点返回 True 并推进基准。"""
        t = float(t)
        if self._t_last is None or t - self._t_last >= self.period_s:
            self._t_last = t
            return True
        return False


class RateLimiter:
    """最小间隔限流器。"""

    def __init__(self, min_period_s: float) -> None:
        self.min_period_s = float(min_period_s)
        self._t_last = None

    def allow(self, t: float) -> bool:
        """距上次放行超过最小间隔则放行。"""
        t = float(t)
        if self._t_last is None or (t - self._t_last) >= self.min_period_s:
            self._t_last = t
            return True
        return False
