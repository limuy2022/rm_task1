# -*- coding: utf-8 -*-
"""``gimbal_common.math_utils`` 测试桩（仅用于无 ROS 环境的单测）。"""

import math
from typing import Sequence

import numpy as np

_TWO_PI = 2.0 * math.pi


def wrap_pi(a: float) -> float:
    """把角度规整到 ``(-π, π]``。"""
    x = math.fmod(float(a) + math.pi, _TWO_PI)
    if x < 0.0:
        x += _TWO_PI
    return float(x - math.pi)


def clamp(v: float, lo: float, hi: float) -> float:
    """幅值限幅。"""
    x = float(v)
    if not math.isfinite(x):
        return float(lo)
    return float(max(float(lo), min(float(hi), x)))


def lowpass(prev: float, new: float, dt: float, cutoff_hz: float) -> float:
    """一阶低通：``α = 1 − exp(−2π f_c dt)``。"""
    if dt <= 0.0 or cutoff_hz <= 0.0:
        return float(new)
    alpha = 1.0 - math.exp(-_TWO_PI * float(cutoff_hz) * float(dt))
    alpha = min(max(alpha, 0.0), 1.0)
    return float(prev) + alpha * (float(new) - float(prev))


def accel_limit(delta, max_delta: float):
    """把增量 ``delta``（标量或 2D 向量）按最大模长 ``max_delta`` 限幅。"""
    d = np.asarray(delta, dtype=np.float64)
    norm = float(np.linalg.norm(d))
    if norm <= float(max_delta) or norm <= 0.0:
        return d
    return d * (float(max_delta) / norm)
