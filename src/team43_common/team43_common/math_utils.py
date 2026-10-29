"""通用数学工具（纯 Python + numpy，不依赖 ROS）。

包含：角度归一、限幅、一阶低通、加速度限幅、DDA 网格射线遍历。
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np


_TWO_PI: float = 2.0 * math.pi


def wrap_pi(a: float) -> float:
    """把角度归一化到 (-pi, pi]。

    Args:
        a: 任意弧度值。

    Returns:
        float: 归一化后的弧度值。输入非有限时返回 0.0。
    """
    if not math.isfinite(float(a)):
        return 0.0
    return (float(a) + math.pi) % _TWO_PI - math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    """标量限幅。``lo``/``hi`` 为 None 时表示不限该侧。"""
    out = float(v)
    if lo is not None:
        out = max(float(lo), out)
    if hi is not None:
        out = min(float(hi), out)
    return out


def norm2(v: Sequence[float]) -> float:
    """二维/三维向量的欧氏范数。"""
    arr = np.asarray(v, dtype=np.float64).reshape(-1)
    return float(np.linalg.norm(arr))


def is_finite_array(a: Sequence[float]) -> bool:
    """判断数组/序列中所有元素都是有限值。"""
    arr = np.asarray(a, dtype=np.float64).reshape(-1)
    return bool(np.all(np.isfinite(arr)))


def lowpass_alpha(dt: float, cutoff_hz: float) -> float:
    """一阶低通系数 ``alpha = 1 - exp(-2*pi*f*dt)``。

    Args:
        dt: 步长（秒），<=0 时返回 1.0（直接跟随）。
        cutoff_hz: 截止频率（Hz），<=0 时返回 0.0（保持原值）。

    Returns:
        float: 位于 [0,1] 的低通系数。
    """
    dt = float(dt)
    cutoff_hz = float(cutoff_hz)
    if cutoff_hz <= 0.0:
        return 0.0
    if dt <= 0.0:
        return 1.0
    return 1.0 - math.exp(-_TWO_PI * cutoff_hz * dt)


def lowpass(prev: float, target: float, dt: float, cutoff_hz: float) -> float:
    """一阶低通滤波（标量）。

    Args:
        prev: 上一时刻滤波值。
        target: 本时刻目标值。
        dt: 步长（秒）。
        cutoff_hz: 截止频率（Hz）。

    Returns:
        float: 本时刻滤波输出；输入非有限时回退为 ``prev``（若 prev 也非有限则回退 0.0）。
    """
    if not math.isfinite(float(target)) or not math.isfinite(float(dt)):
        return float(prev) if math.isfinite(float(prev)) else 0.0
    alpha = lowpass_alpha(dt, cutoff_hz)
    return float(prev) + alpha * (float(target) - float(prev))


def accel_limit(delta_xy: Sequence[float], dt: float, a_max: float) -> np.ndarray:
    """对二维增量做加速度限幅（DESIGN §6.10 Step3）。

    Args:
        delta_xy: 期望的二维增量 ``cmd_lp - cmd_out_prev``。
        dt: 步长（秒）。
        a_max: 最大加速度（m/s²），<=0 表示不限幅。

    Returns:
        np.ndarray: (2,) 限幅后的增量。
    """
    d = np.asarray(delta_xy, dtype=np.float64).reshape(2)
    if not np.all(np.isfinite(d)):
        return np.zeros(2, dtype=np.float64)
    dt = float(dt)
    a_max = float(a_max)
    if a_max <= 0.0 or dt <= 0.0:
        return d
    n = float(np.linalg.norm(d))
    max_step = a_max * dt
    if n <= max_step or n <= 0.0:
        return d
    return d * (max_step / n)


def ray_cells(origin_xy: Sequence[float],
              dir_xy: Sequence[float],
              L: float,
              res: float) -> List[Tuple[int, int]]:
    """Amanatides-Woo DDA 网格遍历（**超覆盖**，避免穿角漏检）。

    坐标约定：``origin_xy`` 与 ``dir_xy`` 均为**栅格局部连续坐标**（即已经减掉栅格
    ``origin_xy`` 角的米制坐标），cell 索引为 ``floor(p / res)``，与
    ``GridMap.world_to_cell`` 保持一致。调用方负责把世界坐标减掉栅格原点。

    Args:
        origin_xy: 射线起点（栅格局部米）。
        dir_xy: 射线方向（无需归一化）。
        L: 射线长度（米）。
        res: 栅格分辨率（米/格）。

    Returns:
        list: 途经 cell 索引 ``[(i, j), ...]``，按由近及远顺序；起点 cell 排在最前。
              退化输入（零方向 / 非正长度 / 非正分辨率）返回空列表。
    """
    res = float(res)
    L = float(L)
    if res <= 0.0 or L <= 0.0:
        return []
    ox = float(np.asarray(origin_xy).reshape(-1)[0])
    oy = float(np.asarray(origin_xy).reshape(-1)[1])
    d = np.asarray(dir_xy, dtype=np.float64).reshape(-1)
    if d.size < 2:
        return []
    n = math.hypot(float(d[0]), float(d[1]))
    if n <= 1e-12:
        return []
    dx = float(d[0]) / n
    dy = float(d[1]) / n

    # 切换到"格"单位下继续推导
    x = ox / res
    y = oy / res
    t_end = L / res
    ix = math.floor(x)
    iy = math.floor(y)

    step_x = 1 if dx > 0.0 else (-1 if dx < 0.0 else 0)
    step_y = 1 if dy > 0.0 else (-1 if dy < 0.0 else 0)

    if step_x > 0:
        t_max_x = (math.floor(x) + 1.0 - x) / dx
    elif step_x < 0:
        t_max_x = (x - math.floor(x)) / (-dx)
    else:
        t_max_x = math.inf
    if step_y > 0:
        t_max_y = (math.floor(y) + 1.0 - y) / dy
    elif step_y < 0:
        t_max_y = (y - math.floor(y)) / (-dy)
    else:
        t_max_y = math.inf

    t_delta_x = abs(1.0 / dx) if dx != 0.0 else math.inf
    t_delta_y = abs(1.0 / dy) if dy != 0.0 else math.inf

    cells: List[Tuple[int, int]] = [(ix, iy)]
    t = 0.0
    guard = 0
    max_iter = int(4.0 * (abs(math.floor(ox / res + dx * t_end) - ix)
                          + abs(math.floor(oy / res + dy * t_end) - iy))) + 64
    while t <= t_end and guard <= max_iter:
        guard += 1
        if t_max_x < t_max_y:
            ix += step_x
            t = t_max_x
            t_max_x += t_delta_x
        elif t_max_y < t_max_x:
            iy += step_y
            t = t_max_y
            t_max_y += t_delta_y
        else:
            # 正好穿过格点：超覆盖，把两个侧向邻居与对角邻居一并加入
            jx = ix + step_x
            jy = iy + step_y
            if jx != ix:
                cells.append((jx, iy))
            if jy != iy:
                cells.append((ix, jy))
            ix, iy = jx, jy
            t = t_max_x
            t_max_x += t_delta_x
            t_max_y += t_delta_y
        if t > t_end:
            break
        cells.append((ix, iy))
    return cells
