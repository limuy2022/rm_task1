"""CSV 行格式化（DESIGN §6.12，⚠ 修正 2：默认严格 8 列）。

列序：``pred_x, pred_y, cur_x, cur_y, base_x, base_y, cmd_vx, cmd_vy``
``csv_extra_cols=true`` 时追加第 9 列 ``timestamp``（相对 t0 的单调秒）与第 10 列 ``track_status``。

不依赖 ROS，可在无 ROS 环境下单测。
"""

from __future__ import annotations

import math
from typing import List, Sequence

__all__ = [
    'COLS8', 'COLS_EXTRA', 'N_COLS8', 'format_row', 'header_line',
    'all_finite', 'sanitize_row',
]

COLS8: List[str] = ['pred_x', 'pred_y', 'cur_x', 'cur_y',
                    'base_x', 'base_y', 'cmd_vx', 'cmd_vy']
COLS_EXTRA: List[str] = ['timestamp', 'track_status']
N_COLS8: int = len(COLS8)


def format_row(vals: Sequence[float], extra: bool = False) -> str:
    """格式化一行 CSV（严格 8 列，或 8+2 列）。

    Args:
        vals: 至少 8 个数值；``extra=True`` 时还需要第 9（float）与第 10（0/1/2）个。
        extra: 是否追加附加列。

    Returns:
        str: ``'%.6f'`` 逗号分隔的一行（不含换行符）。
    """
    line = ','.join('%.6f' % float(v) for v in list(vals)[:N_COLS8])
    if extra:
        v8 = float(vals[8]) if len(vals) > 8 else 0.0
        v9 = int(vals[9]) if len(vals) > 9 else 0
        line += ',%.6f,%d' % (v8, v9)
    return line


def header_line(extra: bool = False) -> str:
    """返回表头行（列数与 :func:`format_row` 一致）。"""
    cols = list(COLS8)
    if extra:
        cols += list(COLS_EXTRA)
    return ','.join(cols)


def all_finite(vals: Sequence[float]) -> bool:
    """判断序列中所有值都是有限值（非 NaN / inf）。"""
    return all(math.isfinite(float(v)) for v in vals)


def sanitize_row(vals: Sequence[float],
                 last_valid: Sequence[float],
                 extra: bool = False) -> List[float]:
    """有限性守卫：任一值非有限 → 用上一次有效值替代（DESIGN §6.12）。

    Args:
        vals: 本行原始值（8 或 10 个）。
        last_valid: 上一次有效值（8 或 10 个）；为空则把非法位置填 0.0。
        extra: 是否含附加列。

    Returns:
        list: 清洗后的数值列表（长度 8 或 10）。
    """
    n = N_COLS8 + (len(COLS_EXTRA) if extra else 0)
    cur = [float(v) for v in list(vals)[:n]]
    while len(cur) < n:
        cur.append(0.0)
    prev = [float(v) for v in list(last_valid)[:n]]
    while len(prev) < n:
        prev.append(0.0)
    out: List[float] = []
    for i in range(n):
        if math.isfinite(cur[i]):
            out.append(cur[i])
        elif math.isfinite(prev[i]):
            out.append(prev[i])
        else:
            out.append(0.0)
    return out
