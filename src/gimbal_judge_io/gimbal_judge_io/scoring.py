"""赛题一评分口径（**以本队口述为准**，本文件是该口径的唯一真源）。

评分（口述原文）：

> 对 CSV 文件的前四个数字与准确值之间做 RMSE，各自乘以 0.1、0.2、0.25、0.25；
> 最后两项采取"不与不可跨越地形相交的、与目标能成的最小角度方向"做标准，
> 以角度为单位计算角度 RMSE 并乘以 0.2 做和。按照加权值之和排名。

展开成公式（每列为一条时间序列，逐拍对齐后统计）：

```
score = 0.10 * RMSE(pred_x)          # 相对给定点，t_tick + 100 ms 的预测 x
      + 0.20 * RMSE(pred_y)
      + 0.25 * RMSE(cur_x)           # t_tick 时刻的实际 x
      + 0.25 * RMSE(cur_y)
      + 0.20 * (RMSE_deg(direction) / 180)      # 角度误差，度
```

- 前四项 = **米**为单位的原始差值 RMSE（不做距离归一化）。
- 方向项 = 每一拍"我们给出的方向"（由 `vx, vy` 决定）与**标准方向**的夹角（度），
  取 RMSE 后除以 180 归一，再乘 0.2 → 1° 的 RMSE 价值 ≈ 0.00111 总分。
- 标准方向 = 从底盘出发、**不与不可跨越地形（占用格）相交**、射线最长 1 m、
  与"底盘→目标"方向**夹角最小**的方向（见 `chassis_advisor.free_dir`）。

权重合计 1.0；`base_x / base_y` 只输出、**不单独计分**（它们只通过"底盘位置"
间接影响方向项：底盘位置估计偏了 → 我们算出的可行方向偏 → 方向项误差变大）。

两个硬规则（沿用主办方规则，与权重无关）：

- **停车陷阱**：`hypot(vx, vy) < 0.02` 视为停车；此时距目标 < 0.5 m 记 0 分误差，
  否则记**满分误差 1.0**（等价 180°）。因此离目标还远时必须保证 `|v| ≥ 0.02`。
- **缺行**：某拍没有行（或行迟到 >0.15 s 作废）按满分误差计（见 `MISSING_*`）。

纯 Python + numpy，不 import rclpy，可直接单测。
"""

from __future__ import annotations

import math
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence

import numpy as np

__all__ = [
    'W_PRED_X',
    'W_PRED_Y',
    'W_CUR_X',
    'W_CUR_Y',
    'W_DIR',
    'W_HEAD4',
    'WEIGHTS',
    'STOP_SPEED',
    'STOP_NEAR_M',
    'DIR_FULL_ERR_DEG',
    'MISSING_POS_ERR_M',
    'MISSING_DIR_ERR_DEG',
    'COLS',
    'rms',
    'rmse',
    'angle_between_deg',
    'direction_error_deg',
    'direction_term',
    'heading_deg',
    'score',
    'format_report',
]

# ---------------------------------------------------------------------------
# 权重（口述口径，合计 1.0）
# ---------------------------------------------------------------------------
W_PRED_X: float = 0.10
W_PRED_Y: float = 0.20
W_CUR_X: float = 0.25
W_CUR_Y: float = 0.25
W_DIR: float = 0.20

#: 前四列权重，按 CSV 列序 ``pred_x, pred_y, cur_x, cur_y``
W_HEAD4: tuple = (W_PRED_X, W_PRED_Y, W_CUR_X, W_CUR_Y)

WEIGHTS: Dict[str, float] = {
    'pred_x': W_PRED_X,
    'pred_y': W_PRED_Y,
    'cur_x': W_CUR_X,
    'cur_y': W_CUR_Y,
    'direction': W_DIR,
}

#: 本队 CSV 的列序（与 `tick_csv.COLS9` 前 8 列一致）
COLS: List[str] = ['pred_x', 'pred_y', 'cur_x', 'cur_y', 'base_x', 'base_y',
                   'vx', 'vy']

# ---------------------------------------------------------------------------
# 硬规则
# ---------------------------------------------------------------------------
#: 判"停车"的速度阈值
STOP_SPEED: float = 0.02
#: 停车时"距目标多近算到位"（到位记 0 分误差）
STOP_NEAR_M: float = 0.5
#: 满分误差（= 1.0）在方向项里等价的角度
DIR_FULL_ERR_DEG: float = 180.0
#: 缺行的位置误差（米）——按满分误差 1.0 计
MISSING_POS_ERR_M: float = 1.0
#: 缺行的方向误差（度）
MISSING_DIR_ERR_DEG: float = DIR_FULL_ERR_DEG

_EPS: float = 1e-12


# ---------------------------------------------------------------------------
# 基础统计
# ---------------------------------------------------------------------------
def rms(vals: Sequence[float]) -> float:
    """均方根；无有效（有限）值时返回 NaN。"""
    a = np.asarray(vals, dtype=np.float64).reshape(-1)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return float('nan')
    return float(math.sqrt(float(np.mean(a * a))))


def rmse(errors: Sequence[float]) -> float:
    """误差序列的 RMSE（= 误差的 RMS）；无有效值返回 NaN。"""
    return rms(errors)


def heading_deg(vx: float, vy: float) -> float:
    """由速度分量求朝向角（度，map 系，atan2(vy, vx)）；零向量返回 NaN。"""
    try:
        x, y = float(vx), float(vy)
    except (TypeError, ValueError):
        return float('nan')
    if not (math.isfinite(x) and math.isfinite(y)):
        return float('nan')
    if math.hypot(x, y) <= _EPS:
        return float('nan')
    return math.degrees(math.atan2(y, x))


def _wrap_deg(a: float) -> float:
    """把角度折到 (-180, 180]。"""
    if not math.isfinite(a):
        return float('nan')
    r = math.fmod(a + 180.0, 360.0)
    if r < 0.0:
        r += 360.0
    return r - 180.0


def angle_between_deg(a_deg: float, b_deg: float) -> float:
    """两个朝向角之间的夹角（度，[0, 180]）。"""
    if not (math.isfinite(a_deg) and math.isfinite(b_deg)):
        return float('nan')
    return abs(_wrap_deg(a_deg - b_deg))


# ---------------------------------------------------------------------------
# 方向项
# ---------------------------------------------------------------------------
def direction_error_deg(vx: float,
                        vy: float,
                        std_dir_deg: float,
                        dist_to_target_m: float,
                        stop_speed: float = STOP_SPEED,
                        stop_near_m: float = STOP_NEAR_M) -> float:
    """单拍的**方向误差**（度）。

    口径：

    - `hypot(vx, vy) < stop_speed` → 判为"停车"：
      距目标 `< stop_near_m` 记 **0**（已到位，最优），否则记 **满分误差** 180°。
    - 否则取 `|heading(vx, vy) − std_dir_deg|`（度，折到 [0, 180]）。

    Args:
        vx: 建议 x 速度（m/s）。
        vy: 建议 y 速度（m/s）。
        std_dir_deg: 该拍的**标准方向**朝向角（度，map 系）。
        dist_to_target_m: 底盘到目标的距离（m）。
        stop_speed: 判停车阈值，默认 0.02。
        stop_near_m: 停车时"算到位"的距离阈值，默认 0.5 m。

    Returns:
        float: 误差（度）；速度或标准方向无效时返回 NaN。
    """
    try:
        x, y = float(vx), float(vy)
        d = float(dist_to_target_m)
    except (TypeError, ValueError):
        return float('nan')
    if not (math.isfinite(x) and math.isfinite(y)):
        return float('nan')
    if math.hypot(x, y) < float(stop_speed):
        if math.isfinite(d) and d < float(stop_near_m):
            return 0.0
        return DIR_FULL_ERR_DEG
    h = heading_deg(x, y)
    if not math.isfinite(h):
        return float('nan')
    return angle_between_deg(h, float(std_dir_deg))


def direction_term(err_deg: Sequence[float],
                   weight: float = W_DIR,
                   full_scale_deg: float = DIR_FULL_ERR_DEG) -> Dict[str, Any]:
    """方向项得分贡献。

    Args:
        err_deg: 逐拍方向误差（度）。
        weight: 权重，默认 0.2。
        full_scale_deg: 归一化满量程，默认 180°。

    Returns:
        dict: ``{'rmse_deg', 'norm_rmse', 'contribution', 'n'}``。
    """
    e = np.asarray(err_deg, dtype=np.float64).reshape(-1)
    e = e[np.isfinite(e)]
    n = int(e.size)
    if n == 0 or not math.isfinite(float(full_scale_deg)) or full_scale_deg <= 0.0:
        return {'rmse_deg': float('nan'), 'norm_rmse': float('nan'),
                'contribution': float('nan'), 'n': n}
    r = float(math.sqrt(float(np.mean(e * e))))
    norm = r / float(full_scale_deg)
    return {'rmse_deg': r, 'norm_rmse': norm,
            'contribution': float(weight) * norm, 'n': n}


# ---------------------------------------------------------------------------
# 总分
# ---------------------------------------------------------------------------
def score(pred: Sequence[Sequence[float]],
          cur: Sequence[Sequence[float]],
          pred_true: Sequence[Sequence[float]],
          cur_true: Sequence[Sequence[float]],
          err_dir_deg: Sequence[float],
          names: Sequence[str] = tuple(WEIGHTS.keys())[:4],
          weights: Sequence[float] = W_HEAD4,
          w_dir: float = W_DIR) -> Dict[str, Any]:
    """按口述口径算加权总分（越小越好）。

    Args:
        pred: 逐拍 ``(pred_x, pred_y)``（本队输出）。
        cur: 逐拍 ``(cur_x, cur_y)``（本队输出）。
        pred_true: 逐拍预测位置**准确值**。
        cur_true: 逐拍当前位置**准确值**。
        err_dir_deg: 逐拍方向误差（度），由 :func:`direction_error_deg` 得到。
        names: 前四项名字（默认 ``pred_x, pred_y, cur_x, cur_y``）。
        weights: 前四项权重（默认 ``0.10, 0.20, 0.25, 0.25``）。
        w_dir: 方向项权重（默认 0.2）。

    Returns:
        dict: ``{'total', 'terms', 'dir'}``；``terms`` 为 ``{列名: {rmse, weight,
        contribution, n}}``。某项无有效数据时其 contribution 为 NaN，
        ``total`` 用 ``nansum`` 方式给出并在 ``'degraded'`` 里列出缺项。
    """
    series = {
        'pred_x': ([p[0] for p in pred], [t[0] for t in pred_true]),
        'pred_y': ([p[1] for p in pred], [t[1] for t in pred_true]),
        'cur_x': ([c[0] for c in cur], [t[0] for t in cur_true]),
        'cur_y': ([c[1] for c in cur], [t[1] for t in cur_true]),
    }
    nm = list(names)[:4]
    wt = list(weights)[:4]
    terms: Dict[str, Dict[str, Any]] = {}
    total = 0.0
    degraded: List[str] = []
    for k, name in enumerate(nm):
        est, tru = series.get(str(name), ([], []))
        n = int(min(len(est), len(tru)))
        diffs: List[float] = []
        for i in range(n):
            try:
                a, b = float(est[i]), float(tru[i])
            except (TypeError, ValueError):
                continue
            if math.isfinite(a) and math.isfinite(b):
                diffs.append(a - b)
        r = rmse(diffs)
        w = float(wt[k]) if k < len(wt) else float('nan')
        c = r * w
        terms[str(name)] = {'rmse': r, 'weight': w, 'contribution': c,
                            'n': len(diffs)}
        if math.isfinite(c):
            total += c
        else:
            degraded.append(str(name))
    d = direction_term(err_dir_deg, weight=w_dir)
    if math.isfinite(d['contribution']):
        total += float(d['contribution'])
    else:
        degraded.append('direction')
    return {'total': float(total), 'terms': terms, 'dir': d,
            'weight_sum': float(sum(wt)) + float(w_dir),
            'degraded': degraded}


def format_report(res: Dict[str, Any],
                  meta: Optional[Dict[str, Any]] = None) -> str:
    """把 :func:`score` 的结果渲染成可读文本（复盘用）。"""
    lines: List[str] = []
    lines.append('# 评分复盘（口述口径）')
    lines.append('')
    lines.append('| 项 | 权重 | RMSE | 加权贡献 | 样本 |')
    lines.append('|---|---|---|---|---|')
    order = ['pred_x', 'pred_y', 'cur_x', 'cur_y']
    for name in order:
        t = res['terms'].get(name)
        if t is None:
            continue
        lines.append('| %s | %.2f | %s | %s | %d |'
                     % (name, float(t['weight']), _f(t['rmse'], 4),
                        _f(t['contribution'], 5), int(t['n'])))
    d = res['dir']
    lines.append('| direction | %.2f | %s ° (÷180) | %s | %d |'
                 % (W_DIR, _f(d['rmse_deg'], 3), _f(d['contribution'], 5),
                    int(d['n'])))
    lines.append('| **合计** | %.2f | — | **%s** | — |'
                 % (float(res['weight_sum']), _f(res['total'], 5)))
    lines.append('')
    if res.get('degraded'):
        lines.append('缺数据项（计 0，实为失真）：%s' % ', '.join(res['degraded']))
        lines.append('')
    lines.append('> 越小越好；方向项 1° RMSE ≈ %.5f 总分。'
                 % (W_DIR / DIR_FULL_ERR_DEG))
    if meta:
        lines.append('')
        for k in sorted(meta.keys()):
            lines.append('- %s: %s' % (k, meta[k]))
    return '\n'.join(lines) + '\n'


def _f(v: Any, nd: int = 4) -> str:
    """数值格式化；None / 非有限显示 'n/a'。"""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return 'n/a'
    if not math.isfinite(f):
        return 'n/a'
    return ('%%.%df' % nd) % f
