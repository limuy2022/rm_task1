"""Task 1 official nine-column scoring, for explicit offline reference data.
Official environment reference: https://blog.secaumene.com/kit/campus-contest-env.html


Weights: pred x/y .05 each, current x/y .15 each, base x/y .20 each,
direction .20. At every tick target-axis absolute errors are divided by
max(TRUE base-to-current-target distance, 1m), base-axis errors by the field's
long side, then clipped to [0,1]. RMSE is applied AFTER per-tick normalization
and clipping. Direction errors are similarly clipped after division by 180deg.

No reference data is discovered or read here. A missing submitted value receives
maximum error; missing reference/context makes an item unevaluable. An incomplete
report has total=None, never a deceptively perfect zero. CSV files alone cannot
prove submission latency or obstacle visibility/feasibility.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional, Sequence
import numpy as np

W_PRED_X = W_PRED_Y = 0.05
W_CUR_X = W_CUR_Y = 0.15
W_BASE_X = W_BASE_Y = 0.20
W_DIR = 0.20
W_HEAD4 = (W_PRED_X, W_PRED_Y, W_CUR_X, W_CUR_Y)
W_HEAD6 = W_HEAD4 + (W_BASE_X, W_BASE_Y)
WEIGHTS = dict(zip(('pred_x', 'pred_y', 'cur_x', 'cur_y', 'base_x', 'base_y', 'direction'),
                   W_HEAD6 + (W_DIR,)))
COLS = ['pred_x', 'pred_y', 'cur_x', 'cur_y', 'base_x', 'base_y', 'vx', 'vy']
STOP_SPEED = 0.02
STOP_NEAR_M = 0.5
DIR_FULL_ERR_DEG = 180.0
MISSING_NORMALIZED_ERROR = 1.0
MISSING_DIR_ERR_DEG = DIR_FULL_ERR_DEG


def rms(vals: Sequence[float]) -> float:
    """RMS of finite diagnostic values; score() separately accounts for missing data."""
    values = np.asarray(vals, dtype=float).reshape(-1)
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(values * values))) if values.size else float('nan')


def rmse(errors: Sequence[float]) -> float:
    return rms(errors)


def heading_deg(vx: float, vy: float) -> float:
    try:
        x, y = float(vx), float(vy)
        if not (math.isfinite(x) and math.isfinite(y)) or math.hypot(x, y) <= 1e-12:
            return float('nan')
        return math.degrees(math.atan2(y, x))
    except (TypeError, ValueError, OverflowError):
        return float('nan')


def angle_between_deg(a_deg: float, b_deg: float) -> float:
    try:
        a, b = float(a_deg), float(b_deg)
        if not (math.isfinite(a) and math.isfinite(b)):
            return float('nan')
        return abs((a - b + 180.0) % 360.0 - 180.0)
    except (TypeError, ValueError, OverflowError):
        return float('nan')


def direction_error_deg(vx: float, vy: float, std_dir_deg: float,
                        dist_to_target_m: float,
                        stop_speed: float = STOP_SPEED,
                        stop_near_m: float = STOP_NEAR_M) -> float:
    """Per-tick direction error. Distance must come from the offline reference.

    Strict boundaries: speed < .02 and TRUE distance < .5 permits stopping.
    At .02m/s the vector direction is scored; at .5m stopping gets full error.
    Invalid submitted velocity receives full error. An unknown moving reference
    direction is unevaluable (NaN), not replaced by the direct target bearing.
    """
    try:
        x, y = float(vx), float(vy)
    except (TypeError, ValueError, OverflowError):
        return DIR_FULL_ERR_DEG
    if not (math.isfinite(x) and math.isfinite(y)):
        return DIR_FULL_ERR_DEG
    try:
        distance = float(dist_to_target_m)
    except (TypeError, ValueError, OverflowError):
        distance = float('nan')
    if math.hypot(x, y) < float(stop_speed):
        return 0.0 if math.isfinite(distance) and 0.0 <= distance < stop_near_m else DIR_FULL_ERR_DEG
    return angle_between_deg(heading_deg(x, y), std_dir_deg)


def direction_term(err_deg: Sequence[float], weight: float = W_DIR,
                   full_scale_deg: float = DIR_FULL_ERR_DEG) -> Dict[str, Any]:
    values = np.asarray(err_deg, dtype=float).reshape(-1)
    finite = values[np.isfinite(values)]
    complete = values.size > 0 and finite.size == values.size
    if not math.isfinite(float(full_scale_deg)) or full_scale_deg <= 0:
        finite = np.empty(0)
        complete = False
    normalized = np.clip(np.abs(finite) / full_scale_deg, 0.0, 1.0)
    norm = rms(normalized)
    return {'rmse_deg': rms(finite), 'norm_rmse': norm,
            'contribution': float(weight) * norm, 'weight': float(weight),
            'n': int(finite.size), 'expected_n': int(values.size), 'complete': bool(complete)}


def _coordinate(series, index, axis):
    try:
        return float(series[index][axis])
    except (TypeError, ValueError, IndexError, KeyError, OverflowError):
        return float('nan')


def _position_term(est, reference, axis, denominators, n, weight):
    raw, normalized = [], []
    missing_estimates = clipped = 0
    for index in range(n):
        truth = _coordinate(reference, index, axis)
        denominator = denominators[index]
        if not math.isfinite(truth) or not math.isfinite(denominator) or denominator <= 0:
            # The reference cannot establish this tick's score.
            continue
        value = _coordinate(est, index, axis)
        if not math.isfinite(value):
            normalized.append(MISSING_NORMALIZED_ERROR)
            missing_estimates += 1
            continue
        error = value - truth
        raw.append(error)
        unbounded = abs(error) / denominator
        clipped += int(unbounded > 1.0)
        normalized.append(min(unbounded, 1.0))
    norm = rms(normalized)
    return {'rmse': rms(raw), 'norm_rmse': norm, 'weight': weight,
            'contribution': weight * norm, 'n': len(normalized), 'expected_n': n,
            'raw_n': len(raw), 'missing_estimates': missing_estimates,
            'clipped_samples': clipped, 'complete': n > 0 and len(normalized) == n}


def score(pred: Sequence[Sequence[float]], cur: Sequence[Sequence[float]],
          pred_true: Sequence[Sequence[float]], cur_true: Sequence[Sequence[float]],
          err_dir_deg: Optional[Sequence[float]] = None, *,
          base: Optional[Sequence[Sequence[float]]] = None,
          base_true: Optional[Sequence[Sequence[float]]] = None,
          field_long_side_m: Optional[float] = None) -> Dict[str, Any]:
    """Score explicit aligned reference ticks; lower is better.

    pred_true is the TRUE target position at tick+0.1s; cur_true/base_true refer
    to tick. Missing estimated rows/NaNs get maximum normalized error. Supply
    actual missing estimates as []/NaNs, not a shorter reference tick list.
    base=None means no base estimate was supplied, whereas base=[] explicitly
    means it missed every known reference tick. Missing reference geometry,
    field length or direction leaves total=None; partial_total is diagnostic.
    """
    n = max(len(pred_true), len(cur_true), len(base_true) if base_true is not None else 0)
    denominators = []
    for index in range(n):
        target = [_coordinate(cur_true, index, axis) for axis in (0, 1)]
        origin = [_coordinate(base_true, index, axis) for axis in (0, 1)]
        if all(math.isfinite(v) for v in target + origin):
            denominators.append(max(math.hypot(target[0] - origin[0], target[1] - origin[1]), 1.0))
        else:
            denominators.append(float('nan'))
    try:
        long_side = float(field_long_side_m)
    except (TypeError, ValueError, OverflowError):
        long_side = float('nan')
    base_denominators = [long_side if math.isfinite(long_side) and long_side > 0
                         and base is not None else float('nan')] * n
    terms = {}
    for name, estimate, reference, axis, scales in (
            ('pred_x', pred, pred_true, 0, denominators),
            ('pred_y', pred, pred_true, 1, denominators),
            ('cur_x', cur, cur_true, 0, denominators),
            ('cur_y', cur, cur_true, 1, denominators),
            ('base_x', base, base_true, 0, base_denominators),
            ('base_y', base, base_true, 1, base_denominators)):
        terms[name] = _position_term(estimate, reference, axis, scales, n, WEIGHTS[name])
    direction_values = list(err_dir_deg) if err_dir_deg is not None else []
    # Missing direction reference is unknown, not zero error. The CSV evaluator
    # explicitly supplies 180deg when a submitted row/velocity is invalid.
    direction_values = (direction_values + [float('nan')] * n)[:n]
    direction = direction_term(direction_values)
    degraded = [name for name, term in terms.items() if not term['complete']]
    if not direction['complete']:
        degraded.append('direction')
    contributions = [term['contribution'] for term in terms.values()] + [direction['contribution']]
    partial = sum(value for value in contributions if math.isfinite(value))
    return {'total': float(partial) if not degraded else None,
            'partial_total': float(partial), 'terms': terms, 'dir': direction,
            'weight_sum': sum(WEIGHTS.values()), 'degraded': degraded,
            'expected_n': n, 'rule': 'task1_official_nine_columns'}


def _f(value, digits=4):
    try:
        value = float(value)
        return f'{value:.{digits}f}' if math.isfinite(value) else '未评估'
    except (TypeError, ValueError, OverflowError):
        return '未评估'


def format_report(res: Dict[str, Any], meta: Optional[Dict[str, Any]] = None) -> str:
    lines = ['# 赛题一离线评分（九列规则）', '',
             '| 项 | 权重 | 原始RMSE | 归一化截断后RMSE | 加权贡献 | 有参考的拍数 |',
             '|---|---|---|---|---|---|']
    for name in ('pred_x', 'pred_y', 'cur_x', 'cur_y', 'base_x', 'base_y'):
        term = res['terms'][name]
        lines.append(f"| {name} | {term['weight']:.2f} | {_f(term['rmse'])} m | "
                     f"{_f(term['norm_rmse'])} | {_f(term['contribution'], 5)} | "
                     f"{term['n']}/{term['expected_n']} |")
    term = res['dir']
    lines.append(f"| direction | {W_DIR:.2f} | {_f(term['rmse_deg'])} ° | "
                 f"{_f(term['norm_rmse'])} | {_f(term['contribution'], 5)} | "
                 f"{term['n']}/{term['expected_n']} |")
    lines += [f"| **合计** | {res['weight_sum']:.2f} | — | — | **{_f(res['total'], 5)}** | — |", '',
              '> 每拍先按真实距离/场地长边归一化并截断，再计算RMSE；越小越好。',
              '> 离线CSV不能验证80ms提交时限；没有障碍参考方向时不评估方向和总分。']
    if res['degraded']:
        lines.append('缺少评分依据：' + '、'.join(res['degraded']) + '。部分项之和不作为总分。')
    for key, value in sorted((meta or {}).items()):
        lines.append(f'- {key}: {value}')
    return '\n'.join(lines) + '\n'
