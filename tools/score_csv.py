#!/usr/bin/env python3
"""Task 1 read-only offline evaluation against an EXPLICIT user-owned reference.
Official environment reference: https://blog.secaumene.com/kit/campus-contest-env.html


Usage: python tools/score_csv.py submission.csv synthetic_reference.csv --field-long-side-m 10
Reference CSV has a header: t_tick,pred_x,pred_y,cur_x,cur_y,base_x,base_y,
and optionally std_dir_deg. pred_* is TRUE position at tick+0.1s. std_dir_deg
must be an independently supplied obstacle-feasible reference direction. This
tool never derives the official direction by just pointing at the target.
No reference direction => position items only, total=None. Without field long
side, base position terms are also unevaluable. No organizer truth is discovered
or read; no simulator/judge/hardware is started and no CSV is modified.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/team43_judge_io'))
from team43_judge_io import scoring as S
from check_csv import _read_submission, _safe_path, _tick_us


def _read_reference(path):
    names = ('t_tick', 'pred_x', 'pred_y', 'cur_x', 'cur_y', 'base_x', 'base_y')
    rows = []
    with _safe_path(path).open('r', encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise ValueError('参考CSV需要无重复列名的表头')
        if not set(names).issubset(reader.fieldnames):
            raise ValueError('参考CSV缺少列：' + ','.join(name for name in names if name not in reader.fieldnames))
        has_direction = 'std_dir_deg' in reader.fieldnames
        for line_number, raw in enumerate(reader, 2):
            if None in raw or any(raw[name] is None for name in names):
                raise ValueError(f'参考CSV第{line_number}行列数不匹配')
            try:
                row = {name: float(raw[name]) for name in names}
                row['_tick_us'] = _tick_us(raw['t_tick'])
                if not all(math.isfinite(row[name]) for name in names):
                    raise ValueError('非有限参考数值')
                if has_direction:
                    row['std_dir_deg'] = float(raw['std_dir_deg'])
                    if not math.isfinite(row['std_dir_deg']):
                        raise ValueError('标准方向不是有限数')
                if rows and row['_tick_us'] <= rows[-1]['_tick_us']:
                    raise ValueError('参考tick必须严格递增')
                rows.append(row)
            except (TypeError, ValueError, OverflowError) as exc:  # 外部CSV/统计输入可能损坏；明确报告无效项，不把它当作有效测量。
                raise ValueError(f'参考CSV第{line_number}行：{exc}') from exc
    if not rows:
        raise ValueError('参考CSV没有数据行，不能评分')
    return rows, has_direction


def evaluate(pred_path, truth_path, tol=0.000001, *, field_long_side_m=None):
    """Evaluate official rules using explicit offline reference ticks.

    tol is timestamp matching tolerance in seconds, not a submission deadline.
    Default matches within one microsecond. A missing/malformed submission tick
    receives maximum errors. Duplicate/ambiguous matches are not counted twice.
    Reference values must be complete/finite; they cannot be fabricated from
    the submitted estimates. Field length must be supplied by the user.
    """
    if not math.isfinite(float(tol)) or tol < 0:
        raise ValueError('tol 必须是非负有限秒数')
    if field_long_side_m is not None:
        if not math.isfinite(float(field_long_side_m)) or float(field_long_side_m) <= 0:
            raise ValueError('场地长边必须为正的有限米数')
    reference, has_direction = _read_reference(truth_path)
    submitted, issues = _read_submission(pred_path)
    reference_times = [row['_tick_us'] for row in reference]
    tolerance_us = float(tol) * 1000000
    assignment, blocked = {}, set()
    extra = ambiguous = 0
    for row in submitted:
        where = bisect.bisect_left(reference_times, row['_tick_us'])
        candidates = [index for index in (where - 1, where) if 0 <= index < len(reference)]
        candidates = [(abs(reference_times[index] - row['_tick_us']), index)
                      for index in candidates
                      if abs(reference_times[index] - row['_tick_us']) <= tolerance_us]
        candidates.sort()
        if not candidates:
            extra += 1
            continue
        if len(candidates) == 2 and candidates[0][0] == candidates[1][0]:
            ambiguous += 1
            continue
        index = candidates[0][1]
        if index in assignment or index in blocked:
            assignment.pop(index, None)
            blocked.add(index)
            ambiguous += 1
            continue
        assignment[index] = row
    pred, cur, base, pred_true, cur_true, base_true, direction_errors = ([] for _ in range(7))
    for index, ref in enumerate(reference):
        row = assignment.get(index)
        pred_true.append((ref['pred_x'], ref['pred_y']))
        cur_true.append((ref['cur_x'], ref['cur_y']))
        base_true.append((ref['base_x'], ref['base_y']))
        if row is None:
            pred.append((float('nan'), float('nan')))
            cur.append((float('nan'), float('nan')))
            base.append((float('nan'), float('nan')))
            if has_direction:
                direction_errors.append(S.MISSING_DIR_ERR_DEG)
            continue
        pred.append((row['pred_x'], row['pred_y']))
        cur.append((row['cur_x'], row['cur_y']))
        base.append((row['base_x'], row['base_y']))
        if has_direction:
            distance = math.hypot(ref['cur_x'] - ref['base_x'], ref['cur_y'] - ref['base_y'])
            direction_errors.append(S.direction_error_deg(row['vx'], row['vy'], ref['std_dir_deg'], distance))
    result = S.score(pred, cur, pred_true, cur_true,
                     direction_errors if has_direction else None,
                     base=base, base_true=base_true, field_long_side_m=field_long_side_m)
    result['meta'] = {'matched': len(assignment), 'reference_ticks': len(reference),
                      'missing': len(reference) - len(assignment), 'unmatched_submission_rows': extra,
                      'ambiguous_submission_rows': ambiguous, 'timestamp_tolerance_s': tol,
                      'field_long_side_m': field_long_side_m,
                      'direction_reference': ('用户显式提供std_dir_deg；本工具未校验障碍通行性'
                                              if has_direction else '未提供障碍参考方向，方向和总分未评估'),
                      'format_issues': issues,
                      'latency': '未评估；CSV时间戳匹配不等于80ms实时提交验证',
                      'reference_source': '仅用户显式指定的自有离线参考文件'}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('submission', help='自有提交CSV')
    parser.add_argument('reference', help='用户显式提供的自有合成/离线参考CSV')
    parser.add_argument('--field-long-side-m', type=float)
    parser.add_argument('--tol', type=float, default=0.000001,
                        help='时间戳对齐容差（秒），不是提交延迟；默认1微秒')
    args = parser.parse_args(argv)
    try:
        result = evaluate(args.submission, args.reference, tol=args.tol,
                          field_long_side_m=args.field_long_side_m)
    except (OSError, ValueError, csv.Error) as exc:  # 外部CSV/统计输入可能损坏；明确报告无效项，不把它当作有效测量。
        print(f'无法评分：{exc}', file=sys.stderr)
        return 1
    print(S.format_report(result, meta=result['meta']))
    return 0 if result['total'] is not None else 2


if __name__ == '__main__':
    raise SystemExit(main())
