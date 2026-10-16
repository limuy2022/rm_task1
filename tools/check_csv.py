#!/usr/bin/env python3
"""Read-only Task 1 submission format check. No ROS/score-system access.
Official environment reference: https://blog.secaumene.com/kit/campus-contest-env.html


Usage: python tools/check_csv.py path/to/47.csv [--expected-rows 1800]
The stop-trap check uses submitted positions and is only a diagnostic; the actual
judge uses true distance. This file cannot establish flush/80ms arrival latency.
"""
from __future__ import annotations

import argparse
import csv
from decimal import Decimal, InvalidOperation
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gimbal_judge_io'))
from gimbal_judge_io.scoring import STOP_NEAR_M, STOP_SPEED
from gimbal_judge_io.tick_csv import COLS9

_FIXED_TICK = re.compile(r'^[+-]?\d+\.\d{6}$')


def _safe_path(path):
    candidate = Path(path).expanduser().resolve()
    if any(part in ('ws_system', 'juger_runs') for part in candidate.parts):
        raise ValueError('不读取主办方保护目录；请提供自有离线CSV')
    return candidate


def _tick_us(text, fixed=False):
    text = text.strip()
    if fixed and not _FIXED_TICK.fullmatch(text):
        raise ValueError('t_tick 必须为固定六位小数（不用科学计数法）')
    try:
        tick = Decimal(text)
        if not tick.is_finite() or tick < 0:
            raise ValueError('t_tick 必须为非负有限数')
        scaled = tick * 1000000
        if scaled != scaled.to_integral_value():
            raise ValueError('参考 t_tick 精度不能超过微秒')
        return int(scaled)
    except InvalidOperation as exc:
        raise ValueError('t_tick 无法解析') from exc


def _read_submission(path):
    """Return parseable ordered rows + diagnostics; keep bad numeric estimates.

    Invalid data values are retained as NaN so score_csv can assign maximum
    component error. Invalid timestamps/row shape cannot be aligned and are not
    returned. The public checker fails all such diagnostics.
    """
    rows, issues = [], []
    previous = None
    with _safe_path(path).open('r', encoding='utf-8-sig', newline='') as stream:
        for line_number, fields in enumerate(csv.reader(stream), 1):
            if len(fields) != 9:
                issues.append(f'❌ 第{line_number}行：应为九列，实际{len(fields)}列')
                continue
            try:
                tick = _tick_us(fields[8], fixed=True)
            except ValueError as exc:
                issues.append(f'❌ 第{line_number}行：{exc}')
                continue
            if previous is not None and tick <= previous:
                issues.append(f'❌ 第{line_number}行：t_tick 必须严格递增，重复或倒退')
                # Keep it for offline evaluator ambiguity handling; the public
                # checker still fails. Never silently discard a duplicate and
                # give the first row an apparently perfect score.
            previous = tick if previous is None else max(previous, tick)
            row = {'_tick_us': tick, '_line': line_number, 't_tick': tick / 1000000}
            for name, raw in zip(COLS9[:8], fields[:8]):
                try:
                    value = float(raw)
                except (TypeError, ValueError, OverflowError):
                    value = float('nan')
                if not math.isfinite(value):
                    issues.append(f'❌ 第{line_number}行 {name}：非有限或不可解析数值 {raw!r}（nan/inf）')
                row[name] = value
            rows.append(row)
    if not rows:
        issues.append('❌ 没有可对齐的九列数据行')
    return rows, issues


def check_csv(path, cols=9, *, expected_rows=None):
    """Return (passed, report_lines); this never modifies the input file."""
    if cols != 9:
        return False, ['❌ 当前赛题一只接受九列，八列兼容格式不能用于正式提交']
    try:
        rows, issues = _read_submission(path)
    except (OSError, ValueError, csv.Error) as exc:
        return False, [f'❌ CSV读取失败：{exc}']
    lines = list(issues)
    for row in rows:
        values = [row[name] for name in ('vx', 'vy', 'cur_x', 'cur_y', 'base_x', 'base_y')]
        if not all(math.isfinite(value) for value in values):
            continue
        speed = math.hypot(row['vx'], row['vy'])
        distance = math.hypot(row['cur_x'] - row['base_x'], row['cur_y'] - row['base_y'])
        if speed < STOP_SPEED and distance >= STOP_NEAR_M:
            lines.append(f"❌ 第{row['_line']}行：停车陷阱，速度{speed:.6f}m/s、"
                         f'本队估计距离{distance:.3f}m；正式评分使用真实距离')
    if expected_rows is not None and len(rows) != expected_rows:
        lines.append(f'❌ 可对齐行数{len(rows)}，期望{expected_rows}；是否缺拍需对照独立tick记录')
    passed = not any(line.startswith('❌') for line in lines)
    if passed:
        lines.append(f'✅ {len(rows)}行：九列、有限数、固定六位tick、严格递增，未发现估计距离停车陷阱')
    lines.append('说明：文件检查不能证明及时flush、80ms提交延迟或目标/底盘真实精度。')
    return passed, lines


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_path', help='自有离线提交CSV')
    parser.add_argument('--expected-rows', type=int)
    args = parser.parse_args(argv)
    if args.expected_rows is not None and args.expected_rows < 1:
        parser.error('--expected-rows 必须为正整数')
    passed, lines = check_csv(args.csv_path, expected_rows=args.expected_rows)
    print('\n'.join(lines))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
