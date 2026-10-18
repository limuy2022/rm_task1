"""独立只读观察CSV完整行首次可见时间；不导入ROS。必须在测试源启动前运行。"""
import argparse
import json
import math
import re
import time
from pathlib import Path


def check_line(line):
    fields = line.split(',')
    if len(fields) != 9:
        raise ValueError('必须九列')
    values = list(map(float, fields))
    if not all(map(math.isfinite, values)):
        raise ValueError('非有限数值')
    if not re.fullmatch(r'\d+\.\d{6}', fields[8]):
        raise ValueError('时间列须固定六位小数')
    expected = (1., 2., 1., 2., .5, -.25, .3, .4)
    if any(abs(a-b) > 1e-6 for a,b in zip(values[:8], expected)):
        raise ValueError('固定状态测试数值不符')
    return values[8]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv', type=Path)
    p.add_argument('--count', type=int, default=1800)
    p.add_argument('--timeout', type=float, default=240)
    args = p.parse_args()
    if args.count < 1 or args.timeout <= 0:
        p.error('count和timeout必须为正数')
    if args.csv.exists() and args.csv.stat().st_size:
        p.error('测试CSV非空；请使用新的测试目录，避免把旧行当成新提交')
    deadline = time.monotonic() + args.timeout
    handle = None
    pending = b''
    delays = []
    errors = []
    previous = None
    try:
        while time.monotonic() < deadline and len(delays) < args.count:
            if handle is None and args.csv.exists():
                handle = args.csv.open('rb')
            if handle:
                pending += handle.read()
                while b'\n' in pending:
                    raw, pending = pending.split(b'\n', 1)
                    visible = time.time()
                    try:
                        stamp = check_line(raw.decode('utf-8'))
                        if previous is not None and stamp <= previous:
                            raise ValueError('时间不递增或重复拍')
                        previous = stamp
                        delay = visible - stamp
                        if delay < -.001:
                            raise ValueError('时间戳在未来，检查时钟')
                        delays.append(delay)
                    except (ValueError, UnicodeError) as exc:  # 外部CSV/统计输入可能损坏；明确报告无效项，不把它当作有效测量。
                        errors.append(str(exc))
            time.sleep(.002)
    finally:
        if handle:
            handle.close()
    ordered = sorted(delays)
    result = {'rows': len(delays), 'expected_rows': args.count,
              'errors': errors, 'partial_tail_bytes': len(pending),
              'max_visible_delay_ms': max(delays)*1000 if delays else None,
              'p99_visible_delay_ms': ordered[min(len(ordered)-1,int(.99*len(ordered)))]*1000 if ordered else None,
              'over_60ms': sum(d >= .060 for d in delays),
              'over_80ms': sum(d > .080 for d in delays)}
    result['passed'] = (len(delays) == args.count and not errors and not pending
                        and result['over_60ms'] == 0)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['passed'] else 1)

if __name__ == '__main__':
    main()
