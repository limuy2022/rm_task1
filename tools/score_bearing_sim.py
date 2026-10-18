"""Score a bearing_sim closed-loop run from its e2e artifacts (offline analysis).

输入：`e2e_evaluator` 落盘的 `final.json` + `final.csv`（逐样本 ``t_s,err_deg``）。
在既有 RMS/max/收敛时间之外补四项闭环质量指标（全部只依赖角度误差序列，
不信任被测节点自报数据）：

* **settle 后超调比**：收敛后 1s 窗口起的峰值 |err| / 稳态中位数 |err|；
* **振荡**：settle 后 ``err − 滑动中位数`` 的符号变化率（次/分钟）；
* **平滑度**：settle 后误差一阶差分的标准差（°/样本，换算 °/s）；
* **丢样**：时间戳间隙 > 3× 中位周期的次数与最大间隙。

并复算 RMS 与 final.json 互检（防读错格式/漏样本）。

用法::

    python3 tools/score_bearing_sim.py build/bearing_sim_final
    python3 tools/score_bearing_sim.py build/bearing_sim_final \
        --max-overshoot-ratio 3 --max-osc-per-min 30 --max-gap-s 0.2

带阈值参数时超限退出码 1；无阈值时纯报告（退出码 0，解析失败才非零）。
"""

from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path

import numpy as np

_SETTLE_EXTRA_S = 1.0     # 收敛后再留 1s 过渡，之后的样本算“稳态”
_MED_WIN = 21             # 滚动中位数窗口（奇数）


def _rolling_median(x: np.ndarray, win: int) -> np.ndarray:
    half = win // 2
    out = np.empty_like(x)
    for i in range(len(x)):
        lo, hi = max(0, i - half), min(len(x), i + half + 1)
        out[i] = np.median(x[lo:hi])
    return out


def score(base: str) -> dict:
    jpath, cpath = Path(base + '.json'), Path(base + '.csv')
    meta = json.loads(jpath.read_text(encoding='utf-8'))
    rows = np.genfromtxt(str(cpath), delimiter=',', names=True)
    t = np.asarray(rows['t_s'], dtype=float)
    err = np.asarray(rows['err_deg'], dtype=float)
    if len(t) < 20:
        raise ValueError('样本过少（%d）' % len(t))

    converged = float(meta.get('converged_at_s') or 0.0)
    n_transient = int(meta.get('n_transient') or 0)
    # 稳态窗口：取“收敛时刻+1s”与“跳过 evaluator 剔除的过渡样本数”的较大者
    settle_idx = int(max(np.searchsorted(t, converged + _SETTLE_EXTRA_S), n_transient))
    settle_idx = min(settle_idx, len(t) - 5)
    ts, es = t[settle_idx:], np.abs(err[settle_idx:])

    # RMS 互检（evaluator 的窗口口径 = 剔除 n_transient 后全部样本）
    rms_re = float(np.sqrt(np.mean(np.square(err[n_transient:]))))
    rms_json = float(meta.get('rms_deg') or float('nan'))

    steady = float(np.median(es)) if len(es) else float('nan')
    peak = float(np.max(es)) if len(es) else float('nan')
    overshoot = (peak / steady) if steady > 1e-6 else float('inf')

    resid = err[settle_idx:] - _rolling_median(err[settle_idx:], _MED_WIN)
    signs = np.sign(resid)
    osc = int(np.sum(signs[1:] * signs[:-1] < 0))
    duration_s = float(t[-1] - t[0])
    osc_per_min = osc / (duration_s / 60.0) if duration_s > 0 else float('nan')

    d_err = np.diff(err[settle_idx:])
    smooth_std = float(np.std(d_err)) if len(d_err) else float('nan')
    rate_hz = (len(t) - 1) / duration_s if duration_s > 0 else float('nan')

    gaps = np.diff(t)
    med_period = float(np.median(gaps)) if len(gaps) else 0.0
    drop_mask = gaps > 3.0 * max(med_period, 1e-9)
    n_drops = int(np.sum(drop_mask))
    max_gap = float(np.max(gaps)) if len(gaps) else 0.0

    return {
        'source': str(Path(base).name),
        'samples': int(len(t)),
        'duration_s': round(duration_s, 3),
        'converged_at_s': converged,
        'settle_window_from_s': round(float(ts[0]) if len(ts) else 0.0, 3),
        'rms_deg_recomputed': round(rms_re, 4),
        'rms_deg_from_json': rms_json,
        'rms_consistent': bool(abs(rms_re - rms_json) <= max(0.02, 0.05 * rms_json)),
        'steady_median_deg': round(steady, 4),
        'post_settle_peak_deg': round(peak, 4),
        'overshoot_ratio': round(overshoot, 3),
        'oscillations_total': osc,
        'oscillations_per_min': round(osc_per_min, 2),
        'smoothness_std_deg': round(smooth_std, 4),
        'smoothness_std_deg_per_s': round(smooth_std * rate_hz, 3) if rate_hz else None,
        'sample_rate_hz': round(rate_hz, 2),
        'n_dropped_gaps': n_drops,
        'max_gap_s': round(max_gap, 4),
    }


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print(__doc__)
        return 2
    base = argv[0]
    limits = {}
    for key in ('--max-overshoot-ratio', '--max-osc-per-min', '--max-gap-s'):
        if key in argv:
            limits[key] = float(argv[argv.index(key) + 1])
    try:
        report = score(base)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print('FAIL 解析 %s: %r' % (base, exc), flush=True)
        return 2

    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if not report['rms_consistent']:
        print('WARN 复算 RMS 与 final.json 不一致，请核对口径', flush=True)
    failed = []
    if '--max-overshoot-ratio' in limits and \
            report['overshoot_ratio'] > limits['--max-overshoot-ratio']:
        failed.append('overshoot_ratio %.2f' % report['overshoot_ratio'])
    if '--max-osc-per-min' in limits and \
            report['oscillations_per_min'] > limits['--max-osc-per-min']:
        failed.append('osc_per_min %.1f' % report['oscillations_per_min'])
    if '--max-gap-s' in limits and report['max_gap_s'] > limits['--max-gap-s']:
        failed.append('max_gap_s %.3f' % report['max_gap_s'])
    if failed:
        print('FAIL 阈值超限: %s' % ', '.join(failed), flush=True)
        return 1
    print('PASS', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
