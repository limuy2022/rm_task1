"""角度误差指标与验收判据（DESIGN §3.12 / §7-T05，对齐 PRD §6.1、hardware_spec §5）。

**纯 numpy + ``gimbal_common.frames``（本身就是纯 numpy），不 import rclpy**，
可直接单测：``python -m pytest src/gimbal_debug/test -q``。

判据定义（PRD §6.1，原文）：

```
e_ang(t) = angle( p_target_true − p_gimbal , 相机光轴方向 )     # 在 world 系计算
RMS  ≤ 2.0°     （对收敛段之后 t > t0 + 2.0s 的数据取 RMS）
max  ≤ 5.0°     （同上时段）
收敛 ≤ 2.0s      （e_ang 进入 2.0° 并"保持"）
时长   30s       （中途不失能、不崩溃）
min_samples = 50，sample_rate_hz = 200（hardware_spec §5）
require_swing：目标摆幅 < 水平半视场 → 本次评估 invalid（hardware_spec §5）
```

相机光轴方向（world 系）：``axis_world = R_world←gimbal · R_gimbal←opt · [0,0,1]``。
其中 ``R_gimbal←opt`` 由 ``gimbal_common.frames.R_opt_from_link()`` 给出（**单一真源**，
本文件不重复定义 rpy 常量），``R_gimbal←opt · [0,0,1]`` 恰好 = gimbal 系 ``+x``。

⚠ 「保持」的可判定近似：用「连续 ``window_s``（默认 2.0s）内全部 ≤ tol」代表「进入并保持」，
且要求该窗口有完整后续数据（否则判为"尚不能判定收敛"）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from dataclasses import field as dc_field
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np

from gimbal_common.frames import R_opt_from_link as _frames_R_opt_from_link
from gimbal_common.frames import quat_to_mat as _frames_quat_to_mat

__all__ = [
    'DEFAULT_RMS_MAX_DEG',
    'DEFAULT_ABS_MAX_DEG',
    'DEFAULT_CONVERGENCE_S',
    'DEFAULT_DURATION_S',
    'DEFAULT_MIN_SAMPLES',
    'DEFAULT_SAMPLE_RATE_HZ',
    'DEFAULT_POST_CONVERGENCE_SKIP_S',
    'DEFAULT_CONVERGENCE_TOL_DEG',
    'ErrorSample',
    'EvalThresholds',
    'EvalResult',
    'unit_vec',
    'optical_axis_world',
    'target_direction',
    'angle_error_deg',
    'bearing_azimuth_rad',
    'swing_rad',
    'swing_deg',
    'rms',
    'rms_deg',
    'max_deg',
    'mean_deg',
    'convergence_time',
    'bucket_rms',
    'evaluate',
    'samples_from_arrays',
    'values_from_samples',
    'format_report_md',
]

# ---------------------------------------------------------------------------
# 默认判据（DESIGN §6.13 `debug.*` + hardware_spec §5）
# ---------------------------------------------------------------------------
DEFAULT_RMS_MAX_DEG: float = 2.0
DEFAULT_ABS_MAX_DEG: float = 5.0
DEFAULT_CONVERGENCE_S: float = 2.0
DEFAULT_DURATION_S: float = 30.0
DEFAULT_MIN_SAMPLES: int = 50
DEFAULT_SAMPLE_RATE_HZ: float = 100.0
DEFAULT_POST_CONVERGENCE_SKIP_S: float = 2.0
DEFAULT_CONVERGENCE_TOL_DEG: float = 2.0

_EPS: float = 1e-12


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------
@dataclass
class ErrorSample:
    """单个采样点。

    Attributes:
        t: 单调时钟时刻（s），全序列同一时间域。
        e_deg: 角度误差 ``e_ang``（度）。
        bearing_rad: 目标相对云台原点的方位角（rad，world 系 XY 平面），用于摆幅统计。
        est_err_m: 目标**估计**位置与真值位置的距离（m）；无估计时为 NaN。
        track_status: 0=丢失 1=本帧有观测 2=外推；未知为 -1。
    """

    t: float = 0.0
    e_deg: float = float('nan')
    bearing_rad: float = float('nan')
    est_err_m: float = float('nan')
    track_status: int = -1


@dataclass
class EvalThresholds:
    """验收判据（对齐 PRD §6.1 / hardware_spec §5）。

    Attributes:
        rms_max_deg: RMS 上限（默认 2.0°）。
        abs_max_deg: max 上限（默认 5.0°）。
        convergence_time_s: 收敛时间上限（默认 2.0s，同时也是收敛判定的"保持"窗口）。
        duration_s: 期望时长（默认 30s），仅用于报告，不参与 PASS 判定。
        min_samples: 最少样本数（默认 50），不足判 invalid。
        post_convergence_skip_s: 统计 RMS 时跳过的起始段（默认 2.0s，PRD「收敛段之后」）。
        convergence_tol_deg: 收敛容差（默认 2.0°）。
        require_swing: True 时摆幅 < half_fov_rad 判 invalid（hardware_spec §5）。
        half_fov_rad: 相机水平半视场（rad）；由调用方从 virtual.yaml 计算后传入，
            **不在此处硬编码**（DESIGN §8.6 单一数据源）。
        swing_margin: 摆幅判定裕度系数（默认 1.0，即严格 < half_fov 判 invalid）。
    """

    rms_max_deg: float = DEFAULT_RMS_MAX_DEG
    abs_max_deg: float = DEFAULT_ABS_MAX_DEG
    convergence_time_s: float = DEFAULT_CONVERGENCE_S
    duration_s: float = DEFAULT_DURATION_S
    min_samples: int = DEFAULT_MIN_SAMPLES
    post_convergence_skip_s: float = DEFAULT_POST_CONVERGENCE_SKIP_S
    convergence_tol_deg: float = DEFAULT_CONVERGENCE_TOL_DEG
    require_swing: bool = False
    half_fov_rad: float = 0.0
    swing_margin: float = 1.0


@dataclass
class EvalResult:
    """评估结果。

    Attributes:
        valid: 数据是否足以评估（样本数够、且（可选）摆幅达标）。
        reason: ``valid`` 为 False 时的原因；否则为空串。
        n_samples: 有效样本数。
        n_used: 参与 RMS/max 统计的样本数（跳过后收敛段之前的部分）。
        rms_deg / max_deg: 角度误差 RMS / 最大值（度）。
        mean_deg: 均值（度）。
        convergence_time_s: 首次进入并保持的**相对**时刻（s，相对首个样本）；未收敛为 None。
        converged: 是否在 ``convergence_time_s`` 上限内完成收敛。
        observed_duration_s: 实际观测时长（s）。
        swing_deg: 摆幅（度）。
        track_status_hist: ``{status: count}``。
        est_err_mean_m / est_err_max_m: 估计 vs 真值位置误差统计（world 系，m）。
        passed: RMS / max / 收敛三项全部达标且 ``valid`` 为 True。
    """

    valid: bool = False
    reason: str = ''
    n_samples: int = 0
    n_used: int = 0
    rms_deg: float = float('nan')
    max_deg: float = float('nan')
    mean_deg: float = float('nan')
    convergence_time_s: Optional[float] = None
    converged: bool = False
    observed_duration_s: float = 0.0
    swing_deg: float = 0.0
    track_status_hist: Dict[int, int] = dc_field(default_factory=dict)
    est_err_mean_m: float = float('nan')
    est_err_max_m: float = float('nan')
    passed: bool = False

    def as_dict(self) -> Dict[str, Any]:
        """转成 JSON 可序列化的 dict（给 run_meta / 报告用）。"""
        return {
            'valid': bool(self.valid),
            'reason': self.reason,
            'n_samples': int(self.n_samples),
            'n_used': int(self.n_used),
            'rms_deg': _f(self.rms_deg),
            'max_deg': _f(self.max_deg),
            'mean_deg': _f(self.mean_deg),
            'convergence_time_s': (None if self.convergence_time_s is None
                                   else float(self.convergence_time_s)),
            'converged': bool(self.converged),
            'observed_duration_s': float(self.observed_duration_s),
            'swing_deg': float(self.swing_deg),
            'track_status_hist': {str(k): int(v)
                                  for k, v in self.track_status_hist.items()},
            'est_err_mean_m': _f(self.est_err_mean_m),
            'est_err_max_m': _f(self.est_err_max_m),
            'passed': bool(self.passed),
        }


# ---------------------------------------------------------------------------
# 基础几何
# ---------------------------------------------------------------------------
def unit_vec(v: Sequence[float], eps: float = _EPS) -> Optional[np.ndarray]:
    """归一化三维向量；退化（零向量/非有限）时返回 None。"""
    a = np.asarray(v, dtype=np.float64).reshape(-1)
    if a.size < 3:
        a = np.concatenate([a, np.zeros(3 - a.size, dtype=np.float64)])
    a = a[:3]
    n = float(np.linalg.norm(a))
    if not math.isfinite(n) or n <= eps:
        return None
    return a / n


def optical_axis_world(quat_wxyz: Sequence[float],
                       R_gimbal_from_opt: Optional[np.ndarray] = None
                       ) -> Optional[np.ndarray]:
    """相机光轴在 **world** 系的单位向量。

    ``axis_world = R_world←gimbal · R_gimbal←opt · [0,0,1]``，
    ``R_world←gimbal = quat_to_mat(quat)``（``GimbalState.quat`` 即 world→gimbal_link）。

    Args:
        quat_wxyz: 四元数 (w,x,y,z)，``GimbalState.quat``。
        R_gimbal_from_opt: (3,3) 相机光学系→云台系旋转；None 时用
            ``gimbal_common.frames.R_opt_from_link()``（单一真源）。

    Returns:
        np.ndarray | None: (3,) 单位向量；输入退化时 None。

    """
    R_wg = _frames_quat_to_mat(np.asarray(quat_wxyz, dtype=np.float64).reshape(4))
    R_go = _resolve_R_gimbal_from_opt(R_gimbal_from_opt)
    axis_gimbal = R_go @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return unit_vec(R_wg @ axis_gimbal)


def _resolve_R_gimbal_from_opt(R_gimbal_from_opt: Optional[np.ndarray]) -> np.ndarray:
    """取 ``R_gimbal←opt``：优先用调用方注入，否则用 gimbal_common.frames。"""
    if R_gimbal_from_opt is not None:
        return np.asarray(R_gimbal_from_opt, dtype=np.float64).reshape(3, 3)
    return np.asarray(_frames_R_opt_from_link(), dtype=np.float64).reshape(3, 3)


def target_direction(p_target_world: Sequence[float],
                     p_gimbal_world: Sequence[float]
                     ) -> Optional[np.ndarray]:
    """``normalize(p_target_true − p_gimbal)``（world 系）；退化时 None。"""
    pt = np.asarray(p_target_world, dtype=np.float64).reshape(-1)
    pg = np.asarray(p_gimbal_world, dtype=np.float64).reshape(-1)
    if pt.size < 3:
        pt = np.concatenate([pt, np.zeros(3 - pt.size, dtype=np.float64)])
    if pg.size < 3:
        pg = np.concatenate([pg, np.zeros(3 - pg.size, dtype=np.float64)])
    return unit_vec(pt[:3] - pg[:3])


def angle_error_deg(p_target_world: Sequence[float],
                    p_gimbal_world: Sequence[float],
                    quat_wxyz: Sequence[float],
                    R_gimbal_from_opt: Optional[np.ndarray] = None) -> float:
    """PRD §6.1 的 ``e_ang``（度）：目标视线与相机光轴的夹角，在 world 系计算。

    Args:
        p_target_world: 目标真值位置（world 系，(3,) 或 (2,)）。
        p_gimbal_world: 云台原点位置（world 系）。
        quat_wxyz: ``GimbalState.quat``（world→gimbal_link）。
        R_gimbal_from_opt: 可选注入的相机外参旋转。

    Returns:
        float: ``e_ang``（度，[0, 180]）；输入退化时返回 NaN。
    """
    d = target_direction(p_target_world, p_gimbal_world)
    ax = optical_axis_world(quat_wxyz, R_gimbal_from_opt)
    if d is None or ax is None:
        return float('nan')
    c = float(np.dot(ax, d))
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def bearing_azimuth_rad(p_target_world: Sequence[float],
                        p_gimbal_world: Sequence[float]) -> float:
    """目标相对云台原点的方位角（rad，world 系 XY 平面）；退化时 NaN。"""
    d = target_direction(p_target_world, p_gimbal_world)
    if d is None:
        return float('nan')
    return math.atan2(float(d[1]), float(d[0]))


def swing_rad(bearings: Sequence[float]) -> float:
    """方位角序列的摆幅（rad，先解缠再取 max−min）；不足 2 个有效值时 0。"""
    b = np.asarray([x for x in np.asarray(bearings, dtype=np.float64).reshape(-1)
                    if math.isfinite(float(x))], dtype=np.float64)
    if b.size < 2:
        return 0.0
    return float(np.max(np.unwrap(b)) - np.min(np.unwrap(b)))


def swing_deg(bearings: Sequence[float]) -> float:
    """方位角摆幅（度）。"""
    return math.degrees(swing_rad(bearings))


# ---------------------------------------------------------------------------
# 统计
# ---------------------------------------------------------------------------
def _finite_array(vals: Sequence[float]) -> np.ndarray:
    """过滤出有限值，返回 float64 一维数组。"""
    a = np.asarray(vals, dtype=np.float64).reshape(-1)
    return a[np.isfinite(a)]


def rms(vals: Sequence[float]) -> float:
    """均方根；无有效值返回 NaN。"""
    a = _finite_array(vals)
    if a.size == 0:
        return float('nan')
    return float(math.sqrt(float(np.mean(a * a))))


def rms_deg(vals_deg: Sequence[float]) -> float:
    """角度 RMS（度）。"""
    return rms(vals_deg)


def max_deg(vals_deg: Sequence[float]) -> float:
    """角度最大值（度）；无有效值返回 NaN。"""
    a = _finite_array(vals_deg)
    return float(np.max(a)) if a.size > 0 else float('nan')


def mean_deg(vals_deg: Sequence[float]) -> float:
    """角度均值（度）；无有效值返回 NaN。"""
    a = _finite_array(vals_deg)
    return float(np.mean(a)) if a.size > 0 else float('nan')


def convergence_time(t_rel: Sequence[float],
                     e_deg: Sequence[float],
                     window_s: float = DEFAULT_CONVERGENCE_S,
                     tol_deg: float = DEFAULT_CONVERGENCE_TOL_DEG
                     ) -> Tuple[Optional[float], bool]:
    """首次「进入 ``tol_deg`` 并在其后 ``window_s`` 内保持」的时刻。

    「保持」用「连续 ``window_s`` 内全部 ≤ tol」近似，并要求该窗口有完整后续数据
    （否则无法判定，继续向后找）。

    Args:
        t_rel: 相对时间序列（s，单调不减即可，内部会排序）。
        e_deg: 与 ``t_rel`` 等长的角度误差序列（度）。
        window_s: 保持窗口（s），默认 2.0。
        tol_deg: 收敛容差（度），默认 2.0。

    Returns:
        tuple: ``(t_converge, converged)``。``t_converge`` 为相对首个样本的秒数；
        未能判定收敛时为 None。
    """
    t = np.asarray(t_rel, dtype=np.float64).reshape(-1)
    e = np.asarray(e_deg, dtype=np.float64).reshape(-1)
    n = int(min(t.size, e.size))
    if n == 0:
        return None, False
    t, e = t[:n], e[:n]
    m_fin = np.isfinite(t) & np.isfinite(e)
    t, e = t[m_fin], e[m_fin]
    if t.size == 0:
        return None, False
    order = np.argsort(t, kind='stable')
    t, e = t[order], e[order]
    t_end = float(t[-1])
    w = max(0.0, float(window_s))
    for k in range(t.size):
        if t[k] + w > t_end + 1e-9:
            break                     # 后续数据不足 window_s，无法判定"保持"
        sel = (t >= t[k] - 1e-12) & (t <= t[k] + w + 1e-12)
        if bool(np.all(e[sel] <= tol_deg)):
            return float(t[k]), True
    return None, False


def bucket_rms(samples: Sequence[ErrorSample],
               bucket_s: float = 5.0,
               t0: Optional[float] = None) -> List[Dict[str, Any]]:
    """按 ``bucket_s`` 秒分段统计 RMS / max，便于定位误差出现在哪个时段。

    Args:
        samples: 采样序列。
        bucket_s: 分段长度（s）。
        t0: 时间基准；None 时取首个样本时刻。

    Returns:
        list[dict]: ``[{'t_start','t_end','n','rms_deg','max_deg'}, ...]``。
    """
    s = [x for x in samples if math.isfinite(float(x.e_deg))]
    if not s:
        return []
    if t0 is None:
        t0 = min(float(x.t) for x in s)
    bw = max(1e-6, float(bucket_s))
    t_end = max(float(x.t) for x in s) - t0
    n_b = int(math.floor(t_end / bw)) + 1
    out: List[Dict[str, Any]] = []
    for b in range(n_b):
        lo = b * bw
        hi = lo + bw
        vals = [float(x.e_deg) for x in s if lo <= (float(x.t) - t0) < hi]
        if not vals:
            continue
        out.append({
            't_start': lo,
            't_end': hi,
            'n': len(vals),
            'rms_deg': rms_deg(vals),
            'max_deg': max_deg(vals),
        })
    return out


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------
def samples_from_arrays(t: Sequence[float],
                        e_deg: Sequence[float],
                        bearings: Optional[Sequence[float]] = None,
                        est_err_m: Optional[Sequence[float]] = None,
                        track_status: Optional[Sequence[int]] = None
                        ) -> List[ErrorSample]:
    """由平行数组构造 ``ErrorSample`` 列表（缺省字段填 NaN / -1）。"""
    n = int(min(len(t), len(e_deg)))
    out: List[ErrorSample] = []
    for k in range(n):
        out.append(ErrorSample(
            t=float(t[k]),
            e_deg=float(e_deg[k]),
            bearing_rad=(float(bearings[k]) if bearings is not None
                         and k < len(bearings) else float('nan')),
            est_err_m=(float(est_err_m[k]) if est_err_m is not None
                       and k < len(est_err_m) else float('nan')),
            track_status=(int(track_status[k]) if track_status is not None
                          and k < len(track_status) else -1),
        ))
    return out


def values_from_samples(samples: Sequence[ErrorSample]) -> Tuple[List[float], List[float]]:
    """抽出 ``(t, e_deg)`` 两个列表（仅保留有限值）。"""
    ts: List[float] = []
    es: List[float] = []
    for s in samples:
        if math.isfinite(float(s.t)) and math.isfinite(float(s.e_deg)):
            ts.append(float(s.t))
            es.append(float(s.e_deg))
    return ts, es


def evaluate(samples: Sequence[ErrorSample],
             thresholds: Optional[EvalThresholds] = None,
             t0: Optional[float] = None) -> EvalResult:
    """按 PRD §6.1 / hardware_spec §5 判据评估一段 ``e_ang`` 采样序列。

    Args:
        samples: ``ErrorSample`` 序列（时间用单调时钟，同一时间域）。
        thresholds: 判据；None 时用默认（RMS 2.0 / max 5.0 / 收敛 2.0 / 30s / ≥50 样本）。
        t0: 时间基准（相对时间的零点）；None 时取首个有限样本时刻。

    Returns:
        EvalResult: 见 :class:`EvalResult`。
    """
    th = thresholds if thresholds is not None else EvalThresholds()
    res = EvalResult()
    ts, es = values_from_samples(samples)
    res.n_samples = len(ts)
    if res.n_samples == 0:
        res.reason = '无有效角度误差样本'
        return res
    if t0 is None:
        t0 = ts[0]
    t_rel = [x - t0 for x in ts]
    res.observed_duration_s = max(t_rel) - min(t_rel)

    # 摆幅（hardware_spec §5）
    res.swing_deg = swing_deg([s.bearing_rad for s in samples])

    # 统计窗口：PRD「收敛段之后 t > t0 + 2.0s」
    skip = max(0.0, float(th.post_convergence_skip_s))
    used = [e for tr, e in zip(t_rel, es) if tr >= skip - 1e-12]
    window_note = ''
    if not used:
        used = es                       # 数据短于跳过段 → 退化为全段，并在 reason 里说明
        window_note = f'（数据不足 {skip:.1f}s，RMS/max 退化为全段统计）'
    res.n_used = len(used)
    res.rms_deg = rms_deg(used)
    res.max_deg = max_deg(used)
    res.mean_deg = mean_deg(used)

    # 收敛
    t_c, ok = convergence_time(t_rel, es,
                               window_s=th.convergence_time_s,
                               tol_deg=th.convergence_tol_deg)
    res.convergence_time_s = t_c
    res.converged = bool(ok and t_c is not None and t_c <= th.convergence_time_s + 1e-9)

    # 样本数 / 摆幅有效性
    reasons: List[str] = []
    if res.n_samples < int(th.min_samples):
        reasons.append(f'样本数 {res.n_samples} < {int(th.min_samples)}')
    if th.require_swing and th.half_fov_rad > 0.0:
        if swing_rad([s.bearing_rad for s in samples]) < th.half_fov_rad * float(th.swing_margin):
            reasons.append('目标摆幅 %.2f° < 半视场 %.2f°（本次评估不可信）'
                           % (res.swing_deg, math.degrees(th.half_fov_rad)))
    if not res.converged:
        reasons.append('未在 %.1fs 内判定收敛（tol %.1f°）'
                       % (th.convergence_time_s, th.convergence_tol_deg))

    # track_status 直方图 + 估计误差统计
    hist: Dict[int, int] = {}
    for s in samples:
        hist[int(s.track_status)] = hist.get(int(s.track_status), 0) + 1
    res.track_status_hist = hist
    ee = [float(s.est_err_m) for s in samples if math.isfinite(float(s.est_err_m))]
    if ee:
        res.est_err_mean_m = float(np.mean(ee))
        res.est_err_max_m = float(np.max(ee))

    # 判定
    hard_fail: List[str] = []
    if not math.isfinite(res.rms_deg) or res.rms_deg > th.rms_max_deg + 1e-9:
        hard_fail.append('RMS %.3f° > %.3f°' % (res.rms_deg, th.rms_max_deg))
    if not math.isfinite(res.max_deg) or res.max_deg > th.abs_max_deg + 1e-9:
        hard_fail.append('max %.3f° > %.3f°' % (res.max_deg, th.abs_max_deg))

    res.valid = not reasons
    res.reason = '；'.join(reasons + ([window_note] if window_note else []))
    res.passed = bool(res.valid and not hard_fail)
    if not res.valid and hard_fail:
        res.reason = res.reason + '；' + '；'.join(hard_fail)
    return res


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
def format_report_md(result: EvalResult,
                     thresholds: Optional[EvalThresholds] = None,
                     meta: Optional[Dict[str, Any]] = None,
                     buckets: Optional[Sequence[Dict[str, Any]]] = None,
                     notes: Optional[Sequence[str]] = None) -> str:
    """把评估结果渲染成 ``eval_report.md`` 正文（纯字符串，可单测）。"""
    th = thresholds if thresholds is not None else EvalThresholds()
    m = dict(meta or {})
    lines: List[str] = []
    lines.append('# 评估报告（eval_report.md）')
    lines.append('')
    lines.append('> 由 `gimbal_debug/gt_error_monitor_node` 在退出时生成。')
    lines.append('> 判据来源：PRD §6.1 与 `docs/hardware_spec.md` §5。')
    lines.append('')
    lines.append('## 1. 结论')
    lines.append('')
    lines.append('| 判据 | 实测 | 目标 | 判定 |')
    lines.append('|---|---|---|---|')
    lines.append('| 角度 RMS | %s ° | ≤ %.2f ° | %s |'
                 % (_fmt(result.rms_deg, 3), th.rms_max_deg,
                    _mark(_le(result.rms_deg, th.rms_max_deg))))
    lines.append('| 角度 max | %s ° | ≤ %.2f ° | %s |'
                 % (_fmt(result.max_deg, 3), th.abs_max_deg,
                    _mark(_le(result.max_deg, th.abs_max_deg))))
    lines.append('| 收敛时间 | %s s | ≤ %.2f s | %s |'
                 % (_fmt(result.convergence_time_s, 3), th.convergence_time_s,
                    _mark(result.converged)))
    lines.append('| 样本数 | %d | ≥ %d | %s |'
                 % (result.n_samples, int(th.min_samples),
                    _mark(result.n_samples >= int(th.min_samples))))
    lines.append('| 观测时长 | %.3f s | %.1f s（仅报告） | — |'
                 % (result.observed_duration_s, th.duration_s))
    lines.append('')
    lines.append('**汇总判定：%s**' % ('PASS' if result.passed else 'FAIL'))
    if result.reason:
        lines.append('')
        lines.append('无效/备注：%s' % result.reason)
    lines.append('')
    lines.append('## 2. 判据与统计口径')
    lines.append('')
    lines.append('- `e_ang = angle(p_target_true − p_gimbal, 相机光轴方向)`，**world 系**。')
    lines.append('- 相机光轴 `= R_world←gimbal · R_gimbal←opt · [0,0,1]`，'
                 '`R_gimbal←opt` 取自 `gimbal_common.frames.R_opt_from_link()`（单一真源）。')
    lines.append('- RMS/max 统计窗口：`t_rel ≥ %.2f s`（收敛段之后），参与样本 %d / %d。'
                 % (th.post_convergence_skip_s, result.n_used, result.n_samples))
    lines.append('- 「保持」判定：连续 %.2f s 内 `e_ang ≤ %.2f°`。'
                 % (th.convergence_time_s, th.convergence_tol_deg))
    lines.append('- 目标摆幅 %.2f°%s。'
                 % (result.swing_deg,
                    ('（半视场 %.2f°，require_swing=%s）'
                     % (math.degrees(th.half_fov_rad), th.require_swing))
                    if th.half_fov_rad > 0.0 else ''))
    lines.append('')
    lines.append('## 3. 其他统计')
    lines.append('')
    lines.append('| 项 | 值 |')
    lines.append('|---|---|')
    lines.append('| 均值(°) | %s |' % _fmt(result.mean_deg, 3))
    lines.append('| 估计 vs 真值 平均距离(m) | %s |' % _fmt(result.est_err_mean_m, 4))
    lines.append('| 估计 vs 真值 最大距离(m) | %s |' % _fmt(result.est_err_max_m, 4))
    lines.append('| track_status 分布 | %s |'
                 % (', '.join('%s×%d' % (k, v)
                              for k, v in sorted(result.track_status_hist.items()))
                    or 'n/a'))
    lines.append('')

    if buckets:
        lines.append('## 4. 分时段 RMS')
        lines.append('')
        lines.append('| 窗口(s) | 样本 | RMS(°) | max(°) |')
        lines.append('|---|---|---|---|')
        for b in buckets:
            lines.append('| %.1f – %.1f | %d | %s | %s |'
                         % (float(b['t_start']), float(b['t_end']), int(b['n']),
                            _fmt(b['rms_deg'], 3), _fmt(b['max_deg'], 3)))
        lines.append('')

    if m:
        lines.append('## 5. 运行元信息')
        lines.append('')
        lines.append('| 键 | 值 |')
        lines.append('|---|---|')
        for k in sorted(m.keys()):
            lines.append('| %s | %s |' % (k, m[k]))
        lines.append('')

    if notes:
        lines.append('## 6. 备注')
        lines.append('')
        for n in notes:
            lines.append('- %s' % n)
        lines.append('')

    return '\n'.join(lines) + '\n'


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _f(v: float) -> Optional[float]:
    """有限值转 float，非有限转 None（便于 JSON）。"""
    f = float(v)
    return f if math.isfinite(f) else None


def _fmt(v: Any, nd: int = 3) -> str:
    """数值格式化；None / 非有限显示 'n/a'。"""
    if v is None:
        return 'n/a'
    try:
        f = float(v)
    except (TypeError, ValueError):  # 外部CSV/统计输入可能损坏；明确报告无效项，不把它当作有效测量。
        return 'n/a'
    if not math.isfinite(f):
        return 'n/a'
    return ('%%.%df' % nd) % f


def _le(v: Any, limit: float) -> bool:
    """``v <= limit``（非有限视为不达标）。"""
    try:
        f = float(v)
    except (TypeError, ValueError):  # 外部CSV/统计输入可能损坏；明确报告无效项，不把它当作有效测量。
        return False
    return bool(math.isfinite(f) and f <= limit + 1e-9)


def _mark(ok: bool) -> str:
    """✅ / ❌。"""
    return '✅' if ok else '❌'
