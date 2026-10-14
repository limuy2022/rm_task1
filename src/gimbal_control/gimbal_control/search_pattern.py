# -*- coding: utf-8 -*-
"""丢失后的搜索扫掠模式 —— DESIGN.md §6.8「丢失与搜索」。

两级扫掠：

| 阶段            | 条件          | yaw 幅值 | 频率   | pitch        |
|-----------------|---------------|----------|--------|--------------|
| 小幅搜索        | ``age > 0.5`` | 0.25 rad | 0.5 Hz | 保持 0       |
| 全范围扫描      | ``age > 5.0`` | 1.40 rad | 0.25 Hz| 0.2 rad 慢扫 |

角速度形式（对正弦位置的解析导数）::

    yaw(t)   = A_yaw · sin(2π f τ)
    ω_yaw(τ) = A_yaw · 2π f · cos(2π f τ)

    pitch(τ) = A_pitch · sin(2π f_p τ)
    ω_pitch(τ) = A_pitch · 2π f_p · cos(2π f_p τ)

其中 ``τ`` 是**自进入搜索阶段起的单调秒**（不是绝对时刻），这样阶段切换时相位连续。

0.25 rad 的选取依据（DESIGN §6.8）：小幅搜索幅值不超过**水平半视场的一半**，
保证扫掠期间目标不会完全出画。

⚠ 水平半视场**不在本文件写死**（DESIGN §8.6：唯一数据源是 ``config/virtual.yaml`` 的
``camera.width`` / ``camera.fx``）。正确用法是由调用方（``gimbal_controller_node``）
读 YAML 后算出 ``half_fov = atan(width/2 / fx)``，通过 ``half_fov_rad`` 形参注入。
本模块只保留一个**保守兜底值** :data:`DEFAULT_HALF_FOV_RAD`，仅在未注入时使用。

纯 Python 实现，**不依赖 ROS**，可直接单测。
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional, Tuple

# --------------------------------------------------------------------------
# 默认参数（DESIGN §6.13 control.*）
# --------------------------------------------------------------------------
DEFAULT_SEARCH_AMP_YAW = 0.25
DEFAULT_SEARCH_FREQ = 0.5
DEFAULT_SEARCH_AMP_PITCH = 0.0

DEFAULT_WIDE_AFTER_S = 5.0
DEFAULT_WIDE_AMP_YAW = 1.4
DEFAULT_WIDE_FREQ = 0.25
DEFAULT_WIDE_AMP_PITCH = 0.2

#: 水平半视场的**保守兜底值**（rad）。
#:
#: 仅在调用方既没有注入 ``half_fov_rad``、也不存在 ``virtual.yaml`` 时使用。
#: 真实值一律由 ``atan(camera.width/2 / camera.fx)`` 计算后注入
#: （禁止在本文件写死内参派生量，见 DESIGN §8.6）。
#: 0.5 rad 取略小于真实值（≈0.52）的保守下界，宁小不大。
DEFAULT_HALF_FOV_RAD = 0.5

TWO_PI = 2.0 * math.pi


def half_fov_rad_from_intrinsics(width: float, fx: float) -> float:
    """由相机内参计算水平半视场 ``atan(width/2 / fx)``（rad）。

    Args:
        width: 图像宽度（像素）。
        fx: 水平焦距（像素）。

    Returns:
        水平半视场（rad，正数）。内参非法时返回 :data:`DEFAULT_HALF_FOV_RAD`。
    """
    w = float(width)
    f = float(fx)
    if not (math.isfinite(w) and math.isfinite(f)) or w <= 0.0 or f <= 0.0:
        return DEFAULT_HALF_FOV_RAD
    out = math.atan(0.5 * w / f)
    return out if math.isfinite(out) and out > 0.0 else DEFAULT_HALF_FOV_RAD


def _coerce_half_fov(half_fov_rad: Optional[float]) -> float:
    """把外部注入的半视场规整为合法正数，非法/缺失时回退保守兜底值。"""
    if half_fov_rad is None:
        return DEFAULT_HALF_FOV_RAD
    v = float(half_fov_rad)
    if not math.isfinite(v) or v <= 0.0:
        return DEFAULT_HALF_FOV_RAD
    return v


class SearchSample(NamedTuple):
    """一次搜索采样结果。

    Attributes:
        yaw_omega: yaw 角速度指令（rad/s，协议域）。
        pitch_omega: pitch 角速度指令（rad/s，协议域）。
        yaw_offset: 相对进入搜索时刻的 yaw 角度偏移（rad，调试用）。
        pitch_offset: 相对进入搜索时刻的 pitch 角度偏移（rad，调试用）。
    """

    yaw_omega: float = 0.0
    pitch_omega: float = 0.0
    yaw_offset: float = 0.0
    pitch_offset: float = 0.0


class SearchPattern:
    """正弦扫掠发生器。

    Args:
        amp_yaw: yaw 扫掠幅值（rad），需为非负。
        freq: 扫掠频率（Hz）。
        amp_pitch: pitch 扫掠幅值（rad），0 表示 pitch 不动。
        pitch_freq: pitch 扫掠频率（Hz）；``None`` 时与 yaw 同频。
        half_fov_rad: 水平半视场（rad）。**应由调用方从 ``virtual.yaml`` 内参算出后注入**
            （``atan(width/2 / fx)``）；``None`` 或非法时回退
            :data:`DEFAULT_HALF_FOV_RAD`（保守兜底）。
        clamp_amp_to_half_fov: ``True`` 时把 ``amp_yaw`` 限制到 ``0.5 × half_fov_rad``
            以内（DESIGN §6.8 的原始约束）。小幅搜索应开启；全范围扫描**故意**
            要扫出视场外，故保持 ``False``。

    Attributes:
        amp_yaw: 实际生效的 yaw 幅值（可能已被半视场上限截断）。
        amp_yaw_unclamped: 调用方传入的原始 yaw 幅值（日志/调试用）。
        clamped_to_half_fov: ``amp_yaw`` 是否被半视场上限截断过。
    """

    def __init__(self,
                 amp_yaw: float = DEFAULT_SEARCH_AMP_YAW,
                 freq: float = DEFAULT_SEARCH_FREQ,
                 amp_pitch: float = DEFAULT_SEARCH_AMP_PITCH,
                 pitch_freq: Optional[float] = None,
                 half_fov_rad: Optional[float] = None,
                 clamp_amp_to_half_fov: bool = False) -> None:
        self.half_fov_rad = _coerce_half_fov(half_fov_rad)

        raw_amp = float(amp_yaw)
        if not math.isfinite(raw_amp):
            raw_amp = 0.0
        raw_amp = max(raw_amp, 0.0)
        self.amp_yaw_unclamped = raw_amp

        self.amp_yaw = raw_amp
        self.clamped_to_half_fov = False
        if clamp_amp_to_half_fov:
            cap = max(0.5 * self.half_fov_rad, 0.0)
            if self.amp_yaw > cap:
                self.amp_yaw = cap
                self.clamped_to_half_fov = True

        self.freq = max(float(freq), 0.0) if math.isfinite(float(freq)) else 0.0
        ap = float(amp_pitch)
        self.amp_pitch = max(ap, 0.0) if math.isfinite(ap) else 0.0
        if pitch_freq is None:
            self.pitch_freq = self.freq
        else:
            pf = float(pitch_freq)
            self.pitch_freq = max(pf, 0.0) if math.isfinite(pf) else 0.0

    # ------------------------------------------------------------------ 接口
    def reset(self) -> None:
        """预留钩子（本实现无内部状态，τ 由调用方持有）。"""
        return None

    def sample(self, t: float) -> Tuple[float, float]:
        """按 DESIGN §4.1 类图签名返回 ``(yaw_omega, pitch_omega)``。

        Args:
            t: 自进入搜索阶段起的单调秒 ``τ``。

        Returns:
            ``(yaw_omega, pitch_omega)`` 二元组（协议域）。
        """
        s = self.sample_full(t)
        return (s.yaw_omega, s.pitch_omega)

    def sample_full(self, t: float) -> SearchSample:
        """同 :meth:`sample`，但额外返回角度偏移（便于绘制/调试）。"""
        tau = float(t)
        if not math.isfinite(tau) or tau < 0.0:
            tau = 0.0

        w_y = TWO_PI * self.freq
        yaw_offset = self.amp_yaw * math.sin(w_y * tau)
        yaw_omega = self.amp_yaw * w_y * math.cos(w_y * tau)

        w_p = TWO_PI * self.pitch_freq
        pitch_offset = self.amp_pitch * math.sin(w_p * tau)
        pitch_omega = self.amp_pitch * w_p * math.cos(w_p * tau)

        if not math.isfinite(yaw_omega):
            yaw_omega = 0.0
        if not math.isfinite(pitch_omega):
            pitch_omega = 0.0
        return SearchSample(
            yaw_omega=float(yaw_omega),
            pitch_omega=float(pitch_omega),
            yaw_offset=float(yaw_offset) if math.isfinite(yaw_offset) else 0.0,
            pitch_offset=float(pitch_offset) if math.isfinite(pitch_offset) else 0.0,
        )

    def max_omega(self) -> Tuple[float, float]:
        """返回该模式下角速度的理论峰值 ``(yaw, pitch)``。"""
        return (abs(self.amp_yaw * TWO_PI * self.freq),
                abs(self.amp_pitch * TWO_PI * self.pitch_freq))

    def within_half_fov(self) -> bool:
        """当前 yaw 幅值是否满足 ``≤ 0.5 × half_fov``（DESIGN §6.8）。"""
        return fits_half_fov(self.amp_yaw, self.half_fov_rad)

    def snapshot(self) -> dict:
        """配置快照（日志用）。"""
        return {
            'amp_yaw': self.amp_yaw,
            'amp_yaw_unclamped': self.amp_yaw_unclamped,
            'clamped_to_half_fov': self.clamped_to_half_fov,
            'half_fov_rad': self.half_fov_rad,
            'freq': self.freq,
            'amp_pitch': self.amp_pitch,
            'pitch_freq': self.pitch_freq,
            'max_omega': list(self.max_omega()),
        }


def make_small_search(amp_yaw: float = DEFAULT_SEARCH_AMP_YAW,
                      freq: float = DEFAULT_SEARCH_FREQ,
                      half_fov_rad: Optional[float] = None) -> SearchPattern:
    """小幅搜索：``A=0.25 rad, f=0.5 Hz``，pitch 保持 0。

    幅值强制限制在 ``0.5 × half_fov_rad`` 以内（DESIGN §6.8 原始约束）。
    """
    return SearchPattern(amp_yaw=amp_yaw, freq=freq,
                         amp_pitch=DEFAULT_SEARCH_AMP_PITCH,
                         half_fov_rad=half_fov_rad,
                         clamp_amp_to_half_fov=True)


def make_wide_search(amp_yaw: float = DEFAULT_WIDE_AMP_YAW,
                     freq: float = DEFAULT_WIDE_FREQ,
                     amp_pitch: float = DEFAULT_WIDE_AMP_PITCH,
                     half_fov_rad: Optional[float] = None) -> SearchPattern:
    """全范围扫描：``A=1.4 rad, f=0.25 Hz``，pitch 0.2 rad 慢扫。

    故意扫出视场外，故**不**按半视场上限截断。
    """
    return SearchPattern(amp_yaw=amp_yaw, freq=freq, amp_pitch=amp_pitch, pitch_freq=freq,
                         half_fov_rad=half_fov_rad, clamp_amp_to_half_fov=False)


def fits_half_fov(amp_yaw: float, half_fov: float = DEFAULT_HALF_FOV_RAD) -> bool:
    """幅值是否不超过半视场的一半（DESIGN §6.8 的约束）。"""
    return float(amp_yaw) <= 0.5 * float(half_fov) + 1e-12
