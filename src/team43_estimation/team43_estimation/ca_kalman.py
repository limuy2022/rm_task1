# -*- coding: utf-8 -*-
"""常加速度（CA）卡尔曼滤波器 —— DESIGN.md §6.7 的参考实现。

纯 Python + numpy，**不依赖 ROS / rclpy**，可在无 ROS 环境下直接 import 并单测。

状态向量（**map 系**）::

    x = [px, py, vx, vy, ax, ay]^T          # 6 维

典型用法（由 ``target_estimator_node`` 驱动）::

    kf = CAKalman(jerk_std=1.0, predict_dt=0.1)
    # 观测到来（t_obs 为 team43_common.time_utils.now() 的单调秒）
    dt = kf.advance(t_obs)          # 内部完成 clamp(dt, 0, max_dt) + predict
    ok = kf.update(z_xy, R_xy)      # NIS 门限内才真正更新
    # 100Hz 定时器
    kf.advance(t_now)
    pred_xy = kf.extrapolate(0.1)   # 100ms 外推

设计要点（与 DESIGN.md 一一对应）:

* 状态转移 ``F``：``F[0,2]=dt; F[0,4]=0.5dt²; F[2,4]=dt``（y 轴同理）
* 过程噪声 ``Q``：piecewise-constant jerk，强度 ``q = jerk_std²``
* 观测 ``H = [[1,0,0,0,0,0],[0,1,0,0,0,0]]``，``R`` 来自 ``observation_builder``（§6.6）
* NIS 门限 16.81（2 自由度 99.9%）；超门限**只 predict 不 update**；``‖d‖ > 1.0m`` → reinit
* 冷启动 ``P = diag(0.05², 0.05², 1.0², 1.0², 5.0², 5.0²)``
* ``trace(P[0:2,0:2]) > 100`` 判发散；出现非有限值 → 恢复/重置
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

# --------------------------------------------------------------------------
# 默认参数（与 DESIGN.md §6.13 的 estimation.* 命名空间一一对应）
# --------------------------------------------------------------------------
DEFAULT_JERK_STD = 1.0            # m/s^3，过程噪声强度 sigma_j
DEFAULT_PREDICT_DT = 0.1          # 100ms 外推
DEFAULT_MAX_DT = 0.5              # dt 上限，防止长时间丢观测后爆炸
DEFAULT_NIS_GATE = 16.81          # 2 自由度 99.9%
DEFAULT_REINIT_JUMP_M = 1.0       # 新息范数超过该值直接 reinit
DEFAULT_P_MAX_TRACE = 100.0       # 位置协方差迹上限，判发散
DEFAULT_INIT_SIGMA_P = 0.05       # 冷启动位置标准差
DEFAULT_INIT_SIGMA_V = 1.0        # 冷启动速度标准差
DEFAULT_INIT_SIGMA_A = 5.0        # 冷启动加速度标准差
DEFAULT_CV_ACC_THRESH = 0.3       # P1：加速度不显著阈值 m/s^2
DEFAULT_CV_HOLD_S = 0.5           # P1：持续时间 s
DEFAULT_CV_DEGRADE_ENABLE = False  # P1：默认关闭

N_STATE = 6
N_OBS = 2

#: 状态块索引 —— x 轴分量 (px, vx, ax) 与 y 轴分量 (py, vy, ay)
IX: Tuple[int, int, int] = (0, 2, 4)
IY: Tuple[int, int, int] = (1, 3, 5)

#: 观测矩阵（只读常量，任何地方不得就地修改）
H_POS = np.array(
    [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
     [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]],
    dtype=np.float64,
)

#: update() 的返回结果枚举
RESULT_INIT = 'init'          # 首次观测，冷启动
RESULT_ACCEPTED = 'accepted'  # 正常更新
RESULT_REJECTED = 'rejected'  # NIS 超门限，本次不更新
RESULT_REINIT = 'reinit'      # 跳变过大 / 发散 / 数值异常后重初始化


def clamp_dt(dt: float, max_dt: float = DEFAULT_MAX_DT) -> float:
    """把 dt 截断到 ``[0, max_dt]``，非有限值返回 0。

    Args:
        dt: 待截断的时间间隔（秒）。
        max_dt: 上限（秒）。

    Returns:
        截断后的 dt，恒为有限值且落在 ``[0, max_dt]``。
    """
    d = float(dt)
    if not math.isfinite(d):
        return 0.0
    if d < 0.0:
        return 0.0
    if d > max_dt:
        return float(max_dt)
    return d


class CAKalman:
    """6 状态（位置/速度/加速度 × x,y）常加速度卡尔曼滤波器。

    Attributes:
        x: 状态向量，shape (6,)。
        P: 协方差矩阵，shape (6, 6)。
        t_last: 上一次推进到的单调时刻（秒）；``None`` 表示尚未推进。
        outlier_count: 被 NIS 门限拒绝的观测计数。
        reinit_count: 重新初始化次数。
        diverged: 是否处于发散标记状态（下一次有效观测会直接 reinit）。
        last_result: 最近一次 ``update`` 的结果枚举字符串。
    """

    def __init__(
        self,
        jerk_std: float = DEFAULT_JERK_STD,
        predict_dt: float = DEFAULT_PREDICT_DT,
        max_dt: float = DEFAULT_MAX_DT,
        nis_gate: float = DEFAULT_NIS_GATE,
        reinit_jump_m: float = DEFAULT_REINIT_JUMP_M,
        p_max_trace: float = DEFAULT_P_MAX_TRACE,
        init_sigma_p: float = DEFAULT_INIT_SIGMA_P,
        init_sigma_v: float = DEFAULT_INIT_SIGMA_V,
        init_sigma_a: float = DEFAULT_INIT_SIGMA_A,
        cv_degrade_enable: bool = DEFAULT_CV_DEGRADE_ENABLE,
        cv_acc_thresh: float = DEFAULT_CV_ACC_THRESH,
        cv_hold_s: float = DEFAULT_CV_HOLD_S,
    ) -> None:
        self.jerk_std = float(jerk_std)
        self.predict_dt = float(predict_dt)
        self.max_dt = float(max_dt)
        self.nis_gate = float(nis_gate)
        self.reinit_jump_m = float(reinit_jump_m)
        self.p_max_trace = float(p_max_trace)
        self.init_sigma_p = float(init_sigma_p)
        self.init_sigma_v = float(init_sigma_v)
        self.init_sigma_a = float(init_sigma_a)
        self.cv_degrade_enable = bool(cv_degrade_enable)
        self.cv_acc_thresh = float(cv_acc_thresh)
        self.cv_hold_s = float(cv_hold_s)

        self.x = np.zeros(N_STATE, dtype=np.float64)
        self.P = self._initial_covariance()
        self.t_last: Optional[float] = None
        self._initialized = False

        self.outlier_count = 0
        self.reinit_count = 0
        self.diverged = False
        self.last_result = RESULT_REJECTED
        self._nis = 0.0
        self._elapsed = 0.0
        self._low_acc_time = 0.0
        self._cv_frozen = False

        self._last_good_x = self.x.copy()
        self._last_good_P = self.P.copy()

    # ------------------------------------------------------------------ 属性
    @property
    def initialized(self) -> bool:
        """是否已完成冷启动（收到过至少一次有效观测）。"""
        return bool(self._initialized)

    @property
    def position(self) -> np.ndarray:
        """当前位置估计 ``[px, py]``，shape (2,)。"""
        return self.x[0:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        """当前速度估计 ``[vx, vy]``，shape (2,)。"""
        return self.x[2:4].copy()

    @property
    def acceleration(self) -> np.ndarray:
        """当前加速度估计 ``[ax, ay]``，shape (2,)。"""
        return self.x[4:6].copy()

    @property
    def cov_xy(self) -> np.ndarray:
        """位置协方差 2x2（``P[0:2, 0:2]``）。"""
        return self.P[0:2, 0:2].copy()

    @property
    def elapsed(self) -> float:
        """自初始化以来累计的预测时间（秒）。"""
        return float(self._elapsed)

    @property
    def cv_frozen(self) -> bool:
        """P1：加速度是否已被冻结（退化为 CV 模型）。"""
        return bool(self._cv_frozen)

    # ------------------------------------------------------------------ 内部
    def _initial_covariance(self) -> np.ndarray:
        """冷启动协方差 ``diag(σp², σp², σv², σv², σa², σa²)``。"""
        sp = self.init_sigma_p
        sv = self.init_sigma_v
        sa = self.init_sigma_a
        return np.diag([sp * sp, sp * sp, sv * sv, sv * sv, sa * sa, sa * sa]).astype(np.float64)

    def _save_good(self) -> None:
        """把当前状态快照为"最近一次有效状态"，供数值异常时回滚。"""
        if self._finite():
            self._last_good_x = self.x.copy()
            self._last_good_P = self.P.copy()

    def _finite(self) -> bool:
        """状态与协方差是否全部有限。"""
        return bool(np.all(np.isfinite(self.x)) and np.all(np.isfinite(self.P)))

    def _restore_or_reinit(self, reason: str) -> str:
        """数值异常后的恢复：优先回滚上一有效状态，否则冷启动。

        Args:
            reason: 触发原因（用于日志/统计）。

        Returns:
            ``RESULT_REINIT``（约定返回值，便于调用方记录）。
        """
        good = self._last_good_x
        good_p = self._last_good_P
        if np.all(np.isfinite(good)) and np.all(np.isfinite(good_p)) and self._initialized:
            self.x = good.copy()
            self.P = good_p.copy()
        else:
            self.x = np.zeros(N_STATE, dtype=np.float64)
            self.P = self._initial_covariance()
            self._initialized = False
        self.reinit_count += 1
        self.diverged = False
        self._nis = 0.0
        self.last_result = RESULT_REINIT
        _ = reason  # 供调用方/断点调试使用，避免 unused 警告
        return RESULT_REINIT

    def _build_F(self, dt: float) -> np.ndarray:
        """构造状态转移矩阵 ``F``（DESIGN §6.7）。"""
        F = np.eye(N_STATE, dtype=np.float64)
        F[0, 2] = dt
        F[0, 4] = 0.5 * dt * dt
        F[2, 4] = dt
        F[1, 3] = dt
        F[1, 5] = 0.5 * dt * dt
        F[3, 5] = dt
        return F

    def _build_Q(self, dt: float) -> np.ndarray:
        """构造过程噪声矩阵 ``Q``（piecewise-constant jerk）。"""
        q = float(self.jerk_std) ** 2
        Q1 = q * np.array(
            [[dt ** 5 / 20.0, dt ** 4 / 8.0, dt ** 3 / 6.0],
             [dt ** 4 / 8.0, dt ** 3 / 3.0, dt ** 2 / 2.0],
             [dt ** 3 / 6.0, dt ** 2 / 2.0, dt]],
            dtype=np.float64,
        )
        Q = np.zeros((N_STATE, N_STATE), dtype=np.float64)
        Q[np.ix_(IX, IX)] = Q1
        Q[np.ix_(IY, IY)] = Q1
        return Q

    def _maybe_degrade_to_cv(self, dt: float) -> None:
        """P1：加速度长期不显著时冻结 ``ax, ay``（退化为 CV 模型）。

        仅在 ``cv_degrade_enable=True`` 时生效；默认关闭。
        """
        if not self.cv_degrade_enable:
            return
        acc_norm = float(np.linalg.norm(self.x[4:6]))
        if acc_norm < self.cv_acc_thresh:
            self._low_acc_time += float(dt)
        else:
            self._low_acc_time = 0.0

        if self._low_acc_time >= self.cv_hold_s:
            self._cv_frozen = True
            self.x[4] = 0.0
            self.x[5] = 0.0
        else:
            self._cv_frozen = False

    # ------------------------------------------------------------------ 接口
    def reset(self) -> None:
        """完全复位到未初始化状态（清掉全部历史）。"""
        self.x = np.zeros(N_STATE, dtype=np.float64)
        self.P = self._initial_covariance()
        self.t_last = None
        self._initialized = False
        self.outlier_count = 0
        self.reinit_count = 0
        self.diverged = False
        self.last_result = RESULT_REJECTED
        self._nis = 0.0
        self._elapsed = 0.0
        self._low_acc_time = 0.0
        self._cv_frozen = False
        self._last_good_x = self.x.copy()
        self._last_good_P = self.P.copy()

    def predict(self, dt: float) -> None:
        """时间更新：``x = F x``；``P = F P Fᵀ + Q``。

        Args:
            dt: 时间间隔（秒）。内部会 clamp 到 ``[0, max_dt]``；``dt <= 0`` 直接返回。
        """
        d = clamp_dt(dt, self.max_dt)
        if d <= 0.0:
            return
        if not self._initialized:
            # 未冷启动：只累计时间，不做状态转移
            self._elapsed += d
            return

        F = self._build_F(d)
        Q = self._build_Q(d)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)  # 强制对称，抑制数值漂移
        self._elapsed += d

        self._maybe_degrade_to_cv(d)

        if not self._finite():
            self._restore_or_reinit('predict 后出现非有限值')
            return
        if float(np.trace(self.P[0:2, 0:2])) > self.p_max_trace:
            self.diverged = True

    def advance(self, t_now: float) -> float:
        """按单调时刻推进滤波器，返回实际使用的 dt。

        等价于 DESIGN §6.7 的时刻管理：``dt = clamp(t_now − t_last, 0, max_dt)``，
        随后 ``predict(dt)`` 并把 ``t_last`` 更新为 ``t_now``。

        Args:
            t_now: 当前单调时刻（``team43_common.time_utils.now()``）。

        Returns:
            实际用于 ``predict`` 的 dt（秒）。首次调用返回 0.0。
        """
        t = float(t_now)
        if self.t_last is None or not math.isfinite(self.t_last):
            self.t_last = t
            return 0.0
        dt = clamp_dt(t - self.t_last, self.max_dt)
        self.predict(dt)
        self.t_last = t
        return dt

    def update(self, z: Sequence[float], R: Sequence[Sequence[float]]) -> bool:
        """观测更新（含 NIS 门限、跳变 reinit、发散保护）。

        Args:
            z: 观测位置 ``[px, py]``（map 系）。
            R: 观测噪声协方差 2x2（来自 ``observation_builder``）。

        Returns:
            ``True`` 表示观测被接受（正常更新 / 冷启动 / reinit）；
            ``False`` 表示被 NIS 门限拒绝（此时只保留 predict 结果）。
        """
        zv = np.asarray(z, dtype=np.float64).reshape(N_OBS)
        Rm = np.asarray(R, dtype=np.float64).reshape(N_OBS, N_OBS)
        if not (np.all(np.isfinite(zv)) and np.all(np.isfinite(Rm))):
            self.last_result = RESULT_REJECTED
            return False

        # 冷启动 / 发散：直接用观测重新播种
        was_initialized = bool(self._initialized)
        if not was_initialized or self.diverged:
            self.reinit(zv)
            self.last_result = RESULT_INIT if not was_initialized else RESULT_REINIT
            return True

        H = H_POS
        PHt = self.P @ H.T                       # (6,2)
        S = H @ PHt + Rm                          # (2,2)
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)

        d = zv - H @ self.x                       # 新息 (2,)
        nis = float(d @ S_inv @ d)
        self._nis = nis

        if not math.isfinite(nis) or nis > self.nis_gate:
            self.outlier_count += 1
            jump = float(np.linalg.norm(d))
            if jump > self.reinit_jump_m:
                self.reinit(zv)
                self.last_result = RESULT_REINIT
                return True
            self.last_result = RESULT_REJECTED
            return False

        # Joseph 形式更新，保证 P 对称正定
        K = PHt @ S_inv                            # (6,2)
        I = np.eye(N_STATE, dtype=np.float64)
        IKH = I - K @ H
        self.x = self.x + K @ d
        self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        if not self._finite():
            self._restore_or_reinit('update 后出现非有限值')
            return False

        self.diverged = bool(float(np.trace(self.P[0:2, 0:2])) > self.p_max_trace)
        self.last_result = RESULT_ACCEPTED
        self._save_good()
        return True

    def reinit(self, pos: Optional[Sequence[float]] = None) -> None:
        """用给定位置重新播种（速度/加速度清零，P 回到冷启动值）。

        Args:
            pos: ``[px, py]``；为 ``None`` 时使用最近一次有效位置（无则 0）。
        """
        if pos is None:
            p = self._last_good_x[0:2].copy() if self._initialized else np.zeros(N_OBS)
        else:
            p = np.asarray(pos, dtype=np.float64).reshape(N_OBS)
        if not np.all(np.isfinite(p)):
            p = np.zeros(N_OBS, dtype=np.float64)

        self.x = np.zeros(N_STATE, dtype=np.float64)
        self.x[0] = p[0]
        self.x[1] = p[1]
        self.P = self._initial_covariance()
        self._initialized = True
        self.diverged = False
        self._nis = 0.0
        self.reinit_count += 1
        self._elapsed = 0.0
        self._low_acc_time = 0.0
        self._cv_frozen = False
        self._save_good()

    def extrapolate(self, dt: Optional[float] = None) -> np.ndarray:
        """外推 ``dt`` 秒后的位置（默认 ``predict_dt = 0.1s``）。

        ``pred = p + v·Δ + 0.5·a·Δ²``

        Args:
            dt: 外推时长（秒）；``None`` 时使用 ``predict_dt``。

        Returns:
            shape (2,) 的外推位置（map 系）。非有限时退化为当前位置。
        """
        d = self.predict_dt if dt is None else float(dt)
        if not math.isfinite(d):
            d = self.predict_dt
        pred = self.x[0:2] + self.x[2:4] * d + 0.5 * self.x[4:6] * d * d
        if not np.all(np.isfinite(pred)):
            return self.x[0:2].copy()
        return pred

    def nis(self) -> float:
        """最近一次 ``update`` 计算的归一化新息平方（NIS）。"""
        return float(self._nis)

    def position_trace(self) -> float:
        """位置协方差的迹，用于发散判据。"""
        return float(np.trace(self.P[0:2, 0:2]))

    def snapshot(self) -> dict:
        """导出一份可读的诊断快照（日志/调试用）。"""
        return {
            'initialized': self.initialized,
            'pos': self.position.tolist(),
            'vel': self.velocity.tolist(),
            'acc': self.acceleration.tolist(),
            'pos_trace': self.position_trace(),
            'nis': self.nis(),
            'outlier_count': int(self.outlier_count),
            'reinit_count': int(self.reinit_count),
            'diverged': bool(self.diverged),
            'last_result': str(self.last_result),
            'cv_frozen': bool(self._cv_frozen),
            'elapsed': float(self._elapsed),
        }
