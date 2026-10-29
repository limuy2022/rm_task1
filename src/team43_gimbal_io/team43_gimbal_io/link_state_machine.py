"""链路使能 / 保活 / 故障恢复状态机（DESIGN §6.3）。

状态::

    [*] --> IDLE : 打开串口
    IDLE --> ACTIVE : 收到第一帧合法 0x01
    IDLE --> IDLE : 有指令请求也丢弃（首帧前不发任何帧）
    ACTIVE --> FAULT_SILENT : 故障（越界/失控/连续校验错/端口写失败）
    FAULT_SILENT --> ACTIVE : 静默 >= fault_silence_s(0.6s)
    ACTIVE/FAULT_SILENT --> SHUTDOWN : 收到退出信号
    SHUTDOWN --> [*] : 连发 5 帧 (0,0) -> close()

| 状态 | 是否发 0x30 |
|---|---|
| IDLE | **否** |
| ACTIVE | 是，100Hz |
| FAULT_SILENT | 否 |
| SHUTDOWN | 是（仅零指令 ×N） |

本模块**不依赖 ROS**，时钟一律走 ``team43_common.time_utils.now``（可注入以便单测）。
"""

from __future__ import annotations

from typing import Deque, Optional, Tuple

from collections import deque

from team43_common.time_utils import now as _default_now


IDLE: str = 'IDLE'
ACTIVE: str = 'ACTIVE'
FAULT_SILENT: str = 'FAULT_SILENT'
SHUTDOWN: str = 'SHUTDOWN'

LINK_STATE_ID = {IDLE: 0, ACTIVE: 1, FAULT_SILENT: 2, SHUTDOWN: 3}
STATE_NAME = {0: IDLE, 1: ACTIVE, 2: FAULT_SILENT, 3: SHUTDOWN}

# 周期比较容差：吸收浮点累加误差，避免 100Hz 回路丢拍（该发没发）。
_PERIOD_EPS_S: float = 1e-9


class LinkStateMachine:
    """链路状态机。

    Args:
        fault_silence_s: 故障静默时长（秒），默认 0.6（下位机要求 0.5，留 0.1 余量）。
        cmd_period_s: 指令周期（秒），默认 0.01（100Hz）。
        yaw_limits: 协议域 yaw 软限位 (min, max)。
        pitch_limits: 协议域 pitch 软限位 (min, max)。
        limit_fault_margin: 越出限位这么多即判定故障。
        runaway_factor: 失控判定系数。
        err_thresh: 失控判定常数项。
        runaway_hold_s: 失控需持续这么久才判定故障。
        bad_frame_window_s: 连续校验错统计窗口（秒）。
        bad_frame_limit: 窗口内 frames_bad 增量上限。
        clock: 时间源（默认 ``time_utils.now``），单测可注入假时钟。
    """

    def __init__(self,
                 fault_silence_s: float = 0.6,
                 cmd_period_s: float = 0.01,
                 yaw_limits: Tuple[float, float] = (-2.8, 2.8),
                 pitch_limits: Tuple[float, float] = (-0.5, 0.35),
                 limit_fault_margin: float = 0.10,
                 runaway_factor: float = 1.5,
                 err_thresh: float = 0.5,
                 runaway_hold_s: float = 0.2,
                 bad_frame_window_s: float = 1.0,
                 bad_frame_limit: int = 50,
                 clock=None) -> None:
        self._fault_silence_s: float = float(fault_silence_s)
        self._cmd_period_s: float = float(cmd_period_s)
        self._yaw_limits: Tuple[float, float] = (float(yaw_limits[0]), float(yaw_limits[1]))
        self._pitch_limits: Tuple[float, float] = (float(pitch_limits[0]), float(pitch_limits[1]))
        self._limit_fault_margin: float = float(limit_fault_margin)
        self._runaway_factor: float = float(runaway_factor)
        self._err_thresh: float = float(err_thresh)
        self._runaway_hold_s: float = float(runaway_hold_s)
        self._bad_window_s: float = float(bad_frame_window_s)
        self._bad_limit: int = int(bad_frame_limit)
        self._now = clock if clock is not None else _default_now

        self._state: str = IDLE
        self._t_enter: float = self._now()
        self._t_last_cmd: Optional[float] = None
        self._imu_seen: bool = False
        self._n_faults: int = 0
        self._last_fault_reason: str = ''
        self._fault_log: Deque[Tuple[float, str]] = deque(maxlen=32)
        self._runaway_since: Optional[float] = None
        self._bad_stamps: Deque[float] = deque()
        self._zero_frames_left: int = 0

    # ---------------------------------------------------------------- 状态查询
    @property
    def state(self) -> str:
        """当前状态名。"""
        return self._state

    @property
    def state_id(self) -> int:
        """当前状态对应的 ``GimbalState.link_state`` 编码。"""
        return LINK_STATE_ID[self._state]

    @property
    def imu_seen(self) -> bool:
        """是否已收到过合法 ``0x01`` 帧。"""
        return self._imu_seen

    @property
    def enabled(self) -> bool:
        """是否处于可下发指令的状态（ACTIVE）。"""
        return self._state == ACTIVE

    @property
    def n_faults(self) -> int:
        """累计故障次数。"""
        return self._n_faults

    @property
    def last_fault_reason(self) -> str:
        """最近一次故障原因。"""
        return self._last_fault_reason

    def time_in_state(self, t: Optional[float] = None) -> float:
        """在当前状态已停留的秒数。"""
        t = self._now() if t is None else float(t)
        return max(0.0, t - self._t_enter)

    def fault_log(self) -> list:
        """返回最近的故障记录 ``[(t, reason), ...]``。"""
        return list(self._fault_log)

    # ---------------------------------------------------------------- 事件
    def on_imu_frame(self, t: Optional[float] = None) -> bool:
        """收到一帧合法 ``0x01``。

        Returns:
            bool: 是否发生了 ``IDLE -> ACTIVE`` 迁移（应打日志）。
        """
        t = self._now() if t is None else float(t)
        transitioned = False
        if not self._imu_seen:
            self._imu_seen = True
        if self._state == IDLE:
            self._set_state(ACTIVE, t)
            transitioned = True
        return transitioned

    def on_fault(self, reason: str, t: Optional[float] = None) -> bool:
        """触发一次故障。

        Args:
            reason: 故障原因（日志必打 WARN）。
            t: 时刻。

        Returns:
            bool: 是否真的发生了状态迁移。
        """
        t = self._now() if t is None else float(t)
        self._n_faults += 1
        self._last_fault_reason = str(reason)
        self._fault_log.append((t, str(reason)))
        self._runaway_since = None
        if self._state == ACTIVE:
            self._set_state(FAULT_SILENT, t)
            return True
        return False

    def tick(self, t: Optional[float] = None) -> bool:
        """周期推进：处理 ``FAULT_SILENT -> ACTIVE`` 恢复。

        Returns:
            bool: 是否发生了状态迁移。
        """
        t = self._now() if t is None else float(t)
        if self._state == FAULT_SILENT and (t - self._t_enter) >= self._fault_silence_s:
            self._set_state(ACTIVE, t)
            return True
        return False

    def cmd_due(self, t: Optional[float] = None) -> bool:
        """是否应当下发一帧 ``0x30``（仅 ACTIVE 状态）。

        Returns:
            bool: 到期则返回 True 并推进相位。
        """
        t = self._now() if t is None else float(t)
        if self._state != ACTIVE:
            return False
        if self._t_last_cmd is None:
            self._t_last_cmd = t
            return True
        elapsed = t - self._t_last_cmd
        if elapsed >= (self._cmd_period_s - _PERIOD_EPS_S):
            # 按既定相位推进，不能以实际回调时刻重新计周期：否则定时器
            # 微小抖动就会导致下一拍被误判为未到期，持续漏发。
            # 错过的周期直接跳过，不集中补发过期指令。
            periods = max(1, int((elapsed + _PERIOD_EPS_S) / self._cmd_period_s))
            self._t_last_cmd += periods * self._cmd_period_s
            return True
        return False

    def request_shutdown(self, zero_frames: int = 5, t: Optional[float] = None) -> int:
        """请求退出。

        Args:
            zero_frames: 退出时连发的零指令帧数。
            t: 时刻。

        Returns:
            int: 需要连发的零指令帧数。
        """
        t = self._now() if t is None else float(t)
        self._zero_frames_left = max(0, int(zero_frames))
        self._set_state(SHUTDOWN, t)
        return self._zero_frames_left

    @property
    def zero_frames_left(self) -> int:
        """退出流程还剩几帧零指令未发。"""
        return self._zero_frames_left

    def take_zero_frame(self) -> bool:
        """取用一帧零指令额度。

        Returns:
            bool: 还有额度则返回 True 并递减。
        """
        if self._zero_frames_left <= 0:
            return False
        self._zero_frames_left -= 1
        return True

    # ---------------------------------------------------------------- 故障判据
    def check_limits(self, yaw: float, pitch: float, t: Optional[float] = None) -> bool:
        """越界判据：超出软限位 + ``limit_fault_margin`` 即故障。

        Args:
            yaw: 协议域 yaw。
            pitch: 协议域 pitch。
            t: 时刻。

        Returns:
            bool: 是否触发了故障。
        """
        t = self._now() if t is None else float(t)
        if self._state != ACTIVE:
            return False
        m = self._limit_fault_margin
        if yaw < self._yaw_limits[0] - m or yaw > self._yaw_limits[1] + m:
            return self.on_fault(f'yaw_out_of_range({yaw:.3f})', t)
        if pitch < self._pitch_limits[0] - m or pitch > self._pitch_limits[1] + m:
            return self.on_fault(f'pitch_out_of_range({pitch:.3f})', t)
        return False

    def check_runaway(self, omega_imu_norm: float, omega_cmd_norm: float,
                      t: Optional[float] = None) -> bool:
        """失控判据：持续 ``runaway_hold_s`` 内 ``|w_imu| > factor*|w_cmd| + err_thresh``。

        Args:
            omega_imu_norm: IMU 角速度范数（rad/s）。
            omega_cmd_norm: 本帧下发指令的范数（rad/s）。
            t: 时刻。

        Returns:
            bool: 是否触发了故障。
        """
        t = self._now() if t is None else float(t)
        if self._state != ACTIVE:
            self._runaway_since = None
            return False
        thresh = self._runaway_factor * float(omega_cmd_norm) + self._err_thresh
        if float(omega_imu_norm) > thresh:
            if self._runaway_since is None:
                self._runaway_since = t
                return False
            if (t - self._runaway_since) >= self._runaway_hold_s:
                self._runaway_since = None
                return self.on_fault(
                    f'runaway(|w_imu|={float(omega_imu_norm):.3f} > {thresh:.3f})', t)
            return False
        self._runaway_since = None
        return False

    def on_bad_frame(self, t: Optional[float] = None) -> bool:
        """记一次校验错帧；窗口内超过上限即故障。

        Args:
            t: 时刻。

        Returns:
            bool: 是否触发了故障。
        """
        t = self._now() if t is None else float(t)
        self._bad_stamps.append(t)
        cutoff = t - self._bad_window_s
        while self._bad_stamps and self._bad_stamps[0] < cutoff:
            self._bad_stamps.popleft()
        if len(self._bad_stamps) > self._bad_limit and self._state == ACTIVE:
            return self.on_fault(f'bad_frames({len(self._bad_stamps)}/{self._bad_window_s}s)', t)
        return False

    def bad_frames_in_window(self) -> int:
        """当前统计窗口内的校验错帧数。"""
        return len(self._bad_stamps)

    def on_write_failure(self, t: Optional[float] = None) -> bool:
        """串口写入失败 → 故障。"""
        return self.on_fault('serial_write_failed', t)

    # ---------------------------------------------------------------- 内部
    def _set_state(self, state: str, t: float) -> None:
        self._state = state
        self._t_enter = float(t)

    def reset(self) -> None:
        """恢复初始状态（主要用于单测）。"""
        self._state = IDLE
        self._t_enter = self._now()
        self._t_last_cmd = None
        self._imu_seen = False
        self._runaway_since = None
        self._bad_stamps.clear()
        self._zero_frames_left = 0
