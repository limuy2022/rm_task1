"""赛题一 CSV 行写出（tick 驱动，每行立即 flush）。

列序：前 8 列是**评分契约**，第 9 列 ``t_tick`` 仅在需要自证拍号时追加：

- **8 列（默认，官方评分要求）**：``pred_x, pred_y, cur_x, cur_y, base_x, base_y, vx, vy``
- **9 列**：上面 8 列 + ``t_tick``（该拍 ``/judge/tick`` 的 ``stamp``，``%.6f``）
  —— 用于自检"有没有丢拍"。官方按**行号**对齐时不需要它。
  由 ``with_tick=True`` 开启（工程侧开关 `judge.emit_t_tick`）。

其余硬规则（主办方，与权重无关）：
- 路径：``~/ws_championship/output/<队伍编号>.csv``
- 每收到一拍写一行，**立即 flush**；只追加，已写过的行**不能回头改**
- ``t_tick`` 必须是该拍 ``/judge/tick`` 的 ``stamp``（秒），``%.6f`` 固定小数位
- 迟到（比所属拍晚 >0.15 s 出现）的行作废 → 该拍按缺失计
- 数值列写 ``nan`` 会被读作 NaN → 该拍对应项记**最大误差 1**
- ``t_tick`` 必须是有限数，否则整行匹配不上拍

设计要点：

- 打开用 ``'a'``（追加）。节点中途重启时不会覆盖已写行（覆盖=作废）。
  若已有文件末行缺 ``\\n``，先补一个换行，避免拼接出坏行。
- 非有限值用**上一次有效值**兜底（比写 NaN 记 1 更好：截断到 1 之前仍有得分机会）。
- ``vx``/``vy`` 由调用方保证有限；``t_tick`` 非有限时**整行不写**（写了也匹配不上）。

纯 Python（不依赖 rclpy），可直接单测。
"""

from __future__ import annotations

import math
import os
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence

__all__ = [
    'COLS8',
    'COLS9',
    'N_COLS',
    'V_STOP',
    'format_row',
    'header_line',
    'TickCsvWriter',
]

#: 数据列（8 列）——口述的"输出八列数字"就是这个顺序
COLS8: List[str] = ['pred_x', 'pred_y', 'cur_x', 'cur_y', 'base_x', 'base_y',
                    'vx', 'vy']

#: 赛题一 9 列列序（主办方写死，不可增删换位）
COLS9: List[str] = COLS8 + ['t_tick']
N_COLS: int = len(COLS9)

#: 评分系统判定"停车"的速度阈值（hypot(vx,vy) < 0.02）
V_STOP: float = 0.02

_INF = float('inf')


def format_row(pred: Sequence[float],
               cur: Sequence[float],
               base: Sequence[float],
               vel: Sequence[float],
               t_tick: float,
               with_tick: bool = True) -> str:
    """格式化一行 CSV（``%.6f``；默认 9 列，8 列需 ``with_tick=False``）。

    Args:
        pred: (pred_x, pred_y)  t_tick+0.1s 的目标预测位置。
        cur: (cur_x, cur_y)     t_tick 时刻目标位置。
        base: (base_x, base_y)  底盘位置。
        vel: (vx, vy)           建议速度。
        t_tick: 该拍 tick 的 stamp（秒）。
        with_tick: True 时追加第 9 列 ``t_tick``；False 时只写 8 列数据。
            本函数默认 True 只是为了保持纯函数的中性；
            **工程默认是 8 列**（`judge.emit_t_tick=false`，官方评分要求）。

    Returns:
        str: 逗号分隔的一行（不含换行符）。非有限数值写成 ``nan``。
    """
    vals = (list(pred)[:2] + list(cur)[:2] + list(base)[:2]
            + list(vel)[:2] + ([t_tick] if with_tick else []))
    out: List[str] = []
    for v in vals:
        try:
            fv = float(v)
        except Exception:
            fv = float('nan')
        out.append('%.6f' % fv if math.isfinite(fv) else 'nan')
    return ','.join(out)


def header_line(with_tick: bool = True) -> str:
    """返回表头行（仅用于调试副本，主文件**不要**写表头）。"""
    return ','.join(COLS9 if with_tick else COLS8)


class TickCsvWriter:
    """按拍写出 CSV（**工程默认 8 列**；`with_tick=True` 时 9 列）。

    Attributes:
        path: 主文件路径。
        rows: 已写入行数。
        n_guarded: 触发"非有限值兜底"的次数。
        n_dropped: 因 ``t_tick`` 非有限而丢弃的行数（仅 9 列模式会触发）。
        with_tick: 是否写第 9 列 ``t_tick``。
    """

    def __init__(self, path: str, logger: Any = None,
                 with_tick: bool = True) -> None:
        self.path = str(path)
        self._log = logger
        self._fh = None
        self._last: List[float] = [0.0] * (N_COLS - 1)   # 前 8 个数据列的兜底值
        self._rows: int = 0
        self._n_guarded: int = 0
        self._n_dropped: int = 0
        self._opened: bool = False
        self._with_tick: bool = bool(with_tick)

    # -- 属性 -----------------------------------------------------------------
    @property
    def rows(self) -> int:
        """已写入行数。"""
        return self._rows

    @property
    def n_guarded(self) -> int:
        """非有限值兜底次数。"""
        return self._n_guarded

    @property
    def n_dropped(self) -> int:
        """因 t_tick 非有限被丢弃的行数。"""
        return self._n_dropped

    @property
    def is_open(self) -> bool:
        """文件是否打开。"""
        return self._fh is not None

    @property
    def n_cols(self) -> int:
        """本写出器每行的列数（9 或 8）。"""
        return N_COLS if self._with_tick else len(COLS8)

    @property
    def with_tick(self) -> bool:
        """是否写 ``t_tick`` 列。"""
        return self._with_tick

    # -- 生命周期 -------------------------------------------------------------
    def open(self) -> bool:
        """以追加模式打开（必要时补尾换行）。

        Returns:
            bool：成功为 True。
        """
        try:
            d = os.path.dirname(os.path.abspath(self.path))
            if d:
                os.makedirs(d, exist_ok=True)
            need_nl = False
            if os.path.exists(self.path) and os.path.getsize(self.path) > 0:
                with open(self.path, 'rb') as rf:
                    rf.seek(-1, os.SEEK_END)
                    need_nl = rf.read(1) != b'\n'
            self._fh = open(self.path, 'a', encoding='utf-8', newline='\n')
            if need_nl:
                self._fh.write('\n')
                self._fh.flush()
        except Exception as exc:
            self._emit('error', 'CSV 打开失败 %s: %r' % (self.path, exc))
            self._fh = None
            return False
        self._opened = True
        self._emit('info', 'CSV 已打开(追加): %s' % self.path)
        return True

    def close(self) -> None:
        """flush + 关闭。"""
        if self._fh is None:
            return
        try:
            self._fh.flush()
        finally:
            try:
                self._fh.close()
            except Exception:
                pass
            self._fh = None

    # -- 写行 -----------------------------------------------------------------
    def write_tick(self,
                   pred: Sequence[float],
                   cur: Sequence[float],
                   base: Sequence[float],
                   vel: Sequence[float],
                   t_tick: float) -> bool:
        """写一拍（9 列）并**立即 flush**。

        非有限数据列用上一次有效值兜底；``t_tick`` 非有限时整行不写。

        Returns:
            bool：写入成功为 True。
        """
        if self._fh is None:
            if not self.open():
                return False
        try:
            t = float(t_tick)
        except Exception:
            t = float('nan')
        if not math.isfinite(t):
            self._n_dropped += 1
            self._emit('warn', 't_tick 非有限，本拍不写（累计 %d）' % self._n_dropped)
            return False

        raw = [self._f(pred, 0), self._f(pred, 1),
               self._f(cur, 0), self._f(cur, 1),
               self._f(base, 0), self._f(base, 1),
               self._f(vel, 0), self._f(vel, 1)]
        out = list(self._last)
        bad = False
        for k in range(8):
            if math.isfinite(raw[k]):
                out[k] = raw[k]
            else:
                bad = True
        if bad:
            self._n_guarded += 1
            self._emit('warn', '出现非有限值，已用上一次有效值兜底（累计 %d）'
                       % self._n_guarded)
        self._last = out

        line = format_row(out[0:2], out[2:4], out[4:6], out[6:8], t,
                          with_tick=self._with_tick)
        try:
            self._fh.write(line + '\n')
            self._fh.flush()
            os.fsync(self._fh.fileno())
        except Exception as exc:
            # fsync 在部分文件系统不可用：退化为只 flush，绝不能因此丢行
            try:
                self._fh.flush()
            except Exception:
                pass
            self._emit('error', 'CSV 写行异常(已尽力 flush): %r' % (exc,))
        self._rows += 1
        return True

    # -- 诊断 -----------------------------------------------------------------
    def stats(self) -> Dict[str, Any]:
        """返回写入统计。"""
        return {'path': self.path, 'rows': self._rows,
                'n_guarded': self._n_guarded, 'n_dropped': self._n_dropped,
                'opened': self._opened, 'n_cols': self.n_cols}

    # -- 内部 -----------------------------------------------------------------
    @staticmethod
    def _f(seq: Sequence[float], k: int) -> float:
        """取序列第 k 个并转 float；失败/越界返回 NaN。"""
        try:
            return float(seq[k])
        except Exception:
            return float('nan')

    def _emit(self, level: str, msg: str) -> None:
        """日志（logger 不可用时静默）。"""
        if self._log is None:
            return
        try:
            getattr(self._log, level if level != 'warn' else 'warning')(msg)
        except Exception:
            try:
                self._log.info(msg)
            except Exception:
                pass
