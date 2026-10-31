"""赛题一 CSV 行写出（tick 驱动，每行立即 flush）。

默认九列：pred_x,pred_y,cur_x,cur_y,base_x,base_y,vx,vy,t_tick。
t_tick 为评分拍时间戳，固定六位小数；列数固定为九列。

其余硬规则（主办方，与权重无关）：
- 路径：``~/ws_championship/output/<队伍编号>.csv``
- 每收到一拍写一行，**立即 flush**；只追加，已写过的行**不能回头改**
- ``t_tick`` 必须是该拍 ``/judge/tick`` 的 ``stamp``（秒），``%.6f`` 固定小数位
- 迟到（比所属拍晚 >0.08 s 出现）的行作废 → 该拍按缺失计
- 数值列写 ``nan`` 会被读作 NaN → 该拍对应项记**最大误差 1**
- ``t_tick`` 必须是有限数，否则整行匹配不上拍

设计要点：

- 打开用 ``'a+'``（追加并校验同一文件）。节点中途重启时不会覆盖已写行（覆盖=作废）。
  先校验全部已有行并恢复最后六位时间；半行或损坏记录必须拒绝打开，不能修补。
- 每拍核对当前路径；评分端删除、替换或截空时重开，不恢复已清理的旧轮次。
- 同一文件相同/倒序的六位时间不得再次提交；写/flush失败后锁存，直到关闭并重新校验文件。
- 非有限值用**上一次有效值**兜底（比写 NaN 记 1 更好：截断到 1 之前仍有得分机会）。
- ``vx``/``vy`` 由调用方保证有限；``t_tick`` 非有限时**整行不写**（写了也匹配不上）。

日志分发复用 ``team43_common.log_utils.emit``：rclpy(Humble) 的 rcutils
按调用点缓存严重级，单一分发帧混用等级会被吞掉并降级成 info（humble
容器实测），log_utils 的每级独立函数已修复该问题。单测需工作区
PYTHONPATH（容器流程即满足）。
"""

from __future__ import annotations

import math
import os
import re
import fcntl
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence

from team43_common import log_utils


#: 赛题一固定九列，不能省略评分时间戳。
COLS9: List[str] = ['pred_x', 'pred_y', 'cur_x', 'cur_y', 'base_x', 'base_y',
                    'vx', 'vy', 't_tick']
N_COLS: int = len(COLS9)

#: 评分系统判定"停车"的速度阈值（hypot(vx,vy) < 0.02）
V_STOP: float = 0.02

_INF = float('inf')
_TICK_FORMAT = re.compile(r'-?\d+\.\d{6}\Z')


def _tick_key(text: str) -> int:
    """Compare exactly the six decimals submitted, without epoch float roundoff."""
    if _TICK_FORMAT.fullmatch(text) is None:
        raise ValueError('t_tick must use fixed six decimals')
    whole, fraction = text.lstrip('-').split('.')
    value = int(whole) * 1000000 + int(fraction)
    return -value if text.startswith('-') else value


def format_row(pred: Sequence[float], cur: Sequence[float],
               base: Sequence[float], vel: Sequence[float], t_tick: float) -> str:
    """格式化固定九列；非有限数据值写成nan，实际提交由写出器先检查。"""
    vals = list(pred)[:2] + list(cur)[:2] + list(base)[:2] + list(vel)[:2] + [t_tick]
    out = []
    for value in vals:
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError):  # 外部评分数值或历史CSV可能损坏；明确拒绝该记录，不能输出不可解析的行。
            value = float('nan')
        out.append('%.6f' % value if math.isfinite(value) else 'nan')
    return ','.join(out)


def header_line() -> str:
    """仅供调试副本使用；正式CSV不写表头。"""
    return ','.join(COLS9)


class TickCsvWriter:
    """按拍写出固定九列 CSV。

    Attributes:
        path: 主文件路径。
        rows: 已写入行数。
        n_guarded: 触发"非有限值兜底"的次数。
        n_dropped: 因非有限、重复或倒序 ``t_tick`` 丢弃的行数。
    """

    def __init__(self, path: str, logger: Any = None) -> None:
        self.path = str(path)
        self._log = logger
        self._fh = None
        self._last: List[float] = [0.0] * (N_COLS - 1)   # 前 8 个数据列的兜底值
        self._rows: int = 0
        self._n_guarded: int = 0
        self._n_dropped: int = 0
        self._opened: bool = False
        self._last_tick: Optional[int] = None
        self._recovered_rows = 0
        self._failed = False
        self._file_id = None
        self._fd = None
        self._file_size = 0
        self._reopens = 0

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
        """非有限、重复或倒序tick的丢弃次数。"""
        return self._n_dropped

    @property
    def is_open(self) -> bool:
        """文件是否打开。"""
        return self._fh is not None

    # -- 生命周期 -------------------------------------------------------------
    def open(self) -> bool:
        """Lock one append writer and validate existing bytes without repairing them.

        Returns:
            bool：成功为 True。
        """
        if self._fh is not None:
            return not self._failed
        handle = None
        try:
            d = os.path.dirname(os.path.abspath(self.path))
            if d:
                os.makedirs(d, exist_ok=True)
            handle = open(self.path, 'a+', encoding='utf-8', newline='\n')
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            last, tick, recovered = self._recover(handle)
            status = os.fstat(handle.fileno())
        except (OSError, ValueError, UnicodeError) as exc:
            # Lock/disk failure or corrupt existing bytes: refuse, never repair the submission.
            if handle is not None:
                handle.close()
            self._emit('error', 'CSV 打开失败 %s: %r' % (self.path, exc))
            self._fh = None
            return False
        self._fh = handle
        self._fd = handle.fileno()
        self._file_id = (status.st_dev, status.st_ino)
        self._file_size = status.st_size
        self._last, self._last_tick, self._recovered_rows = last, tick, recovered
        self._failed = False
        self._opened = True
        self._emit('info', 'CSV 已打开(追加): %s' % self.path)
        return True

    def _recover(self, handle):
        """校验锁定的同一文件，不另开路径，以免读取到评分端刚替换的文件。"""
        last = [0.0] * 8
        tick = None
        recovered = 0
        handle.seek(0)
        for number, raw in enumerate(handle, 1):
            if not raw.endswith('\n'):
                raise ValueError('incomplete CSV tail at line %d; no records modified' % number)
            body = raw[:-1]
            if body.endswith('\r'):
                body = body[:-1]
            fields = body.split(',')
            if len(fields) != N_COLS:
                raise ValueError('wrong CSV column count at line %d' % number)
            values = [float(value) for value in fields[:8]]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('nonfinite existing data at line %d' % number)
            current = _tick_key(fields[8])
            if tick is not None and current <= tick:
                raise ValueError('duplicate/out-of-order existing tick at line %d' % number)
            last, tick = values, current
            recovered += 1
        handle.seek(0, os.SEEK_END)
        return last, tick, recovered

    def _path_changed(self):
        # 测试注入的写入故障对象没有真实文件身份；真实open一定设置此字段。
        if self._file_id is None:
            return False
        try:
            current = os.stat(self.path)
        except FileNotFoundError:
            return True
        return ((current.st_dev, current.st_ino) != self._file_id
                or current.st_size < self._file_size)

    def _refresh_path(self):
        """评分端删除/替换/截空CSV后，重新打开当前路径；不复制旧轮次内容。"""
        try:
            changed = self._path_changed()
        except OSError as exc:
            self._emit('error', 'CSV路径状态查询失败: %r' % (exc,))
            return False
        if not changed:
            return True
        self.close()
        self._emit('warning', 'CSV被外部删除、替换或截空，重新打开当前评分文件')
        if not self.open():
            return False
        self._reopens += 1
        return True

    def close(self) -> None:
        """flush + 关闭。"""
        if self._fh is None:
            return
        try:
            self._fh.flush()
        except (OSError, ValueError) as exc:  # 关闭文件时磁盘可能失败；标记写出失败并报告，仍释放文件锁。
            self._failed = True
            self._emit('error', 'CSV close flush失败；文件须重新校验: %r' % (exc,))
        finally:
            try:
                self._fh.close()
            finally:
                self._fh = None
                self._fd = None
                self._file_id = None

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
        if self._fh is not None and not self._refresh_path():
            return False
        if self._failed:
            self._emit('error', 'CSV上次写入失败；先关闭并校验文件，不继续追加')
            return False
        if self._fh is None:
            if not self.open():
                return False
        try:
            t = float(t_tick)
        except (TypeError, ValueError, OverflowError):  # 外部评分数值或历史CSV可能损坏；明确拒绝该记录，不能输出不可解析的行。
            t = float('nan')
        if not math.isfinite(t):
            self._n_dropped += 1
            self._emit('warning', 't_tick 非有限，本拍不写（累计 %d）' % self._n_dropped)
            return False
        serialized_tick = '%.6f' % t
        key = _tick_key(serialized_tick)
        if self._last_tick is not None and key <= self._last_tick:
            self._n_dropped += 1
            self._emit('warning', '重复或倒序tick %s，本拍不重复追加' % serialized_tick)
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
            self._emit('warning', '出现非有限值，已用上一次有效值兜底（累计 %d）'
                       % self._n_guarded)

        line = format_row(out[0:2], out[2:4], out[4:6], out[6:8], t)
        payload = line + '\n'
        # 路径若恰好在写/flush期间被清理，再向新文件补当前一拍；不补历史行。
        for attempt in range(2):
            try:
                expected_size = self._file_size + len(payload.encode('utf-8'))
                if self._fh.write(payload) != len(payload):
                    raise OSError('short CSV write')
                self._fh.flush()
                if self._fd is not None:
                    self._file_size = max(expected_size, os.fstat(self._fd).st_size)
                if not self._path_changed():
                    break
            except (OSError, ValueError) as exc:
                self._failed = True
                self._emit('error', 'CSV 写入/flush异常，停止追加，未计成功: %r' % (exc,))
                return False
            if not self._refresh_path():
                return False
            if self._last_tick is not None and key <= self._last_tick:
                self._n_dropped += 1
                return False
        else:
            self._emit('error', 'CSV连续被外部清理，本拍无法确认路径可见')
            return False
        self._last = out
        self._last_tick = key
        self._rows += 1
        return True

    # -- 诊断 -----------------------------------------------------------------
    def stats(self) -> Dict[str, Any]:
        """返回写入统计。"""
        return {'path': self.path, 'rows': self._rows,
                'n_guarded': self._n_guarded, 'n_dropped': self._n_dropped,
                'opened': self._opened, 'n_cols': N_COLS,
                'recovered_rows': self._recovered_rows, 'failed': self._failed,
                'external_reopens': self._reopens}

    # -- 内部 -----------------------------------------------------------------
    @staticmethod
    def _f(seq: Sequence[float], k: int) -> float:
        """取序列第 k 个并转 float；失败/越界返回 NaN。"""
        try:
            return float(seq[k])
        except (TypeError, ValueError, OverflowError, IndexError):  # 外部评分数值或历史CSV可能损坏；明确拒绝该记录，不能输出不可解析的行。
            return float('nan')

    def _emit(self, level: str, msg: str) -> None:
        """无日志器时可独立写 CSV；有日志器时按真实 ROS 日志级别分发。"""
        if self._log is not None:
            log_utils.emit(self._log, level, msg)
