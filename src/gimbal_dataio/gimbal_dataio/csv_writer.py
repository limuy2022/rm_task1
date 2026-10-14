"""CSV 行组装与落盘（**默认严格 8 列**，DESIGN §6.12 / 主理人拍板 3）。

⚠⚠ **主理人拍板 3（最高优先级）**：
需求原文写死「输出八列数字」。**默认只写 8 列**：
`pred_x, pred_y, cur_x, cur_y, base_x, base_y, cmd_vx, cmd_vy`，
**无表头**、逗号分隔、`%.6f`。
附加列（`timestamp` / `track_status`）由参数 `csv_extra_cols`（**默认 false**）开启，
开启时**追加到第 9、10 列**，绝不能挤占前 8 列。
同时另写一份 `result_with_header.csv`（同数据，首行表头，列数与主文件一致）。

行为规格：
- 20Hz（50ms）单调定时器驱动（定时器在节点里，本模块只管写）
- 每 `flush_period_s`（默认 1.0s）flush 一次；退出 `close()`
- **有限性守卫**：任一值非有限 → 用**上一次有效值**替代，`track_status` 置 0，WARN 节流 1s

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

try:  # 优先用 T01 的 gimbal_common
    from gimbal_common.csv_utils import COLS8, COLS_EXTRA, format_row
    from gimbal_common.log_utils import get_logger, throttled
    from gimbal_common.time_utils import now
except Exception:  # pragma: no cover - Windows 无 ROS2 兜底
    from ._compat import (COLS8, COLS_EXTRA, format_row, get_logger, now,
                          throttled)

__all__ = [
    'COLS8',
    'COLS_EXTRA',
    'N_COLS_BASE',
    'N_COLS_EXTRA',
    'CsvWriter',
]

N_COLS_BASE: int = 8
N_COLS_EXTRA: int = 10


class CsvWriter:
    """严格 8 列 CSV 写出器（可选追加 2 列）。

    Attributes:
        extra_cols: 是否写第 9/10 列（timestamp / track_status），默认 False。
        flush_period_s: 自动 flush 周期（s）。
        row_count: 已写入的行数。
        n_guarded: 触发有限性守卫的次数。
    """

    def __init__(self,
                 extra_cols: bool = False,
                 flush_period_s: float = 1.0,
                 logger: Any = None) -> None:
        self.extra_cols = bool(extra_cols)
        self.flush_period_s = float(flush_period_s)
        self._log = logger if logger is not None else _safe_logger('csv_writer')

        self._fh = None
        self._fh_hdr = None
        self._path: str = ''
        self._header_path: str = ''
        self._last: List[float] = [0.0] * N_COLS_EXTRA
        self._row_count: int = 0
        self._n_guarded: int = 0
        self._t_last_flush: float = 0.0
        self._closed: bool = True

    # -- 属性 -----------------------------------------------------------------
    @property
    def path(self) -> str:
        """主文件路径。"""
        return self._path

    @property
    def header_path(self) -> str:
        """带表头副本的路径。"""
        return self._header_path

    @property
    def row_count(self) -> int:
        """已写入行数。"""
        return self._row_count

    @property
    def n_guarded(self) -> int:
        """有限性守卫触发次数（诊断用）。"""
        return self._n_guarded

    @property
    def is_open(self) -> bool:
        """文件是否处于打开状态。"""
        return self._fh is not None

    @property
    def last_values(self) -> List[float]:
        """上一次写出的 10 个值（前 8 为数据列）。"""
        return list(self._last)

    # -- 生命周期 -------------------------------------------------------------
    def open(self, path: str, header_path: Optional[str] = None) -> bool:
        """打开主文件与带表头副本。

        主文件**无表头**；副本首行写表头（列数与主文件一致）。

        Args:
            path: 主文件路径（如 `outputs/latest/result.csv`）。
            header_path: 副本路径；None 时自动取同目录下 `result_with_header.csv`。

        Returns:
            bool：成功为 True。
        """
        if header_path is None:
            d = os.path.dirname(os.path.abspath(path))
            header_path = os.path.join(d, 'result_with_header.csv')
        try:
            _ensure_dir(os.path.dirname(os.path.abspath(path)))
            _ensure_dir(os.path.dirname(os.path.abspath(header_path)))
            self._fh = open(path, 'w', encoding='utf-8', newline='\n')
            self._fh_hdr = open(header_path, 'w', encoding='utf-8', newline='\n')
            cols = list(COLS8) + (list(COLS_EXTRA) if self.extra_cols else [])
            self._fh_hdr.write(','.join(cols) + '\n')
        except Exception as exc:
            self._log.error('CSV 打开失败: %r' % (exc,))
            self._fh = None
            self._fh_hdr = None
            return False
        self._path = path
        self._header_path = header_path
        self._closed = False
        self._t_last_flush = now()
        self._row_count = 0
        return True

    def flush(self) -> None:
        """把缓冲区刷到磁盘。"""
        for fh in (self._fh, self._fh_hdr):
            if fh is not None:
                try:
                    fh.flush()
                    os.fsync(fh.fileno())
                except Exception:  # pragma: no cover - 某些文件系统不支持 fsync
                    try:
                        fh.flush()
                    except Exception:
                        pass

    def close(self) -> None:
        """flush + 关闭（退出时必须调用）。"""
        if self._closed and self._fh is None and self._fh_hdr is None:
            return
        try:
            self.flush()
        finally:
            for fh in (self._fh, self._fh_hdr):
                if fh is not None:
                    try:
                        fh.close()
                    except Exception:  # pragma: no cover
                        pass
            self._fh = None
            self._fh_hdr = None
            self._closed = True

    # -- 写行 -----------------------------------------------------------------
    def write_row(self, vals: Sequence[float]) -> bool:
        """写一行（严格 8 列；`extra_cols=True` 时 10 列）。

        有限性守卫：前 8 列中任一值非有限 → 用**上一次有效值**替代，
        `track_status` 置 0，并打 WARN（节流 1s）。

        Args:
            vals: 长度 ≥8（extra 时 ≥10）的数值序列；顺序见 `COLS8`。

        Returns:
            bool：写入成功为 True（未打开 / 异常时 False）。
        """
        if self._fh is None or self._fh_hdr is None:
            return False
        seq = list(vals)
        bad = False
        out: List[float] = list(self._last)

        for k in range(N_COLS_BASE):
            v = _safe_float(seq[k]) if k < len(seq) else float('nan')
            if math.isfinite(v):
                out[k] = v
            else:
                bad = True                     # 保留 out[k] = 上一次有效值

        if self.extra_cols:
            ts = _safe_float(seq[8]) if len(seq) > 8 else float('nan')
            out[8] = ts if math.isfinite(ts) else self._last[8]
            st = int(_safe_float(seq[9])) if len(seq) > 9 else 0
            out[9] = float(st)

        if bad:
            self._n_guarded += 1
            if self.extra_cols:
                out[9] = 0.0                   # track_status 置 0
            throttled(self._log, 'warn',
                      'CSV 出现非有限值，已用上一次有效值替代（累计 %d 次）'
                      % self._n_guarded,
                      'csv_writer.nonfinite', 1.0)

        line = format_row(out, extra=self.extra_cols)
        try:
            self._fh.write(line + '\n')
            self._fh_hdr.write(line + '\n')
        except Exception as exc:  # pragma: no cover
            self._log.error('CSV 写行失败: %r' % (exc,))
            return False

        self._last = out
        self._row_count += 1

        if self.flush_period_s > 0.0 and (now() - self._t_last_flush) >= self.flush_period_s:
            self.flush()
            self._t_last_flush = now()
        return True

    # -- 便捷接口 -------------------------------------------------------------
    def write_estimate(self,
                       pred_xy: Sequence[float],
                       cur_xy: Sequence[float],
                       base_xy: Sequence[float],
                       cmd_vxy: Sequence[float],
                       timestamp: float = 0.0,
                       track_status: int = 0) -> bool:
        """按语义参数写一行（内部按 `COLS8` 顺序组装）。

        Args:
            pred_xy: 目标预测位置 (x, y)。
            cur_xy: 目标当前位置 (x, y)。
            base_xy: 底盘位置 (x, y)。
            cmd_vxy: 底盘建议速度 (vx, vy)。
            timestamp: 相对 t0 的单调秒（extra 时写入第 9 列）。
            track_status: 0/1/2（extra 时写入第 10 列）。

        Returns:
            bool：写入成功为 True。
        """
        vals: List[float] = [
            _conv(pred_xy[0]), _conv(pred_xy[1]),
            _conv(cur_xy[0]), _conv(cur_xy[1]),
            _conv(base_xy[0]), _conv(base_xy[1]),
            _conv(cmd_vxy[0]), _conv(cmd_vxy[1]),
            float(timestamp), int(track_status),
        ]
        return self.write_row(vals)

    def stats(self) -> Dict[str, Any]:
        """返回写入统计（供 run_meta / 日志用）。"""
        return {
            'path': self._path,
            'header_path': self._header_path,
            'n_cols': N_COLS_EXTRA if self.extra_cols else N_COLS_BASE,
            'extra_cols': self.extra_cols,
            'rows': self._row_count,
            'n_guarded': self._n_guarded,
        }


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _ensure_dir(d: str) -> None:
    """确保目录存在。"""
    if d:
        os.makedirs(d, exist_ok=True)


class _FallbackLogger:
    """get_logger 不可用时的极简 logger（保证写盘路径永不因日志抛异常）。"""

    def __init__(self, name: str = 'csv_writer') -> None:
        self._name = str(name)

    def _emit(self, level: str, msg: str) -> None:
        print('[%s] %s %s' % (self._name, level, msg))

    def debug(self, msg: str) -> None:
        self._emit('DEBUG', msg)

    def info(self, msg: str) -> None:
        self._emit('INFO', msg)

    def warn(self, msg: str) -> None:
        self._emit('WARN', msg)

    def error(self, msg: str) -> None:
        self._emit('ERROR', msg)

    def fatal(self, msg: str) -> None:
        self._emit('FATAL', msg)


def _safe_logger(name: str) -> Any:
    """取一个可用的 logger：gimbal_common → 极简终端 logger。"""
    try:
        return get_logger(None, name)
    except Exception:  # pragma: no cover
        return _FallbackLogger(name)


def _safe_float(v: Any) -> float:
    """尽量把 v 转成 float；失败返回 NaN（触发有限性守卫）。"""
    try:
        return float(v)
    except Exception:
        return float('nan')


def _conv(v: Any) -> float:
    """把 msg 字段/数值转成 float（不可转时返回 NaN，由守卫兜底）。"""
    return _safe_float(v)
