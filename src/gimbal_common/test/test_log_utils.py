# -*- coding: utf-8 -*-
"""log_utils 回归：修复 rclpy(Humble)「同帧混用日志等级即抛错」的封装缺陷。

rclpy 的 rcutils_logger 按调用点帧缓存上下文；旧版 _emit/_FormatLogger.emit
把所有等级收敛到同一个分发帧，任何 logger 混用两个等级都会抛
ValueError('Logger severity cannot be changed between calls.')。
本测试用同款检查逻辑的假 logger 复现该约束，验证修复后不再触发。
"""

from __future__ import annotations

import inspect
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))  # noqa: E402

from gimbal_common.log_utils import _FormatLogger, throttled  # noqa: E402


class SeverityContextLogger:
    """模拟 rclpy CallerId 上下文检查：按调用点记录等级，同点换等级即抛错。"""

    def __init__(self):
        self._contexts = {}
        self.calls = []

    def _log(self, severity, msg):
        frame = inspect.currentframe().f_back.f_back   # 跳过 _log 与严重级方法
        key = (frame.f_code.co_filename, frame.f_code.co_name, frame.f_lineno)
        prev = self._contexts.get(key)
        if prev is not None and prev != severity:
            raise ValueError('Logger severity cannot be changed between calls.')
        self._contexts[key] = severity
        self.calls.append((severity, msg))

    def debug(self, msg):
        self._log(10, msg)

    def info(self, msg):
        self._log(20, msg)

    def warning(self, msg):
        self._log(30, msg)

    def error(self, msg):
        self._log(40, msg)

    def fatal(self, msg):
        self._log(50, msg)


def test_format_logger_mixed_severities_do_not_collide():
    raw = SeverityContextLogger()
    wrapped = _FormatLogger(raw)
    wrapped.info('启动 rate=%.1fHz', 50.0)     # %-style 参数仍被格式化
    wrapped.warning('丢失 %s', 'marker')
    wrapped.error('严重')
    wrapped.fatal('致命')
    assert [c[0] for c in raw.calls] == [20, 30, 40, 50]
    assert raw.calls[0][1] == '启动 rate=50.0Hz'
    assert raw.calls[1][1] == '丢失 marker'


def test_throttled_mixed_severities_do_not_collide():
    raw = SeverityContextLogger()
    from gimbal_common import log_utils
    log_utils.reset_throttle()
    throttled(raw, 'warn', 'w1', 'k1', 0.0)
    throttled(raw, 'error', 'e1', 'k2', 0.0)
    throttled(raw, 'info', 'i1', 'k3', 0.0)
    assert [c[0] for c in raw.calls] == [30, 40, 20]


def test_non_logging_attributes_pass_through():
    raw = SeverityContextLogger()
    raw.marker = 7
    wrapped = _FormatLogger(raw)
    assert wrapped.marker == 7
