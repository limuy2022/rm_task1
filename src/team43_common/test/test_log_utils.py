# -*- coding: utf-8 -*-
"""log_utils 回归：rclpy(Humble)「同帧混用日志等级即抛错」约束下的分发安全。

rclpy 的 rcutils_logger 按调用点帧缓存上下文；旧版单帧分发（_emit）把所有
等级收敛到同一个分发帧，任何 logger 混用两个等级都会抛
ValueError('Logger severity cannot be changed between calls.')。
本测试用同款检查逻辑的假 logger 复现该约束，验证 emit/throttled 的每级
分发不再触发；并冒烟验证 get_logger 返回的 rclpy logger 可直接单串调用。
"""

from __future__ import annotations

import inspect
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))  # noqa: E402

from team43_common.log_utils import get_logger, throttled  # noqa: E402


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


def test_throttled_mixed_severities_do_not_collide():
    raw = SeverityContextLogger()
    from team43_common import log_utils
    log_utils.reset_throttle()
    throttled(raw, 'warning', 'w1', 'k1', 0.0)
    throttled(raw, 'error', 'e1', 'k2', 0.0)
    throttled(raw, 'info', 'i1', 'k3', 0.0)
    assert [c[0] for c in raw.calls] == [30, 40, 20]


def test_get_logger_prefixes_node_name(monkeypatch):
    from team43_common import log_utils

    class FakeLogger:
        def __init__(self, name):
            self.name = name

    monkeypatch.setattr(log_utils.rclpy.logging, 'get_logger', FakeLogger)
    node = type('Node', (), {'get_name': lambda self: 'my_node'})()
    assert log_utils.get_logger(node, 'wheel').name == 'my_node.wheel'
    assert log_utils.get_logger(None, 'x').name == 'gimbal.x'


def test_get_logger_direct_calls_accept_single_string():
    log = get_logger(None, 'smoke')
    log.debug('单串')
    log.info('单串')
    log.warning('单串')
    log.error('单串')
    log.fatal('单串')
