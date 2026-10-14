# -*- coding: utf-8 -*-
"""``gimbal_common.log_utils`` 测试桩（仅用于无 ROS 环境的单测）。"""

import time
from typing import Dict, Tuple

_LAST_LOG: Dict[str, float] = {}


class _StubLogger:
    """极简 logger 桩：把消息写到 stdout，带 ``[name]`` 前缀。"""

    def __init__(self, name: str = 'stub') -> None:
        self.name = str(name)

    def _emit(self, level: str, msg: str) -> None:
        print('[%s] %s %s' % (self.name, level.upper(), msg))

    def info(self, msg, *args) -> None:
        self._emit('info', msg % args if args else msg)

    def warn(self, msg, *args) -> None:
        self._emit('warn', msg % args if args else msg)

    def error(self, msg, *args) -> None:
        self._emit('error', msg % args if args else msg)

    def debug(self, msg, *args) -> None:
        self._emit('debug', msg % args if args else msg)


def get_logger(node, name: str = 'node') -> _StubLogger:
    """返回统一前缀的 logger（真实实现包装 ``node.get_logger()``）。"""
    return _StubLogger(name)


def throttled(logger, level: str, msg: str, key: str, period_s: float = 1.0) -> bool:
    """节流日志：同一 ``key`` 在 ``period_s`` 内只打一次。

    Returns:
        本次是否真的打印。
    """
    t = time.monotonic()
    last = _LAST_LOG.get(key)
    if last is not None and (t - last) < float(period_s):
        return False
    _LAST_LOG[key] = t
    fn = getattr(logger, str(level).lower(), None)
    if fn is None:
        fn = logger.info
    fn(msg)
    return True
