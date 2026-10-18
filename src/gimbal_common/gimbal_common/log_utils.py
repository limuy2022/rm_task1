"""统一日志封装：前缀 ``[<node>.<name>]`` + 基于单调时钟的节流。

* ``get_logger(node, name)`` 返回 rclpy logger（名为 ``<node_name>.<name>``，
  rclpy 输出已自带 ``[node_name]`` 前缀）。
* ``throttled(...)`` 用 :func:`gimbal_common.time_utils.now` 做节流，
  **不用** 墙上时钟，避免系统时钟跳变导致节流失效。
* 状态迁移 / 首次冻结 / reinit / 丢失重捕获等必打日志用 WARN 或 INFO。
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Union

import rclpy.logging

from . import time_utils

__all__ = ['emit', 'get_logger', 'throttled', 'reset_throttle', 'LEVELS']

LEVELS = ('debug', 'info', 'warn', 'error', 'fatal')

_LAST_EMIT: Dict[str, float] = {}


# --------------------------------------------------------------------------
# 每个严重级一个独立入口函数。
#
# rclpy(Humble) 的 rcutils_logger 按「调用点帧」缓存日志上下文，同一帧先 INFO
# 再 WARN 会抛 ValueError('Logger severity cannot be changed between calls.')。
# 若所有等级共用一个分发帧（旧版 _emit/_FormatLogger.emit），任何混用等级的
# logger 都会崩溃——ROS 实跑下 gimbal_controller/bearing 节点均会踩中。
# 每级一个模块级函数即可让上下文按严重级自然分离。
# --------------------------------------------------------------------------
def _log_debug(logger: Any, message: str) -> None:
    logger.debug(message)


def _log_info(logger: Any, message: str) -> None:
    logger.info(message)


def _log_warning(logger: Any, message: str) -> None:
    logger.warning(message)


def _log_error(logger: Any, message: str) -> None:
    logger.error(message)


def _log_fatal(logger: Any, message: str) -> None:
    logger.fatal(message)


_EMIT_BY_LEVEL = {
    'debug': _log_debug, 10: _log_debug,
    'info': _log_info, 20: _log_info,
    'warn': _log_warning, 'warning': _log_warning, 30: _log_warning,
    'error': _log_error, 40: _log_error,
    'fatal': _log_fatal, 'critical': _log_fatal, 50: _log_fatal,
}


def emit(logger: Any, level: Union[str, int], msg: str) -> None:
    """按等级输出一条日志（兼容 rclpy logger 与标准 logging）。

    供持有任意 logger 的模块复用；等级分发必须走上面每级一个的模块级
    函数，不要在调用方重新写单一分发帧（rclpy 按调用点缓存严重级）。
    """
    lv = str(level).lower() if isinstance(level, str) else level
    _EMIT_BY_LEVEL[lv](logger, msg)


class _FormatLogger:
    """把既有logging风格的格式参数转成rclpy要求的单个消息字符串。"""
    def __init__(self, logger):
        self._logger = logger

    def __getattr__(self, name):
        if name not in _EMIT_BY_LEVEL:
            return getattr(self._logger, name)
        emit_fn = _EMIT_BY_LEVEL[name]
        logger = self._logger
        def emit(message, *args):
            return emit_fn(logger, message % args if args else message)
        return emit


def get_logger(node: Any = None, name: str = 'gimbal') -> Any:
    """取得带节点前缀的 logger。

    Args:
        node: rclpy 节点（可为 None，此时只用 name）。
        name: 子节点/模块名，用于日志定位。

    Returns:
        rclpy logger。
    """
    base = node.get_name() if node is not None else 'gimbal'
    return _FormatLogger(rclpy.logging.get_logger(f'{base}.{name}'))


def throttled(logger: Any, level: Union[str, int], msg: str, key: str,
              period_s: float = 1.0) -> bool:
    """节流日志：同一 ``key`` 在 ``period_s`` 内只输出一次。

    Args:
        logger: 日志对象。
        level: ``'debug'/'info'/'warn'/'error'/'fatal'``。
        msg: 日志内容。
        key: 节流键（全局唯一，建议带上节点/模块名）。
        period_s: 节流周期（秒）。

    Returns:
        bool: 本次是否真的输出。
    """
    t = time_utils.now()
    last = _LAST_EMIT.get(key, None)
    if last is not None and (t - last) < float(period_s):
        return False
    _LAST_EMIT[key] = t
    emit(logger, level, msg)
    return True


def reset_throttle(key: Optional[str] = None) -> None:
    """清空节流记录；``key=None`` 时清空全部。"""
    if key is None:
        _LAST_EMIT.clear()
    else:
        _LAST_EMIT.pop(key, None)
