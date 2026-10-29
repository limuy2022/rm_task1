"""共享 QoS 构造器。

本模块依赖 rclpy，**只在 ROS 节点文件中使用**（纯算法模块不要 import，
约定同 :mod:`team43_common.node_utils`）。

三种工程内标准 profile，各节点一律从这里取，不要再本地复制构造器：
QoS 语义（尤其 ``TRANSIENT_LOCAL`` 锁存）无法满足时应当在启动时失败，
不允许静默降级。
"""

from __future__ import annotations

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


def sensor_qos(depth: int = 10) -> QoSProfile:
    """传感器数据流：BEST_EFFORT + VOLATILE + KEEP_LAST(depth)。"""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def latched_qos(depth: int = 1) -> QoSProfile:
    """锁存话题：RELIABLE + TRANSIENT_LOCAL，晚加入的订阅者也能收到最新一条。"""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def reliable_qos(depth: int = 10) -> QoSProfile:
    """可靠流：RELIABLE + VOLATILE + KEEP_LAST(depth)（如 /judge/tick 官方要求）。"""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )
