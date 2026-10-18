"""rclpy 节点工厂与生命周期封装。

⚠ 主理人拍板 1「混合时钟」：**不做 ``node._clock`` 替换，不使用 ``ClockType.STEADY_TIME``**。
理由：替换为单调时钟后我方 TF 与外部 ``gimbal_link_gt`` 时间轴不同域，RViz 显示不出 TF 树。

约定：

* ``header.stamp`` / TF 时间戳 → ROS 系统时钟（``node.get_clock().now()``）。
* 所有 dt、超时、预测、定时器周期计算 → :func:`gimbal_common.time_utils.now`（``time.monotonic()``）。

本模块依赖 rclpy，**只在 ROS 节点文件中使用**（纯算法模块不要 import）。
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

__all__ = ['create_node', 'declare_params', 'spin_with_finally', 'shutdown']


def create_node(name: str, **kwargs: Any) -> Node:
    """创建 rclpy 节点（不含时钟替换）。

    统一做三件事：
    1. 强制 ``use_sim_time=false``（DESIGN §8.2 第 4 条）；
    2. 打开 ``automatically_declare_parameters_from_overrides``，让 YAML/命令行注入的参数可用；
    3. 返回标准 ``rclpy.node.Node``，不触碰 ``node._clock``。

    Args:
        name: 节点名。
        **kwargs: 透传给 ``rclpy.create_node``（``namespace`` / ``parameter_overrides`` 等）。

    Returns:
        rclpy.node.Node: 已创建的节点。
    """
    overrides: List[Parameter] = list(kwargs.pop('parameter_overrides', None) or [])
    if not any(p.name == 'use_sim_time' for p in overrides):
        overrides.append(Parameter('use_sim_time', Parameter.Type.BOOL, False))
    kwargs.setdefault('automatically_declare_parameters_from_overrides', True)
    return rclpy.create_node(name, parameter_overrides=overrides, **kwargs)


def declare_params(node: Node, defaults: Dict[str, Any]) -> Dict[str, Any]:
    """批量声明并返回参数当前值（含 YAML/launch 注入的 override）。

    已声明过的参数不会重复声明（重复声明会抛 ``ParameterAlreadyDeclaredException``）。
    声明或读取失败时直接抛错，避免节点使用与实际配置不一致的默认值。

    Args:
        node: 节点。
        defaults: ``{参数名: 默认值}``。

    Returns:
        dict: ``{参数名: 当前值}``。
    """
    out: Dict[str, Any] = {}
    for key, value in defaults.items():
        if not node.has_parameter(key):
            node.declare_parameter(key, value)
        out[key] = node.get_parameter(key).value
    return out


def spin_with_finally(node: Node,
                      on_shutdown: Optional[Callable[[], None]] = None) -> None:
    """``rclpy.spin`` 包装：退出（含异常）时保证执行收尾钩子。

    Args:
        node: 节点。
        on_shutdown: 单个收尾回调（flush / 落盘 / 发零指令）。
    """
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # Ctrl+C 是正常停机请求；finally 仍执行写盘或发零。
        pass
    finally:
        if on_shutdown is not None:
            on_shutdown()


def shutdown() -> None:
    """安全关闭 rclpy（已初始化时才调用）。"""
    if rclpy.ok():
        rclpy.shutdown()
