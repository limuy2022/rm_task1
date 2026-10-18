"""rclpy 节点工厂与生命周期封装。

⚠ 主理人拍板 1「混合时钟」：**不做 ``node._clock`` 替换，不使用 ``ClockType.STEADY_TIME``**。
理由：替换为单调时钟后我方 TF 与外部 ``gimbal_link_gt`` 时间轴不同域，RViz 显示不出 TF 树。

约定：

* ``header.stamp`` / TF 时间戳 → ROS 系统时钟（``node.get_clock().now()``）。
* 所有 dt、超时、预测、定时器周期计算 → :func:`gimbal_common.time_utils.now`（``time.monotonic()``）。

本模块依赖 rclpy，**只在 ROS 节点文件中使用**（纯算法模块不要 import）。
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Sequence

import rclpy
from rclpy.exceptions import ParameterException
from rclpy.node import Node
from rclpy.parameter import Parameter

__all__ = ['create_node', 'declare_params', 'get_param', 'spin_with_finally',
           'SpinUntilShutdown', 'shutdown']


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
    声明失败（如 override 与默认值类型不匹配）时告警并回退默认值：
    节点仍可启动，但配置错误不静默吞掉。

    Args:
        node: 节点。
        defaults: ``{参数名: 默认值}``。

    Returns:
        dict: ``{参数名: 当前值}``。
    """
    out: Dict[str, Any] = {}
    for key, value in defaults.items():
        if not node.has_parameter(key):
            try:
                node.declare_parameter(key, value)
            except ParameterException as exc:
                node.get_logger().warning(
                    f'参数 {key} 声明失败: {exc!r}，回退默认值 {value!r}')
                out[key] = value
                continue
        try:
            out[key] = node.get_parameter(key).value
        except Exception:
            out[key] = value
    return out


def get_param(node: Node, name: str, default: Any = None) -> Any:
    """取参数；不存在时用 ``default`` 声明并返回。"""
    if not node.has_parameter(name):
        if default is None:
            return None
        try:
            node.declare_parameter(name, default)
        except ParameterException as exc:
            node.get_logger().warning(
                f'参数 {name} 声明失败: {exc!r}，回退默认值 {default!r}')
            return default
    try:
        value = node.get_parameter(name).value
    except Exception:
        return default
    return default if value is None else value


def spin_with_finally(node: Node,
                      on_shutdown: Optional[Callable[[], None]] = None,
                      hooks: Optional[Sequence[Callable[[], None]]] = None) -> None:
    """``rclpy.spin`` 包装：退出（含异常）时保证执行收尾钩子。

    Args:
        node: 节点。
        on_shutdown: 单个收尾回调（flush / 落盘 / 发零指令）。
        hooks: 多个收尾回调，按序执行（``on_shutdown`` 最后执行）。
    """
    calls: List[Callable[[], None]] = list(hooks or [])
    if on_shutdown is not None:
        calls.append(on_shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for fn in calls:
            try:
                fn()
            except Exception as exc:  # 收尾失败不能影响其它钩子
                node.get_logger().error(f'shutdown hook 失败: {exc!r}')


class SpinUntilShutdown:
    """带 ``finally`` 钩子的 spin 封装（面向对象版本）。"""

    def __init__(self, node: Node) -> None:
        self._node: Node = node
        self._hooks: List[Callable[[], None]] = []

    def add_hook(self, fn: Callable[[], None]) -> None:
        """注册一个收尾钩子（按注册顺序执行）。"""
        self._hooks.append(fn)

    def run(self) -> None:
        """进入 spin，退出时执行全部钩子。"""
        spin_with_finally(self._node, hooks=self._hooks)


def shutdown() -> None:
    """安全关闭 rclpy（已初始化时才调用）。"""
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
