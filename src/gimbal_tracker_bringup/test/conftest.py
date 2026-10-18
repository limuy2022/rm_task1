"""pytest 公共设施：路径注入 + **ROS 桩环境**（本机无 ROS2 时验证 launch/节点）。

本目录的三个测试是 T05「集成联调层」的回归网，**无需 ROS2 即可在任意机器上跑**：

| 文件 | 验证对象 | 手法 |
|---|---|---|
| `test_launch_sim.py` | `launch/*.launch.py` 的 `_launch_setup` | 桩掉 `launch`/`launch_ros`，真跑 `_launch_setup`，检查生成的 Node/TimerAction |
| `test_node_itest.py` | `gimbal_debug/gt_error_monitor_node.py` | 桩掉 `rclpy` 与消息类型 + 假节点/假单调时钟，真跑节点与收尾落盘 |
| `test_consistency.py` | 全工程静态一致性 | 纯文本/AST 扫描（entry_points、话题、参数、禁用字面量、墙上时钟） |

设计要点
--------
* **自包含**：所有路径由 `__file__` 推导，不使用绝对路径、不使用 `%TEMP%`；临时产物一律用
  pytest 的 `tmp_path`。
* **零污染**：`stub_ros_env()` 会快照 `sys.modules`，退出时删除窗口内新增的模块并还原被替换的
  模块，保证 `rclpy` / `launch` / 消息桩不会泄漏到其它包的测试（`pytest src -q` 单进程跑全套）。
* **不假装通过**：无法在本机验证的项用 `pytest.skip("需 Ubuntu + ROS2")` 明确跳过。
* `ament_index_python` 由测试桩提供包目录，生产代码直接使用安装索引。
"""

from __future__ import annotations

import contextlib
import os
import sys
import types
from typing import Any
from typing import Dict
from typing import List

import pytest

# ---------------------------------------------------------------------------
# 路径（全部由 __file__ 推导，无绝对路径）
# ---------------------------------------------------------------------------
TEST_DIR: str = os.path.dirname(os.path.abspath(__file__))
PKG_DIR: str = os.path.abspath(os.path.join(TEST_DIR, '..'))        # src/gimbal_tracker_bringup
SRC_DIR: str = os.path.abspath(os.path.join(PKG_DIR, '..'))         # src
WS_DIR: str = os.path.abspath(os.path.join(SRC_DIR, '..'))          # 工作区根

CONFIG_DIR: str = os.path.join(PKG_DIR, 'config')
LAUNCH_DIR: str = os.path.join(PKG_DIR, 'launch')
RVIZ_DIR: str = os.path.join(PKG_DIR, 'rviz')

VIRTUAL_YAML: str = os.path.join(CONFIG_DIR, 'virtual.yaml')
FIELD_YAML: str = os.path.join(CONFIG_DIR, 'field.yaml')
TRACKER_PARAMS: str = os.path.join(CONFIG_DIR, 'tracker_params.yaml')
RViz_CONFIG: str = os.path.join(RVIZ_DIR, 'gimbal_tracker.rviz')

for _p in (PKG_DIR,
           os.path.join(SRC_DIR, 'gimbal_common'),
           os.path.join(SRC_DIR, 'gimbal_debug')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

#: 受桩影响的模块命名空间前缀（只有这些会在退出时被清理/还原）
STUB_PREFIXES = frozenset({
    'launch', 'launch_ros', 'rclpy', 'ament_index_python',
    'geometry_msgs', 'nav_msgs', 'std_msgs', 'gimbal_interfaces',
    'gimbal_common', 'gimbal_debug',
})


# ---------------------------------------------------------------------------
# 消息 / 节点桩
# ---------------------------------------------------------------------------
class _LoggerProxy(object):
    """把 stdlib logger 包一层，让 `warn()` 走 `warning()`。

    rclpy 的 `RcutilsLogger` 提供的是 `warn()`（非 deprecated），工程内统一用
    `log_utils.throttled(logger, 'warn', ...)`。本机走 stdlib 兜底时会触发
    `DeprecationWarning: The 'warn' method is deprecated`，这里对齐 rclpy 语义、
    消除噪声（**不改生产代码**）。
    """

    def __init__(self, inner: Any) -> None:
        self._inner = inner

    def warn(self, *args: Any, **kwargs: Any) -> Any:
        return self._inner.warning(*args, **kwargs)

    def __getattr__(self, item: str) -> Any:
        return getattr(self._inner, item)


class _Msg(object):
    """宽松消息基类：允许任意字段。"""

    def __init__(self, **kw: Any) -> None:
        for k, v in kw.items():
            setattr(self, k, v)


class StubRos(object):
    """一整套 ROS2 桩（launch / launch_ros / rclpy / 消息）。

    Attributes:
        nodes: 桩 `Node` 被构造时收集的 kwargs 列表（按构造顺序）。
        published: `(topic, msg)` 列表。
        subs: `{topic: callback}`。
        timers: `[(period, callback)]`。
    """

    def __init__(self) -> None:
        self.nodes: List[Dict[str, Any]] = []
        self.published: List[Any] = []
        self.subs: Dict[str, Any] = {}
        self.timers: List[Any] = []
        self._build_msg_types()
        self._build_rclpy()
        self._build_launch()

    # -- 消息类型 -----------------------------------------------------------
    def _build_msg_types(self) -> None:
        class Header(_Msg):
            def __init__(self) -> None:
                self.frame_id = ''
                self.stamp = 'stub-stamp'

        class Vector3(_Msg):
            def __init__(self) -> None:
                self.x = self.y = self.z = 0.0

        class Quaternion(_Msg):
            def __init__(self) -> None:
                self.x = self.y = self.z = 0.0
                self.w = 1.0

        class Point(_Msg):
            def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
                self.x, self.y, self.z = x, y, z

        class Pose(_Msg):
            def __init__(self) -> None:
                self.position = Vector3()
                self.orientation = Quaternion()

        class Transform(_Msg):
            def __init__(self) -> None:
                self.translation = Vector3()
                self.rotation = Quaternion()

        class PointStamped(_Msg):
            def __init__(self) -> None:
                self.header = Header()
                self.point = Point()

        class PoseStamped(_Msg):
            def __init__(self) -> None:
                self.header = Header()
                self.pose = Pose()

        class TransformStamped(_Msg):
            def __init__(self) -> None:
                self.header = Header()
                self.child_frame_id = ''
                self.transform = Transform()

        class Path(_Msg):
            def __init__(self) -> None:
                self.header = Header()
                self.poses: List[Any] = []

        class Float64(_Msg):
            def __init__(self) -> None:
                self.data = 0.0

        class GimbalState(_Msg):
            def __init__(self) -> None:
                self.header = Header()
                self.yaw = self.pitch = 0.0
                self.omega_yaw = self.omega_pitch = 0.0
                self.gyro = [0.0, 0.0, 0.0]
                self.accel = [0.0, 0.0, 9.8]
                self.quat = [1.0, 0.0, 0.0, 0.0]        # w,x,y,z
                self.link_state = 1
                self.enabled = True
                self.frames_ok = 1
                self.frames_bad = 0
                self.seq_last = 1

        class TargetState(_Msg):
            def __init__(self) -> None:
                self.header = Header()
                self.cur_x = self.cur_y = 0.0
                self.pred_x = self.pred_y = 0.0
                self.vel_x = self.vel_y = 0.0
                self.acc_x = self.acc_y = 0.0
                self.cov_xy = [0.0] * 4
                self.track_status = 1
                self.last_obs_age = 0.0
                self.valid = True

        self.Header = Header
        self.Point = Point
        self.PointStamped = PointStamped
        self.PoseStamped = PoseStamped
        self.TransformStamped = TransformStamped
        self.Path = Path
        self.Float64 = Float64
        self.GimbalState = GimbalState
        self.TargetState = TargetState

    # -- rclpy --------------------------------------------------------------
    def _build_rclpy(self) -> None:
        rclpy = types.ModuleType('rclpy')

        node_mod = types.ModuleType('rclpy.node')
        node_mod.Node = type('Node', (), {})

        param_mod = types.ModuleType('rclpy.parameter')

        class _ParamType(object):
            BOOL = 'bool'
            DOUBLE = 'double'
            STRING = 'string'
            INTEGER = 'integer'

        class Parameter(object):
            Type = _ParamType

            def __init__(self, name: str, type_: Any = None, value: Any = None) -> None:
                self.name = name
                self.type_ = type_
                self.value = value

        param_mod.Parameter = Parameter

        qos_mod = types.ModuleType('rclpy.qos')

        class QoSProfile(object):
            def __init__(self, **kw: Any) -> None:
                self.kw = kw

        class _Enum(object):
            KEEP_LAST = 1
            TRANSIENT_LOCAL = 2
            VOLATILE = 3
            RELIABLE = 4
            BEST_EFFORT = 5

        qos_mod.QoSProfile = QoSProfile
        qos_mod.QoSHistoryPolicy = _Enum
        qos_mod.QoSDurabilityPolicy = _Enum
        qos_mod.QoSReliabilityPolicy = _Enum

        logging_mod = types.ModuleType('rclpy.logging')

        def _get_logger(name: str) -> Any:
            import logging as _logging
            return _LoggerProxy(_logging.getLogger(name))

        logging_mod.get_logger = _get_logger

        rclpy.node = node_mod
        rclpy.parameter = param_mod
        rclpy.qos = qos_mod
        rclpy.logging = logging_mod

        self.rclpy = rclpy

        # 消息包
        geo = types.ModuleType('geometry_msgs')
        geo_msg = types.ModuleType('geometry_msgs.msg')
        geo_msg.PointStamped = self.PointStamped
        geo_msg.PoseStamped = self.PoseStamped
        geo_msg.TransformStamped = self.TransformStamped
        geo.msg = geo_msg

        nav = types.ModuleType('nav_msgs')
        nav_msg = types.ModuleType('nav_msgs.msg')
        nav_msg.Path = self.Path
        nav.msg = nav_msg

        std = types.ModuleType('std_msgs')
        std_msg = types.ModuleType('std_msgs.msg')
        std_msg.Float64 = self.Float64
        std.msg = std_msg

        gi = types.ModuleType('gimbal_interfaces')
        gi_msg = types.ModuleType('gimbal_interfaces.msg')
        gi_msg.GimbalState = self.GimbalState
        gi_msg.TargetState = self.TargetState
        gi.msg = gi_msg

        self.msg_modules = {
            'rclpy': rclpy, 'rclpy.node': node_mod, 'rclpy.parameter': param_mod,
            'rclpy.qos': qos_mod, 'rclpy.logging': logging_mod,
            'geometry_msgs': geo, 'geometry_msgs.msg': geo_msg,
            'nav_msgs': nav, 'nav_msgs.msg': nav_msg,
            'std_msgs': std, 'std_msgs.msg': std_msg,
            'gimbal_interfaces': gi, 'gimbal_interfaces.msg': gi_msg,
        }

    # -- launch / launch_ros -------------------------------------------------
    def _build_launch(self) -> None:
        env = self

        class LaunchDescription(object):
            def __init__(self, entities: Any = None) -> None:
                self.entities = list(entities or [])

        class LaunchContext(object):
            pass

        class LaunchConfiguration(object):
            def __init__(self, name: str, default: Any = None) -> None:
                self.name = name
                self.default = default

            def perform(self, context: Any) -> str:
                return env.args.get(self.name, self.default if isinstance(
                    self.default, str) else '')

            def __str__(self) -> str:
                return self.name

        class DeclareLaunchArgument(object):
            def __init__(self, name: str, *args: Any, default_value: Any = None,
                         description: str = '', **kw: Any) -> None:
                self.name = name
                self.kw = {'name': name, 'default_value': default_value,
                           'description': description}
                self.args = list(args)

        class OpaqueFunction(object):
            def __init__(self, *args: Any, function: Any = None, **kw: Any) -> None:
                self.kw = {'function': function}

        class TimerAction(object):
            def __init__(self, *args: Any, period: float = 0.0, actions: Any = None,
                         **kw: Any) -> None:
                self.kw = {'period': period, 'actions': list(actions or [])}

        class LogInfo(object):
            def __init__(self, *args: Any, msg: str = '', **kw: Any) -> None:
                self.kw = {'msg': msg}

        class _Source(object):
            def __init__(self, *args: Any, **kw: Any) -> None:
                self.path = args[0] if args else kw.get('path', '')

            def __str__(self) -> str:
                return str(self.path)

            def __repr__(self) -> str:
                return 'PythonLaunchDescriptionSource(%r)' % (self.path,)

        class IncludeLaunchDescription(object):
            def __init__(self, *args: Any, launch_description_source: Any = None,
                         launch_arguments: Any = None, **kw: Any) -> None:
                src = launch_description_source if launch_description_source is not None \
                    else (args[0] if args else None)
                self.kw = {'launch_description_source': src,
                           'launch_arguments': launch_arguments}

        class EmitEvent(object):
            def __init__(self, *args: Any, event: Any = None, **kw: Any) -> None:
                self.kw = {'event': event}

        class OnProcessExit(object):
            def __init__(self, *args: Any, target_action: Any = None,
                         on_exit: Any = None, **kw: Any) -> None:
                self.kw = {'target_action': target_action,
                           'on_exit': list(on_exit or [])}

        class RegisterEventHandler(object):
            def __init__(self, *args: Any, event_handler: Any = None, **kw: Any) -> None:
                eh = event_handler if event_handler is not None else (
                    args[0] if args else None)
                self.kw = {'event_handler': eh}
                if eh is not None:
                    self.kw.update(getattr(eh, 'kw', {}))

        class Shutdown(object):
            def __init__(self, *args: Any, reason: str = '', **kw: Any) -> None:
                self.kw = {'reason': reason}

        class Node(object):
            def __init__(self, *args: Any, **kw: Any) -> None:
                self.kw = dict(kw)
                env.nodes.append(self)

        launch = types.ModuleType('launch')
        launch.LaunchDescription = LaunchDescription
        launch.LaunchContext = LaunchContext

        actions = types.ModuleType('launch.actions')
        actions.DeclareLaunchArgument = DeclareLaunchArgument
        actions.OpaqueFunction = OpaqueFunction
        actions.TimerAction = TimerAction
        actions.LogInfo = LogInfo
        actions.IncludeLaunchDescription = IncludeLaunchDescription
        actions.EmitEvent = EmitEvent
        actions.RegisterEventHandler = RegisterEventHandler
        actions.SetEnvironmentVariable = LogInfo

        substitutions = types.ModuleType('launch.substitutions')
        substitutions.LaunchConfiguration = LaunchConfiguration

        sources = types.ModuleType('launch.launch_description_sources')
        sources.PythonLaunchDescriptionSource = _Source

        handlers = types.ModuleType('launch.event_handlers')
        handlers.OnProcessExit = OnProcessExit
        handlers.OnProcessStart = OnProcessExit

        events = types.ModuleType('launch.events')
        events.Shutdown = Shutdown

        launch_ros = types.ModuleType('launch_ros')
        launch_ros_actions = types.ModuleType('launch_ros.actions')
        launch_ros_actions.Node = Node

        self.LaunchDescription = LaunchDescription
        self.DeclareLaunchArgument = DeclareLaunchArgument
        self.OpaqueFunction = OpaqueFunction
        self.TimerAction = TimerAction
        self.LogInfo = LogInfo
        self.IncludeLaunchDescription = IncludeLaunchDescription
        self.EmitEvent = EmitEvent
        self.RegisterEventHandler = RegisterEventHandler
        self.Shutdown = Shutdown
        self.Node = Node

        # launch 时可通过 launch_arguments 覆盖的参数值
        self.args: Dict[str, str] = {}

        ament = types.ModuleType('ament_index_python')
        packages = types.ModuleType('ament_index_python.packages')
        def get_package_share_directory(package):
            path = os.path.join(SRC_DIR, package)
            if not os.path.isdir(path):
                raise LookupError(package)
            return path
        packages.get_package_share_directory = get_package_share_directory
        ament.packages = packages

        self.launch_modules = {
            'ament_index_python': ament, 'ament_index_python.packages': packages,
            'launch': launch,
            'launch.actions': actions,
            'launch.substitutions': substitutions,
            'launch.launch_description_sources': sources,
            'launch.event_handlers': handlers,
            'launch.events': events,
            'launch_ros': launch_ros,
            'launch_ros.actions': launch_ros_actions,
        }

    # -- 安装 / 使用 --------------------------------------------------------
    def install(self) -> None:
        """把桩写进 `sys.modules`。"""
        for name, mod in self.msg_modules.items():
            sys.modules[name] = mod
        for name, mod in self.launch_modules.items():
            sys.modules[name] = mod

    def load_launch_module(self, rel_path: str, mod_name: str) -> Any:
        """把 `launch/...` 下的文件当模块执行（`launch`/`launch_ros` 用桩）。

        Args:
            rel_path: 相对 `launch/` 的路径，如 `include/static_tf.launch.py`。
            mod_name: 给模块起的唯一名字。

        Returns:
            types.ModuleType：已执行的模块对象（其 `__file__` 指向源码，供 share 兜底）。
        """
        path = os.path.join(LAUNCH_DIR, rel_path)
        mod = types.ModuleType(mod_name)
        mod.__file__ = path
        with open(path, 'r', encoding='utf-8') as f:
            code = compile(f.read(), path, 'exec')
        exec(code, mod.__dict__)          # noqa: S102 - 测试内主动执行本仓库源码
        return mod

    def opaque_setup(self, launch_description: Any) -> Any:
        """取出 LaunchDescription 里的 `OpaqueFunction.function`。"""
        for entity in launch_description.entities:
            if isinstance(entity, self.OpaqueFunction):
                return entity.kw['function']
        raise AssertionError('LaunchDescription 中未找到 OpaqueFunction')

    def declared_args(self, launch_description: Any) -> Dict[str, Any]:
        """取出 `{参数名: 默认值}`。"""
        out: Dict[str, Any] = {}
        for entity in launch_description.entities:
            if isinstance(entity, self.DeclareLaunchArgument):
                out[entity.name] = entity.kw.get('default_value')
        return out


@contextlib.contextmanager
def stub_ros_env():
    """安装 ROS 桩的上下文管理器，退出时**只按需还原** `sys.modules`。

    为什么不能"整表快照 + 整表还原"：某些测试是**第一次**导入 numpy（或其它带 C 扩展的
    第三方库），若退出时把这些模块从 `sys.modules` 删掉，后续再 import 会触发
    ``ImportError: cannot load module more than once per process``（CPython 不允许同一进程
    内二次加载同一扩展模块）。因此这里只清理**受桩影响**的模块命名空间前缀，
    并把被桩覆盖的真实模块还原回去。
    """
    saved = dict(sys.modules)
    env = StubRos()
    env.install()
    try:
        yield env
    finally:
        # 1) 删除窗口内新增的、且属于桩命名空间的模块（用桩导入过，必须重导）
        for name in list(sys.modules):
            if name in saved:
                continue
            if name.split('.')[0] in STUB_PREFIXES:
                del sys.modules[name]
        # 2) 还原被桩覆盖的真实模块（Ubuntu 上 launch/rclpy 是真模块）
        for name, module in saved.items():
            if name.split('.')[0] in STUB_PREFIXES:
                sys.modules[name] = module
        # 3) 本机无 ROS 时，确保桩名不会残留在 sys.modules
        for name in list(env.msg_modules) + list(env.launch_modules):
            if name not in saved:
                sys.modules.pop(name, None)


@pytest.fixture
def stub_ros():
    """函数级 ROS 桩环境（见 :class:`StubRos`）。"""
    with stub_ros_env() as env:
        yield env
