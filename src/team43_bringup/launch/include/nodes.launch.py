"""节点清单与**阶梯启动**（被 `competition.launch.py` / `sim.launch.py` include 复用）。

启动顺序与依赖（DESIGN §3.13 / §6.14 / §7-T05）：

```
team43_grid_mapper → base_pose_provider → map_frame_broadcaster → vgimbal_bridge
  → marker_pnp → target_estimator → gimbal_controller → team43_chassis_advisor
  → judge_io(官方 CSV+地图) → trajectory_annotator [→ gt_error_monitor(可选)]
```

* `grid_mapper_node` **必须最先起**：它以 `TRANSIENT_LOCAL` 发 `/map/grid` 与 `/map/ray_grid`，
  `chassis_advisor_node` / `trajectory_annotator_node` 依赖。
* 每个节点用 `TimerAction` 阶梯启动，步长由 `startup_step_s`（默认 0.3s）控制。
* 允许通过 `serial_port` / `base_pose_source` / `target_digit` 等参数**按需覆盖**
  `tracker_params.yaml`（留空 = 不覆盖，使用参数文件里的值 = 单一数据源）。
* 收尾：`TimerAction(period = run_duration_s + shutdown_grace_s) → Shutdown(reason='mission complete')`
  （DESIGN §6.14）。各节点在自己的 `finally` 里 flush / close / 落盘。
* `vgimbal_bridge_node` 异常退出（例如串口被占用 → FATAL）时立即触发收尾，避免干等 30s。
* **禁止**启动 `robo_bridge_edu` 或任何串口桥；`world → gimbal_link` 只由 `vgimbal_bridge_node` 发布。
"""

from __future__ import annotations

import json
import os
from team43_bringup.parameter_files import package_config
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PACKAGE_NAME = 'team43_bringup'

# (阶梯序号, 包名, 可执行文件名)
# 可执行文件名必须与各包 setup.py 的 console_scripts 完全一致。
NODE_LADDER: List[Tuple[int, str, str]] = [
    (0, 'team43_grid_mapper', 'grid_mapper_node'),
    (1, 'team43_localization', 'base_pose_provider_node'),
    (2, 'team43_localization', 'map_frame_broadcaster_node'),
    (3, 'team43_gimbal_io', 'vgimbal_bridge_node'),
    (4, 'team43_perception', 'aruco_detector_node'),      # /image_raw → 角点
    (5, 'team43_perception', 'marker_pnp_node'),          # 角点 → PnP 位姿
    (6, 'team43_estimation', 'target_estimator_node'),
    (7, 'team43_control', 'gimbal_controller_node'),
    (8, 'team43_chassis_advisor', 'chassis_advisor_node'),
    (9, 'team43_judge_io', 'judge_io_node'),              # CSV(9列)+地图
    (10, 'team43_dataio', 'trajectory_annotator_node'),
    (11, 'team43_debug', 'gt_error_monitor_node'),        # 可选
]

# 串口桥异常退出时需要立刻收尾的节点
_CRITICAL_EXECUTABLE = 'vgimbal_bridge_node'


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------
def _share_dir(package: str = _PACKAGE_NAME) -> str:
    """从 ament 安装索引定位包的 share 目录。"""
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def _default_config(name: str) -> str:
    """share/config/<name> 默认路径。"""
    return os.path.join(_share_dir(), 'config', name)


def _default_rviz() -> str:
    """share/rviz/gimbal_tracker.rviz 默认路径。"""
    return os.path.join(_share_dir(), 'rviz', 'gimbal_tracker.rviz')


def _as_bool(text: Any, default: bool = False) -> bool:
    """字符串转布尔。"""
    if text is None:
        return default
    s = str(text).strip().lower()
    if s == '':
        return default
    if s in ('1', 'true', 'yes', 'on', 'y', 't'):
        return True
    if s in ('0', 'false', 'no', 'off', 'n', 'f'):
        return False
    raise ValueError(f'非法布尔参数: {text!r}')


def _as_float(text: Any, default: float) -> float:
    """字符串转 float（空串用默认）。"""
    s = str(text).strip() if text is not None else ''
    if s == '':
        return float(default)
    return float(s)


def _overrides_from_json(text: str) -> Dict[str, Any]:
    """解析参数覆盖；空值表示无覆盖，损坏的 JSON 直接报错。"""
    s = str(text or '').strip()
    if not s:
        return {}
    data = json.loads(s)
    if not isinstance(data, dict):
        raise ValueError('overrides_json 必须为 JSON 对象')
    return data


def _node_params(params_file: str, overrides: Dict[str, Any],
                 use_sim_time: bool) -> List[Any]:
    """组装 `parameters=[参数文件, 覆盖字典]`（字典在文件之后 → 覆盖文件值）。"""
    over = dict(overrides)
    over.setdefault('use_sim_time', bool(use_sim_time))
    return [params_file, over]


# ---------------------------------------------------------------------------
# launch setup
# ---------------------------------------------------------------------------
def _launch_setup(context: LaunchContext, *_args: Any, **_kwargs: Any) -> List[Any]:
    """按阶梯生成节点动作 / 静态 TF / RViz / 收尾定时器。"""
    params_file = LaunchConfiguration('params_file').perform(context)
    virtual_yaml = LaunchConfiguration('virtual_yaml').perform(context)
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context), False)
    startup_step = max(0.0, _as_float(LaunchConfiguration('startup_step_s').perform(context), 0.3))
    enable_gt = _as_bool(LaunchConfiguration('enable_gt_monitor').perform(context), False)
    rviz_enabled = _as_bool(LaunchConfiguration('rviz').perform(context), False)
    team_id = str(LaunchConfiguration('team_id').perform(context)).strip()
    output_dir = str(LaunchConfiguration('output_dir').perform(context)).strip()
    mount_tf = _as_bool(LaunchConfiguration('publish_mount_tf').perform(context), True)
    run_duration = _as_float(LaunchConfiguration('run_duration_s').perform(context), 30.0)
    shutdown_grace = _as_float(LaunchConfiguration('shutdown_grace_s').perform(context), 1.5)

    overrides = _overrides_from_json(LaunchConfiguration('overrides_json').perform(context))
    # 队伍编号 / 输出目录：官方路径 ~/ws_championship/output/<队伍编号>.csv
    if team_id:
        overrides['judge.team_id'] = team_id
    if output_dir:
        overrides['judge.output_dir'] = output_dir
    if not enable_gt:
        # 未开启调试监测时，显式把开关也置 false（保证节点内参数一致）
        overrides.setdefault('debug.enable_gt_monitor', False)
    else:
        overrides['debug.enable_gt_monitor'] = True

    actions: List[Any] = []
    ladder: List[Tuple[int, str, str]] = list(NODE_LADDER)

    # ---- 环境变量：必须与评分程序一致，否则互相收不到话题
    actions.append(SetEnvironmentVariable(
        name='ROS_LOCALHOST_ONLY', value='1'))

    # ---- 轮次横幅（便于在 log 里定位）
    actions.append(LogInfo(msg='[bringup] 阶梯启动：步长 %.2fs，共 %d 个节点'
                                '（team=%s，GT监测=%s）'
                                % (startup_step, len(ladder), team_id,
                                   'on' if enable_gt else 'off')))

    # ---- 静态 TF（从 virtual.yaml 读外参）
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_share_dir(), 'launch', 'include', 'static_tf.launch.py')),
        launch_arguments={
            'virtual_yaml': virtual_yaml,
            'tracker_params': params_file,
            'publish_mount_tf': 'true' if mount_tf else 'false',
            'use_sim_time': 'true' if use_sim_time else 'false',
        }.items()))

    # ---- 节点阶梯
    bridge_action: Optional[Node] = None
    for step, pkg, exe in ladder:
        if exe == 'gt_error_monitor_node' and not enable_gt:
            continue
        node = Node(
            package=pkg,
            executable=exe,
            name=exe,
            output='screen',
            emulate_tty=True,
            parameters=([package_config(pkg)] if pkg != 'team43_dataio' else [])
                       + _node_params(params_file, overrides, use_sim_time),
        )
        if exe == _CRITICAL_EXECUTABLE:
            bridge_action = node
        if step == 0:
            actions.append(node)
        else:
            actions.append(TimerAction(period=step * startup_step, actions=[node]))

    # ---- 可选 RViz
    if rviz_enabled:
        actions.append(TimerAction(period=3.0, actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2',
                 output='screen', emulate_tty=True,
                 arguments=['-d', _default_rviz()],
                 parameters=[{'use_sim_time': use_sim_time}]),
        ]))

    # ---- 关键节点意外退出 → 立即收尾（串口被占用等 FATAL 场景，避免干等到 30s）
    if bridge_action is not None:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=bridge_action,
            on_exit=[
                LogInfo(msg='[bringup] 串口桥进程已退出 → 触发整体收尾'),
                EmitEvent(event=Shutdown(reason='vgimbal_bridge exited')),
            ])))

    # ---- 定时收尾（DESIGN §6.14）
    total = max(0.0, run_duration) + max(0.0, shutdown_grace)
    actions.append(LogInfo(msg='[bringup] 任务窗口 %.1fs + 收尾宽限 %.1fs → %.1fs 后自动收尾'
                                % (run_duration, shutdown_grace, total)))
    actions.append(TimerAction(period=total, actions=[
        LogInfo(msg='[bringup] mission complete → Shutdown'),
        EmitEvent(event=Shutdown(reason='mission complete')),
    ]))
    return actions


def generate_launch_description() -> LaunchDescription:
    """节点清单 LaunchDescription。"""
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=_default_config('tracker_params.yaml'),
                              description='整车共享/自定义覆盖参数文件（通过 /** 通配符注入所有节点）'),
        DeclareLaunchArgument('virtual_yaml',
                              default_value=_default_config('virtual.yaml'),
                              description='相机内参/外参单一数据源'),
        DeclareLaunchArgument('startup_step_s', default_value='0.3',
                              description='阶梯启动步长（s）'),
        DeclareLaunchArgument('enable_gt_monitor', default_value='false',
                              description='是否启动 GT 真值监测（P1，默认关）'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('publish_mount_tf', default_value='true',
                              description='是否发布 base_link → gimbal_mount 静态 TF'),
        DeclareLaunchArgument('team_id', default_value='',
                              description='队伍编号（决定 CSV/地图文件名）'),
        DeclareLaunchArgument('output_dir',
                              default_value='',
                              description='官方输出目录（留空用参数文件里的值）'),
        # 赛题一 tick 持续 180 s；评分节点启动后约 3 s 才开始发 tick，
        # 最后一拍后 1 s 才复制文件 → 必须活过 180+3+1 s，留足余量。
        DeclareLaunchArgument('run_duration_s', default_value='210.0',
                              description='任务窗口（s）'),
        DeclareLaunchArgument('shutdown_grace_s', default_value='5.0',
                              description='收尾宽限（s）'),
        DeclareLaunchArgument('overrides_json', default_value='{}',
                              description='参数覆盖（JSON 字典，顶层 launch 组装）'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='固定 false（DESIGN §8.2 第 4 条）'),
        OpaqueFunction(function=_launch_setup),
    ])
