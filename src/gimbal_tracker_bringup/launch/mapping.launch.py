"""`mapping.launch.py` — 赛前离线建图（**只起 `grid_mapper_node`**，DESIGN §3.13 / §7-T05）。

流程（PRD §3.6「赛前（离线）mapping」）：

```
读取 config/field.yaml → 栅格化（0.05 m/cell，12×12m，origin [-6,-6,0]）
→ 写 maps/map.pgm + maps/map.yaml + maps/map_preview.png
→ 发布锁存 /map/grid 与 /map/ray_grid（供 RViz 查看）
```

用法::

    ros2 launch gimbal_tracker_bringup mapping.launch.py
    ros2 launch gimbal_tracker_bringup mapping.launch.py maps_dir:=maps rviz:=true
    ros2 launch gimbal_tracker_bringup mapping.launch.py exit_after_build_s:=5.0

⚠ 产出的 `map.pgm` / `map.yaml` 必须能被 `nav2_map_server` 加载（PRD A4）。
⚠ 本文件无法在 Windows（无 ROS 2）上运行验证。
"""

from __future__ import annotations

import json
import os
from typing import Any
from typing import Dict
from typing import List

import yaml
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PACKAGE_NAME = 'gimbal_tracker_bringup'
_EXECUTABLE = 'grid_mapper_node'


def _share_dir(package: str = _PACKAGE_NAME) -> str:
    """从 ament 安装索引定位包的 share 目录。"""
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def _default_config(name: str) -> str:
    """share/config/<name> 默认路径。"""
    return os.path.join(_share_dir(), 'config', name)


def _share_config(name: str) -> str:
    """把（可能是包内相对名的）配置文件解析成 share/config 下的绝对路径。"""
    return os.path.join(_share_dir(), 'config', os.path.basename(str(name)))


def _flat_params(path: str) -> Dict[str, Any]:
    """读取 `tracker_params.yaml` 的 `/**/ros__parameters` 扁平字典（失败返回空）。"""
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except Exception:  # pragma: no cover
        return {}
    if not isinstance(data, dict):
        return {}
    wildcard = data.get('/**')
    if not isinstance(wildcard, dict):
        return {}
    params = wildcard.get('ros__parameters')
    return params if isinstance(params, dict) else {}


def _default_rviz() -> str:
    """share/rviz/gimbal_tracker.rviz 默认路径。"""
    return os.path.join(_share_dir(), 'rviz', 'gimbal_tracker.rviz')


def _as_bool(text: Any, default: bool = False) -> bool:
    """字符串转布尔。"""
    s = str(text).strip().lower() if text is not None else ''
    if s == '':
        return default
    return s in ('1', 'true', 'yes', 'on', 'y', 't')


def _launch_setup(context: LaunchContext, *_args: Any, **_kwargs: Any) -> List[Any]:
    """组装 grid_mapper 的独立建图流程。"""
    params_file = LaunchConfiguration('params_file').perform(context)
    flat = _flat_params(params_file)

    virtual_yaml = str(LaunchConfiguration('virtual_yaml').perform(context)).strip()
    if virtual_yaml == '':
        virtual_yaml = _share_config(str(flat.get('config.virtual_yaml', 'virtual.yaml')
                                         or 'virtual.yaml'))
    field_yaml = str(LaunchConfiguration('field_yaml').perform(context)).strip()
    if field_yaml == '':
        field_yaml = _share_config(str(flat.get('config.field_yaml', 'field.yaml')
                                       or 'field.yaml'))
    maps_dir = str(LaunchConfiguration('maps_dir').perform(context)).strip()
    if maps_dir == '':
        maps_dir = str(flat.get('config.maps_dir', '') or '').strip() or 'maps'

    map_source = str(LaunchConfiguration('grid_map_source').perform(context)).strip()
    rviz_enabled = _as_bool(LaunchConfiguration('rviz').perform(context), False)
    exit_after = str(LaunchConfiguration('exit_after_build_s').perform(context)).strip()

    over: Dict[str, Any] = {
        'config.virtual_yaml': virtual_yaml,
        'grid.field_yaml': field_yaml,          # ⚠ 显式注入，禁止依赖自动定位
        'use_sim_time': False,
    }
    if maps_dir:
        over['grid.out_dir'] = maps_dir
    if map_source:
        over['grid.map_source'] = map_source
    if map_source == 'topic':
        topic = str(LaunchConfiguration('grid_map_topic').perform(context)).strip()
        if topic:
            over['grid.map_topic'] = topic

    actions: List[Any] = [
        LogInfo(msg='[mapping] 场地文件=%s → 目标目录=%s（参数文件=%s）'
                    % (field_yaml, maps_dir, params_file)),
        Node(
            package='grid_mapper',
            executable=_EXECUTABLE,
            name=_EXECUTABLE,
            output='screen',
            emulate_tty=True,
            parameters=[params_file, over],
        ),
    ]

    if rviz_enabled:
        actions.append(TimerAction(period=2.0, actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2',
                 output='screen', emulate_tty=True,
                 arguments=['-d', _default_rviz()],
                 parameters=[{'use_sim_time': False}]),
        ]))

    if exit_after:
        try:
            period = max(0.5, float(exit_after))
        except ValueError:
            period = 0.0
        if period > 0.0:
            actions.append(LogInfo(msg='[mapping] %.1fs 后自动退出（建图已落盘）' % period))
            actions.append(TimerAction(period=period, actions=[
                EmitEvent(event=Shutdown(reason='mapping done')),
            ]))
    return actions


def generate_launch_description() -> LaunchDescription:
    """建图入口 LaunchDescription。"""
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=_default_config('tracker_params.yaml'),
                              description='§6.13 全表参数文件'),
        DeclareLaunchArgument('virtual_yaml', default_value=_default_config('virtual.yaml'),
                              description='相机内参/外参单一数据源'),
        DeclareLaunchArgument('field_yaml', default_value=_default_config('field.yaml'),
                              description='场地边界与障碍；显式注入 grid.field_yaml（空 = 用 config.field_yaml）'),
        DeclareLaunchArgument('maps_dir', default_value='',
                              description='地图输出目录（空 = 用参数文件 config.maps_dir，再退到 maps）'),
        DeclareLaunchArgument('grid_map_source', default_value='config',
                              description='地图来源 config|topic'),
        DeclareLaunchArgument('grid_map_topic', default_value='/map',
                              description='map_source=topic 时订阅的话题'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否启动 RViz2 查看 /map/grid'),
        DeclareLaunchArgument('exit_after_build_s', default_value='',
                              description='建图后 N 秒自动退出（空 = 常驻，Ctrl-C 退出）'),
        OpaqueFunction(function=_launch_setup),
    ])
