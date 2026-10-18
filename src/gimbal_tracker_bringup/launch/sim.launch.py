"""`sim.launch.py` — 仿真预设（基座 mock + 虚拟云台串口 + 可选调试）。

等价于::

    ros2 launch gimbal_tracker_bringup competition.launch.py \
        serial_port:=/tmp/ttyVGIMBAL base_pose_source:=mock \
        enable_gt_monitor:=true rviz:=true

差别只在**默认值**（DESIGN §3.13 / §7-T05）：

| 参数 | competition | sim |
|---|---|---|
| `serial_port` | 参数文件（`/tmp/ttyVGIMBAL`） | `/tmp/ttyVGIMBAL` |
| `base_pose_source` | 参数文件（`mock`） | `mock` |
| `enable_gt_monitor` | `false` | **`true`**（仿真才有真值可对比） |
| `rviz` | `false` | **`true`** |

⚠ 仿真**不要**启动 `robo_bridge_edu` 或其它串口桥：`world → gimbal_link` 只能有一个发布者。
⚠ 本文件无法在 Windows（无 ROS 2）上运行验证。
"""

from __future__ import annotations

import os
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_PACKAGE_NAME = 'gimbal_tracker_bringup'


def _share_dir(package: str = _PACKAGE_NAME) -> str:
    """从 ament 安装索引定位包的 share 目录。"""
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def _default_config(name: str) -> str:
    """share/config/<name> 默认路径。"""
    return os.path.join(_share_dir(), 'config', name)


def generate_launch_description() -> LaunchDescription:
    """仿真入口 LaunchDescription。"""
    args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument('params_file',
                              default_value=_default_config('tracker_params.yaml'),
                              description='§6.13 全表参数文件'),
        DeclareLaunchArgument('virtual_yaml',
                              default_value=_default_config('virtual.yaml'),
                              description='相机内参/外参单一数据源'),
        DeclareLaunchArgument('field_yaml',
                              default_value=_default_config('field.yaml'),
                              description='场地描述'),
        DeclareLaunchArgument('serial_port', default_value='/tmp/ttyVGIMBAL',
                              description='虚拟云台 pty 软链'),
        DeclareLaunchArgument('base_pose_source', default_value='mock',
                              description='仿真无底盘 → mock'),
        DeclareLaunchArgument('enable_gt_monitor', default_value='true',
                              description='仿真默认开启 GT 真值评估'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='仿真默认开 RViz2'),
        DeclareLaunchArgument('run_duration_s', default_value='',
                              description='任务窗口 s（空 = 用参数文件值）'),
        DeclareLaunchArgument('startup_step_s', default_value='0.3',
                              description='阶梯启动步长 s'),
        DeclareLaunchArgument('target_digit', default_value='',
                              description='要跟踪的 marker ID（空 = 用参数值）'),
    ]

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_share_dir(), 'launch', 'competition.launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'virtual_yaml': LaunchConfiguration('virtual_yaml'),
            'field_yaml': LaunchConfiguration('field_yaml'),
            'serial_port': LaunchConfiguration('serial_port'),
            'base_pose_source': LaunchConfiguration('base_pose_source'),
            'enable_gt_monitor': LaunchConfiguration('enable_gt_monitor'),
            'rviz': LaunchConfiguration('rviz'),
            'run_duration_s': LaunchConfiguration('run_duration_s'),
            'startup_step_s': LaunchConfiguration('startup_step_s'),
            'target_digit': LaunchConfiguration('target_digit'),
        }.items())

    return LaunchDescription(args + [
        LogInfo(msg='[sim] 仿真预设：base_pose_source=mock, serial_port=/tmp/ttyVGIMBAL, '
                    'GT 监测与 RViz 默认开启'),
        include,
    ])
