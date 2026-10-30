"""`competition.launch.py` — **主入口**：一键启动全部节点 + 静态 TF + 30s 自动收尾。

对应 DESIGN §3.13 / §6.14 / §7-T05。流程：

```
读取 config/virtual.yaml（内参+外参）→ 传给 static_tf.launch.py 与 perception
读取 config/field.yaml             → team43_grid_mapper
读取各包 config/tracker_defaults.yaml + bringup/config/tracker_params.yaml → 各节点
节点：team43_grid_mapper → base_pose_provider → map_frame_broadcaster → vgimbal_bridge
      → marker_pnp → target_estimator → gimbal_controller → team43_chassis_advisor
      → judge_io → trajectory_annotator [→ gt_error_monitor(可选)]
TimerAction(period = run_duration_s + shutdown_grace_s) → Shutdown(reason='mission complete')
```

用法::

    # 默认：功能包默认参数 + tracker_params.yaml 共享配置（仿真/虚拟云台环境可直接跑通）
    ros2 launch team43_bringup competition.launch.py

    # 正式赛机（串口在 /dev/gimbal、底盘位姿来自主办方 odom）
    ros2 launch team43_bringup competition.launch.py \
        serial_port:=/dev/gimbal base_pose_source:=odom

    # 带 RViz 与调试评估
    ros2 launch team43_bringup competition.launch.py rviz:=true enable_gt_monitor:=true

    # 临时覆盖**任意**参数（不改 tracker_params.yaml；值按 JSON 猜类型）
    ros2 launch team43_bringup competition.launch.py \
        extra_params:="perception.marker_size_m=0.105;judge.team_id=T7"

**单一数据源**：本文件刻意把大部分 launch 参数默认留空 = 「不覆盖」，
即沿用各功能包默认值与 `config/tracker_params.yaml` 的共享值；只有场地文件路径与任务时长会被显式注入，
以保证 launch 与节点看到同一份配置。

运行环境：Ubuntu 22.04 / ROS2 Humble。
"""

from __future__ import annotations

import json
import os
from typing import Any
from typing import Dict
from typing import List

from team43_common.config_loader import load_ros_params
from team43_bringup.launch_utils import as_bool
from team43_bringup.launch_utils import default_config
from team43_bringup.launch_utils import share_config
from team43_bringup.launch_utils import share_dir
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------
def _put_if_set(over: Dict[str, Any], key: str, text: Any) -> None:
    """把非空的字符串参数写入覆盖字典（用于数值以外的一般参数）。"""
    s = str(text).strip() if text is not None else ''
    if s != '':
        over[key] = s


def _put_num_if_set(over: Dict[str, Any], key: str, text: Any) -> None:
    """把非空的数值参数写入覆盖字典（解析成 float）。"""
    s = str(text).strip() if text is not None else ''
    if s == '':
        return
    over[key] = float(s)


def _put_int_if_set(over: Dict[str, Any], key: str, text: Any) -> None:
    """把非空的整数参数写入覆盖字典（解析成 int）。"""
    s = str(text).strip() if text is not None else ''
    if s == '':
        return
    over[key] = int(s)


def _parse_extra_params(text: Any) -> Dict[str, Any]:
    """解析 ``extra_params`` 的「任意参数覆盖」文本（纯标准库，可离线单测）。

    形式为 ``键=值``，多项之间用 ``;`` 或 ``,`` 分隔；分隔符出现在 ``[]`` ``{}`` ``()``
    或引号内部时不算分隔（所以 ``[0.1,0,0.05]`` 这种数组不会被切碎）。

    值按 JSON 解析：``0.35``→float、``true``→bool、``[0.1,0,0.05]``→list、
    ``"abc"``→str；不是合法 JSON 时按**字符串**处理（``/dev/gimbal``、``~/ws/x``）。

    Args:
        text: 命令行给的那串文本（可以带引号，launch 会自己剥掉）。

    Returns:
        覆盖字典（可空）。

    Raises:
        ValueError: 某项缺少 ``=`` 或键为空。
    """
    items: List[str] = []
    buf: List[str] = []
    depth = 0
    quote = ''
    for ch in str(text or ''):
        if quote:
            buf.append(ch)
            if ch == quote:
                quote = ''
            continue
        if ch in '"\'':
            quote = ch
            buf.append(ch)
            continue
        if ch in '[{(':
            depth += 1
            buf.append(ch)
            continue
        if ch in ']})':
            depth = max(0, depth - 1)
            buf.append(ch)
            continue
        if depth == 0 and ch in ';,':
            items.append(''.join(buf))
            buf = []
            continue
        buf.append(ch)
    items.append(''.join(buf))

    over: Dict[str, Any] = {}
    for item in items:
        item = item.strip()
        if not item:
            continue
        key, sep, raw = item.partition('=')
        key = key.strip()
        if not sep or not key:
            raise ValueError('extra_params 里这一项应为 键=值，收到 %r' % item)
        raw = raw.strip()
        try:
            over[key] = json.loads(raw)
        except json.JSONDecodeError:
            # 未加引号的串口路径/普通文本合法；损坏的 JSON 容器不能当文本放行。
            if raw.startswith(('[', '{', '"')):
                raise
            over[key] = raw
    return over


# ---------------------------------------------------------------------------
# launch setup
# ---------------------------------------------------------------------------
def _launch_setup(context: LaunchContext, *_args: Any, **_kwargs: Any) -> List[Any]:
    """读取 YAML、组装参数覆盖、include 节点清单。"""
    params_file = LaunchConfiguration('params_file').perform(context)
    flat = load_ros_params(params_file)

    # ---- 路径类参数：launch 参数留空时，回退到 `tracker_params.yaml` 的 config.* 键
    #      （单一数据源；路径一律按 share/config 解析，不使用相对路径）
    virtual_yaml = str(LaunchConfiguration('virtual_yaml').perform(context)).strip()
    if virtual_yaml == '':
        virtual_yaml = share_config(str(flat.get('config.virtual_yaml', 'virtual.yaml')
                                         or 'virtual.yaml'))
    field_yaml = str(LaunchConfiguration('field_yaml').perform(context)).strip()
    if field_yaml == '':
        field_yaml = share_config(str(flat.get('config.field_yaml', 'field.yaml')
                                       or 'field.yaml'))
    maps_dir = str(LaunchConfiguration('maps_dir').perform(context)).strip()
    if maps_dir == '':
        maps_dir = str(flat.get('config.maps_dir', '') or '').strip()
    csv_out_dir = str(LaunchConfiguration('csv_out_dir').perform(context)).strip()
    if csv_out_dir == '':
        csv_out_dir = str(flat.get('config.outputs_dir', '') or '').strip()

    enable_gt = as_bool(LaunchConfiguration('enable_gt_monitor').perform(context), False)
    rviz_enabled = as_bool(LaunchConfiguration('rviz').perform(context), False)

    run_duration = str(LaunchConfiguration('run_duration_s').perform(context)).strip()
    if run_duration == '':
        run_duration = str(flat.get('mission.run_duration_s', 30.0))
    shutdown_grace = str(LaunchConfiguration('shutdown_grace_s').perform(context)).strip()
    if shutdown_grace == '':
        shutdown_grace = str(flat.get('mission.shutdown_grace_s', 1.5))

    over: Dict[str, Any] = {}

    # ---- 单一数据源路径：显式注入，避免节点各自"猜"路径
    over['config.virtual_yaml'] = virtual_yaml
    over['grid.field_yaml'] = field_yaml            # ⚠ 显式注入（不再依赖环境变量自动定位）
    if maps_dir:
        over['grid.out_dir'] = maps_dir
    over['mission.run_duration_s'] = float(run_duration)
    over['mission.shutdown_grace_s'] = float(shutdown_grace)

    # ---- 按需覆盖（留空 = 沿用 tracker_params.yaml）
    _put_if_set(over, 'judge.team_id', LaunchConfiguration('team_id').perform(context))
    _put_if_set(over, 'serial.port',
                LaunchConfiguration('serial_port').perform(context))
    _put_if_set(over, 'localization.base_pose_source',
                LaunchConfiguration('base_pose_source').perform(context))
    _put_if_set(over, 'perception.id_filter_mode',
                LaunchConfiguration('id_filter_mode').perform(context))
    _put_if_set(over, 'chassis.cmd_frame',
                LaunchConfiguration('cmd_frame').perform(context))
    _put_int_if_set(over, 'perception.target_digit',
                    LaunchConfiguration('target_digit').perform(context))
    _put_num_if_set(over, 'chassis.v_max',
                    LaunchConfiguration('v_max').perform(context))
    if csv_out_dir:
        over['csv.out_dir'] = csv_out_dir
    _put_if_set(over, 'csv.run_id',
                LaunchConfiguration('csv_run_id').perform(context))
    _put_if_set(over, 'grid.map_source',
                LaunchConfiguration('grid_map_source').perform(context))
    _put_if_set(over, 'grid.map_topic',
                LaunchConfiguration('grid_map_topic').perform(context))
    _put_num_if_set(over, 'grid.republish_period_s',
                    LaunchConfiguration('grid_republish_period_s').perform(context))
    preview = str(LaunchConfiguration('grid_write_preview_png').perform(context)).strip()
    if preview != '':
        over['grid.write_preview_png'] = as_bool(preview, True)

    # ---- 任意参数覆盖（最后应用 → 优先级最高）：`extra_params:="键=值;键=值"`
    extra_text = str(LaunchConfiguration('extra_params').perform(context)).strip()
    if extra_text:
        extra_over = _parse_extra_params(extra_text)
        over.update(extra_over)
        bad = [k for k in extra_over if '.' not in k]
        if bad:
            # 点分键是本工程的统一约定，不带点的多半是写错了
            print('[competition] ⚠ extra_params 里这些键没有命名空间前缀（可能写错）: %s'
                  % ', '.join(bad))

    actions: List[Any] = [
        LogInfo(msg='[competition] 参数文件=%s' % params_file),
        LogInfo(msg='[competition] 场地文件=%s（显式注入 grid.field_yaml）' % field_yaml),
        LogInfo(msg='[competition] 任务窗口=%ss，收尾宽限=%ss，GT 监测=%s，RViz=%s'
                    % (run_duration, shutdown_grace, enable_gt, rviz_enabled)),
        LogInfo(msg='[competition] 参数覆盖共 %d 项%s'
                    % (len(over), ('：' + json.dumps(over, ensure_ascii=False))
                       if over else '')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share_dir(), 'launch', 'include', 'nodes.launch.py')),
            launch_arguments={
                'params_file': params_file,
                'virtual_yaml': virtual_yaml,
                'use_sim_time': 'false',
                'startup_step_s': LaunchConfiguration('startup_step_s'),
                'enable_gt_monitor': 'true' if enable_gt else 'false',
                'rviz': 'true' if rviz_enabled else 'false',
                'publish_mount_tf': LaunchConfiguration('publish_mount_tf'),
                'run_duration_s': run_duration,
                'shutdown_grace_s': shutdown_grace,
                'overrides_json': json.dumps(over, ensure_ascii=False),
            }.items()),
    ]
    return actions


def generate_launch_description() -> LaunchDescription:
    """主入口 LaunchDescription。"""
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config('tracker_params.yaml'),
                              description='整车共享/自定义覆盖参数文件'),
        DeclareLaunchArgument('virtual_yaml', default_value=default_config('virtual.yaml'),
                              description='相机内参/外参单一数据源（空 = 用 config.virtual_yaml）'),
        DeclareLaunchArgument('field_yaml', default_value=default_config('field.yaml'),
                              description='场地边界与障碍；显式注入 grid.field_yaml（空 = 用 config.field_yaml）'),
        DeclareLaunchArgument('maps_dir', default_value='',
                              description='地图输出目录（空 = 用参数文件 config.maps_dir / grid.out_dir）'),
        DeclareLaunchArgument('team_id', default_value='', description='队号；空则沿用参数文件'),
        DeclareLaunchArgument('serial_port', default_value='',
                              description='串口路径（空 = 用参数值；正式赛填 /dev/gimbal）'),
        DeclareLaunchArgument('base_pose_source', default_value='',
                              description='底盘位姿源 mock|odom（空 = 用参数值）'),
        DeclareLaunchArgument('target_digit', default_value='',
                              description='要跟踪的 marker ID（空 = 用参数值）'),
        DeclareLaunchArgument('id_filter_mode', default_value='',
                              description='marker ID 过滤 off|on（空 = 用参数值）'),
        DeclareLaunchArgument('v_max', default_value='',
                              description='底盘建议速度上限 m/s（空 = 用参数值）'),
        DeclareLaunchArgument('cmd_frame', default_value='',
                              description='cmd_v 参考系 map|base_link（空 = 用参数值）'),
        DeclareLaunchArgument('csv_out_dir', default_value='',
                              description='产物根目录（空 = 用参数文件 config.outputs_dir / csv.out_dir）'),
        DeclareLaunchArgument('csv_run_id', default_value='',
                              description='run_id 子目录（空 = 用参数值）'),
        DeclareLaunchArgument('grid_map_source', default_value='',
                              description='地图来源 config|topic（空 = 用参数值）'),
        DeclareLaunchArgument('grid_map_topic', default_value='',
                              description='map_source=topic 时订阅的话题（空 = 用参数值）'),
        DeclareLaunchArgument('grid_republish_period_s', default_value='',
                              description='锁存地图重发周期 s，0=只发一次（空 = 用参数值）'),
        DeclareLaunchArgument('grid_write_preview_png', default_value='',
                              description='是否绘制 map_preview.png（空 = 用参数值）'),
        DeclareLaunchArgument('extra_params', default_value='',
                              description='任意参数覆盖，键=值 多项用 ; 分隔（空 = 不覆盖）。'
                                          '例: "perception.marker_size_m=0.105;judge.team_id=T7"'),
        DeclareLaunchArgument('run_duration_s', default_value='',
                              description='任务窗口 s（空 = 用参数文件 mission.run_duration_s）'),
        DeclareLaunchArgument('shutdown_grace_s', default_value='',
                              description='收尾宽限 s（空 = 用参数文件 mission.shutdown_grace_s）'),
        DeclareLaunchArgument('startup_step_s', default_value='0.3',
                              description='阶梯启动步长 s'),
        DeclareLaunchArgument('enable_gt_monitor', default_value='false',
                              description='启动 GT 真值监测（P1，默认关）'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='启动 RViz2'),
        DeclareLaunchArgument('publish_mount_tf', default_value='true',
                              description='发布 base_link → gimbal_mount 静态 TF'),
        OpaqueFunction(function=_launch_setup),
    ])
