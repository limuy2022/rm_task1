"""`launch/*.launch.py` 的功能性回归（**无需 ROS2**）。

手法：把 `launch` / `launch_ros` 换成桩（见 `conftest.StubRos`），然后**真跑**
每个 launch 文件的 `generate_launch_description()` 与 `OpaqueFunction._launch_setup`，
对生成出来的 `Node` / `TimerAction` / `IncludeLaunchDescription` 做断言。

覆盖 DESIGN §3.13 / §6.14 / §7-T05 的硬要求：
  * `static_tf.launch.py` 的外参**必须来自 `virtual.yaml`**（不手抄、缺失即 FATAL）
  * 阶梯启动顺序与周期（`grid_mapper` 在 t=0）
  * 参数注入（params_file + overrides 字典）与 `use_sim_time=false`
  * 收尾 `TimerAction(period = run_duration + grace) → Shutdown('mission complete')`
  * `vgimbal_bridge_node` 异常退出的兜底收尾
  * `competition` / `sim` / `mapping` 三者的默认差异

⚠ 这里验证的是 **launch 的 Python 逻辑**；`colcon build`、`ros2 launch` 的进程编排、
`static_transform_publisher` 是否接受 `--flag` 风格，仍需在 Ubuntu 24.04 + ROS 2 Jazzy 实跑。
"""

from __future__ import annotations

import json
import math
import os
from typing import Any
from typing import Dict
from typing import List
from typing import Tuple

import pytest
import yaml

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.abspath(os.path.join(TEST_DIR, '..'))
CONFIG_DIR = os.path.join(PKG_DIR, 'config')
LAUNCH_DIR = os.path.join(PKG_DIR, 'launch')
RVIZ_DIR = os.path.join(PKG_DIR, 'rviz')

VIRTUAL_YAML = os.path.join(CONFIG_DIR, 'virtual.yaml')
TRACKER_PARAMS = os.path.join(CONFIG_DIR, 'tracker_params.yaml')
FIELD_YAML = os.path.join(CONFIG_DIR, 'field.yaml')

# DESIGN §6.14 规定的启动顺序（package, executable）
EXPECTED_ORDER: List[Tuple[str, str]] = [
    ('grid_mapper', 'grid_mapper_node'),
    ('gimbal_localization', 'base_pose_provider_node'),
    ('gimbal_localization', 'map_frame_broadcaster_node'),
    ('gimbal_serial_bridge', 'vgimbal_bridge_node'),
    ('gimbal_perception', 'marker_pnp_node'),
    ('gimbal_estimation', 'target_estimator_node'),
    ('gimbal_control', 'gimbal_controller_node'),
    ('chassis_advisor', 'chassis_advisor_node'),
    ('gimbal_dataio', 'csv_writer_node'),
    ('gimbal_dataio', 'trajectory_annotator_node'),
]

STEP = 0.3


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------
def _flat_params(path: str) -> Dict[str, Any]:
    """读 `tracker_params.yaml` 的 `/**/ros__parameters`。"""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return data['/**']['ros__parameters']


def _collect_nodes(entries: List[Any], env: Any) -> List[Tuple[float, str, str, Dict]]:
    """按**实际启动顺序**抽取 `(period, package, executable, node_kwargs)`。

    裸 `Node` 视为 t=0；`TimerAction` 内的 `Node` 取该定时器的 period。
    """
    out: List[Tuple[float, str, str, Dict]] = []
    for entity in entries:
        if isinstance(entity, env.TimerAction):
            for sub in entity.kw['actions']:
                if isinstance(sub, env.Node):
                    out.append((float(entity.kw['period']), sub.kw['package'],
                                sub.kw['executable'], sub.kw))
        elif isinstance(entity, env.Node):
            out.append((0.0, entity.kw['package'], entity.kw['executable'], entity.kw))
    return out


def _shutdown_periods(entries: List[Any], env: Any) -> List[Tuple[float, str]]:
    """找出所有 `TimerAction → EmitEvent(Shutdown)` 的 `(period, reason)`。"""
    out: List[Tuple[float, str]] = []
    for entity in entries:
        if not isinstance(entity, env.TimerAction):
            continue
        for sub in entity.kw['actions']:
            if isinstance(sub, env.EmitEvent) and isinstance(sub.kw.get('event'),
                                                            env.Shutdown):
                out.append((float(entity.kw['period']),
                            str(sub.kw['event'].kw.get('reason', ''))))
    return out


def _arg_of(args: List[str], flag: str) -> str:
    """取 `--flag value` 里的 value。"""
    return args[args.index(flag) + 1]


# ---------------------------------------------------------------------------
# 1. static_tf.launch.py
# ---------------------------------------------------------------------------
def test_static_tf_declares_expected_args(stub_ros):
    """声明 4 个 launch 参数，默认值合理。"""
    mod = stub_ros.load_launch_module('include/static_tf.launch.py', 'lf_static_tf_1')
    declared = stub_ros.declared_args(mod.generate_launch_description())
    assert set(declared) == {'virtual_yaml', 'tracker_params', 'publish_mount_tf',
                             'use_sim_time'}
    assert declared['virtual_yaml'].endswith(os.path.join('config', 'virtual.yaml'))
    assert declared['publish_mount_tf'] == 'true'
    assert declared['use_sim_time'] == 'false'


def test_static_tf_publishes_three_tfs_from_yaml(stub_ros):
    """三个静态 TF 的数值/帧名必须来自 `virtual.yaml` 与 tracker_params（不手抄）。"""
    mod = stub_ros.load_launch_module('include/static_tf.launch.py', 'lf_static_tf_2')
    ld = mod.generate_launch_description()
    stub_ros.args.update({'virtual_yaml': VIRTUAL_YAML,
                          'tracker_params': TRACKER_PARAMS,
                          'publish_mount_tf': 'true',
                          'use_sim_time': 'false'})
    stub_ros.nodes.clear()
    entries = stub_ros.opaque_setup(ld)(None)

    assert len(entries) == 3, '应发布 3 个静态 TF'
    by_name = {n.kw['name']: n.kw for n in entries}

    with open(VIRTUAL_YAML, 'r', encoding='utf-8') as f:
        ex = yaml.safe_load(f)['extrinsics']
    cam_xyz = [float(v) for v in ex['camera_xyz']]
    cam_rpy = [float(v) for v in ex['camera_rpy']]

    # (a) gimbal_link -> camera_link，数值取自 virtual.yaml
    a = by_name['static_tf_gimbal_camera']
    assert (a['package'], a['executable']) == ('tf2_ros', 'static_transform_publisher')
    args = a['arguments']
    for flag, val in (('--x', cam_xyz[0]), ('--y', cam_xyz[1]), ('--z', cam_xyz[2]),
                      ('--roll', cam_rpy[0]), ('--pitch', cam_rpy[1]),
                      ('--yaw', cam_rpy[2])):
        assert float(_arg_of(args, flag)) == pytest.approx(val), flag
    assert _arg_of(args, '--frame-id') == ex['frames']['gimbal']
    assert _arg_of(args, '--child-frame-id') == ex['frames']['camera']

    # (b) camera_link -> camera_optical_frame，rpy = (-pi/2, 0, -pi/2)
    b = by_name['static_tf_camera_optical']['arguments']
    assert _arg_of(b, '--frame-id') == ex['frames']['camera']
    assert _arg_of(b, '--child-frame-id') == ex['frames']['optical']
    assert float(_arg_of(b, '--roll')) == pytest.approx(-math.pi / 2.0)
    assert float(_arg_of(b, '--pitch')) == pytest.approx(0.0)
    assert float(_arg_of(b, '--yaw')) == pytest.approx(-math.pi / 2.0)

    # (c) base_link -> gimbal_mount，数值取自 tracker_params
    mount = [float(v) for v in
             _flat_params(TRACKER_PARAMS)['localization.gimbal_mount_xyz_rpy']]
    c = by_name['static_tf_base_gimbal_mount']['arguments']
    assert _arg_of(c, '--frame-id') == 'base_link'
    assert _arg_of(c, '--child-frame-id') == 'gimbal_mount'
    for flag, val in (('--x', mount[0]), ('--y', mount[1]), ('--z', mount[2]),
                      ('--roll', mount[3]), ('--pitch', mount[4]), ('--yaw', mount[5])):
        assert float(_arg_of(c, flag)) == pytest.approx(val), flag

    # 全部静态 TF 显式 use_sim_time=false
    for n in entries:
        assert n.kw['parameters'] == [{'use_sim_time': False}]

    # 禁止出现 world→odom / map→odom（那两个归 map_frame_broadcaster_node，
    # 否则会出现双发布者）
    all_args = ' '.join(' '.join(n.kw['arguments']) for n in entries)
    assert '--frame-id world' not in all_args
    assert '--frame-id map' not in all_args


def test_static_tf_optical_rotation_matches_frames_single_source(stub_ros):
    """launch 里的光学系 rpy 必须与 `gimbal_common.frames` 的单一真源一致。"""
    import numpy as np
    from gimbal_common import frames as FR

    mod = stub_ros.load_launch_module('include/static_tf.launch.py', 'lf_static_tf_3')
    R_launch = FR.rpy_to_mat(list(mod._OPTICAL_RPY))
    assert float(np.abs(R_launch - FR.R_opt_from_link()).max()) < 1e-9
    mod._assert_optical_rpy_matches_frames()          # 不得抛错


def test_static_tf_missing_yaml_is_fatal(stub_ros):
    """`virtual.yaml` 缺失 → 必须抛错（DESIGN §8.4：配置文件缺失 = FATAL）。"""
    mod = stub_ros.load_launch_module('include/static_tf.launch.py', 'lf_static_tf_4')
    ld = mod.generate_launch_description()
    stub_ros.args.update({'virtual_yaml': os.path.join(CONFIG_DIR, 'nope.yaml'),
                          'tracker_params': TRACKER_PARAMS,
                          'publish_mount_tf': 'true',
                          'use_sim_time': 'false'})
    with pytest.raises(RuntimeError):
        stub_ros.opaque_setup(ld)(None)


def test_static_tf_mount_tf_can_be_disabled(stub_ros):
    """`publish_mount_tf=false` 时只发 2 个 TF。"""
    mod = stub_ros.load_launch_module('include/static_tf.launch.py', 'lf_static_tf_5')
    ld = mod.generate_launch_description()
    stub_ros.args.update({'virtual_yaml': VIRTUAL_YAML,
                          'tracker_params': TRACKER_PARAMS,
                          'publish_mount_tf': 'false', 'use_sim_time': 'false'})
    assert len(stub_ros.opaque_setup(ld)(None)) == 2


# ---------------------------------------------------------------------------
# 2. include/nodes.launch.py
# ---------------------------------------------------------------------------
def _nodes_setup(stub_ros, tag: str, **arg_over: str) -> List[Any]:
    """加载 nodes.launch.py，填好参数并跑 `_launch_setup`，返回实体列表。"""
    mod = stub_ros.load_launch_module('include/nodes.launch.py', 'lf_nodes_%s' % tag)
    ld = mod.generate_launch_description()
    args = {'params_file': TRACKER_PARAMS, 'virtual_yaml': VIRTUAL_YAML,
            'startup_step_s': str(STEP), 'enable_gt_monitor': 'false',
            'rviz': 'false', 'publish_mount_tf': 'true',
            'run_duration_s': '30.0', 'shutdown_grace_s': '1.5',
            'overrides_json': json.dumps({'serial.port': '/dev/gimbal'}),
            'use_sim_time': 'false'}
    args.update(arg_over)
    stub_ros.args.clear()
    stub_ros.args.update(args)
    stub_ros.nodes.clear()
    return stub_ros.opaque_setup(ld)(None)


def test_nodes_ladder_order_and_periods(stub_ros):
    """启动顺序 = DESIGN §6.14，`grid_mapper` 在 t=0，其余按步长阶梯。"""
    entries = _nodes_setup(stub_ros, 'order')
    got = _collect_nodes(entries, stub_ros)
    assert [(pkg, exe) for _t, pkg, exe, _kw in got] == EXPECTED_ORDER
    assert got[0][0] == 0.0, 'grid_mapper_node 必须最先（t=0）'
    for i, (period, _pkg, _exe, _kw) in enumerate(got):
        assert period == pytest.approx(i * STEP), '第 %d 个节点阶梯周期不对' % i


def test_nodes_inject_params_and_use_sim_time(stub_ros):
    """每个节点注入 params_file + 覆盖字典，且 use_sim_time 强制 false。"""
    entries = _nodes_setup(stub_ros, 'params')
    for _t, _pkg, _exe, kw in _collect_nodes(entries, stub_ros):
        params = kw['parameters']
        assert params[0] == TRACKER_PARAMS
        assert isinstance(params[1], dict)
        assert params[1]['use_sim_time'] is False
        assert params[1]['serial.port'] == '/dev/gimbal'
        assert kw['name'] == kw['executable']


def test_nodes_gt_monitor_off_by_default(stub_ros):
    """`enable_gt_monitor=false` 时不创建该节点，并显式下发 debug 开关 false。"""
    entries = _nodes_setup(stub_ros, 'gtoff')
    got = _collect_nodes(entries, stub_ros)
    assert 'gt_error_monitor_node' not in [e for _t, _p, e, _kw in got]
    assert len(got) == len(EXPECTED_ORDER)
    for _t, _p, _e, kw in got:
        assert kw['parameters'][1]['debug.enable_gt_monitor'] is False


def test_nodes_gt_monitor_and_rviz_on(stub_ros):
    """开启后追加 gt 节点与 rviz2（且 rviz 指向本包 rviz 配置）。"""
    entries = _nodes_setup(stub_ros, 'gton', enable_gt_monitor='true', rviz='true')
    got = _collect_nodes(entries, stub_ros)
    exes = [e for _t, _p, e, _kw in got]
    assert 'gt_error_monitor_node' in exes
    assert 'rviz2' in exes
    gt = [kw for _t, _p, e, kw in got if e == 'gt_error_monitor_node'][0]
    assert gt['package'] == 'gimbal_debug'
    assert gt['parameters'][1]['debug.enable_gt_monitor'] is True
    rv = [kw for _t, _p, e, kw in got if e == 'rviz2'][0]
    assert rv['package'] == 'rviz2'
    assert rv['arguments'][0] == '-d'
    assert rv['arguments'][1].endswith('gimbal_tracker.rviz')
    assert os.path.isfile(rv['arguments'][1])


def test_nodes_shutdown_timer_and_bridge_watchdog(stub_ros):
    """收尾定时器 = run+grace；串口桥异常退出有兜底收尾。"""
    entries = _nodes_setup(stub_ros, 'shutdown')
    periods = _shutdown_periods(entries, stub_ros)
    assert periods, '缺少 Shutdown 定时器'
    timer_period, reason = periods[-1]
    assert timer_period == pytest.approx(30.0 + 1.5)
    assert reason == 'mission complete'

    handlers = [e for e in entries if isinstance(e, stub_ros.RegisterEventHandler)]
    assert len(handlers) == 1
    assert handlers[0].kw['target_action'].kw['executable'] == 'vgimbal_bridge_node'
    assert any(isinstance(a, stub_ros.EmitEvent) for a in handlers[0].kw['on_exit']), \
        '串口桥退出后必须触发 Shutdown（避免串口被占用时干等 30s）'


def test_nodes_duration_is_parameterised(stub_ros):
    """`run_duration_s` / `shutdown_grace_s` 可调，收尾周期随之变化。"""
    entries = _nodes_setup(stub_ros, 'dur', run_duration_s='12.0',
                           shutdown_grace_s='2.0')
    periods = _shutdown_periods(entries, stub_ros)
    assert periods[-1][0] == pytest.approx(14.0)


def test_nodes_includes_static_tf(stub_ros):
    """节点清单必须 include static_tf.launch.py，且路径真实存在。"""
    entries = _nodes_setup(stub_ros, 'incl')
    inc = [e for e in entries if isinstance(e, stub_ros.IncludeLaunchDescription)]
    assert len(inc) == 1
    src = str(inc[0].kw['launch_description_source'])
    assert src.endswith(os.path.join('launch', 'include', 'static_tf.launch.py'))
    assert os.path.isfile(src)


# ---------------------------------------------------------------------------
# 3. competition.launch.py
# ---------------------------------------------------------------------------
def _competition_args(**over: str) -> Dict[str, str]:
    """competition 的默认 launch 参数（全部置为"用参数文件"的空值，便于测回退）。"""
    args = {'params_file': TRACKER_PARAMS, 'virtual_yaml': VIRTUAL_YAML,
            'field_yaml': FIELD_YAML, 'maps_dir': '', 'serial_port': '',
            'base_pose_source': '', 'target_digit': '', 'id_filter_mode': '',
            'v_max': '', 'cmd_frame': '', 'csv_extra_cols': '', 'csv_out_dir': '',
            'csv_run_id': '', 'grid_map_source': '', 'grid_map_topic': '',
            'grid_republish_period_s': '', 'grid_write_preview_png': '',
            'run_duration_s': '', 'shutdown_grace_s': '',
            'enable_gt_monitor': 'false', 'rviz': 'false',
            'startup_step_s': str(STEP), 'publish_mount_tf': 'true'}
    args.update(over)
    return args


def test_competition_declares_args(stub_ros):
    """主入口声明全部常用 launch 参数，且 debug/rviz 默认关闭。"""
    mod = stub_ros.load_launch_module('competition.launch.py', 'lf_comp_1')
    declared = stub_ros.declared_args(mod.generate_launch_description())
    for key in ('params_file', 'virtual_yaml', 'field_yaml', 'maps_dir',
                'serial_port', 'base_pose_source', 'target_digit', 'id_filter_mode',
                'v_max', 'cmd_frame', 'csv_extra_cols', 'csv_out_dir', 'csv_run_id',
                'grid_map_source', 'grid_map_topic', 'grid_republish_period_s',
                'grid_write_preview_png', 'run_duration_s', 'shutdown_grace_s',
                'startup_step_s', 'enable_gt_monitor', 'rviz', 'publish_mount_tf'):
        assert key in declared, '缺少 launch 参数 %s' % key
    assert declared['enable_gt_monitor'] == 'false'
    assert declared['rviz'] == 'false'


def test_competition_defaults_fall_back_to_params_file(stub_ros):
    """未覆盖时：时长/目录从 tracker_params.yaml 读回，路径显式注入。"""
    mod = stub_ros.load_launch_module('competition.launch.py', 'lf_comp_2')
    ld = mod.generate_launch_description()
    stub_ros.args.clear()
    stub_ros.args.update(_competition_args())
    stub_ros.nodes.clear()
    entries = stub_ros.opaque_setup(ld)(None)

    inc = [e for e in entries if isinstance(e, stub_ros.IncludeLaunchDescription)]
    assert len(inc) == 1
    la = dict(inc[0].kw['launch_arguments'])
    assert la['use_sim_time'] == 'false'
    assert la['enable_gt_monitor'] == 'false'
    assert la['rviz'] == 'false'
    # `startup_step_s` 以 LaunchConfiguration 透传（延迟求值），这里取一次实参
    step = la['startup_step_s']
    if not isinstance(step, str):
        step = step.perform(None)
    assert step == str(STEP)

    over = json.loads(la['overrides_json'])
    params = _flat_params(TRACKER_PARAMS)
    assert over['grid.field_yaml'] == FIELD_YAML, '场地路径必须显式注入绝对路径'
    assert over['config.virtual_yaml'] == VIRTUAL_YAML
    assert over['mission.run_duration_s'] == pytest.approx(
        float(params['mission.run_duration_s']))
    assert over['mission.shutdown_grace_s'] == pytest.approx(
        float(params['mission.shutdown_grace_s']))
    assert over['grid.out_dir'] == params['config.maps_dir']
    assert over['csv.out_dir'] == params['config.outputs_dir']
    # 留空的参数不得进入覆盖字典（保持"沿用参数文件"的语义）
    for absent in ('serial.port', 'localization.base_pose_source',
                   'perception.target_digit', 'chassis.v_max', 'chassis.cmd_frame',
                   'csv.csv_extra_cols', 'csv.run_id', 'grid.map_source'):
        assert absent not in over, '%s 不应出现在覆盖字典中' % absent


def test_competition_honours_explicit_overrides(stub_ros):
    """显式给出的 launch 参数必须写进覆盖字典，且类型正确。"""
    mod = stub_ros.load_launch_module('competition.launch.py', 'lf_comp_3')
    ld = mod.generate_launch_description()
    stub_ros.args.clear()
    stub_ros.args.update(_competition_args(
        serial_port='/dev/gimbal', base_pose_source='odom', target_digit='7',
        id_filter_mode='on', v_max='0.8', cmd_frame='base_link',
        csv_extra_cols='true', csv_run_id='run01', grid_map_source='topic',
        grid_map_topic='/ext/map', grid_republish_period_s='1.0',
        grid_write_preview_png='false', maps_dir='maps2', csv_out_dir='out2',
        run_duration_s='12.5', shutdown_grace_s='2.0'))
    stub_ros.nodes.clear()
    entries = stub_ros.opaque_setup(ld)(None)
    la = dict([e for e in entries
               if isinstance(e, stub_ros.IncludeLaunchDescription)][0]
              .kw['launch_arguments'])
    over = json.loads(la['overrides_json'])
    assert over['serial.port'] == '/dev/gimbal'
    assert over['localization.base_pose_source'] == 'odom'
    assert over['perception.target_digit'] == 7
    assert isinstance(over['perception.target_digit'], int)
    assert over['perception.id_filter_mode'] == 'on'
    assert over['chassis.v_max'] == pytest.approx(0.8)
    assert over['chassis.cmd_frame'] == 'base_link'
    assert over['csv.csv_extra_cols'] is True
    assert over['csv.run_id'] == 'run01'
    assert over['grid.map_source'] == 'topic'
    assert over['grid.map_topic'] == '/ext/map'
    assert over['grid.republish_period_s'] == pytest.approx(1.0)
    assert over['grid.write_preview_png'] is False
    assert over['grid.out_dir'] == 'maps2'
    assert over['csv.out_dir'] == 'out2'
    assert over['mission.run_duration_s'] == pytest.approx(12.5)
    assert over['mission.shutdown_grace_s'] == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# 4. sim.launch.py
# ---------------------------------------------------------------------------
def test_sim_defaults(stub_ros):
    """仿真预设：mock + /tmp/ttyVGIMBAL + GT 评估/RViz 默认开；复用 competition。"""
    mod = stub_ros.load_launch_module('sim.launch.py', 'lf_sim_1')
    ld = mod.generate_launch_description()
    declared = stub_ros.declared_args(ld)
    assert declared['serial_port'] == '/tmp/ttyVGIMBAL'
    assert declared['base_pose_source'] == 'mock'
    assert declared['enable_gt_monitor'] == 'true'
    assert declared['rviz'] == 'true'

    inc = [e for e in ld.entities if isinstance(e, stub_ros.IncludeLaunchDescription)]
    assert len(inc) == 1
    assert str(inc[0].kw['launch_description_source']).endswith('competition.launch.py')
    keys = dict(inc[0].kw['launch_arguments'])
    assert {'base_pose_source', 'serial_port', 'enable_gt_monitor', 'rviz'} <= set(keys)


# ---------------------------------------------------------------------------
# 5. mapping.launch.py
# ---------------------------------------------------------------------------
def _mapping_setup(stub_ros, tag: str, **arg_over: str) -> List[Any]:
    mod = stub_ros.load_launch_module('mapping.launch.py', 'lf_map_%s' % tag)
    ld = mod.generate_launch_description()
    args = {'params_file': TRACKER_PARAMS, 'virtual_yaml': '', 'field_yaml': FIELD_YAML,
            'maps_dir': '', 'grid_map_source': 'config', 'grid_map_topic': '',
            'rviz': 'false', 'exit_after_build_s': ''}
    args.update(arg_over)
    stub_ros.args.clear()
    stub_ros.args.update(args)
    stub_ros.nodes.clear()
    return stub_ros.opaque_setup(ld)(None)


def test_mapping_only_grid_mapper(stub_ros):
    """只起 `grid_mapper_node`，显式注入 `grid.field_yaml`，空参数走回退链。"""
    entries = _mapping_setup(stub_ros, 'only')
    nodes = [e for e in entries if isinstance(e, stub_ros.Node)]
    assert len(nodes) == 1
    kw = nodes[0].kw
    assert (kw['package'], kw['executable']) == ('grid_mapper', 'grid_mapper_node')
    over = kw['parameters'][1]
    assert kw['parameters'][0] == TRACKER_PARAMS
    assert over['grid.field_yaml'] == FIELD_YAML
    assert over['grid.map_source'] == 'config'
    assert over['use_sim_time'] is False
    assert over['grid.out_dir'] == _flat_params(TRACKER_PARAMS)['config.maps_dir']
    assert over['config.virtual_yaml'].endswith(os.path.join('config', 'virtual.yaml'))
    assert not _shutdown_periods(entries, stub_ros), '未请求自动退出时不应有 Shutdown'


def test_mapping_exit_after_build_and_maps_dir(stub_ros):
    """`exit_after_build_s` 非空 → 自动退出定时器；`maps_dir` 覆盖输出目录。"""
    entries = _mapping_setup(stub_ros, 'exit', virtual_yaml=VIRTUAL_YAML,
                             maps_dir='maps_out', exit_after_build_s='5')
    periods = _shutdown_periods(entries, stub_ros)
    assert periods and periods[-1][0] == pytest.approx(5.0)
    over = [e for e in entries if isinstance(e, stub_ros.Node)][0].kw['parameters'][1]
    assert over['grid.out_dir'] == 'maps_out'


# ---------------------------------------------------------------------------
# 6. RViz 配置（无需 ROS）
# ---------------------------------------------------------------------------
def test_rviz_config_is_valid_and_covers_required_displays():
    """RViz 配置可解析、Fixed Frame = map、含 TF/栅格/图像/轨迹、话题正确。"""
    path = os.path.join(RVIZ_DIR, 'gimbal_tracker.rviz')
    assert os.path.isfile(path)
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    assert isinstance(cfg, dict)
    vm = cfg['Visualization Manager']
    assert vm['Global Options']['Fixed Frame'] == 'map'
    classes = [d.get('Class', '') for d in vm['Displays']]
    for want in ('rviz_default_plugins/Grid', 'rviz_default_plugins/TF',
                 'rviz_default_plugins/Map', 'rviz_default_plugins/Image',
                 'rviz_default_plugins/Path'):
        assert want in classes, '缺少显示项 %s' % want
    topics = [d.get('Topic', {}).get('Value') for d in vm['Displays']]
    for want in ('/map/grid', '/map/ray_grid', '/virtual_gimbal/image_raw',
                 '/debug/target_path'):
        assert want in topics, '缺少话题 %s' % want


def test_rviz_config_does_not_reference_ground_truth_topic():
    """RViz 配置不得引用真值话题（DESIGN §8.7：除 gimbal_debug 外不得出现该字面量）。"""
    with open(os.path.join(RVIZ_DIR, 'gimbal_tracker.rviz'), 'r',
              encoding='utf-8') as f:
        text = f.read()
    assert ('target' + '_gt') not in text


# ---------------------------------------------------------------------------
# 7. 只能在 Ubuntu + ROS2 上执行的项（本机明确 skip，绝不假装通过）
# ---------------------------------------------------------------------------
def _has_real_ros2() -> bool:
    """本机是否装了真实 ROS 2（而不是测试用的桩）。"""
    try:
        import rclpy                                    # noqa: F401
        from launch import LaunchDescription            # noqa: F401
        return True
    except Exception:
        return False


@pytest.mark.skipif(not _has_real_ros2(),
                    reason='需 Ubuntu + ROS2：本机无 ROS2，无法执行 ros2 launch / ament 解析')
@pytest.mark.parametrize('launch_name', ['competition.launch.py', 'sim.launch.py',
                                         'mapping.launch.py'])
def test_ros2_launch_show_args(launch_name):
    """真机冒烟：`ros2 launch <pkg> <file> --show-args` 必须能解析并列出参数。

    这一项验证本机桩测覆盖不到的部分：ament 把 `launch/`（含 `launch/include/`）安装到位、
    `LaunchDescription` 能被真实 `launch` 解析、`OpaqueFunction` 的延迟求值不被提前触发。
    """
    import shutil
    import subprocess

    ros2 = shutil.which('ros2')
    if ros2 is None:
        pytest.skip('需 Ubuntu + ROS2：找不到 ros2 可执行文件')
    proc = subprocess.run(
        [ros2, 'launch', 'gimbal_tracker_bringup', launch_name, '--show-args'],
        capture_output=True, text=True, timeout=120)
    out = (proc.stdout or '') + (proc.stderr or '')
    if 'PackageNotFoundError' in out or 'not found' in out.lower():
        pytest.skip('需 Ubuntu + ROS2：请先 colcon build --symlink-install 并 '
                    'source install/setup.bash')
    assert proc.returncode == 0, '--show-args 失败:\n%s' % out
    assert 'params_file' in out, '未列出 params_file 参数'


@pytest.mark.skipif(not _has_real_ros2(),
                    reason='需 Ubuntu + ROS2：本机无 ROS2，无法执行 static_transform_publisher')
def test_static_transform_publisher_accepts_flag_style():
    """真机冒烟：Jazzy 的 `static_transform_publisher` 必须接受 `--flag` 风格参数。

    若这一项失败，说明需要退回旧的位置参数风格
    （`x y z yaw pitch roll frame_id child_frame_id`）。
    """
    import shutil
    import subprocess

    exe = shutil.which('static_transform_publisher')
    if exe is None:
        pytest.skip('需 Ubuntu + ROS2：找不到 static_transform_publisher')
    proc = subprocess.run([exe, '--help'], capture_output=True, text=True, timeout=60)
    out = (proc.stdout or '') + (proc.stderr or '')
    for flag in ('--frame-id', '--child-frame-id', '--x', '--roll'):
        assert flag in out, 'static_transform_publisher 不支持 %s（需改参数风格）' % flag
