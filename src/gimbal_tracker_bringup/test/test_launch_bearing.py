"""`launch/bearing_bench / bearing_hardware / cloud_mapping` 的功能性回归。

手法与 `test_launch_sim.py` 相同（真 ROS 2 Humble launch 内省，不起进程），
辅助函数直接从该文件按路径加载复用（升级 ROS 发行版时只需复核一处）。

覆盖：
  * bearing_bench：参数校验（epoch 必填 / target 范围 / camera 存在）、
    节点组合、内参与 epoch/target 注入、四道开关默认全关、
    hardware_interface 的 5 项透传（含 yaw/pitch_sign）
  * bearing_hardware：未实测模板被 load_vehicle_config 拒绝、
    实测确认后的合法整车配置通过、control.enabled=true 拒绝、
    节点组合与 hardware_interface 的 3 项透传
  * cloud_mapping：team47 命名空间、extrinsic_T → sensor_origin_in_body、
    floor/output 注入、extrinsic_est_en=true 拒绝

⚠ 需要能 import launch / launch_ros 的环境（如验证容器），否则整体跳过。
"""

from __future__ import annotations

import importlib.util
import os

import pytest
import yaml

pytest.importorskip('launch')
pytest.importorskip('launch_ros')

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.abspath(os.path.join(TEST_DIR, '..'))
CONFIG_DIR = os.path.join(PKG_DIR, 'config')
LAUNCH_DIR = os.path.join(PKG_DIR, 'launch')

CAMERA_TEAM47 = os.path.join(CONFIG_DIR, 'camera_team47.yaml')
VEHICLE_TEMPLATE = os.path.join(CONFIG_DIR, 'vehicle_template.yaml')
TRACKER_PARAMS = os.path.join(CONFIG_DIR, 'tracker_params.yaml')
LIO_PARAMS = os.path.join(CONFIG_DIR, 'point_lio_mid360.yaml')

# 复用 test_launch_sim 封装好的 Humble 私有属性辅助函数（唯一来源）。
_SPEC = importlib.util.spec_from_file_location(
    'test_launch_sim_helpers', os.path.join(TEST_DIR, 'test_launch_sim.py'))
SIM = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(SIM)

_BENCH_EXES = ['base_pose_provider_node', 'aruco_detector_node', 'marker_pnp_node',
               'gimbal_kinematics_node', 'gimbal_bearing_node']
# 实机组合没有 base_pose_provider（真实底盘位姿来自 Point-LIO odom）
_HW_EXES = ['aruco_detector_node', 'marker_pnp_node',
            'gimbal_kinematics_node', 'gimbal_bearing_node']


def _namespace_of(node) -> str:
    """Humble 私有属性：`_Node__node_namespace`。"""
    return node.__dict__.get('_Node__node_namespace') or ''


# ---------------------------------------------------------------------------
# 1. bearing_bench.launch.py
# ---------------------------------------------------------------------------
def _bench_args(**over):
    args = {'epoch': '7', 'target_id': '3', 'port': '/dev/ttyUSB0',
            'camera_config': CAMERA_TEAM47, 'yaw_sign': '1', 'pitch_sign': '1',
            'allow_command_subscription': 'false', 'control_enabled': 'false',
            'bearing_enabled': 'false', 'geometry_verified': 'false'}
    args.update(over)
    return args


def _bench_setup(tag, **arg_over):
    mod = SIM._load_launch_module('bearing_bench.launch.py', 'lf_bench_%s' % tag)
    ld = mod.generate_launch_description()
    ctx = SIM._make_context(_bench_args(**arg_over))
    return SIM._opaque_setup(ld)(ctx), ctx


def test_bench_declares_args_and_defaults():
    mod = SIM._load_launch_module('bearing_bench.launch.py', 'lf_bench_decl')
    declared = SIM._declared_args(mod.generate_launch_description())
    for key in ('epoch', 'target_id', 'port', 'camera_config', 'yaw_sign',
                'pitch_sign', 'allow_command_subscription', 'control_enabled',
                'bearing_enabled', 'geometry_verified'):
        assert key in declared, '缺少 launch 参数 %s' % key
    assert declared['epoch'] == '-1', 'epoch 无默认值（必填）'
    assert declared['target_id'] == '0'
    assert declared['port'] == '/dev/ttyUSB0'
    assert declared['camera_config'].endswith(os.path.join('config', 'camera_team47.yaml'))
    for switch in ('allow_command_subscription', 'control_enabled',
                   'bearing_enabled', 'geometry_verified'):
        assert declared[switch] == 'false', '%s 必须默认全关' % switch


def test_bench_node_set_and_injection():
    entries, ctx = _bench_setup('main')
    inc = SIM._include_views(entries)
    assert len(inc) == 1
    assert inc[0]['source'].endswith('hardware_interface.launch.py')
    assert os.path.isfile(inc[0]['source'])
    assert set(inc[0]['launch_arguments']) == {
        'port', 'allow_command_subscription', 'control_enabled',
        'yaw_sign', 'pitch_sign'}, '桥的 5 项透传（含轴向符号）不得缺'

    got = SIM._collect_nodes(entries, ctx)
    assert [exe for _t, _p, exe, _kw in got] == _BENCH_EXES
    by_exe = {exe: kw for _t, _p, exe, kw in got}

    # 感知节点：tracker + bench + epoch/target 覆盖 + 相机内参覆盖
    for exe in ('aruco_detector_node', 'marker_pnp_node'):
        params = by_exe[exe]['parameters']
        assert params[0].endswith(os.path.join('config', 'tracker_params.yaml'))
        assert params[1].endswith(os.path.join('config', 'vehicle_bench.yaml'))
        assert params[2] == {'kinematics.calibrated_epoch': 7,
                             'perception.target_digit': 3}
        assert params[3] == {'config.virtual_yaml': CAMERA_TEAM47}, \
            'camera_config 必须真正注入感知节点'

    # bearing 节点：开关覆盖键存在（值由 ParameterValue 延迟求值）
    bearing_params = by_exe['gimbal_bearing_node']['parameters']
    assert set(bearing_params[3]) == {'control.bearing.enabled',
                                      'control.bearing.geometry_verified'}
    # 其余节点只吃 tracker+bench+epoch/target
    assert by_exe['gimbal_kinematics_node']['parameters'][:3] == \
        by_exe['aruco_detector_node']['parameters'][:3]


def test_bench_rejects_bad_epoch_target_camera():
    for over, fragment in (({'epoch': '-1'}, 'epoch'),
                           ({'target_id': '50'}, 'target_id'),
                           ({'target_id': '-2'}, 'target_id'),
                           ({'camera_config': '/nonexistent.yaml'}, 'camera_config')):
        with pytest.raises(ValueError):
            _bench_setup('bad_%s' % fragment, **over)


# ---------------------------------------------------------------------------
# 2. bearing_hardware.launch.py
# ---------------------------------------------------------------------------
_CONFIRM_FLIPS = (
    ('localization.sensor_extrinsics_confirmed: false',
     'localization.sensor_extrinsics_confirmed: true'),
    ('localization.parent_transform_confirmed: false',
     'localization.parent_transform_confirmed: true'),
    ('kinematics.calibration_confirmed: false',
     'kinematics.calibration_confirmed: true'),
    ('kinematics.joint_zero_known: false',
     'kinematics.joint_zero_known: true'),
    ('kinematics.mounted_on_base_confirmed: false',
     'kinematics.mounted_on_base_confirmed: true'),
    ('estimation.geometry_confirmed: false',
     'estimation.geometry_confirmed: true'),
)


def _measured_vehicle(tmp_path, *, enable_world_control=False):
    """从模板生成实测确认的整车配置（六开关翻 true + epoch 对齐）。"""
    text = open(VEHICLE_TEMPLATE, encoding='utf-8').read()
    for old, new in _CONFIRM_FLIPS:
        assert old in text, '模板缺少 %s' % old.split(':')[0]
        text = text.replace(old, new)
    text = text.replace('kinematics.calibrated_epoch: -1',
                        'kinematics.calibrated_epoch: 5')
    if enable_world_control:
        text = text.replace('control.enabled: false', 'control.enabled: true')
    path = tmp_path / 'vehicle_measured.yaml'
    path.write_text(text, encoding='utf-8')
    return str(path)


def _hw_args(vehicle, **over):
    args = {'vehicle_config': vehicle, 'camera_config': CAMERA_TEAM47,
            'target_id': '3', 'port': '/dev/ttyUSB0',
            'allow_command_subscription': 'false', 'control_enabled': 'false',
            'bearing_enabled': 'false', 'geometry_verified': 'false'}
    args.update(over)
    return args


def _hw_setup(tag, vehicle, **arg_over):
    mod = SIM._load_launch_module('bearing_hardware.launch.py', 'lf_hw_%s' % tag)
    ld = mod.generate_launch_description()
    ctx = SIM._make_context(_hw_args(vehicle, **arg_over))
    return SIM._opaque_setup(ld)(ctx), ctx


def test_hw_rejects_untested_template():
    mod = SIM._load_launch_module('bearing_hardware.launch.py', 'lf_hw_tmpl')
    ld = mod.generate_launch_description()
    ctx = SIM._make_context(_hw_args(VEHICLE_TEMPLATE))
    with pytest.raises(ValueError, match='未确认'):
        SIM._opaque_setup(ld)(ctx)


def test_hw_rejects_world_control_enabled(tmp_path):
    cfg = _measured_vehicle(tmp_path, enable_world_control=True)
    mod = SIM._load_launch_module('bearing_hardware.launch.py', 'lf_hw_wc')
    ld = mod.generate_launch_description()
    ctx = SIM._make_context(_hw_args(cfg))
    # load_vehicle_config 先拒绝（processing 入口不接受运动使能）
    with pytest.raises(ValueError, match='使能'):
        SIM._opaque_setup(ld)(ctx)


def test_hw_node_set_with_measured_config(tmp_path):
    cfg = _measured_vehicle(tmp_path)
    entries, ctx = _hw_setup('main', cfg)
    inc = SIM._include_views(entries)
    assert len(inc) == 1
    assert inc[0]['source'].endswith('hardware_interface.launch.py')
    assert set(inc[0]['launch_arguments']) == {
        'port', 'allow_command_subscription', 'control_enabled'}

    got = SIM._collect_nodes(entries, ctx)
    assert [exe for _t, _p, exe, _kw in got] == _HW_EXES
    by_exe = {exe: kw for _t, _p, exe, kw in got}
    for exe in ('aruco_detector_node', 'marker_pnp_node'):
        params = by_exe[exe]['parameters']
        assert params[0].endswith(os.path.join('config', 'tracker_params.yaml'))
        assert params[1] == cfg and params[1].endswith('vehicle_measured.yaml'), \
            '实测整车配置必须注入'
        assert params[2] == {'config.virtual_yaml': CAMERA_TEAM47,
                             'perception.target_digit': 3}


def test_hw_rejects_bad_target(tmp_path):
    cfg = _measured_vehicle(tmp_path)
    with pytest.raises(ValueError):
        _hw_setup('badt', cfg, target_id='99')


# ---------------------------------------------------------------------------
# 3. cloud_mapping.launch.py
# ---------------------------------------------------------------------------
def _cm_args(**over):
    args = {'params_file': os.path.join(CONFIG_DIR, 'cloud_mapping.yaml'),
            'lio_params_file': LIO_PARAMS, 'floor_z': '0.0',
            'floor_confirmed': 'false',
            'output_dir': '/tmp/cm_out'}
    args.update(over)
    return args


def _cm_setup(tag, **arg_over):
    mod = SIM._load_launch_module('cloud_mapping.launch.py', 'lf_cm_%s' % tag)
    ld = mod.generate_launch_description()
    ctx = SIM._make_context(_cm_args(**arg_over))
    return SIM._opaque_setup(ld)(ctx), ctx


def test_cm_single_node_namespace_and_injection():
    entries, ctx = _cm_setup('main')
    nodes = [e for e in entries if isinstance(e, SIM.Node)]
    assert len(nodes) == 1
    node = nodes[0]
    view = SIM._node_view(ctx, node)
    assert (view['package'], view['executable'], view['name']) == \
        ('grid_mapper', 'cloud_mapping_node', 'cloud_mapping_node')
    assert _namespace_of(node) == 'team47'

    with open(LIO_PARAMS, encoding='utf-8') as f:
        lio = yaml.safe_load(f)['/**']['ros__parameters']
    offset = [float(x) for x in lio['mapping']['extrinsic_T']]

    params = view['parameters']
    assert len(params) == 2
    assert SIM._perform(ctx, params[0]) == os.path.join(CONFIG_DIR, 'cloud_mapping.yaml')
    over = params[1]
    assert over['sensor_origin_in_body'] == pytest.approx(offset), \
        '光心偏移必须来自 Point-LIO extrinsic_T，不得手抄'
    assert over['origin_model_confirmed'] is True
    assert over['floor_z'] == pytest.approx(0.0)
    assert over['floor_confirmed'] is False
    assert over['output_dir'] == '/tmp/cm_out'


def test_cm_floor_override_reaches_parameters():
    entries, ctx = _cm_setup('floor', floor_z='0.12', floor_confirmed='true')
    view = SIM._node_view(ctx, [e for e in entries
                                if isinstance(e, SIM.Node)][0])
    over = view['parameters'][1]
    assert over['floor_z'] == pytest.approx(0.12)
    assert over['floor_confirmed'] is True


def test_cm_rejects_estimated_extrinsic(tmp_path):
    bad = tmp_path / 'lio_est.yaml'
    bad.write_text(
        '/**:\n  ros__parameters:\n    mapping:\n'
        '      extrinsic_est_en: true\n      extrinsic_T: [0.0, 0.0, 0.05]\n',
        encoding='utf-8')
    with pytest.raises(ValueError):
        _cm_setup('est', lio_params_file=str(bad))
