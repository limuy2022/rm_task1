"""验证拆分配置已安装、ROS 能解析，且路径解析不静默回退。"""
from pathlib import Path

import pytest
import yaml
from rclpy.context import Context
from rclpy.node import Node
import rclpy

from team43_bringup.parameter_files import package_config

SRC = Path(__file__).resolve().parents[2]
DEFAULTS = sorted(SRC.glob('team43_*/config/tracker_defaults.yaml'))


def ros_parameters(*paths):
    context = Context()
    rclpy.init(context=context)
    try:
        node = Node('config_probe', namespace='team43', context=context,
                    cli_args=['--ros-args'] + [arg for path in paths
                                              for arg in ('--params-file', str(path))],
                    automatically_declare_parameters_from_overrides=True)
        try:
            return node.get_parameters_by_prefix('')
        finally:
            node.destroy_node()
    finally:
        context.shutdown()


@pytest.mark.parametrize('source', DEFAULTS, ids=lambda p: p.parents[1].name)
def test_installed_defaults_match_source_and_ros_can_parse(source):
    installed = Path(package_config(source.parents[1].name))
    assert installed.read_bytes() == source.read_bytes()
    # 使用 ROS 自己的参数文件解析器，能抓到 YAML 类型与 ROS 参数格式不兼容。
    loaded = ros_parameters(installed)
    expected = yaml.safe_load(source.read_text(encoding='utf-8'))['/**']['ros__parameters']
    for key, value in expected.items():
        actual = loaded[key].value
        assert (list(actual) if isinstance(value, list) else actual) == value


@pytest.mark.parametrize('package, filename', [
    ('team43_control', 'visual_tracking.yaml'),
    ('team43_grid_mapper', 'cloud_mapping.yaml'),
])
def test_moved_node_config_installed_and_ros_can_parse(package, filename):
    installed = Path(package_config(package, filename))
    assert installed.read_bytes() == (SRC/package/'config'/filename).read_bytes()
    assert ros_parameters(installed)


def test_missing_configuration_is_fatal():
    with pytest.raises(FileNotFoundError, match='配置未安装'):
        package_config('team43_control', 'missing.yaml')


def test_scene_and_explicit_overrides_keep_priority(tmp_path):
    defaults = package_config('team43_control')
    shared = package_config('team43_bringup', 'tracker_params.yaml')
    bench = package_config('team43_bringup', 'vehicle_bench.yaml')
    assert ros_parameters(defaults, shared)['control.kp_yaw'].value == 6.0
    params = ros_parameters(defaults, shared, bench)
    assert params['control.kp_yaw'].value == 0.5
    assert params['control.bearing.enabled'].value is False
    override = tmp_path/'override.yaml'
    override.write_text('/**:\n  ros__parameters:\n    control.kp_yaw: 0.75\n', encoding='utf-8')
    params = ros_parameters(defaults, shared, bench, override)
    assert params['control.kp_yaw'].value == 0.75
    assert params['control.bearing.enabled'].value is False
