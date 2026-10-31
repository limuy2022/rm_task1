"""真实ROS launch内省与网络配置回归；不启动硬件驱动或访问网卡。"""
import copy
import importlib.util
import json
from pathlib import Path
import tarfile

import pytest
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters

from team43_bringup.livox_config import load_driver_config, main, mid360_config


ROOT = Path(__file__).resolve().parents[3]


def test_generated_config_matches_bundled_driver_schema():
    # 独立对照实际被编译的驱动模板，防止端口、型号或键名手写错误。
    with tarfile.open(ROOT / 'archives/livox_ros_driver2.tar.gz') as archive:
        official = json.load(archive.extractfile('livox_ros_driver2/config/MID360_config.json'))
    host = official['MID360']['host_net_info']['point_data_ip']
    lidar = official['lidar_configs'][0]['ip']
    assert mid360_config(host, lidar) == official


@pytest.mark.parametrize('host,lidar', [
    ('', '192.168.1.3'), ('127.0.0.1', '192.168.1.3'),
    ('0.0.0.0', '192.168.1.3'), ('::1', '192.168.1.3'),
    ('192.168.1.50', '192.168.1.50'), ('192.168.1.50', '224.0.0.1'),
])
def test_invalid_or_duplicate_addresses_rejected(host, lidar):
    with pytest.raises(ValueError):
        mid360_config(host, lidar)


def test_cli_writes_config_without_overwriting_existing_file(tmp_path):
    path = tmp_path / 'field.json'
    args = ['--model', 'mid360', '--host-ip', '192.168.1.50',
            '--lidar-ip', '192.168.1.3', '--output', str(path)]
    main(args)
    resolved, data = load_driver_config(str(path))
    assert resolved == path
    assert data['MID360']['host_net_info']['imu_data_ip'] == '192.168.1.50'
    original = path.read_bytes()
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2
    assert path.read_bytes() == original


def test_campus_defaults_are_published_addresses(tmp_path):
    path = tmp_path / 'campus.json'
    main(['--output', str(path)])
    _, data = load_driver_config(str(path))
    assert data['lidar_configs'][0]['ip'] == '192.168.1.3'
    assert data['MID360']['host_net_info']['point_data_ip'] == '192.168.1.50'


@pytest.mark.parametrize('change', ['extrinsic', 'model', 'multiple_devices'])
def test_driver_config_cannot_silently_change_geometry(tmp_path, change):
    data = mid360_config('192.168.1.50', '192.168.1.3')
    if change == 'extrinsic':
        data['lidar_configs'][0]['extrinsic_parameter']['yaw'] = 90.
    elif change == 'model':
        data['lidar_summary_info']['lidar_type'] = 1
    else:
        data['lidar_configs'].append(copy.deepcopy(data['lidar_configs'][0]))
    path = tmp_path / 'invalid.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_driver_config(str(path))


def test_launch_starts_only_pointcloud2_driver_with_competition_topics(tmp_path):
    spec = importlib.util.spec_from_file_location(
        'livox_launch', ROOT / 'src/team43_bringup/launch/livox.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = tmp_path / 'lidar.json'
    config.write_text(json.dumps(mid360_config('192.168.1.50', '192.168.1.3')))
    context = LaunchContext()
    context.launch_configurations.update(config_file=str(config))
    nodes = module._setup(context)
    assert len(nodes) == 1
    node = nodes[0]
    assert node.node_package == 'livox_ros_driver2'
    assert node.node_executable == 'livox_ros_driver2_node'
    parameters = evaluate_parameters(context, node._Node__parameters)[0]
    assert parameters['user_config_path'] == str(config)
    assert parameters['xfer_format'] == 0
    assert parameters['multi_topic'] == 0
    assert parameters['data_src'] == 0
    assert parameters['use_sim_time'] is False
    assert parameters['frame_id'] == 'livox_frame'
    remaps = [(perform_substitutions(context, a), perform_substitutions(context, b))
              for a, b in node._Node__remappings]
    assert remaps == [('livox/lidar', '/red_standard_robot1/livox/lidar'),
                      ('livox/imu', '/red_standard_robot1/livox/imu')]
