"""静止重力和相同物理量程在g/SI输入下必须得到相同饱和判定。"""
import pytest
from team43_gazebo import sensor_config


@pytest.mark.parametrize('normal,limit', [(1., 3.), (9.81, 29.43)])
def test_equivalent_units_preserve_acceleration_saturation(normal, limit):
    physical_accelerations_g = [0., 1., 2., 3.1]
    expected = [abs(a*normal) >= .99*limit for a in physical_accelerations_g]
    mapping = {'acc_norm': normal, 'satu_acc': limit}
    sensor_config.use_si_acceleration(mapping)
    measured = [abs(a*9.81) >= .99*mapping['satu_acc'] for a in physical_accelerations_g]
    assert measured == expected
    assert measured[1] is False  # 静止重力不能被屏蔽
    assert mapping['acc_norm'] == 9.81


def test_inconsistent_si_saturation_is_rejected_before_launch():
    with pytest.raises(ValueError, match='静止重力'):
        sensor_config.use_si_acceleration({'acc_norm': 9.81, 'satu_acc': 3.})


@pytest.mark.parametrize('normal', [0., -1., float('nan'), float('inf')])
def test_invalid_acceleration_unit_scale_is_rejected(normal):
    with pytest.raises(ValueError):
        sensor_config.use_si_acceleration({'acc_norm': normal, 'satu_acc': 3.})


def test_simulation_does_not_silently_retune_estimator():
    from pathlib import Path
    import copy
    import yaml
    source = yaml.safe_load((Path(__file__).resolve().parents[2]/'team43_lio_backend/config/small_point_lio_mid360.yaml').read_text())['/**']['ros__parameters']
    before = copy.deepcopy(source)
    result = sensor_config.simulation_lio_parameters(source,'/sim/lidar','/sim/imu','/tmp/audit')
    for key in ('map_resolution','space_downsample_leaf_size','space_downsample','plane_threshold',
                'match_sqaured','laser_point_cov','imu_meas_acc_cov','velocity_cov','acceleration_cov'):
        assert result[key] == source[key]
    assert source == before
    assert result['map_resolution'] == result['space_downsample_leaf_size'] == .2


def test_custom_estimator_voxel_sizes_are_preserved():
    source = {'acc_norm':1.,'satu_acc':3.,'map_resolution':.35,'space_downsample_leaf_size':.12}
    result = sensor_config.simulation_lio_parameters(source,'/sim/lidar','/sim/imu','/tmp/audit')
    assert result['map_resolution'] == .35
    assert result['space_downsample_leaf_size'] == .12
