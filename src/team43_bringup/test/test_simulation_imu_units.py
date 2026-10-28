"""静止重力和相同物理量程在g/SI输入下必须得到相同饱和判定。"""
import importlib.util
from pathlib import Path

import pytest


path = Path(__file__).resolve().parents[3]/'tools/gazebo/sensor_config.py'
spec = importlib.util.spec_from_file_location('simulation_sensor_config', path)
sensor_config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor_config)


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
