"""发现订阅后才放行传感器源，超时或接收进程退出必须失败。"""
from types import SimpleNamespace

import pytest
from team43_gazebo import simulation_startup as startup


def endpoint(name='team43_small_point_lio', namespace='/sim/gazebo/lio'):
    return SimpleNamespace(node_name=name, node_namespace=namespace)


def test_waits_for_both_backend_readers():
    calls = []

    def readers(topic):
        return [endpoint()] if topic == 'lidar' or calls else []

    node = SimpleNamespace(get_subscriptions_info_by_topic=readers)
    startup.wait_for_sensor_readers(node, ('lidar', 'imu'), lambda: calls.append('pump'))
    assert calls == ['pump']


@pytest.mark.parametrize('reader', [endpoint(name='observer'), endpoint(namespace='/other')])
def test_unrelated_reader_does_not_release_source(monkeypatch, reader):
    times = iter((0., 0., 16.))
    monkeypatch.setattr(startup.time, 'monotonic', lambda: next(times))
    node = SimpleNamespace(get_subscriptions_info_by_topic=lambda topic: [reader])
    with pytest.raises(RuntimeError, match='lidar, imu'):
        startup.wait_for_sensor_readers(node, ('lidar', 'imu'), lambda: None)


def test_receiver_exit_during_discovery_is_not_hidden():
    node = SimpleNamespace(get_subscriptions_info_by_topic=lambda topic: [])

    def pump():
        raise RuntimeError('point_lio 提前退出')

    with pytest.raises(RuntimeError, match='point_lio 提前退出'):
        startup.wait_for_sensor_readers(node, ('imu',), pump)
