"""停止信号与订阅取消息竞争：正常关闭可收尾，运行中异常不可吞掉。"""
import pytest
from rclpy.executors import ExternalShutdownException
from gimbal_common import node_utils


@pytest.mark.parametrize('error', [RuntimeError('take_message'), ExternalShutdownException()])
@pytest.mark.parametrize('context_active', [True, False])
def test_spin_shutdown_race_preserves_runtime_failures(monkeypatch, error, context_active):
    finished = []

    def spin(_node):
        raise error

    monkeypatch.setattr(node_utils.rclpy, 'spin', spin)
    monkeypatch.setattr(node_utils.rclpy, 'ok', lambda: context_active)
    if context_active:
        with pytest.raises(type(error)):
            node_utils.spin_with_finally(object(), lambda: finished.append(True))
    else:
        node_utils.spin_with_finally(object(), lambda: finished.append(True))
    assert finished == [True]
