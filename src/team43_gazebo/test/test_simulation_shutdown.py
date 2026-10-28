"""仿真收尾必须保留真实退出码，并在停止服务不可用时回收进程。"""
import signal
import subprocess
from types import SimpleNamespace

import pytest
from team43_gazebo import process_shutdown as shutdown


class Process:
    pid = 12345

    def __init__(self, returncode=None, ignores_interrupt=False):
        self.returncode = returncode
        self.ignores_interrupt = ignores_interrupt

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        if self.returncode is None:
            raise subprocess.TimeoutExpired('仿真进程', timeout)
        return self.returncode


def capture_signals(monkeypatch, process):
    signals = []

    def killpg(pid, sig):
        assert pid == process.pid
        signals.append(sig)
        if sig == signal.SIGKILL:
            process.returncode = -9
        elif not process.ignores_interrupt:
            process.returncode = 0

    monkeypatch.setattr(shutdown.os, 'killpg', killpg)
    return signals


def test_gazebo_service_stops_before_interrupt(monkeypatch, tmp_path):
    process = Process()
    signals = capture_signals(monkeypatch, process)

    def service(command, **kwargs):
        assert command[command.index('-s')+1] == '/server_control'
        assert command[-1] == 'stop: true'
        process.returncode = 0
        return SimpleNamespace(returncode=0, stdout='data: true\n', stderr='')

    monkeypatch.setattr(shutdown.subprocess, 'run', service)
    assert shutdown.stop_processes([('gazebo', process)], tmp_path) == {'gazebo': 0}
    assert signals == []
    assert 'data: true' in (tmp_path/'gazebo_stop.log').read_text()


@pytest.mark.parametrize('failure', ['rejected', 'timeout', 'unavailable'])
def test_service_failure_still_reclaims_process(monkeypatch, tmp_path, failure):
    process = Process()
    signals = capture_signals(monkeypatch, process)

    def service(*args, **kwargs):
        if failure == 'timeout':
            raise subprocess.TimeoutExpired('ign', 5.)
        if failure == 'unavailable':
            raise FileNotFoundError('ign')
        return SimpleNamespace(returncode=0, stdout='data: false\n', stderr='')

    monkeypatch.setattr(shutdown.subprocess, 'run', service)
    assert shutdown.stop_processes([('gazebo', process)], tmp_path) == {'gazebo': 0}
    assert signals == [signal.SIGINT]


def test_acknowledged_but_stuck_server_still_fails(monkeypatch, tmp_path):
    process = Process(ignores_interrupt=True)
    signals = capture_signals(monkeypatch, process)
    monkeypatch.setattr(shutdown.subprocess, 'run', lambda *a, **k:
                        SimpleNamespace(returncode=0, stdout='data: true\n', stderr=''))
    assert shutdown.stop_processes([('gazebo', process)], tmp_path) == {'gazebo': -9}
    assert signals == [signal.SIGINT, signal.SIGKILL]


def test_already_failed_process_keeps_failure(monkeypatch, tmp_path):
    process = Process(returncode=1)
    signals = capture_signals(monkeypatch, process)

    def unexpected_service(*args, **kwargs):
        pytest.fail('已经退出的进程不应再调用停止服务')

    monkeypatch.setattr(shutdown.subprocess, 'run', unexpected_service)
    assert shutdown.stop_processes([('gazebo', process)], tmp_path) == {'gazebo': 1}
    assert signals == []
