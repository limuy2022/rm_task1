"""用真实 PTY 验证串口轮询不会等待下一帧，避免饿死同线程发送定时器。"""

import os
import time

import pytest

from team43_gimbal_io.serial_port import SerialPort, SerialPortError


def test_poll_empty_port_does_not_wait_for_read_timeout():
    master, slave = os.openpty()
    port = SerialPort(os.ttyname(slave), read_timeout_s=0.5, read_chunk=4)
    try:
        port.open()
        started = time.monotonic()
        assert port.read_bytes() == b''
        assert time.monotonic() - started < 0.1, '空缓冲轮询阻塞了发送回调'
        os.write(master, b'abcdef')
        deadline = time.monotonic() + 1.0
        received = bytearray()
        while len(received) < 6 and time.monotonic() < deadline:
            chunk = port.read_bytes()
            assert len(chunk) <= 4
            received.extend(chunk)
        assert received == b'abcdef'
        assert port.bytes_rx == 6
    finally:
        port.close()
        os.close(master)
        os.close(slave)


@pytest.mark.parametrize('empty', ['', '  '])
def test_unconfigured_port_fails_fast_with_actionable_message(empty):
    # 配置默认留空：必须给出"如何确认设备"的指引，而不是把占位串当路径传给 pyserial。
    port = SerialPort(empty)
    with pytest.raises(SerialPortError, match='ls /dev/ttyUSB'):
        port.open()
    assert not port.is_open
