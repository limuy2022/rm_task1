"""串口独占读写封装（docs/hardware_spec.md §1）。

要点：

* ``serial.Serial(port, 921600, exclusive=True)``：独占打开；**被拒即报错退出，不静默重试**。
* 8N1、无流控、raw 模式。
* ``read_bytes()`` 非阻塞（timeout=read_timeout_s）读一块，供上层定时器轮询。
* ``write()`` 失败抛 :class:`SerialPortError`，由链路状态机转成故障。

依赖 pyserial；缺少依赖直接报 ImportError。构造对象不会打开串口。
"""

from __future__ import annotations

from typing import Optional

import serial

__all__ = ['SerialPortError', 'SerialPort']


class SerialPortError(RuntimeError):
    """串口打开/读写失败。按 DESIGN §8.4：打开失败应 FATAL 退出。"""


class SerialPort:
    """独占串口收发器（带字节/帧统计）。"""

    def __init__(self,
                 port: str,
                 baud: int = 921600,
                 exclusive: bool = True,
                 read_timeout_s: float = 0.01,
                 write_timeout_s: float = 0.05,
                 read_chunk: int = 256) -> None:
        """构造串口对象（**不**立即打开，需调用 :meth:`open`）。

        Args:
            port: 设备路径，如 ``/tmp/ttyVGIMBAL`` / ``/dev/gimbal``。
            baud: 波特率，默认 921600。
            exclusive: 独占打开。
            read_timeout_s: 读超时（秒），None 表示阻塞。
            write_timeout_s: 写超时（秒）。
            read_chunk: 单次 read 的最大字节数。
        """
        self._port: str = str(port)
        self._baud: int = int(baud)
        self._exclusive: bool = bool(exclusive)
        self._read_timeout_s: Optional[float] = float(read_timeout_s)
        self._write_timeout_s: Optional[float] = float(write_timeout_s)
        self._read_chunk: int = int(read_chunk)
        self._ser = None
        self.bytes_rx: int = 0
        self.bytes_tx: int = 0
        self.frames_tx: int = 0
        self.write_errors: int = 0
        self.read_errors: int = 0

    @property
    def port(self) -> str:
        """设备路径。"""
        return self._port

    @property
    def is_open(self) -> bool:
        """串口是否已打开。"""
        return self._ser is not None and self._ser.is_open

    def open(self) -> None:
        """独占打开串口。

        Raises:
            SerialPortError: 打开失败或被其它进程占用。
        """
        if self.is_open:
            return
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=self._read_timeout_s,
                write_timeout=self._write_timeout_s,
                exclusive=self._exclusive,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )
        except OSError as exc:  # 设备可能断连或被占用；向上抛链路故障，不掩盖程序异常。
            raise SerialPortError(
                f'打开串口失败 port={self._port} baud={self._baud} '
                f'exclusive={self._exclusive}：{exc!r}。'
                '常见原因：被其它进程占用（课1-3 的 robo_bridge_edu 没关）、'
                '权限不足（加 dialout 组）或路径不存在。按硬约束不静默重试。'
            ) from exc
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except OSError as exc:
            # Opening is incomplete if the device disconnects while buffers reset.
            self.close()
            raise SerialPortError(f'串口初始化失败 port={self._port}: {exc!r}') from exc

    def read_bytes(self) -> bytes:
        """读一块字节（非阻塞，最多 ``read_chunk`` 字节）。

        Returns:
            bytes: 读到的字节；无数据返回 ``b''``；串口异常抛出 SerialPortError 并计数。
        """
        if not self.is_open:
            return b''
        try:
            n = self._ser.in_waiting or self._read_chunk
            data = self._ser.read(min(int(n), self._read_chunk) or self._read_chunk)
        except OSError as exc:
            # A disconnected TTY is a transport fault, not an empty normal read.
            self.read_errors += 1
            raise SerialPortError(f'串口读取失败 port={self._port}: {exc!r}') from exc
        if data:
            self.bytes_rx += len(data)
        return bytes(data)

    def write(self, data: bytes) -> int:
        """写一帧。

        Args:
            data: 待写字节。

        Returns:
            int: 写入字节数。

        Raises:
            SerialPortError: 串口未打开或写入失败（上层应据此进FAULT_SILENT）。
        """
        if not self.is_open:
            raise SerialPortError(f'串口未打开，无法写入 port={self._port}')
        try:
            n = self._ser.write(bytes(data))
            self.bytes_tx += int(n)
            self.frames_tx += 1
            return int(n)
        except OSError as exc:  # 设备可能断连或被占用；向上抛链路故障，不掩盖程序异常。
            self.write_errors += 1
            raise SerialPortError(f'串口写入失败 port={self._port}：{exc!r}') from exc

    def flush(self) -> None:
        """刷新输出缓冲。"""
        if not self.is_open:
            return
        try:
            self._ser.flush()
        except OSError as exc:  # 设备可能断连或被占用；向上抛链路故障，不掩盖程序异常。
            self.write_errors += 1
            raise SerialPortError(f'串口flush失败 port={self._port}: {exc!r}') from exc

    def close(self) -> None:
        """关闭串口（幂等）。"""
        if self._ser is None:
            return
        try:
            if self._ser.is_open:
                self._ser.close()
        finally:
            self._ser = None

    def stats(self) -> dict:
        """返回收发统计。"""
        return {
            'bytes_rx': int(self.bytes_rx),
            'bytes_tx': int(self.bytes_tx),
            'frames_tx': int(self.frames_tx),
            'write_errors': int(self.write_errors),
            'read_errors': int(self.read_errors),
            'is_open': bool(self.is_open),
        }
