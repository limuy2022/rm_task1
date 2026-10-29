"""虚拟云台串口协议编解码（DESIGN §6.1 / §6.2，硬参数见 docs/hardware_spec.md）。

帧格式::

    偏移  0     1     2     3     4 .. 4+len-1   4+len   5+len
          AA    addr  func  len   data           sum     sum_sum

* ``head`` 固定 ``0xAA``
* ``addr``：``0xEE`` = PC→下位机；``0xFF`` = 下位机→PC
* ``func``：``0x30`` 角速度指令（PC→下位机，data 8 字节）；``0x01`` IMU（下位机→PC，data 26 字节）
* 帧总长 = len + 6 ≤ 60

⚠ ``calc_verify`` 中 ``sum_sum`` 累加的是**更新后的 sum**，不是原始字节（最易写错处）。
⚠ ``0.00718260498046875`` / ``0.00106526443603169529841533860381`` / ``32760.0``
   只允许在本文件（与 ``virtual.yaml``）出现。

本模块只依赖 numpy / struct / math，不依赖 ROS，可在无 ROS 环境下单测。
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


# --------------------------------------------------------------------------------------
# 常量
# --------------------------------------------------------------------------------------
HEAD: int = 0xAA
ADDR_PC2MCU: int = 0xEE
ADDR_MCU2PC: int = 0xFF
FUNC_OMEGA_CMD: int = 0x30   # data 8 字节（两个 float32）
FUNC_IMU: int = 0x01         # data 26 字节
LEN_OF_FUNC: Dict[int, int] = {FUNC_OMEGA_CMD: 8, FUNC_IMU: 26}
MAX_DATA_LEN: int = 54       # 帧总长 = len + 6 ≤ 60
MAX_FRAME_LEN: int = MAX_DATA_LEN + 6
BUF_LIMIT: int = 512         # 缓冲区膨胀保护：超过就整段丢弃

ACC_SCALE: float = 0.00718260498046875
GYRO_SCALE: float = 0.00106526443603169529841533860381
Q_SCALE: float = 32760.0     # 必须浮点除法

IMU_DATA_LEN: int = 26
IMU_FRAME_LEN: int = IMU_DATA_LEN + 6
QUAT_NORM_MIN: float = 0.5   # 四元数范数低于此值视为损坏帧，丢弃


# --------------------------------------------------------------------------------------
# 校验与组帧
# --------------------------------------------------------------------------------------
def calc_verify(buf: bytes, length: int) -> Tuple[int, int]:
    """计算 (sum, sum_sum)。

    ⚠ ``sum_sum`` 累加的是**更新后的 sum**，不是原始字节。

    Args:
        buf: 从 ``0xAA`` 开始的帧缓冲（至少 ``length + 4`` 字节）。
        length: data 区字节数。

    Returns:
        tuple: ``(sum, sum_sum)``，均为 0..255。
    """
    s = 0
    ss = 0
    for i in range(length + 4):     # head..data末，共 len+4 字节
        s = (s + buf[i]) & 0xFF
        ss = (ss + s) & 0xFF        # 累加「更新后的 s」
    return s, ss


def pack_frame(addr: int, func: int, data: bytes) -> bytes:
    """组帧：``AA addr func len data... sum sum_sum``。

    Args:
        addr: 地址字节。
        func: 功能码。
        data: 数据区字节。

    Returns:
        bytes: 完整帧。
    """
    payload = bytes(data)
    buf = bytes([HEAD, addr & 0xFF, func & 0xFF, len(payload) & 0xFF]) + payload
    s, ss = calc_verify(buf, len(payload))
    return buf + bytes([s, ss])


def frame_ok(buf: bytes, length: int) -> bool:
    """校验一帧是否合法（长度足够且校验字节匹配）。"""
    if buf is None or len(buf) < length + 6:
        return False
    s, ss = calc_verify(buf, length)
    return s == buf[length + 4] and ss == buf[length + 5]


def encode_omega(yaw: float, pitch: float, omega_max: float = 2.0) -> bytes:
    """编码 ``0x30`` 角速度指令帧。

    Args:
        yaw: 协议域 yaw 角速度（rad/s）。
        pitch: 协议域 pitch 角速度（rad/s，低头为正）。
        omega_max: 限幅上限（rad/s）。传 ``float('inf')`` 可关闭限幅
            （仅用于复现官方文档的原始黄金向量字节流）。

    Returns:
        bytes: 14 字节指令帧。非有限输入一律降为零指令。
    """
    if not (math.isfinite(yaw) and math.isfinite(pitch)):
        yaw = 0.0
        pitch = 0.0
    lo = -abs(float(omega_max))
    hi = abs(float(omega_max))
    yaw = max(lo, min(hi, float(yaw)))
    pitch = max(lo, min(hi, float(pitch)))
    return pack_frame(ADDR_PC2MCU, FUNC_OMEGA_CMD, struct.pack('<ff', yaw, pitch))


def encode_imu(seq: int,
               stamp_us: int,
               accel_xyz: Tuple[float, float, float],
               gyro_xyz: Tuple[float, float, float],
               quat_wxyz: Tuple[float, float, float, float]) -> bytes:
    """编码 ``0x01`` IMU 帧（主要用于单测 / 硬件在环回放）。

    Args:
        seq: 帧序号 u16。
        stamp_us: 下位机微秒时钟 u32。
        accel_xyz: 加速度原始物理量（m/s²），会被除以 ACC_SCALE 转回 i16。
        gyro_xyz: 角速度原始物理量（rad/s），会被除以 GYRO_SCALE 转回 i16。
        quat_wxyz: 四元数 (w,x,y,z)，会被乘以 Q_SCALE 转回 i16。

    Returns:
        bytes: 32 字节 IMU 帧。
    """

    def _i16(v: float) -> int:
        return int(max(-32768, min(32767, round(float(v)))))

    data = struct.pack('<HI', seq & 0xFFFF, stamp_us & 0xFFFFFFFF)
    data += struct.pack('<3h',
                        _i16(accel_xyz[0] / ACC_SCALE),
                        _i16(accel_xyz[1] / ACC_SCALE),
                        _i16(accel_xyz[2] / ACC_SCALE))
    data += struct.pack('<3h',
                        _i16(gyro_xyz[0] / GYRO_SCALE),
                        _i16(gyro_xyz[1] / GYRO_SCALE),
                        _i16(gyro_xyz[2] / GYRO_SCALE))
    data += struct.pack('<4h',
                        _i16(quat_wxyz[0] * Q_SCALE),
                        _i16(quat_wxyz[1] * Q_SCALE),
                        _i16(quat_wxyz[2] * Q_SCALE),
                        _i16(quat_wxyz[3] * Q_SCALE))
    return pack_frame(ADDR_MCU2PC, FUNC_IMU, data)


# --------------------------------------------------------------------------------------
# 0x01 IMU 解码
# --------------------------------------------------------------------------------------
@dataclass
class ImuFrame:
    """一帧解码后的 IMU 数据。"""

    seq: int = 0
    stamp_us: int = 0
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    quat_wxyz: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    t_mono: float = 0.0
    valid: bool = False
    reject_reason: str = ''


def decode_imu(frame: bytes, t_mono: Optional[float] = None, verify: bool = True) -> ImuFrame:
    """解码 ``0x01`` IMU 帧。

    Args:
        frame: 完整帧字节（含 head 与校验字节）。
        t_mono: 主机收到并通过校验的单调时刻；None 时取 ``time.monotonic()``。
        verify: 是否先做整帧校验。

    Returns:
        ImuFrame: ``valid=False`` 时 ``reject_reason`` 给出原因。
    """
    if t_mono is None:
        import time as _time
        t_mono = _time.monotonic()
    out = ImuFrame(t_mono=float(t_mono))

    if frame is None or len(frame) < IMU_FRAME_LEN:
        out.reject_reason = 'short'
        return out
    if frame[0] != HEAD:
        out.reject_reason = 'bad_head'
        return out
    if frame[2] != FUNC_IMU:
        out.reject_reason = 'not_imu'
        return out
    ln = frame[3]
    if ln != IMU_DATA_LEN:
        out.reject_reason = 'bad_len'
        return out
    if verify and not frame_ok(frame, ln):
        out.reject_reason = 'bad_crc'
        return out

    data = frame[4:4 + IMU_DATA_LEN]
    out.seq = int(struct.unpack_from('<H', data, 0)[0])
    out.stamp_us = int(struct.unpack_from('<I', data, 2)[0])
    # i16：struct '<h' 已按「先按 u16 组装两字节再转有符号」处理
    out.accel = np.asarray(struct.unpack_from('<3h', data, 6), dtype=np.float64) * ACC_SCALE
    out.gyro = np.asarray(struct.unpack_from('<3h', data, 12), dtype=np.float64) * GYRO_SCALE
    q_raw = np.asarray(struct.unpack_from('<4h', data, 18), dtype=np.float64)
    q = q_raw / Q_SCALE
    n = float(np.linalg.norm(q))
    if not math.isfinite(n) or n < QUAT_NORM_MIN:
        out.reject_reason = 'bad_quat'
        return out
    out.quat_wxyz = q / n
    if not (np.all(np.isfinite(out.accel)) and np.all(np.isfinite(out.gyro))):
        out.reject_reason = 'nonfinite'
        return out
    out.valid = True
    return out


class FrameParser:
    """无边界字节流的增量帧解析状态机（DESIGN §6.1 六步）。

    用法::

        parser = FrameParser()
        for frame in parser.feed(chunk):
            handle(frame)
    """

    def __init__(self, buf_limit: int = BUF_LIMIT) -> None:
        self._buf = bytearray()
        self._buf_limit: int = int(buf_limit)
        self._frames_ok: int = 0
        self._frames_bad: int = 0
        self._framing_err: int = 0

    def feed(self, chunk: bytes) -> List[bytes]:
        """喂入一段字节流，返回本次解析出的所有合法帧。

        Args:
            chunk: 新读取到的字节（可为 ``b''``）。

        Returns:
            list: 合法帧列表（``bytes``）。
        """
        frames: List[bytes] = []
        if chunk:
            self._buf.extend(chunk)

        while True:
            # 2) 缓冲区膨胀保护：整段丢弃
            if len(self._buf) > self._buf_limit:
                self._framing_err += len(self._buf)
                self._buf.clear()
                break
            # 3) 找帧头
            i = self._buf.find(HEAD)
            if i < 0:
                self._buf.clear()
                break
            if i > 0:
                del self._buf[:i]
                self._framing_err += i
            # 4) 至少要有 head/addr/func/len
            if len(self._buf) < 4:
                break
            ln = self._buf[3]
            func = self._buf[2]
            # 5) 长度合法性
            if ln == 0 or ln > MAX_DATA_LEN or (func in LEN_OF_FUNC and ln != LEN_OF_FUNC[func]):
                del self._buf[:1]          # 这个 0xAA 是假的
                self._framing_err += 1
                continue
            total = ln + 6
            if len(self._buf) < total:
                break                       # 等更多字节
            frame = bytes(self._buf[:total])
            del self._buf[:total]
            if frame_ok(frame, ln):
                self._frames_ok += 1
                frames.append(frame)
            else:
                self._frames_bad += 1
            # goto 3
        return frames

    def stats(self) -> Dict[str, int]:
        """返回解析统计。"""
        return {
            'frames_ok': int(self._frames_ok),
            'frames_bad': int(self._frames_bad),
            'framing_err': int(self._framing_err),
            'buf_len': int(len(self._buf)),
        }

    def reset(self) -> None:
        """清空缓冲区（计数器保留）。"""
        self._buf.clear()


class ImuStreamMonitor:
    """IMU 流的丢帧 / 复位 / 时序监测（DESIGN §6.2）。

    ``stamp_us`` 一律用**无符号减法**（约 71.6 分钟回绕）。
    """

    def __init__(self, expect_period_us: int = 2000, tol_us: int = 200,
                 max_seq_jump: int = 1000) -> None:
        self._expect_us: int = int(expect_period_us)
        self._tol_us: int = int(tol_us)
        self._max_seq_jump: int = int(max_seq_jump)
        self._seq_prev: Optional[int] = None
        self._stamp_prev: Optional[int] = None
        self.lost_frames: int = 0
        self.resets: int = 0
        self.timing_warn: int = 0

    def update(self, seq: int, stamp_us: int) -> Dict[str, int]:
        """喂入一帧的 seq / stamp_us，返回本次产生的事件增量。

        Returns:
            dict: ``{'lost': int, 'reset': int, 'timing_warn': int}``。
        """
        seq = int(seq) & 0xFFFF
        stamp_us = int(stamp_us) & 0xFFFFFFFF
        evt = {'lost': 0, 'reset': 0, 'timing_warn': 0}
        if self._seq_prev is None or self._stamp_prev is None:
            self._seq_prev = seq
            self._stamp_prev = stamp_us
            return evt
        d_seq = (seq - self._seq_prev) & 0xFFFF
        d_us = (stamp_us - self._stamp_prev) & 0xFFFFFFFF
        if d_us > (1 << 31) or d_seq > self._max_seq_jump:
            # 下位机复位：重置基准，不算丢帧
            self.resets += 1
            evt['reset'] = 1
        elif d_seq != 1:
            self.lost_frames += int(d_seq) - 1
            evt['lost'] = int(d_seq) - 1
        elif abs(int(d_us) - self._expect_us) > self._tol_us:
            self.timing_warn += 1
            evt['timing_warn'] = 1
        self._seq_prev = seq
        self._stamp_prev = stamp_us
        return evt

    def stats(self) -> Dict[str, int]:
        """返回累计统计。"""
        return {
            'lost_frames': int(self.lost_frames),
            'resets': int(self.resets),
            'timing_warn': int(self.timing_warn),
        }

    def reset(self) -> None:
        """清空基准（滤波器随之重设）。"""
        self._seq_prev = None
        self._stamp_prev = None
