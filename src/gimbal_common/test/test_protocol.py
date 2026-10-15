"""协议层单测（DESIGN §6.1 黄金向量 + §6.2 解码 + 解析状态机）。

纯 Python，不依赖 ROS。运行::

    python -m pytest src/gimbal_common/test/test_protocol.py -q
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from gimbal_common import protocol as P


def _hex(bs: bytes) -> str:
    return ' '.join('%02X' % b for b in bs)


# --------------------------------------------------------------------------------------
# 黄金向量（DESIGN §6.1 / docs/hardware_spec.md）
# --------------------------------------------------------------------------------------
def test_pack_frame_zero_omega_golden():
    """AA EE 30 08 00×8 D0 5A"""
    frame = P.pack_frame(P.ADDR_PC2MCU, P.FUNC_OMEGA_CMD, b'\x00' * 8)
    assert _hex(frame) == 'AA EE 30 08 00 00 00 00 00 00 00 00 D0 5A'
    assert P.frame_ok(frame, 8)


def test_encode_omega_golden():
    """AA EE 30 08 00 00 40 40 00 00 00 BE 0E D8

    官方文档示例是**限幅前**的原始字节流（yaw=3.0），因此这里显式关闭限幅复现。
    """
    frame = P.encode_omega(3.0, -0.125, omega_max=float('inf'))
    assert _hex(frame) == 'AA EE 30 08 00 00 40 40 00 00 00 BE 0E D8'
    assert P.frame_ok(frame, 8)


def test_encode_omega_clamps_by_default():
    """默认 omega_max=2.0：越界必须被自己先限幅（硬约束）。"""
    frame = P.encode_omega(3.0, -99.0)
    assert P.frame_ok(frame, 8)
    import struct
    yaw, pitch = struct.unpack_from('<ff', frame, 4)
    assert yaw == pytest.approx(2.0)
    assert pitch == pytest.approx(-2.0)


def test_encode_omega_nonfinite_falls_back_to_zero():
    frame = P.encode_omega(float('nan'), float('inf'))
    import struct
    yaw, pitch = struct.unpack_from('<ff', frame, 4)
    assert yaw == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)


def test_imu_frame_golden():
    """AA FF 01 1A 00 00 40 42 0F 00 00 00 00 00 55 05 ... 26 B5"""
    expected = ('AA FF 01 1A 00 00 40 42 0F 00 00 00 00 00 55 05 '
                '00 00 00 00 00 00 F8 7F 00 00 00 00 00 00 26 B5')
    frame = P.encode_imu(seq=0, stamp_us=1000000,
                         accel_xyz=(0.0, 0.0, 0x0555 * P.ACC_SCALE),
                         gyro_xyz=(0.0, 0.0, 0.0),
                         quat_wxyz=(1.0, 0.0, 0.0, 0.0))
    assert _hex(frame) == expected
    assert P.frame_ok(frame, 26)

    imu = P.decode_imu(frame, t_mono=1.0)
    assert imu.valid, imu.reject_reason
    assert imu.seq == 0
    assert imu.stamp_us == 1000000
    assert imu.accel[2] == pytest.approx(9.80425, abs=1e-3)
    assert np.linalg.norm(imu.quat_wxyz) == pytest.approx(1.0)


def test_verify_uses_updated_sum():
    """sum_sum 累加的是更新后的 sum，不是原始字节（最易错处）。"""
    buf = bytes([0xAA, 0xEE, 0x30, 0x08] + [0x00] * 8)
    s, ss = P.calc_verify(buf, 8)
    assert s == 0xD0
    assert ss == 0x5A


# --------------------------------------------------------------------------------------
# decode_imu
# --------------------------------------------------------------------------------------
def test_decode_imu_rejects_bad_crc():
    frame = bytearray(P.encode_imu(1, 2000, (0, 0, 9.8), (0, 0, 0), (1, 0, 0, 0)))
    frame[-1] ^= 0xFF
    imu = P.decode_imu(bytes(frame), t_mono=0.0)
    assert not imu.valid
    assert imu.reject_reason == 'bad_crc'


def test_decode_imu_rejects_small_quat_norm():
    frame = P.encode_imu(2, 4000, (0, 0, 9.8), (0, 0, 0), (0.1, 0.0, 0.0, 0.0))
    imu = P.decode_imu(frame, t_mono=0.0)
    assert not imu.valid
    assert imu.reject_reason == 'bad_quat'


def test_decode_imu_rejects_bad_len_and_func():
    # 整帧应为 32 字节（4 头 + 26 data + 2 校验）
    assert P.decode_imu(b'\xAA\xFF\x01\x05' + b'\x00' * 28, t_mono=0.0).reject_reason == 'bad_len'
    assert P.decode_imu(b'\xAA\xFF\x30\x1A' + b'\x00' * 28, t_mono=0.0).reject_reason == 'not_imu'
    assert P.decode_imu(b'\xAA\xFF\x01\x1A', t_mono=0.0).reject_reason == 'short'


def test_decode_imu_rotated_quat_normalized():
    q = np.array([0.9238795, 0.0, 0.3826834, 0.0])
    frame = P.encode_imu(5, 10000, (0.0, 0.0, 9.8), (0.1, 0.2, -0.3), tuple(q))
    imu = P.decode_imu(frame, t_mono=2.5)
    assert imu.valid
    assert np.linalg.norm(imu.quat_wxyz) == pytest.approx(1.0)
    assert np.allclose(imu.quat_wxyz, q / np.linalg.norm(q), atol=1e-4)
    assert imu.gyro[0] == pytest.approx(0.1, abs=1e-3)
    assert imu.gyro[2] == pytest.approx(-0.3, abs=1e-3)
    assert imu.t_mono == pytest.approx(2.5)


# --------------------------------------------------------------------------------------
# FrameParser 状态机
# --------------------------------------------------------------------------------------
def test_parser_handles_garbage_prefix_and_partial_frames():
    parser = P.FrameParser()
    good = P.encode_omega(0.1, 0.2)
    assert parser.feed(b'\x11\x22\x33') == []
    assert parser.feed(good[:7]) == []
    out = parser.feed(good[7:])
    assert out == [good]
    assert parser.stats()['frames_ok'] == 1
    assert parser.stats()['frames_bad'] == 0


def test_parser_drops_bad_crc_frame():
    parser = P.FrameParser()
    bad = bytearray(P.encode_omega(0.0, 0.0))
    bad[-2] ^= 0x01
    assert parser.feed(bytes(bad)) == []
    assert parser.stats()['frames_bad'] == 1


def test_parser_drops_false_head_and_bad_len():
    parser = P.FrameParser()
    # 假帧头：len 字段为 0x00 → 长度非法 → 丢弃这个 0xAA
    assert parser.feed(b'\xAA\xEE\x30\x00' + b'\x00' * 8) == []
    # len 超过 54 → 同样丢弃
    assert parser.feed(b'\xAA\xEE\x30\xFF' + b'\x00' * 8) == []
    # func=0x30 但 len 不是 8 → 丢弃
    assert parser.feed(b'\xAA\xEE\x30\x1A' + b'\x00' * 26) == []


def test_parser_buffer_overflow_discards_everything():
    parser = P.FrameParser(buf_limit=32)
    assert parser.feed(b'\x77' * 64) == []
    stats = parser.stats()
    assert stats['buf_len'] == 0
    assert stats['framing_err'] >= 64


def test_parser_multiple_frames_in_one_chunk():
    parser = P.FrameParser()
    a = P.encode_omega(0.0, 0.0)
    b = P.encode_imu(1, 2000, (0, 0, 9.8), (0, 0, 0), (1, 0, 0, 0))
    out = parser.feed(a + b)
    assert out == [a, b]


# --------------------------------------------------------------------------------------
# 丢帧 / 复位监测
# --------------------------------------------------------------------------------------
def test_imu_stream_monitor_normal():
    mon = P.ImuStreamMonitor()
    assert mon.update(0, 0)['lost'] == 0
    assert mon.update(1, 2000)['lost'] == 0
    assert mon.stats()['lost_frames'] == 0


def test_imu_stream_monitor_counts_lost_frames():
    mon = P.ImuStreamMonitor()
    mon.update(0, 0)
    evt = mon.update(4, 8000)
    assert evt['lost'] == 3
    assert mon.stats()['lost_frames'] == 3


def test_imu_stream_monitor_reset_on_seq_jump():
    mon = P.ImuStreamMonitor()
    mon.update(100, 200000)
    evt = mon.update(5, 10000)          # stamp 回退 → 复位
    assert evt['reset'] == 1
    assert evt['lost'] == 0
    assert mon.stats()['resets'] == 1


def test_imu_stream_monitor_seq_wrap_is_one_step():
    mon = P.ImuStreamMonitor()
    mon.update(65535, 0)
    evt = mon.update(0, 2000)
    assert evt['lost'] == 0


def test_imu_stream_monitor_timing_warn():
    mon = P.ImuStreamMonitor()
    mon.update(0, 0)
    evt = mon.update(1, 9000)           # 间隔 9000us ≠ 2000±200
    assert evt['timing_warn'] == 1
