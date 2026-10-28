"""`team43_gimbal_io/vgimbal_bridge_node.py` 的功能性回归（真 ROS2 Humble）。

手法：**真 rclpy 消息类型 + 注入式外部节点（`_FakeNode`）+ 内存 FakePort**——
节点构造函数接收外部 node（同 `gt_error_monitor_node` 模式），串口对象在
`start()` 才真正打开，构造后把 `app._port` 换成内存替身即可完整驱动
收包解析（`P.encode_imu` 真 0x01 帧字节流）、IMU/GimbalState 发布、
100Hz 下发门控、epoch 语义与 shutdown 收尾。

时间控制：`TU.now`（节点内所有单调时钟判定）patch 为 `_Clock`；
`LinkStateMachine` 在构造时捕获了真实单调钟，故构造后补注入
`app._lsm._now`（测试内唯一触碰的私有状态，升级时需复核）。

⚠ 这里验证的是**节点逻辑**：消息类、协议编解码都是真的；DDS 传输、
真实串口设备与 spin 线程不在本文件范围。
"""

from __future__ import annotations

import json
import math
import os
import sys
from types import SimpleNamespace

import pytest

_PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
_SRC_DIR = os.path.abspath(os.path.join(_PKG_DIR, '..'))
sys.path.insert(0, _PKG_DIR)
sys.path.insert(0, os.path.join(_SRC_DIR, 'team43_common'))   # team43_common 源码目录

pytest.importorskip('rclpy')

from builtin_interfaces.msg import Time as BuiltinTime  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from team43_common import frames as FR  # noqa: E402
from team43_common import protocol as P  # noqa: E402
from team43_interfaces.msg import GimbalOmegaCmd  # noqa: E402

import team43_gimbal_io.vgimbal_bridge_node as NODE  # noqa: E402
from team43_gimbal_io.link_state_machine import (  # noqa: E402
    ACTIVE, FAULT_SILENT, IDLE, LINK_STATE_ID, SHUTDOWN,
)


# ---------------------------------------------------------------------------
# 注入式替身（真消息类型 + 可控时钟 + 内存串口）
# ---------------------------------------------------------------------------
class _Clock(object):
    def __init__(self, t0: float = 1000.0) -> None:
        self.t = float(t0)

    def now(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)

    def to_msg(self) -> BuiltinTime:
        sec = math.floor(self.t)
        nanosec = int(round((self.t - sec) * 1e9))
        if nanosec >= 1000000000:
            sec += 1
            nanosec -= 1000000000
        return BuiltinTime(sec=int(sec), nanosec=nanosec)


class _ClockHandle(object):
    def __init__(self, clock: _Clock) -> None:
        self._clock = clock

    def now(self) -> '_ClockHandle':
        return self

    def to_msg(self) -> BuiltinTime:
        return self._clock.to_msg()


class _Param(object):
    def __init__(self, value) -> None:
        self.value = value


class _Publisher(object):
    def __init__(self, sink: list, topic: str) -> None:
        self._sink, self._topic = sink, topic

    def publish(self, msg) -> None:
        self._sink.append((self._topic, msg))


class _RecordingLogger(object):
    """记录等级调用的 logger 替身（兼容直接调用与 LU.throttled 路径）。"""

    def __init__(self) -> None:
        self.records = []

    def _rec(self, level):
        return lambda message: self.records.append((level, str(message)))

    def debug(self, message):
        self._rec('debug')(message)

    info = property(lambda self: self._rec('info'))
    warning = property(lambda self: self._rec('warning'))
    warn = property(lambda self: self._rec('warning'))
    error = property(lambda self: self._rec('error'))
    fatal = property(lambda self: self._rec('fatal'))


class _FakeNode(object):
    """vgimbal_bridge_node 用到的那一小部分 rclpy Node API 的替身。"""

    def __init__(self, overrides: dict, clock: _Clock) -> None:
        self._params = dict(overrides)
        self._clock_handle = _ClockHandle(clock)
        self.subs, self.pubs, self.timers, self.published = {}, {}, [], []

    def declare_parameter(self, name, default=None):
        self._params.setdefault(name, default)
        return self._params[name]

    def has_parameter(self, name) -> bool:
        return name in self._params

    def get_parameter(self, name):
        return _Param(self._params.get(name))

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs[topic] = cb
        return object()

    def create_publisher(self, msg_type, topic, qos):
        pub = _Publisher(self.published, topic)
        self.pubs[topic] = pub
        return pub

    def create_timer(self, period, cb):
        self.timers.append((float(period), cb))
        return object()

    def get_clock(self) -> _ClockHandle:
        return self._clock_handle

    def get_name(self) -> str:
        return NODE.NODE_NAME

    def get_logger(self):
        import logging
        return logging.getLogger(NODE.NODE_NAME)

    def messages(self, topic: str) -> list:
        return [m for tp, m in self.published if tp == topic]


class _FakePort(object):
    """内存串口：rx 队列供 read_bytes 出队，write 记录原始字节。"""

    def __init__(self) -> None:
        self.rx = bytearray()
        self.written = []
        self.closed = False

    def read_bytes(self):
        chunk = bytes(self.rx[:256])
        del self.rx[:len(chunk)]
        return chunk

    def write(self, data) -> None:
        self.written.append(bytes(data))

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True

    def stats(self):
        return {'bytes_rx': 0, 'bytes_tx': 0}

    @property
    def port(self) -> str:
        return '/tmp/fake_port'


@pytest.fixture()
def rig():
    saved_now = NODE.TU.now
    saved_sign = FR.PITCH_SIGN
    NODE.TU.now = lambda: 0.0            # 先占位，构造后换成可控时钟
    try:
        clock = _Clock(1000.0)
        NODE.TU.now = clock.now
        overrides = dict(NODE.PARAM_DEFAULTS)
        overrides.update({'serial.publish_tf': False, 'serial.sim_epoch': 7})
        fake = _FakeNode(overrides, clock)
        app = NODE.VgimbalBridgeNode(fake)
        # LinkStateMachine 构造时捕获了真实单调钟；测试注入可控时钟。
        app._lsm._now = clock.now
        port = _FakePort()
        app._port = port
        yield SimpleNamespace(node=NODE, app=app, fake=fake, clock=clock, port=port)
    finally:
        NODE.TU.now = saved_now
        FR.set_pitch_sign(saved_sign)


def _feed_imu(rig, seq, stamp_us, accel=(0., 0., 9.8), gyro=(0.05, -0.1, 0.2),
              quat_wxyz=(1., 0., 0., 0.)):
    rig.port.rx += P.encode_imu(seq, stamp_us, accel, gyro, quat_wxyz)
    rig.app._rx_tick()


def _quat_for(yaw: float, pitch: float):
    return FR.quat_from_mat(FR.compose_yaw_pitch(yaw, pitch))


def _send_cmd(rig, yaw_omega, pitch_omega, valid=True):
    msg = GimbalOmegaCmd()
    msg.yaw_omega = float(yaw_omega)
    msg.pitch_omega = float(pitch_omega)
    msg.valid = bool(valid)
    rig.fake.subs['/gimbal/cmd_omega'](msg)


# ---------------------------------------------------------------------------
# 1. 接线与启动语义
# ---------------------------------------------------------------------------
def test_wiring_and_initial_epoch(rig):
    """发布 IMU/state/epoch，订阅 cmd/status，双定时器（200Hz rx / 100Hz tx）；
    启动即发一次 epoch（latched，值 = serial.sim_epoch）。"""
    fake = rig.fake
    assert set(fake.subs) == {'/gimbal/cmd_omega', '/virtual_gimbal/status'}
    assert '/offboardlink/imu_data' in fake.pubs
    assert '/gimbal/state' in fake.pubs
    assert '/task1/gimbal_reset_epoch' in fake.pubs
    assert len(fake.timers) == 2
    assert fake.timers[0][0] == pytest.approx(1.0 / 200.0)
    assert fake.timers[1][0] == pytest.approx(1.0 / 100.0)
    epochs = fake.messages('/task1/gimbal_reset_epoch')
    assert [e.data for e in epochs] == [7]
    assert rig.app._tf_broadcaster is None, 'publish_tf=false 时不得创建 TF 广播器'


def test_no_tx_before_first_imu_frame(rig):
    """IDLE（首帧 0x01 前）绝不发任何 0x30——即使有指令请求。"""
    _send_cmd(rig, 0.5, -0.2)
    for _ in range(5):
        rig.clock.advance(0.011)
        rig.app._tx_tick()
    assert rig.port.written == []
    assert rig.app._lsm.state == IDLE


# ---------------------------------------------------------------------------
# 2. 收包 → 发布
# ---------------------------------------------------------------------------
def test_imu_frame_publishes_imu_and_state(rig):
    accel, gyro = (0.12, -0.34, 9.79), (0.05, -0.1, 0.2)
    yaw, pitch = 0.35, -0.12
    quat = _quat_for(yaw, pitch)
    rig.clock.advance(0.01)
    _feed_imu(rig, seq=10, stamp_us=2_000_000, accel=accel, gyro=gyro,
              quat_wxyz=tuple(quat))

    imus = rig.fake.messages('/offboardlink/imu_data')
    assert len(imus) == 1
    imu = imus[0]
    assert imu.header.frame_id == 'gimbal_link'
    assert imu.header.stamp.sec == 1000 and imu.header.stamp.nanosec == 10_000_000
    # 协议链路对 quat/陀螺/加计做定点量化（Q_SCALE / _i16），往返误差 ~1e-4
    assert (imu.orientation.w, imu.orientation.x,
            imu.orientation.y, imu.orientation.z) == pytest.approx(list(quat), abs=1e-4)
    assert (imu.angular_velocity.x, imu.angular_velocity.y,
            imu.angular_velocity.z) == pytest.approx(list(gyro), abs=1e-2)
    assert (imu.linear_acceleration.x, imu.linear_acceleration.y,
            imu.linear_acceleration.z) == pytest.approx(list(accel), abs=1e-2)

    states = rig.fake.messages('/gimbal/state')
    assert len(states) == 1
    st = states[0]
    # 角度由量化 quat 分解而来，误差 ~1e-4；角速度由量化陀螺派生
    assert st.yaw == pytest.approx(yaw, abs=5e-4)
    assert st.pitch == pytest.approx(pitch, abs=5e-4)
    assert st.omega_pitch == pytest.approx(gyro[1], abs=5e-2)
    assert st.omega_yaw == pytest.approx(gyro[2] / math.cos(pitch), abs=5e-2)
    assert st.link_state == LINK_STATE_ID[ACTIVE] == 1   # 消息里是 int 编码
    assert st.enabled is True
    assert st.seq_last == 10
    assert rig.app._imu_frames == 1 and rig.app._imu_bad == 0


def test_rx_tick_drains_multiple_frames(rig):
    """一次 rx tick 内把缓冲读空：3 帧 0x01 → 3 条 IMU 发布。"""
    for seq, stamp in ((1, 2_000), (2, 4_000), (3, 6_000)):
        rig.port.rx += P.encode_imu(seq, stamp, (0., 0., 9.8), (0., 0., 0.),
                                     (1., 0., 0., 0.))
    rig.app._rx_tick()
    assert len(rig.fake.messages('/offboardlink/imu_data')) == 3


def test_seq_regression_bumps_epoch(rig):
    """下位机复位（seq 回退）→ epoch +1 并重发。"""
    _feed_imu(rig, seq=5, stamp_us=10_000)
    assert [e.data for e in rig.fake.messages('/task1/gimbal_reset_epoch')] == [7]
    _feed_imu(rig, seq=2, stamp_us=4_000)
    assert [e.data for e in rig.fake.messages('/task1/gimbal_reset_epoch')] == [7, 8]


def test_bad_imu_payload_is_counted_not_published(rig):
    """ quat 模长过小的 0x01 帧解码失败 → 不发布、计入 _imu_bad。"""
    _feed_imu(rig, seq=1, stamp_us=2_000, quat_wxyz=(0.05, 0., 0., 0.))
    assert rig.fake.messages('/offboardlink/imu_data') == []
    assert rig.app._imu_bad == 1 and rig.app._imu_frames == 0


# ---------------------------------------------------------------------------
# 3. 下发门控
# ---------------------------------------------------------------------------
def test_active_tx_uses_fresh_command(rig):
    _feed_imu(rig, seq=1, stamp_us=2_000)          # IDLE → ACTIVE
    rig.clock.advance(0.011)
    rig.app._tx_tick()
    assert rig.port.written == [P.encode_omega(0.0, 0.0)]   # 无指令：保活零帧

    _send_cmd(rig, 0.3, -0.1)
    rig.clock.advance(0.011)
    rig.app._tx_tick()
    assert rig.port.written[-1] == P.encode_omega(0.3, -0.1)

    # 周期内未到期：不重复发送
    n = len(rig.port.written)
    rig.app._tx_tick()
    assert len(rig.port.written) == n


def test_stale_command_falls_back_to_zero(rig):
    _feed_imu(rig, seq=1, stamp_us=2_000)
    _send_cmd(rig, 0.4, 0.1)
    rig.clock.advance(0.6)                          # > serial.cmd_fresh_s(0.5)
    rig.app._tx_tick()
    assert rig.port.written[-1] == P.encode_omega(0.0, 0.0)


def test_nonfinite_command_clears_cache(rig):
    _feed_imu(rig, seq=1, stamp_us=2_000)
    _send_cmd(rig, float('nan'), 0.1)
    assert rig.app._cmd_valid is False
    rig.clock.advance(0.011)
    rig.app._tx_tick()
    assert rig.port.written[-1] == P.encode_omega(0.0, 0.0)


# ---------------------------------------------------------------------------
# 4. 故障与收尾
# ---------------------------------------------------------------------------
def test_out_of_range_yaw_enters_fault_silence(rig):
    _feed_imu(rig, seq=1, stamp_us=2_000)           # 先 ACTIVE
    assert rig.app._lsm.state == ACTIVE
    rig.clock.advance(0.011)
    rig.app._tx_tick()
    assert len(rig.port.written) == 1               # ACTIVE 期间那帧保活

    _feed_imu(rig, seq=2, stamp_us=4_000,
              quat_wxyz=tuple(_quat_for(2.95, 0.0)))  # |yaw|=2.95 > 2.8+0.1，
    # 且不越过 ±π（decompose 会把 3.5 回绕成 -2.78 落回限位内）
    assert rig.app._lsm.state == FAULT_SILENT
    assert rig.app._lsm.n_faults >= 1
    for _ in range(5):
        rig.clock.advance(0.011)
        rig.app._tx_tick()
    assert len(rig.port.written) == 1               # FAULT_SILENT 期间不再发帧


def test_shutdown_sends_five_zero_frames_and_closes(rig):
    _feed_imu(rig, seq=1, stamp_us=2_000)
    rig.app.shutdown()
    assert rig.port.closed is True
    tail = rig.port.written[-5:]
    assert tail == [P.encode_omega(0.0, 0.0)] * 5
    assert rig.app._lsm.state == SHUTDOWN


# ---------------------------------------------------------------------------
# 5. /virtual_gimbal/status 监控
# ---------------------------------------------------------------------------
def test_status_sim_skips_logs_error(rig):
    log = _RecordingLogger()
    rig.app._log = log
    msg = String()
    msg.data = json.dumps({'state': 'ACTIVE', 'sim_skips': 3})
    rig.fake.subs['/virtual_gimbal/status'](msg)
    assert any(level == 'error' and 'sim_skips' in text
               for level, text in log.records)


def test_status_non_json_warns_and_state_fault_flags(rig):
    log = _RecordingLogger()
    rig.app._log = log
    msg = String()
    msg.data = 'not-json{{'
    rig.fake.subs['/virtual_gimbal/status'](msg)      # 不得抛错
    msg.data = json.dumps({'state': 'FAULT_SILENT', 'sim_skips': 0})
    rig.fake.subs['/virtual_gimbal/status'](msg)
    assert any('fault' in text.lower() for _l, text in log.records)
