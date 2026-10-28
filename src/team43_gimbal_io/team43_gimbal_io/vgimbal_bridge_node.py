"""虚拟云台串口桥主节点（DESIGN §3.4 / §6.1-§6.3 / §6.9）。

职责：

1. 独占打开串口（失败 FATAL 退出，不静默重试）；
2. 轮询收包 → ``FrameParser`` → ``decode_imu`` → 发布 ``sensor_msgs/Imu``、
   ``team43_interfaces/GimbalState``、TF ``world→gimbal_link``；
3. 100Hz 单调定时器组 ``0x30`` 下发（无指令发 ``(0,0)`` 保活）；
4. 链路使能/故障状态机（**首帧 ``0x01`` 前不发任何帧**）；
5. 监控 ``/virtual_gimbal/status``：``sim_skips != 0`` 记 ERROR。

⚠ 时钟（主理人拍板 1「混合时钟」）：

* ``header.stamp`` / TF 时间戳 → ROS 系统时钟 ``node.get_clock().now()``（保证 RViz 同域）。
* 所有 dt、超时、周期计算 → ``team43_common.time_utils.now()``（``time.monotonic()``）。
* **不得**用 ``header.stamp`` 或外部话题时间戳做任何运算。

⚠ 本文件依赖 ROS 2，**已在 Ubuntu 22.04 + ROS 2 Humble 容器中编译/运行验证**。
"""

from __future__ import annotations

import json
import math
import sys
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt64
from tf2_ros import TransformBroadcaster

from team43_common import frames as FR
from team43_common.log_utils import get_logger, throttled
from team43_common import node_utils as NU
from team43_common import protocol as P
from team43_common import time_utils as TU
from team43_common.qos_utils import latched_qos, sensor_qos
from team43_interfaces.msg import GimbalOmegaCmd, GimbalState

from .link_state_machine import (ACTIVE, FAULT_SILENT, IDLE, SHUTDOWN,
                                 LinkStateMachine)
from .serial_port import SerialPort, SerialPortError

NODE_NAME = 'vgimbal_bridge_node'

# 一次 rx tick 内最多连续读多少块（防止单 tick 被串口读空循环占满）
_RX_MAX_READS_PER_TICK: int = 32

PARAM_DEFAULTS = {
    'serial.port': '/tmp/ttyVGIMBAL',
    'serial.baud': 921600,
    'serial.exclusive': True,
    'serial.read_timeout_s': 0.01,
    'serial.read_chunk': 256,
    'serial.rx_rate_hz': 200.0,
    'serial.cmd_rate_hz': 100.0,
    'serial.omega_max': 2.0,
    'serial.fault_silence_s': 0.6,
    'serial.shutdown_zero_frames': 5,
    'serial.cmd_fresh_s': 0.5,
    'serial.imu_timeout_s': 1.0,
    'serial.bad_frame_window_s': 1.0,
    'serial.bad_frame_limit': 50,
    'serial.frame_origin_xyz': [0.0, 0.0, 0.0],
    # 仿真 epoch：固定值（实机 C++ 串口桥用系统时钟 ns，每次启动不同）。
    # kinematics.calibrated_epoch 与 gimbal_bearing 的 epoch 锁存都以它为准。
    'serial.sim_epoch': 1,
    # world→gimbal_link TF 开关；关闭时该边让位（如 kinematics 发布
    # base_link→gimbal_link 的链路，避免 gimbal_link 出现两个父节点）。
    'serial.publish_tf': True,
    'gimbal.pitch_sign': -1.0,
    'gimbal.yaw_limits': [-2.8, 2.8],
    'gimbal.pitch_limits': [-0.5, 0.35],
    'gimbal.limit_fault_margin': 0.10,
    'gimbal.runaway_factor': 1.5,
    'gimbal.err_thresh': 0.5,
    'gimbal.runaway_hold_s': 0.2,
}


class VgimbalBridgeNode:
    """串口桥节点：收包解码 + 100Hz 下发 + 链路状态机。"""

    def __init__(self, node) -> None:
        self._node = node
        self._log = get_logger(node, 'bridge')
        self._params = NU.declare_params(node, PARAM_DEFAULTS)
        p = self._params

        # ⚠ pitch 符号唯一入口：由 ROS 参数初始化 frames.PITCH_SIGN
        FR.set_pitch_sign(float(p['gimbal.pitch_sign']))

        self._port = SerialPort(
            port=str(p['serial.port']),
            baud=int(p['serial.baud']),
            exclusive=bool(p['serial.exclusive']),
            read_timeout_s=float(p['serial.read_timeout_s']),
            read_chunk=int(p['serial.read_chunk']),
        )
        self._parser = P.FrameParser()
        self._imu_monitor = P.ImuStreamMonitor()
        self._lsm = LinkStateMachine(
            fault_silence_s=float(p['serial.fault_silence_s']),
            cmd_period_s=1.0 / max(1e-6, float(p['serial.cmd_rate_hz'])),
            yaw_limits=(float(p['gimbal.yaw_limits'][0]), float(p['gimbal.yaw_limits'][1])),
            pitch_limits=(float(p['gimbal.pitch_limits'][0]), float(p['gimbal.pitch_limits'][1])),
            limit_fault_margin=float(p['gimbal.limit_fault_margin']),
            runaway_factor=float(p['gimbal.runaway_factor']),
            err_thresh=float(p['gimbal.err_thresh']),
            runaway_hold_s=float(p['gimbal.runaway_hold_s']),
            bad_frame_window_s=float(p['serial.bad_frame_window_s']),
            bad_frame_limit=int(p['serial.bad_frame_limit']),
        )

        # ---- 最新指令（协议域）
        self._cmd_yaw: float = 0.0
        self._cmd_pitch: float = 0.0
        self._cmd_valid: bool = False
        self._cmd_t_mono: float = 0.0

        # ---- 统计
        self._t_last_imu_mono: Optional[float] = None
        self._frames_bad_prev: int = 0
        self._imu_frames: int = 0
        self._imu_bad: int = 0
        self._last_state: str = self._lsm.state
        self._origin_xyz = [float(v) for v in list(p['serial.frame_origin_xyz'])[:3]]
        while len(self._origin_xyz) < 3:
            self._origin_xyz.append(0.0)

        # ---- 发布/订阅
        qos = sensor_qos(10)
        self._pub_imu = node.create_publisher(Imu, '/offboardlink/imu_data', qos)
        self._pub_state = node.create_publisher(GimbalState, '/gimbal/state', qos)
        self._tf_broadcaster = TransformBroadcaster(node) if bool(p['serial.publish_tf']) else None
        # epoch：镜像 C++ 串口桥——启动发布一次（TRANSIENT_LOCAL 迟到订阅者可取），
        # 检测到下位机复位（stamp/seq 回退）后 +1 重发。
        self._pub_epoch = node.create_publisher(
            UInt64, '/task1/gimbal_reset_epoch', latched_qos(1))
        self._epoch_value = int(p['serial.sim_epoch'])
        self._publish_epoch()
        node.create_subscription(GimbalOmegaCmd, '/gimbal/cmd_omega', self._on_cmd_omega, qos)
        node.create_subscription(String, '/virtual_gimbal/status', self._on_status, qos)

        # ---- 定时器（周期由参数给出；回调内部一律用单调时钟做判定）
        rx_hz = max(1.0, float(p['serial.rx_rate_hz']))
        cmd_hz = max(1.0, float(p['serial.cmd_rate_hz']))
        node.create_timer(1.0 / rx_hz, self._rx_tick)
        node.create_timer(1.0 / cmd_hz, self._tx_tick)

        self._log.info(
            f'串口桥启动 port={p["serial.port"]} baud={p["serial.baud"]} '
            f'cmd_rate={cmd_hz:.0f}Hz rx_rate={rx_hz:.0f}Hz '
            f'pitch_sign={FR.PITCH_SIGN} fault_silence={p["serial.fault_silence_s"]}s')

    # ------------------------------------------------------------------ 生命周期
    def start(self) -> bool:
        """打开串口；失败返回 False（调用方应 FATAL 退出）。"""
        try:
            self._port.open()
        except SerialPortError as exc:  # 真实串口故障需进入静默停止状态，避免继续下发旧速度。
            self._log.fatal(str(exc))
            return False
        self._log.info(f'串口已独占打开: {self._port.port}')
        return True

    def shutdown(self) -> None:
        """收尾：连发零指令 → 关闭串口。"""
        try:
            n_zero = self._lsm.request_shutdown(int(self._params['serial.shutdown_zero_frames']))
            for _ in range(n_zero):
                self._write_frame(P.encode_omega(0.0, 0.0,
                                                 omega_max=float(self._params['serial.omega_max'])))
            self._port.flush()
        except SerialPortError as exc:
            # An unplugged/failing serial link must not prevent port cleanup.
            self._log.error(f'退出零指令发送失败: {exc!r}')
        finally:
            self._port.close()
            stats = self._port.stats()
            self._log.info(
                f'串口桥退出: 收到 IMU {self._imu_frames} 帧（丢弃 {self._imu_bad}），'
                f'parser={self._parser.stats()}, port={stats}, '
                f'faults={self._lsm.n_faults}')

    def _publish_epoch(self) -> None:
        """发布当前 epoch（TRANSIENT_LOCAL；与 C++ 串口桥同一话题/语义）。"""
        msg = UInt64()
        msg.data = int(self._epoch_value)
        self._pub_epoch.publish(msg)
        self._log.info(f'epoch 发布: {msg.data}')

    # ------------------------------------------------------------------ 收包
    def _rx_tick(self) -> None:
        """收包定时器回调：连续读到空为止 → 解析 → 逐帧处理。

        IMU 500Hz × 32B = 16KB/s；单次 read 上限 256B，故一次 tick 内循环读空，
        避免缓冲区积压（收包成簇到达时尤其重要）。
        """
        try:
            for _ in range(_RX_MAX_READS_PER_TICK):
                chunk = self._port.read_bytes()
                if not chunk:
                    break
                frames = self._parser.feed(chunk)
                self._absorb_bad_frames()
                for frame in frames:
                    self._handle_frame(frame)
        except SerialPortError as exc:
            # Physical read failures leave the watchdog active; programming errors surface.
            throttled(self._log, 'error', f'串口读取失败: {exc!r}', 'rx_tick')
        finally:
            self._lsm.tick()
            self._log_state_change()
            self._check_imu_watchdog()

    def _handle_frame(self, frame: bytes) -> None:
        """处理一帧合法数据（目前只认 ``0x01``）。"""
        if len(frame) < 4 or frame[2] != P.FUNC_IMU:
            return
        t_mono = TU.now()
        imu = P.decode_imu(frame, t_mono=t_mono)
        if not imu.valid:
            self._imu_bad += 1
            throttled(self._log, 'warning',
                         f'IMU 帧解码失败: {imu.reject_reason}', f'imu_bad_{imu.reject_reason}')
            return
        self._imu_frames += 1
        self._t_last_imu_mono = t_mono

        # 首帧 0x01 之前绝不发任何 0x30；迁移日志由 _log_state_change 统一打印
        if self._lsm.on_imu_frame(t_mono):
            self._imu_monitor.reset()

        evt = self._imu_monitor.update(imu.seq, imu.stamp_us)
        if evt['reset']:
            throttled(self._log, 'warning', '下位机复位：重置 IMU 基准', 'imu_reset')
            self._epoch_value += 1
            self._publish_epoch()
        if evt['lost']:
            throttled(self._log, 'warning',
                         f"IMU 丢帧 {evt['lost']} 帧（累计 {self._imu_monitor.stats()['lost_frames']}）",
                         'imu_lost')

        self._publish_imu(imu)
        self._publish_state(imu)

    def _publish_imu(self, imu: P.ImuFrame) -> None:
        """发布 ``sensor_msgs/Imu``（时间戳用 ROS 系统时钟）。"""
        msg = Imu()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_link'
        q = imu.quat_wxyz
        msg.orientation.w = float(q[0])
        msg.orientation.x = float(q[1])
        msg.orientation.y = float(q[2])
        msg.orientation.z = float(q[3])
        msg.angular_velocity.x = float(imu.gyro[0])
        msg.angular_velocity.y = float(imu.gyro[1])
        msg.angular_velocity.z = float(imu.gyro[2])
        msg.linear_acceleration.x = float(imu.accel[0])
        msg.linear_acceleration.y = float(imu.accel[1])
        msg.linear_acceleration.z = float(imu.accel[2])
        self._pub_imu.publish(msg)

    def _publish_state(self, imu: P.ImuFrame) -> None:
        """发布 ``GimbalState``（协议域 yaw/pitch/关节角速度）与 TF。"""
        R = FR.quat_to_mat(imu.quat_wxyz)
        yaw, pitch = FR.decompose_yaw_pitch(R)
        omega_pitch = float(imu.gyro[1])
        omega_yaw = float(imu.gyro[2]) / max(math.cos(pitch), 0.1)

        # 越界 / 失控判据
        self._lsm.check_limits(yaw, pitch)
        self._lsm.check_runaway(
            float(np.linalg.norm(imu.gyro)),
            math.hypot(self._cmd_yaw if self._cmd_valid else 0.0,
                       self._cmd_pitch if self._cmd_valid else 0.0))

        msg = GimbalState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_link'
        msg.yaw = float(yaw)
        msg.pitch = float(pitch)
        msg.omega_yaw = omega_yaw
        msg.omega_pitch = omega_pitch
        msg.gyro = [float(imu.gyro[0]), float(imu.gyro[1]), float(imu.gyro[2])]
        msg.accel = [float(imu.accel[0]), float(imu.accel[1]), float(imu.accel[2])]
        msg.quat = [float(imu.quat_wxyz[0]), float(imu.quat_wxyz[1]),
                    float(imu.quat_wxyz[2]), float(imu.quat_wxyz[3])]
        msg.link_state = int(self._lsm.state_id)
        msg.enabled = bool(self._lsm.enabled)
        stats = self._parser.stats()
        msg.frames_ok = int(stats['frames_ok'])
        msg.frames_bad = int(stats['frames_bad'])
        msg.seq_last = int(imu.seq)
        self._pub_state.publish(msg)

        if self._tf_broadcaster is not None:
            self._publish_tf(imu.quat_wxyz)

    def _publish_tf(self, quat_wxyz) -> None:
        """发布 TF ``world → gimbal_link``（**本机是唯一发布者**）。"""
        tf = TransformStamped()
        tf.header.stamp = self._node.get_clock().now().to_msg()
        tf.header.frame_id = 'world'
        tf.child_frame_id = 'gimbal_link'
        tf.transform.translation.x = float(self._origin_xyz[0])
        tf.transform.translation.y = float(self._origin_xyz[1])
        tf.transform.translation.z = float(self._origin_xyz[2])
        tf.transform.rotation.w = float(quat_wxyz[0])
        tf.transform.rotation.x = float(quat_wxyz[1])
        tf.transform.rotation.y = float(quat_wxyz[2])
        tf.transform.rotation.z = float(quat_wxyz[3])
        self._tf_broadcaster.sendTransform(tf)

    # ------------------------------------------------------------------ 下发
    def _tx_tick(self) -> None:
        """100Hz 下发 ``0x30``；ACTIVE 且到期才发（首帧前一律不发）。"""
        if self._lsm.state == SHUTDOWN:
            if self._lsm.take_zero_frame():
                self._write_frame(P.encode_omega(0.0, 0.0))
            return
        if not self._lsm.cmd_due():
            return
        fresh = (TU.now() - self._cmd_t_mono) <= float(self._params['serial.cmd_fresh_s'])
        if self._cmd_valid and fresh:
            frame = P.encode_omega(self._cmd_yaw, self._cmd_pitch,
                                   omega_max=float(self._params['serial.omega_max']))
        else:
            frame = P.encode_omega(0.0, 0.0,
                                   omega_max=float(self._params['serial.omega_max']))
        self._write_frame(frame)

    def _write_frame(self, frame: bytes) -> None:
        """写串口；失败进故障静默。"""
        try:
            self._port.write(frame)
        except SerialPortError as exc:  # 真实串口故障需进入静默停止状态，避免继续下发旧速度。
            throttled(self._log, 'error', str(exc), 'write_fail')
            self._lsm.on_write_failure()

    # ------------------------------------------------------------------ 订阅
    def _on_cmd_omega(self, msg: GimbalOmegaCmd) -> None:
        """缓存最新角速度指令（协议域）。

        外部话题时间戳**只用于日志**，一律用本地单调时刻做新鲜度判定。
        """
        if msg is None:
            return
        if not (math.isfinite(float(msg.yaw_omega)) and math.isfinite(float(msg.pitch_omega))):
            self._cmd_valid = False
            self._cmd_yaw = 0.0
            self._cmd_pitch = 0.0
            return
        self._cmd_yaw = float(msg.yaw_omega)
        self._cmd_pitch = float(msg.pitch_omega)
        self._cmd_valid = bool(msg.valid)
        self._cmd_t_mono = TU.now()

    def _on_status(self, msg: String) -> None:
        """监控 ``/virtual_gimbal/status``（JSON）：``sim_skips != 0`` 记 ERROR。"""
        if msg is None or not msg.data:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            # The simulator publishes external JSON; malformed status is discarded.
            throttled(self._log, 'warning', f'status 非 JSON: {msg.data[:64]!r}', 'status_json')
            return
        if not isinstance(data, dict):
            throttled(self._log, 'warning', 'status 必须是JSON对象', 'status_type')
            return
        state = str(data.get('state', ''))
        if 'fault' in state.lower() or 'relax' in state.lower():
            throttled(self._log, 'warning', f'下位机状态异常: {state}', 'status_state')
        try:
            skips = int(data.get('sim_skips', 0))
        except (TypeError, ValueError):
            # An invalid external counter is unknown, not proof of zero skipped frames.
            throttled(self._log, 'warning', 'status的sim_skips不是有效整数', 'status_skips')
            return
        if skips != 0:
            self._log.error(f'sim_skips={skips} ≠ 0 —— 本次评估不可信（仿真跳帧）')
            throttled(self._log, 'error', f'sim_skips={skips} ≠ 0，本次评估不可信',
                         'sim_skips')

    # ------------------------------------------------------------------ 辅助
    def _absorb_bad_frames(self) -> None:
        """把 parser 新增的校验错帧喂给状态机（1s 内 >50 进故障）。"""
        bad = int(self._parser.stats()['frames_bad'])
        delta = bad - self._frames_bad_prev
        self._frames_bad_prev = bad
        for _ in range(max(0, delta)):
            self._lsm.on_bad_frame()

    def _check_imu_watchdog(self) -> None:
        """ACTIVE 后长时间没有 IMU → WARN（不进故障，避免误伤）。"""
        if not self._lsm.imu_seen or self._lsm.state == SHUTDOWN:
            return
        if self._t_last_imu_mono is None:
            return
        age = TU.now() - self._t_last_imu_mono
        if age > float(self._params['serial.imu_timeout_s']):
            throttled(self._log, 'warning',
                         f'已 {age:.2f}s 未收到 0x01（下位机可能停止发帧）', 'imu_timeout')

    def _log_state_change(self) -> None:
        """状态迁移必打日志（WARN for fault，INFO for 恢复）。"""
        cur = self._lsm.state
        if cur == self._last_state:
            return
        if cur == FAULT_SILENT:
            self._log.warning(f'链路故障：{self._last_state} -> {FAULT_SILENT} '
                              f'（reason={self._lsm.last_fault_reason}），静默 '
                              f'{self._params["serial.fault_silence_s"]}s')
        elif cur == ACTIVE and self._last_state in (FAULT_SILENT, IDLE):
            self._log.info(f'链路使能/恢复：{self._last_state} -> {ACTIVE}')
        elif cur == SHUTDOWN:
            self._log.info(f'链路关闭：{self._last_state} -> {SHUTDOWN}')
        self._last_state = cur


def main(args=None) -> int:
    """入口：建节点 → 开串口（失败 FATAL）→ spin → 收尾。"""
    rclpy.init(args=args)
    node = NU.create_node(NODE_NAME)
    bridge = VgimbalBridgeNode(node)
    if not bridge.start():
        bridge.shutdown()
        node.destroy_node()
        NU.shutdown()
        # 硬约束：串口打开失败 FATAL 退出，不静默重试
        return 1
    try:
        NU.spin_with_finally(node, on_shutdown=bridge.shutdown)
    finally:
        node.destroy_node()
        NU.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
