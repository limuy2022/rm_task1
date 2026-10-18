# -*- coding: utf-8 -*-
"""相机方位闭环节点：``MarkerObservation → BearingControlLaw → GimbalOmegaCmd``。

默认全关（``control.bearing.enabled`` / ``geometry_verified`` 均为 False）。
这是审计 P0「控制ROS接入」的闭环入口：不依赖 map/LIO/world 对齐，只需要

- 新鲜的光学系 ArUco 观测（``/perception/marker_cam``，感知节点已完成 ID 过滤）
- 派生机械关节角（``/gimbal/joint_states``，gimbal_kinematics_node）
- 复位 epoch（``/task1/gimbal_reset_epoch``）

与旧 ``gimbal_controller_node``（world 系链）发布同一 ``/gimbal/cmd_omega``：
两者互斥，由 launch 保证同时只启动一个。每拍无论有效与否都发布一条消息，
使下游 CommandGate 的指令超时不触发；停止靠 ``valid=false`` + 固件看门狗。

实机入口与分步启用检查单见 ``gimbal_tracker_bringup/launch/bearing_hardware.launch.py``
文件头（上机前置、三步启动、四道安全开关）。运行时动态参数速查
（``ros2 param set /gimbal_bearing_node <参数> <值>``，无需重启）：

======================  =====================================================
control.bearing.enabled         总开关；关→开上升沿清 epoch fault（MCU复位后
                                的重新上膛动作就是关一下再开）
control.kp_yaw / kp_pitch       增益。★仿真实测 kp=6 在 0.5s 检测延迟下过冲3倍
                                丢码死锁；首动 0.5 起步，按实测延迟整定
control.bearing.max_age_s       观测最大龄（默认0.15s）；按实测检测端到端延迟
                                的 2 倍设置
control.omega_max_yaw/pitch     角速度上限（协议上限2 rad/s；首动压 0.5）
control.omega_slew              斜坡 rad/s²（默认15，与固件一致）
gimbal.yaw_limits/pitch_limits  软限位（机械关节角域，来自 joint_states；
                                绝不可用 IMU world 姿态替代）
perception.target_digit         目标 ID 运行时切换
======================  =====================================================

停止语义：观测丢失/超龄、关节角缺失或超龄、epoch 复位、相机旋转外参非零、
任一开关为 false → 每拍发布 ``valid=false`` 零指令（下游清斜坡，固件看门狗
在 0.5s 内停转、3.5s 松弛）。
"""

from __future__ import annotations

import math
import sys
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import UInt64

from gimbal_interfaces.msg import GimbalOmegaCmd, MarkerObservation

from gimbal_common import frames
from gimbal_common.log_utils import get_logger, throttled
from gimbal_common.qos_utils import latched_qos, sensor_qos
from gimbal_common.time_utils import now

from .bearing_supervisor import BearingSupervisor
from .control_law import (
    DEFAULT_FF_GAIN,
    DEFAULT_KP_PITCH,
    DEFAULT_KP_YAW,
    DEFAULT_LIMIT_MARGIN,
    DEFAULT_OMEGA_MAX_PITCH,
    DEFAULT_OMEGA_MAX_YAW,
    DEFAULT_OMEGA_SLEW,
    DEFAULT_PITCH_MAX,
    DEFAULT_PITCH_MIN,
    DEFAULT_YAW_MAX,
    DEFAULT_YAW_MIN,
    BearingControlLaw,
)

# --------------------------------------------------------------------------
# 话题名（与 gimbal_controller_node 单点定义保持一致）
# --------------------------------------------------------------------------
TOPIC_MARKER_CAM = '/perception/marker_cam'
TOPIC_JOINT_STATE = '/gimbal/joint_states'
TOPIC_CMD_OMEGA = '/gimbal/cmd_omega'
TOPIC_EPOCH = '/task1/gimbal_reset_epoch'

MARKER_FRAME_ID = 'camera_optical_frame'

QOS_SENSOR = sensor_qos(10)

QOS_LATCHED = latched_qos(1)


class GimbalBearingNode(Node):
    """相机方位闭环：唯一职责是把话题搬运进 BearingSupervisor 并发布指令。"""

    def __init__(self) -> None:
        super().__init__('gimbal_bearing_node')
        self._log = get_logger(self, 'gimbal_bearing')

        # ---------------------------------------------------------- 参数声明
        d = self.declare_parameter
        d('control.bearing.enabled', False)
        d('control.bearing.geometry_verified', False)
        d('control.bearing.max_age_s', 0.15)
        d('control.bearing.joint_timeout_s', 0.2)
        d('control.bearing.rate_hz', 50.0)
        d('control.joint_yaw_name', 'yaw')
        d('control.joint_pitch_name', 'pitch')
        d('control.camera_xyz_rpy', [0.0] * 6)
        d('control.kp_yaw', DEFAULT_KP_YAW)
        d('control.kp_pitch', DEFAULT_KP_PITCH)
        d('control.ff_gain', DEFAULT_FF_GAIN)
        d('control.omega_max_yaw', DEFAULT_OMEGA_MAX_YAW)
        d('control.omega_max_pitch', DEFAULT_OMEGA_MAX_PITCH)
        d('control.omega_slew', DEFAULT_OMEGA_SLEW)
        d('gimbal.yaw_limits', [DEFAULT_YAW_MIN, DEFAULT_YAW_MAX])
        d('gimbal.pitch_limits', [DEFAULT_PITCH_MIN, DEFAULT_PITCH_MAX])
        d('gimbal.limit_margin', DEFAULT_LIMIT_MARGIN)

        # pitch 符号的唯一入口在 gimbal_common.frames；与旧控制器同款透传。
        d('gimbal.pitch_sign', float(frames.PITCH_SIGN))
        frames.set_pitch_sign(float(self.get_parameter('gimbal.pitch_sign').value))

        p = self.get_parameter
        rate_hz = float(p('control.bearing.rate_hz').value)
        self._rate_hz = rate_hz if rate_hz > 0.0 else 50.0

        yaw_lim = [float(v) for v in p('gimbal.yaw_limits').value]
        pitch_lim = [float(v) for v in p('gimbal.pitch_limits').value]
        law = BearingControlLaw(
            kp_yaw=float(p('control.kp_yaw').value),
            kp_pitch=float(p('control.kp_pitch').value),
            ff_gain=float(p('control.ff_gain').value),
            omega_max_yaw=float(p('control.omega_max_yaw').value),
            omega_max_pitch=float(p('control.omega_max_pitch').value),
            omega_slew=float(p('control.omega_slew').value),
            yaw_min=yaw_lim[0], yaw_max=yaw_lim[1],
            pitch_min=pitch_lim[0], pitch_max=pitch_lim[1],
            limit_margin=float(p('gimbal.limit_margin').value),
        )
        camera_rpy = np.asarray(p('control.camera_xyz_rpy').value,
                                dtype=float).reshape(6)[3:6]
        self._sup = BearingSupervisor(
            law,
            max_age_s=float(p('control.bearing.max_age_s').value),
            joint_timeout_s=float(p('control.bearing.joint_timeout_s').value),
            camera_rpy=camera_rpy,
            default_dt_s=1.0 / self._rate_hz,
        )
        self._prev_enabled = bool(p('control.bearing.enabled').value)

        # ---------------------------------------------------------- 话题
        self._pub = self.create_publisher(GimbalOmegaCmd, TOPIC_CMD_OMEGA, QOS_SENSOR)
        self.create_subscription(MarkerObservation, TOPIC_MARKER_CAM,
                                 self.on_marker, QOS_SENSOR)
        self.create_subscription(JointState, TOPIC_JOINT_STATE,
                                 self.on_joint_state, QOS_SENSOR)
        self.create_subscription(UInt64, TOPIC_EPOCH, self.on_epoch, QOS_LATCHED)
        self.create_timer(1.0 / self._rate_hz, self.tick)

        self._t_start = now()
        self._log.info(
            'gimbal_bearing_node 启动 | rate=%.1fHz kp=(%.2f,%.2f) ff=%.2f '
            'omega_max=(%.2f,%.2f) slew=%.1f max_age=%.2fs joint_timeout=%.2fs '
            'enabled=%s geometry_verified=%s',
            self._rate_hz, law.kp_yaw, law.kp_pitch, law.ff_gain,
            law.omega_max_yaw, law.omega_max_pitch, law.omega_slew,
            self._sup.max_age_s, self._sup.joint_timeout_s,
            bool(p('control.bearing.enabled').value),
            bool(p('control.bearing.geometry_verified').value),
        )

    # ------------------------------------------------------------------ 时间
    def _source_time(self, msg, timeout: float) -> Optional[float]:
        """拒绝旧/未来/零 ROS stamp：到达本身不等于新鲜（与旧控制器同款）。"""
        try:
            stamp = msg.header.stamp
            source = float(stamp.sec) + 1e-9 * float(stamp.nanosec)
            ros_now = float(self.get_clock().now().nanoseconds) * 1e-9
            age = ros_now - source
            if source <= 0.0 or not math.isfinite(source) or not -0.03 <= age <= timeout:
                return None
            return source
        except (AttributeError, TypeError, ValueError):
            return None

    def _mono_of(self, source_t: float) -> float:
        """把 ROS 源时刻折算到单调时钟域：``now() - source_age``。"""
        ros_now = float(self.get_clock().now().nanoseconds) * 1e-9
        return now() - max(0.0, ros_now - source_t)

    # ------------------------------------------------------------------ 回调
    def on_marker(self, msg: MarkerObservation) -> None:
        try:
            source_t = self._source_time(msg, self._sup.max_age_s)
            if str(msg.header.frame_id) != MARKER_FRAME_ID:
                self._sup.on_marker(0.0, 0.0, 0.0, False, None)
                return
            t_msg = None if source_t is None else self._mono_of(source_t)
            marker_id = int(msg.marker_id)
            if self._sup.last_marker_id not in (None, marker_id):
                self._log.info('目标 ID 切换: %s → %d', self._sup.last_marker_id, marker_id)
            self._sup.on_marker(msg.x, msg.y, msg.z, bool(msg.valid),
                                t_msg, marker_id=marker_id)
        except Exception as exc:  # pragma: no cover - 回调异常不得中断回路
            throttled(self._log, 'error', 'on_marker 异常: %s' % exc, 'mk_exc', 1.0)
            self._sup.on_marker(0.0, 0.0, 0.0, False, None)

    def on_joint_state(self, msg: JointState) -> None:
        """只接受按名匹配、数值有限的机械关节角（与旧控制器同款）。"""
        try:
            source_t = self._source_time(msg, self._sup.joint_timeout_s)
            t_msg = None if source_t is None else self._mono_of(source_t)
            names = list(msg.name)
            yaw_name = str(self.get_parameter('control.joint_yaw_name').value)
            pitch_name = str(self.get_parameter('control.joint_pitch_name').value)
            if names.count(yaw_name) != 1 or names.count(pitch_name) != 1:
                self._sup.on_joints(0.0, 0.0, None)
                return
            yaw = float(msg.position[names.index(yaw_name)])
            pitch = float(msg.position[names.index(pitch_name)])
            self._sup.on_joints(yaw, pitch, t_msg)
        except (AttributeError, IndexError, TypeError, ValueError):
            self._sup.on_joints(0.0, 0.0, None)

    def on_epoch(self, msg: UInt64) -> None:
        try:
            if self._sup.on_epoch(int(msg.data)):
                self._log.error('云台 MCU 复位（epoch 变化）：输出已停止。'
                                '需将 control.bearing.enabled 关→开后重新上膛，'
                                '并按手册重新确认外参/零位。')
        except (TypeError, ValueError):
            return

    # ------------------------------------------------------------------ 定时
    def tick(self) -> None:
        try:
            p = self.get_parameter
            enabled = bool(p('control.bearing.enabled').value)
            if enabled and not self._prev_enabled:
                self._sup.rearm()
                self._log.info('control.bearing.enabled 上升沿：重新上膛（清 epoch fault）')
            self._prev_enabled = enabled
            self._sup.enabled = enabled
            self._sup.geometry_verified = bool(
                p('control.bearing.geometry_verified').value)

            decision = self._sup.tick(now())
            msg = GimbalOmegaCmd()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gimbal_link'
            msg.yaw_omega = decision.yaw_omega
            msg.pitch_omega = decision.pitch_omega
            msg.valid = decision.valid
            self._pub.publish(msg)
            if not decision.valid:
                throttled(self._log, 'warn',
                          '闭环停止: %s（marker_age=%.3fs）'
                          % (decision.reason, decision.marker_age_s),
                          'bearing_stop_' + decision.reason, 1.0)
        except Exception as exc:  # pragma: no cover - 定时器异常不得中断回路
            self._sup.law.reset()
            throttled(self._log, 'error', 'tick 异常: %s' % exc, 'tick_exc', 1.0)
            try:
                msg = GimbalOmegaCmd()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.yaw_omega = 0.0
                msg.pitch_omega = 0.0
                msg.valid = False
                self._pub.publish(msg)
            except Exception:
                pass

    # ------------------------------------------------------------------ 收尾
    def publish_stop(self) -> None:
        """退出前发一帧零指令（下游 CommandGate 清斜坡，固件看门狗停转）。"""
        try:
            msg = GimbalOmegaCmd()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.yaw_omega = 0.0
            msg.pitch_omega = 0.0
            msg.valid = False
            self._pub.publish(msg)
        except Exception:
            pass

    def shutdown_summary(self) -> None:
        self._log.info('gimbal_bearing_node 退出 | 运行 %.1fs, %s',
                       now() - self._t_start, self._sup.law.snapshot())


def main(args=None) -> int:
    """节点入口。"""
    rclpy.init(args=args)
    node = GimbalBearingNode()
    code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        code = 0
    except Exception as exc:  # pragma: no cover
        node._log.error('gimbal_bearing_node 异常退出: %s' % exc)
        code = 1
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
        try:
            node.shutdown_summary()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return code


if __name__ == '__main__':
    sys.exit(main())
