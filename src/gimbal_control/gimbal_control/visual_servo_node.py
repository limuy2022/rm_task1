"""Bench image PID: selected ArUco corners -> angular velocity, no world/TF/LIO.

No serial is opened here. Disabled means silence, not a zero-frame heartbeat.
Only a human on the NUC should enable motion after the dry-run checks.
"""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt64
from gimbal_interfaces.msg import MarkerCorners
from gimbal_common.config_loader import load_yaml
from gimbal_common.qos_utils import latched_qos, sensor_qos
from .visual_pid import AxisPID, ErrorConditioner


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def image_error(corners, camera):
    """Return yaw/pitch rad errors and pixel offsets from the optical axis."""
    xy = np.asarray(corners, dtype=float).reshape(4, 2)
    width, height = int(camera['width']), int(camera['height'])
    if (not np.all(np.isfinite(xy)) or np.any(xy < 0)
            or np.any(xy[:, 0] >= width) or np.any(xy[:, 1] >= height)):
        raise ValueError('invalid image corners')
    if not cv2.isContourConvex(xy.astype(np.float32).reshape(4, 1, 2)):
        raise ValueError('crossed/degenerate corners')
    center = xy.mean(axis=0)
    K = np.array([[camera['fx'], 0., camera['cx']],
                  [0., camera['fy'], camera['cy']], [0., 0., 1.]], dtype=float)
    normalized = cv2.undistortPoints(center.reshape(1, 1, 2), K,
                                    np.asarray(camera['distortion'], dtype=float))[0, 0]
    x, y = map(float, normalized)
    # Image right -> negative yaw (right); image down -> positive pitch.
    error = (-math.atan2(x, 1.), math.atan2(y, math.hypot(1., x)))
    if not all(math.isfinite(v) for v in error):
        raise ValueError('nonfinite visual error')
    return error, (float(center[0]-camera['cx']), float(center[1]-camera['cy']))


class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        defaults = {'camera_config': '', 'motion_enabled': False, 'axes': 'both',
                    'kp_yaw': 2., 'kp_pitch': 2., 'ki_yaw': 0., 'ki_pitch': 0.,
                    'kd_yaw': 0., 'kd_pitch': 0., 'max_omega': .25,
                    'slew': 1., 'yaw_sign': 1., 'pitch_sign': 1.,
                    'image_timeout': .15, 'imu_timeout': .15,
                    'run_seconds': 15., 'max_rotation': .35,
                    'imu_topic': '/red_standard_robot1/gimbal',
                    'control_hz': 50., 'filter_tau': .04, 'deadband_px': 2.,
                    'integral_limit': .1, 'derivative_tau': .05,
                    'loss_grace_s': .08, 'loss_grace_frames': 2}
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.camera = load_yaml(str(self.p('camera_config')))['camera']
        for key in ('width', 'height', 'fx', 'fy'):
            if not math.isfinite(float(self.camera[key])) or self.camera[key] <= 0:
                raise ValueError('invalid camera '+key)
        image_error([[1, 1], [3, 1], [3, 3], [1, 3]], self.camera)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'status', 10)
        sensor = sensor_qos(1)
        self.create_subscription(MarkerCorners, 'marker_corners', self.on_corners, sensor)
        self.create_subscription(Imu, str(self.p('imu_topic')), self.on_imu, sensor)
        self.create_subscription(UInt64, '/task1/gimbal_reset_epoch', self.on_epoch,
                                 latched_qos(1))
        self.pid = [AxisPID(), AxisPID()]; self.settings = None
        self.error_filter = ErrorConditioner(); self.filter_settings = None
        self.filtered_error = None; self.control_error = None
        self.observation = None; self.last_source = None; self.marker_id = None
        self.last_packet_source = None; self.lost_count = 0
        self.loss_velocity = (0., 0.); self.loss_start = None
        self.imu = None; self.epoch = None; self.armed = False; self.fault = ''
        self.arm_time = None; self.arm_q = None; self.arm_source = None
        self.desired = (0., 0.); self.output = (0., 0.)
        self.sent_motion = False; self.stop_frames = 0
        self.last_tick = time.monotonic(); self.last_status = 0.
        self.control_hz = 50.
        self.timer = self.create_timer(1./self.control_hz, self.tick)
        self.get_logger().info('图像PID就绪：motion_enabled=false时不发串口速度；不需要地图/底盘TF')

    def p(self, name): return self.get_parameter(name).value

    def age(self, source):
        return self.get_clock().now().nanoseconds * 1e-9 - source

    def reset_pid(self, reset_filter=True):
        for pid in self.pid: pid.reset()
        self.desired = (0., 0.)
        if reset_filter:
            self.error_filter.reset()
            self.filtered_error = None; self.control_error = None

    def invalidate_image(self):
        self.observation = None; self.lost_count = 0; self.loss_start = None
        self.loss_velocity = (0., 0.); self.reset_pid()

    def on_epoch(self, msg):
        if self.epoch is not None and msg.data != self.epoch and self.armed:
            self.fault = '开发板/串口桥已复位，先关闭再重新开启motion_enabled'
        self.epoch = msg.data

    def on_imu(self, msg):
        try:
            source = stamp_seconds(msg.header)
            q = np.array([msg.orientation.w, msg.orientation.x,
                          msg.orientation.y, msg.orientation.z], dtype=float)
            if (msg.header.frame_id != 'gimbal_link' or msg.orientation_covariance[0] < 0
                    or not -.03 <= self.age(source) <= float(self.p('imu_timeout'))
                    or not np.all(np.isfinite(q)) or np.linalg.norm(q) < .5):
                self.imu = None; return
            self.imu = (source, q / np.linalg.norm(q))
        except (TypeError, ValueError):
            # 外部IMU数值非法时清姿态；新鲜IMU门禁停止输出。
            self.imu = None

    def refresh_pid(self):
        settings = tuple(float(self.p(name)) for name in (
            'kp_yaw', 'ki_yaw', 'kd_yaw', 'kp_pitch', 'ki_pitch', 'kd_pitch',
            'max_omega', 'integral_limit', 'derivative_tau'))
        if not 0 < settings[6] <= 2.: raise ValueError('max_omega须在(0,2]')
        if settings != self.settings:
            self.pid = [AxisPID(*settings[:3], output_limit=settings[6],
                               integral_limit=settings[7], derivative_tau=settings[8]),
                        AxisPID(*settings[3:6], output_limit=settings[6],
                               integral_limit=settings[7], derivative_tau=settings[8])]
            self.settings = settings
            self.desired = (0., 0.)
        filter_settings = tuple(float(self.p(name)) for name in ('filter_tau', 'deadband_px'))
        if not 0 <= filter_settings[0] <= .5 or not 0 <= filter_settings[1] <= 50.:
            raise ValueError('filter_tau须0..0.5秒；deadband_px须0..50像素')
        if filter_settings != self.filter_settings:
            width = filter_settings[1]
            self.error_filter = ErrorConditioner(filter_settings[0],
                (math.atan(width/self.camera['fx']), math.atan(width/self.camera['fy'])))
            self.filter_settings = filter_settings
            self.reset_pid()

    def refresh_runtime(self):
        self.refresh_pid()
        hz = float(self.p('control_hz'))
        if not math.isfinite(hz) or not 20. <= hz <= 100.:
            raise ValueError('control_hz须20..100Hz')
        if hz != self.control_hz:
            if self.timer is not None: self.timer.timer_period_ns = round(1e9/hz)
            self.control_hz = hz
        grace = float(self.p('loss_grace_s')); frames = int(self.p('loss_grace_frames'))
        if not math.isfinite(grace) or not 0 <= grace <= .1 or not 0 <= frames <= 3:
            raise ValueError('loss_grace_s须0..0.1秒；loss_grace_frames须0..3')
        for name in ('image_timeout', 'imu_timeout'):
            value = float(self.p(name))
            if not math.isfinite(value) or not .02 <= value <= .3:
                raise ValueError(name+'须0.02..0.3秒')

    def on_corners(self, msg):
        try:
            source = stamp_seconds(msg.header)
            if (msg.marker_id < 0 or msg.header.frame_id != 'camera_optical_frame'
                    or not -.03 <= self.age(source) <= float(self.p('image_timeout'))):
                self.invalidate_image(); return
            if (msg.valid and len(msg.corners) != 4) or (not msg.valid and len(msg.corners) != 0):
                self.invalidate_image(); return
            if self.last_packet_source is not None and source <= self.last_packet_source:
                return  # re-delivery must not integrate the same image again
            self.last_packet_source = source
            if not msg.valid:
                if (len(msg.corners) == 0 and msg.marker_id == self.marker_id
                        and self.observation is not None):
                    if self.loss_start is None:
                        self.loss_start = time.monotonic(); self.loss_velocity = self.output
                    self.lost_count += 1
                    self.reset_pid(reset_filter=False)  # no integration without an observation
                else: self.invalidate_image()
                return
            if len(msg.corners) != 4: raise ValueError('expected four corners')
            error, pixels = image_error([(p.x, p.y) for p in msg.corners], self.camera)
            if msg.marker_id != self.marker_id or self.lost_count:
                self.reset_pid(); self.last_source = None
            self.refresh_pid()
            dt = 1./30 if self.last_source is None else source-self.last_source
            if dt > float(self.p('image_timeout')): self.reset_pid(); dt = 1./30
            self.last_source = source; self.marker_id = msg.marker_id
            self.observation = (source, error, pixels)
            self.lost_count = 0; self.loss_start = None
            self.filtered_error, self.control_error = self.error_filter.update(error, dt)
            if self.armed and source > self.arm_source:
                outputs = []
                for pid, e in zip(self.pid, self.control_error):
                    if e == 0.: pid.reset()  # no integral/derivative twitch inside dead zone
                    outputs.append(pid.update(e, dt))
                self.desired = tuple(outputs)
        except (TypeError, ValueError, cv2.error):
            # 畸形角点/投影错误属于无观测，必须清滤波和积分历史。
            self.invalidate_image()

    def send(self, yaw, pitch):
        msg = Twist(); msg.angular.z = float(yaw); msg.angular.y = float(pitch)
        self.pub.publish(msg)

    def stop(self):
        # A bounded zero burst requests stop; then silence lets MCU watchdog expire.
        if self.sent_motion:
            self.stop_frames = 3; self.sent_motion = False
        self.output = (0., 0.); self.reset_pid(reset_filter=False)
        if self.stop_frames:
            self.send(0., 0.); self.stop_frames -= 1

    def tick(self):
        now = time.monotonic(); dt = min(max(now-self.last_tick, 0.), .05); self.last_tick = now
        reason = '只看不动'
        try:
            self.refresh_runtime()
            enabled = bool(self.p('motion_enabled'))
            if not enabled:
                self.armed = False; self.fault = ''; self.arm_q = None
                self.stop()
            else:
                if not self.armed:
                    self.armed = True; self.arm_time = now; self.arm_q = None
                    self.arm_source = self.get_clock().now().nanoseconds * 1e-9
                    self.reset_pid(); self.fault = ''
                imu_ok = (self.imu is not None and self.imu[0] > self.arm_source
                          and -.03 <= self.age(self.imu[0]) <= float(self.p('imu_timeout')))
                if not imu_ok:
                    was_active = self.sent_motion or self.arm_q is not None
                    reason = '等待新云台IMU'; self.stop()
                    if was_active: self.fault = '云台IMU中断'
                elif self.fault:
                    reason = self.fault; self.stop()
                else:
                    if self.arm_q is None: self.arm_q = self.imu[1].copy()
                    run_seconds = float(self.p('run_seconds')); rotation = float(self.p('max_rotation'))
                    if not (1. <= run_seconds <= 180. and .02 <= rotation <= 1.):
                        raise ValueError('run_seconds须1..180；max_rotation须.02..1rad')
                    moved = 2.*math.acos(min(1., abs(float(self.arm_q @ self.imu[1]))))
                    if now-self.arm_time >= run_seconds: self.fault = '本次试转时间结束'
                    if moved >= rotation: self.fault = '本次转动达到角度范围'
                    fresh = self.observation is not None and -.03 <= self.age(self.observation[0]) <= float(self.p('image_timeout'))
                    grace = float(self.p('loss_grace_s'))
                    coast_age = self.age(self.observation[0]) if fresh else float('inf')
                    if self.loss_start is not None: coast_age = max(coast_age, now-self.loss_start)
                    coast = (self.lost_count > 0 and 0 < self.lost_count <= int(self.p('loss_grace_frames'))
                             and grace > 0 and 0 <= coast_age < grace)
                    if self.fault or not fresh or self.observation[0] <= self.arm_source or (self.lost_count and not coast):
                        reason = self.fault or '等待指定码的新图像'; self.stop()
                    else:
                        self.refresh_pid()
                        axes = str(self.p('axes'))
                        if axes not in ('yaw', 'pitch', 'both'): raise ValueError('axes须yaw/pitch/both')
                        signs = (float(self.p('yaw_sign')), float(self.p('pitch_sign')))
                        if any(s not in (-1., 1.) for s in signs): raise ValueError('sign须+1或-1')
                        slew = float(self.p('slew'))
                        if not math.isfinite(slew) or not 0 < slew <= 15.: raise ValueError('slew须在(0,15]')
                        wanted = ([v*(1.-coast_age/grace) for v in self.loss_velocity] if coast else
                                  [v*s for v, s in zip(self.desired, signs)])
                        if axes == 'yaw': wanted[1] = 0.
                        if axes == 'pitch': wanted[0] = 0.
                        bound = float(self.p('max_omega'))
                        # Lost-target decay is a stop path: do not let the slew limit
                        # prolong it or increase an already decayed command.
                        self.output = tuple(max(-bound, min(bound, v if coast else max(old-slew*dt, min(old+slew*dt, v))))
                                            for old, v in zip(self.output, wanted))
                        self.send(*self.output); self.sent_motion = True
                        self.stop_frames = 0; reason = '丢失衰减' if coast else '跟踪中'
            if now-self.last_status >= .9/self.control_hz:
                status = {'state': reason, 'id': self.marker_id, 'yaw_omega': self.output[0],
                          'pitch_omega': self.output[1], 'fault': self.fault,
                          'status_stamp': self.get_clock().now().nanoseconds*1e-9,
                          'image_stamp': self.observation[0] if self.observation else None,
                          'lost_count': self.lost_count, 'control_hz': self.control_hz,
                          'filtered_error_deg': None if self.filtered_error is None else [math.degrees(e) for e in self.filtered_error],
                          'control_error_deg': None if self.control_error is None else [math.degrees(e) for e in self.control_error]}
                if self.observation:
                    status['error_px'] = self.observation[2]
                    status['error_deg'] = [math.degrees(e) for e in self.observation[1]]
                message = String(); message.data = json.dumps(status, ensure_ascii=False)
                self.status_pub.publish(message); self.last_status = now
        except (TypeError, ValueError, OverflowError) as exc:
            # 运行时参数来自ROS服务；非法值锁故障并停止，不沿用旧输出。
            self.fault = str(exc); self.stop()
            self.get_logger().error('图像PID已停止：'+str(exc))

    def shutdown_stop(self):
        if self.sent_motion or self.stop_frames:
            for _ in range(3): self.send(0., 0.)


def main(args=None):
    rclpy.init(args=args); node = VisualServoNode()
    try: rclpy.spin(node)
    # Ctrl+C 是正常停机请求；finally 仍执行停止/关闭资源。
    except KeyboardInterrupt: pass
    finally:
        node.shutdown_stop(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
