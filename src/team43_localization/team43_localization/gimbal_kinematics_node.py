"""Calibrated IMU/base-derived joint estimates and a single camera TF chain.

No encoders are fabricated, no motor/serial output. Defaults deliberately cannot
publish until measured geometry, base mounting, joint zero and IMU epoch agree.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import UInt64
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from team43_common import frames as FR
from team43_common.log_utils import get_logger
from team43_common.qos_utils import latched_qos, sensor_qos
from team43_common.time_buffer import stamp_seconds
from .gimbal_kinematics import GimbalKinematics, SynchronizedKinematics, quaternion_rotation


DEFAULTS = {
    'kinematics.calibration_confirmed': False,
    'kinematics.require_confirmation': True,
    'kinematics.bind_current_epoch': False,
    'kinematics.align_world_at_start': False,
    'kinematics.startup_reference_joint_rad': [0., 0.],
    'kinematics.startup_alignment_ready': False,
    'kinematics.runtime_world_rotation': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
    'kinematics.joint_zero_known': False,
    'kinematics.mounted_on_base_confirmed': False,
    'kinematics.calibrated_epoch': -1,
    'kinematics.base_mount_xyz': [0.]*3, 'kinematics.base_mount_rpy': [0.]*3,
    'kinematics.yaw_to_pitch_xyz': [0.]*3,
    'kinematics.pitch_gimbal_xyz': [0.]*3, 'kinematics.pitch_gimbal_rpy': [0.]*3,
    'kinematics.gimbal_camera_xyz': [0.]*3, 'kinematics.gimbal_camera_rpy': [0.]*3,
    'kinematics.odom_world_rpy': [0.]*3,
    'kinematics.joint_zero_rad': [0.]*2,
    'kinematics.yaw_limits_rad': [-2.8, 2.8],
    'kinematics.pitch_limits_rad': [-.5, .35],
    # 10Hz位姿的100ms周期允许20ms调度抖动；超过120ms仍拒绝插值。
    'kinematics.max_age_s': .2, 'kinematics.max_gap_s': .12,
    'kinematics.imu_topic': '/red_standard_robot1/gimbal',
    'kinematics.odom_frame': 'odom', 'kinematics.base_frame': 'base_link',
}


class GimbalKinematicsNode(Node):
    def __init__(self):
        super().__init__('gimbal_kinematics_node')
        for name, value in DEFAULTS.items(): self.declare_parameter(name, value)
        self.cfg = {k.removeprefix('kinematics.'): self.get_parameter(k).value for k in DEFAULTS}
        c = self.cfg
        model = None
        if c['calibration_confirmed'] or not c['require_confirmation']:
            model = GimbalKinematics(c, require_confirmation=c['require_confirmation'])  # invalid/partial calibration is fatal, never identity
        self.sync = SynchronizedKinematics(model, calibrated_epoch=c['calibrated_epoch'],
                       max_age=c['max_age_s'], max_gap=c['max_gap_s'],
                       bind_current_epoch=c['bind_current_epoch'],
                       startup_reference=(c['startup_reference_joint_rad'] if c['align_world_at_start'] else None))
        self.alignment_reported = False
        self.dynamic = TransformBroadcaster(self); self.static = StaticTransformBroadcaster(self)
        self.sent_static = False; self.last_now = None
        best = sensor_qos(50)
        self.pub = self.create_publisher(JointState, '/gimbal/joint_states', best)
        self.create_subscription(Odometry, '/gimbal/base_pose_raw', self.on_base, best)
        self.create_subscription(Imu, c['imu_topic'], self.on_imu, best)
        self.create_subscription(UInt64, '/task1/gimbal_reset_epoch', self.on_epoch,
                                 latched_qos(1))
        self.create_timer(.01, self.tick)
        self._log = get_logger(self, 'kinematics'); self._log.info(f'关节角由配置几何与IMU解算；人工确认门槛={c["require_confirmation"]}，绑定当前epoch={c["bind_current_epoch"]}')

    def now_sec(self): return self.get_clock().now().nanoseconds*1e-9

    def on_epoch(self, msg):
        self.sync.observe_epoch(msg.data)
        if self.sync.fault:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('kinematics.startup_alignment_ready', value=False)])
            self._log.error(self.sync.last_error)

    def on_base(self, msg):
        try:
            if msg.header.frame_id != self.cfg['odom_frame'] or msg.child_frame_id != self.cfg['base_frame']: return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            T = FR.compose_T(quaternion_rotation([q.w, q.x, q.y, q.z]), [p.x, p.y, p.z])
            self.sync.add_base(stamp_seconds(msg.header.stamp), T, self.now_sec())
        # 外部姿态/标定值可能无效或无可达解；明确拒绝该样本，不发布伪造姿态。
        except (TypeError, ValueError): return

    def on_imu(self, msg):
        try:
            if msg.header.frame_id != 'gimbal_link' or msg.orientation_covariance[0] < 0: return
            q = msg.orientation
            self.sync.add_imu(stamp_seconds(msg.header.stamp),
                quaternion_rotation([q.w, q.x, q.y, q.z]), self.now_sec())
        # 外部姿态/标定值可能无效或无可达解；明确拒绝该样本，不发布伪造姿态。
        except (TypeError, ValueError): return

    @staticmethod
    def message(T, parent, child, time_msg):
        msg = TransformStamped(); msg.header.stamp = time_msg
        msg.header.frame_id = parent; msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, T[:3, 3])
        q = FR.quat_from_mat(T[:3, :3])
        msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z = map(float, q)
        return msg

    def tick(self):
        now = self.now_sec()
        if self.last_now is not None and now < self.last_now-.01:
            self.sync.fault = True; self.sync.base.clear(); self.sync.pending.clear()
        self.last_now = now
        samples = self.sync.drain(now)
        if self.sync.alignment_stamp is not None and not self.alignment_reported:
            from rclpy.parameter import Parameter
            rotation = self.sync.model.R_odom_world.reshape(-1).tolist()
            self.set_parameters([Parameter('kinematics.runtime_world_rotation', value=rotation),
                                 Parameter('kinematics.startup_alignment_ready', value=True)])
            self._log.info(f'按启动水平朝前参考完成世界对齐：stamp={self.sync.alignment_stamp:.6f}，'
                           f'epoch={self.sync.epoch}，R_odom_world={rotation}')
            self.alignment_reported = True
        for item in samples:
            from builtin_interfaces.msg import Time
            ns = round(item.stamp*1e9); time_msg = Time(sec=ns//1000000000, nanosec=ns%1000000000)
            chain = item.transforms
            if not self.sent_static:
                self.static.sendTransform([
                    self.message(chain.gimbal_camera, 'gimbal_link', 'camera_link', time_msg),
                    self.message(chain.camera_optical, 'camera_link', 'camera_optical_frame', time_msg)])
                self.sent_static = True
            self.dynamic.sendTransform(self.message(chain.base_gimbal, self.cfg['base_frame'], 'gimbal_link', time_msg))
            joints = JointState(); joints.header.stamp = time_msg; joints.header.frame_id = self.cfg['base_frame']
            joints.name = ['yaw', 'pitch']; joints.position = [item.joints.yaw, item.joints.pitch]
            # velocity/effort remain empty: they were not measured.
            self.pub.publish(joints)


def main(args=None):
    rclpy.init(args=args); node = GimbalKinematicsNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
