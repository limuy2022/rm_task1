"""底盘位姿提供节点（DESIGN §3.6 / §5.4 / §6.9）。

* 按 ``base_pose_source`` 选择 ``OdomTopicSource``（正式赛）或 ``MockSource``（仿真）。
* 50Hz 发布 ``/gimbal/base_pose_raw``（``nav_msgs/Odometry``，``frame_id=odom``，
  ``child_frame_id=base_link``）。
* ``publish_base_tf=true`` 时发布动态 TF ``odom→base_link``（**只能有一个发布者**）。

时钟：外部位姿和 TF 严格保留输入 stamp；模拟源才使用当前 ROS 时钟。
目标环境：Ubuntu 22.04 + ROS 2 Humble；离线测试不替代 ROS 通信验证。
"""

from __future__ import annotations

import math
import sys

import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from gimbal_common.log_utils import get_logger, throttled
from gimbal_common import node_utils as NU
from gimbal_common import time_utils as TU
from gimbal_common.qos_utils import latched_qos, sensor_qos

from .base_pose_source import make_base_pose_source
from .odometry_geometry import matrix_quaternion

NODE_NAME = 'base_pose_provider_node'

PARAM_DEFAULTS = {
    'localization.base_pose_source': 'mock',
    'localization.odom_topic': '/odom',
    'localization.odom_source': 'topic',
    'localization.odom_timeout_s': 0.5,
    'localization.odom_frame_id': 'odom',
    'localization.odom_child_frame_id': 'base_link',
    'localization.output_frame_id': 'odom',
    'localization.base_frame_id': 'base_link',
    'localization.sensor_in_base_xyz': [0.0, 0.0, 0.0],
    'localization.sensor_in_base_rpy': [0.0, 0.0, 0.0],
    'localization.sensor_extrinsics_confirmed': False,
    'localization.source_parent_in_output_xyz': [0.0, 0.0, 0.0],
    'localization.source_parent_in_output_rpy': [0.0, 0.0, 0.0],
    'localization.parent_transform_confirmed': False,
    'localization.odom_twist_source': 'message',
    'localization.odom_future_tolerance_s': 0.05,
    'localization.publish_base_tf': True,
    'localization.mock_motion': 'static',
    'localization.mock_origin_xy': [0.0, 0.0],
    'localization.mock_yaw0': 0.0,
    'localization.mock_amp_xy': [1.0, 0.5],
    'localization.mock_radius': 1.5,
    'localization.mock_freq': 0.1,
    'localization.base_pose_rate_hz': 50.0,
}


class BasePoseProviderNode:
    """底盘位姿提供节点。"""

    def __init__(self, node) -> None:
        self._node = node
        self._log = get_logger(node, 'base_pose')
        self._params = NU.declare_params(node, PARAM_DEFAULTS)
        p = self._params

        cfg = {key.removeprefix('localization.'): value for key, value in p.items()}
        self._source = make_base_pose_source(cfg)
        self._publish_tf: bool = bool(p['localization.publish_base_tf'])
        self._source_name: str = self._source.name()
        self._last_published_stamp = None
        self._last_replay_start = None

        qos = sensor_qos(10)
        self._pub = node.create_publisher(Odometry, '/gimbal/base_pose_raw', qos)
        self._tf = TransformBroadcaster(node) if self._publish_tf else None

        if self._source_name.startswith('odom'):
            node.create_subscription(Odometry, str(p['localization.odom_topic']),
                                     self._on_odom, qos)
        elif str(p['localization.odom_source']).lower() == 'tf':
            self._log.warning('odom_source=tf 为 P1 备选，当前 base_pose_source=mock，忽略')

        if self._source_name.startswith('odom'):
            from std_msgs.msg import Header
            node.create_subscription(Header, '/judge/replay_start', self._on_replay_start,
                                     latched_qos(1))
        rate = max(1.0, float(p['localization.base_pose_rate_hz']))
        node.create_timer(1.0 / rate, self._tick)
        self._log.info(f'底盘位姿源启动: {self._source_name}，发布 {rate:.0f}Hz，'
                       f'publish_base_tf={self._publish_tf}')

    # ------------------------------------------------------------------ 回调
    def _on_odom(self, msg: Odometry) -> None:
        """外部 ``/odom`` 回调（odom 模式）。本地时刻做新鲜度判定。"""
        try:
            self._source.on_odom(msg, t_mono=TU.now())
        except (TypeError, ValueError) as exc:
            # Reject invalid source pose/stamps; do not publish a fabricated pose.
            throttled(self._log, 'error', f'odom 数据无效: {exc!r}', 'odom_cb')

    def _on_replay_start(self, msg) -> None:
        key = (int(msg.stamp.sec), int(msg.stamp.nanosec))
        if key != self._last_replay_start:
            self._source.reset()
            self._last_published_stamp = None
            self._last_replay_start = key

    def _tick(self) -> None:
        """Publish each source sample once, at the source measurement time."""
        pose = self._source.poll(TU.now())
        if not pose.valid:
            throttled(self._log, 'warning',
                         f'底盘位姿无效（源={self._source_name}）', 'base_invalid')
            return
        sample = self._source.latest
        stamp = self._node.get_clock().now().to_msg()
        if sample is not None:
            stamp.sec, stamp.nanosec = sample.stamp_sec, sample.stamp_nanosec
            key = (sample.stamp_sec, sample.stamp_nanosec)
            if key == self._last_published_stamp:
                return
            age = self._node.get_clock().now().nanoseconds * 1e-9 - sample.stamp_seconds
            if age > float(self._params['localization.odom_timeout_s']) or age < -float(self._params['localization.odom_future_tolerance_s']):
                throttled(self._log, 'warning', f'底盘源时间戳不新鲜或时钟域不同: age={age:.3f}s',
                             'base_stamp')
                return
        else:
            key = None
        self._publish_odom(pose, stamp)
        if self._tf is not None:
            self._publish_tf_msg(pose, stamp)
        self._last_published_stamp = key

    def _full_pose(self, pose):
        sample = self._source.latest
        if sample is not None:
            return sample.T_output_base[:3, 3], matrix_quaternion(sample.T_output_base[:3, :3])
        return np.array([pose.x, pose.y, 0.0]), np.array([0., 0., math.sin(pose.yaw/2), math.cos(pose.yaw/2)])

    def _publish_odom(self, pose, stamp) -> None:
        """ROS twist is expressed in child_frame_id (base frame), per Odometry."""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._source.frame_id
        msg.child_frame_id = self._source.child_frame_id
        position, quaternion = self._full_pose(pose)
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, position)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = map(float, quaternion)
        sample = self._source.latest
        if sample is not None:
            linear, angular = sample.linear_base, sample.angular_base
            if not sample.velocity_valid:
                for i in range(6):
                    msg.twist.covariance[7*i] = 1e6
        else:
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            linear = [c*pose.vx+s*pose.vy, -s*pose.vx+c*pose.vy, 0.]
            angular = [0., 0., 0.]
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, linear)
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, angular)
        self._pub.publish(msg)

    def _publish_tf_msg(self, pose, stamp) -> None:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._source.frame_id
        tf.child_frame_id = self._source.child_frame_id
        position, quaternion = self._full_pose(pose)
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, position)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, quaternion)
        self._tf.sendTransform(tf)

    def shutdown(self) -> None:
        """收尾日志。"""
        self._log.info(f'底盘位姿节点退出（源={self._source_name}）')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = NU.create_node(NODE_NAME)
    app = BasePoseProviderNode(node)
    try:
        NU.spin_with_finally(node, on_shutdown=app.shutdown)
    finally:
        node.destroy_node()
        NU.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
