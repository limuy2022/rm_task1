"""底盘位姿提供节点（DESIGN §3.6 / §5.4 / §6.9）。

* 按 ``base_pose_source`` 选择 ``OdomTopicSource``（正式赛）或 ``MockSource``（仿真）。
* 50Hz 发布 ``/gimbal/base_pose_raw``（``nav_msgs/Odometry``，``frame_id=odom``，
  ``child_frame_id=base_link``）。
* ``publish_base_tf=true`` 时发布动态 TF ``odom→base_link``（**只能有一个发布者**）。

⚠ 时钟：``header.stamp`` / TF 用 ROS 系统时钟；周期与新鲜度判定用单调时钟。
⚠ 本文件依赖 ROS 2，只能在 Ubuntu 24.04 + ROS 2 Jazzy 上编译/运行验证。
"""

from __future__ import annotations

import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_ros import TransformBroadcaster

from gimbal_common import log_utils as LU
from gimbal_common import node_utils as NU
from gimbal_common import time_utils as TU

from .base_pose_source import make_base_pose_source

NODE_NAME = 'base_pose_provider_node'

PARAM_DEFAULTS = {
    'localization.base_pose_source': 'mock',
    'localization.odom_topic': '/odom',
    'localization.odom_source': 'topic',
    'localization.odom_timeout_s': 0.5,
    'localization.publish_base_tf': True,
    'localization.mock_motion': 'static',
    'localization.mock_origin_xy': [0.0, 0.0],
    'localization.mock_yaw0': 0.0,
    'localization.mock_amp_xy': [1.0, 0.5],
    'localization.mock_radius': 1.5,
    'localization.mock_freq': 0.1,
    'localization.base_pose_rate_hz': 50.0,
}


def _sensor_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class BasePoseProviderNode:
    """底盘位姿提供节点。"""

    def __init__(self, node) -> None:
        self._node = node
        self._log = LU.get_logger(node, 'base_pose')
        self._params = NU.declare_params(node, PARAM_DEFAULTS)
        p = self._params

        cfg = {
            'base_pose_source': p['localization.base_pose_source'],
            'odom_topic': p['localization.odom_topic'],
            'odom_timeout_s': p['localization.odom_timeout_s'],
            'mock_motion': p['localization.mock_motion'],
            'mock_origin_xy': p['localization.mock_origin_xy'],
            'mock_yaw0': p['localization.mock_yaw0'],
            'mock_amp_xy': p['localization.mock_amp_xy'],
            'mock_radius': p['localization.mock_radius'],
            'mock_freq': p['localization.mock_freq'],
        }
        self._source = make_base_pose_source(cfg)
        self._publish_tf: bool = bool(p['localization.publish_base_tf'])
        self._source_name: str = self._source.name()

        qos = _sensor_qos(10)
        self._pub = node.create_publisher(Odometry, '/gimbal/base_pose_raw', qos)
        self._tf = TransformBroadcaster(node) if self._publish_tf else None

        if self._source_name.startswith('odom'):
            node.create_subscription(Odometry, str(p['localization.odom_topic']),
                                     self._on_odom, qos)
        elif str(p['localization.odom_source']).lower() == 'tf':
            self._log.warning('odom_source=tf 为 P1 备选，当前 base_pose_source=mock，忽略')

        rate = max(1.0, float(p['localization.base_pose_rate_hz']))
        node.create_timer(1.0 / rate, self._tick)
        self._log.info(f'底盘位姿源启动: {self._source_name}，发布 {rate:.0f}Hz，'
                       f'publish_base_tf={self._publish_tf}')

    # ------------------------------------------------------------------ 回调
    def _on_odom(self, msg: Odometry) -> None:
        """外部 ``/odom`` 回调（odom 模式）。本地时刻做新鲜度判定。"""
        try:
            self._source.on_odom(msg, t_mono=TU.now())
        except Exception as exc:
            LU.throttled(self._log, 'error', f'odom 回调异常: {exc!r}', 'odom_cb', 1.0)

    def _tick(self) -> None:
        """定时发布底盘位姿（+ 可选 TF）。"""
        try:
            t_mono = TU.now()
            pose = self._source.poll(t_mono)
            if not pose.valid:
                LU.throttled(self._log, 'warn',
                             f'底盘位姿无效（源={self._source_name}）', 'base_invalid', 1.0)
                return
            stamp = self._node.get_clock().now().to_msg()
            self._publish_odom(pose, stamp)
            if self._tf is not None:
                self._publish_tf_msg(pose, stamp)
        except Exception as exc:
            LU.throttled(self._log, 'error', f'base_pose tick 异常: {exc!r}', 'base_tick', 1.0)

    def _publish_odom(self, pose, stamp) -> None:
        """发布 ``/gimbal/base_pose_raw``。"""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._source.frame_id
        msg.child_frame_id = self._source.child_frame_id
        msg.pose.pose.position.x = float(pose.x)
        msg.pose.pose.position.y = float(pose.y)
        msg.pose.pose.position.z = 0.0
        cy = math.cos(0.5 * pose.yaw)
        sy = math.sin(0.5 * pose.yaw)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        msg.twist.twist.linear.x = float(pose.vx)
        msg.twist.twist.linear.y = float(pose.vy)
        msg.twist.twist.linear.z = 0.0
        self._pub.publish(msg)

    def _publish_tf_msg(self, pose, stamp) -> None:
        """发布动态 TF ``odom→base_link``。"""
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._source.frame_id
        tf.child_frame_id = self._source.child_frame_id
        tf.transform.translation.x = float(pose.x)
        tf.transform.translation.y = float(pose.y)
        tf.transform.translation.z = 0.0
        cy = math.cos(0.5 * pose.yaw)
        sy = math.sin(0.5 * pose.yaw)
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = sy
        tf.transform.rotation.w = cy
        try:
            self._tf.sendTransform(tf)
        except Exception as exc:
            LU.throttled(self._log, 'warning', f'TF odom->base_link 发布失败: {exc!r}',
                         'base_tf', 1.0)

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
