"""Freeze match map at the replay's measurement stamp, not callback arrival.

Only map->odom is broadcast. The optional IMU-world alignment is an internal
matrix topic; publishing both world->odom and map->odom would give odom two
parents. Source odometry retains its original stamp, full pose, and body twist.
"""
from __future__ import annotations
import copy
import math
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from gimbal_common import frames as FR
from gimbal_common import log_utils as LU
from gimbal_common import node_utils as NU
from gimbal_common import time_utils as TU
from gimbal_common.time_buffer import PoseHistory, stamp_seconds

NODE_NAME='map_frame_broadcaster_node'
PARAM_DEFAULTS={
    'localization.origin_mode':'judge_replay_start',
    'localization.origin_settle_s':.5,
    'localization.origin_timeout_s':5.,
    'localization.origin_max_gap_s':.25,
    'localization.odom_timeout_s':.5,
    'localization.output_frame_id':'odom',
    'localization.base_frame_id':'base_link',
    'localization.publish_map_odom':True,
    'localization.publish_world_odom':False,
    'localization.world_alignment_confirmed':False,
    'localization.world_odom_xyz_rpy':[0.]*6,
    'localization.base_pose_rate_hz':50.,
}

def _sensor_qos(depth=20):
    return QoSProfile(depth=depth,reliability=ReliabilityPolicy.BEST_EFFORT)

def _latched_qos(depth=1):
    return QoSProfile(depth=depth,reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)

def _xyz_rpy_to_T(v):
    v=np.asarray(v,float).reshape(6)
    if not np.all(np.isfinite(v)):raise ValueError('invalid world alignment')
    return FR.compose_T(FR.rpy_to_mat(v[3:]),v[:3])


class MapFrameBroadcasterNode:
    def __init__(self,node):
        self._node=node;self._log=LU.get_logger(node,'map_frame')
        self._params=NU.declare_params(node,PARAM_DEFAULTS);p=self._params
        self._origin_mode=str(p['localization.origin_mode'])
        if self._origin_mode not in ('judge_replay_start','base_init'):
            raise ValueError('origin_mode must be judge_replay_start or explicit base_init')
        self._history=PoseHistory(capacity=2000)
        self._frozen=False;self._T_map_odom=None;self._T_map_world=None
        self._origin_stamp=None;self._last_msg=None;self._last_published_stamp=None
        self._t_first_valid=None;self._n_pose=0;self._freeze_requested=False
        self._T_world_odom=_xyz_rpy_to_T(p['localization.world_odom_xyz_rpy'])
        self._world_confirmed=bool(p['localization.world_alignment_confirmed'])
        self._publish_map_odom=bool(p['localization.publish_map_odom'])
        self._pub_t=node.create_publisher(TransformStamped,'/gimbal/t_map_world',_latched_qos())
        self._pub_pose=node.create_publisher(Odometry,'/gimbal/base_pose',_sensor_qos())
        self._static_tf=StaticTransformBroadcaster(node)
        node.create_subscription(Odometry,'/gimbal/base_pose_raw',self._on_base_pose,_sensor_qos())
        if self._origin_mode=='judge_replay_start':
            node.create_subscription(Header,'/gimbal/origin_freeze',self._on_origin_freeze,_latched_qos())
            node.create_subscription(Header,'/judge/replay_start',self._on_origin_freeze,_latched_qos())
        node.create_timer(1/max(1.,float(p['localization.base_pose_rate_hz'])),self._tick)
        if p['localization.publish_world_odom']:
            self._log.warning('不发布world->odom：会与map->odom形成双父节点；对齐仅走内部话题')

    def _on_base_pose(self,msg):
        try:
            if (msg.header.frame_id != self._params['localization.output_frame_id']
                    or msg.child_frame_id != self._params['localization.base_frame_id']):
                raise ValueError('unexpected internal odometry frames')
            stamp=stamp_seconds(msg.header.stamp)
            q=msg.pose.pose.orientation;pos=msg.pose.pose.position
            quat=np.array([q.w,q.x,q.y,q.z],float)
            xyz=np.array([pos.x,pos.y,pos.z],float)
            if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(quat)) or np.linalg.norm(quat)<.5:
                raise ValueError('invalid base pose')
            T=FR.compose_T(FR.quat_to_mat(quat),xyz)
            self._history.put(stamp,T);self._n_pose+=1
            if self._last_msg is None or stamp >= stamp_seconds(self._last_msg.header.stamp):
                self._last_msg=copy.deepcopy(msg)
            now=TU.now()
            if self._t_first_valid is None:self._t_first_valid=now
            if self._origin_mode=='base_init' and self._origin_stamp is None:
                if now-self._t_first_valid >= float(self._params['localization.origin_settle_s']):
                    self._origin_stamp=stamp
            self._try_freeze()
        except (TypeError,ValueError) as exc:
            # Bad external pose/stamp must not freeze an invented origin.
            LU.throttled(self._log,'warning',f'底盘源拒绝: {exc}','base_invalid',1.)

    def _on_origin_freeze(self,msg):
        try:stamp=stamp_seconds(msg.stamp)
        # 坏的回放时间戳不能冻结比赛起点；拒绝该输入。
        except (TypeError,ValueError):return
        if self._origin_stamp is not None:
            if abs(self._origin_stamp-stamp)>1e-6:
                self._log.error('收到不同回放起点；本进程保留原轮次，请在新轮开始前重启输出/定位')
            return
        self._freeze_requested=True;self._origin_stamp=stamp
        self._try_freeze()

    def _try_freeze(self):
        if self._frozen or self._origin_stamp is None:return
        T=self._history.sample(self._origin_stamp,float(self._params['localization.origin_max_gap_s']))
        if T is None:
            LU.throttled(self._log,'warning','等待覆盖回放起点stamp的底盘位姿，禁止拿最新位置替代','origin',2.)
            return
        yaw=FR.yaw_of_mat(T[:3,:3])
        R=FR.rotz(-yaw)
        self._T_map_odom=FR.compose_T(R,-R@T[:3,3])
        self._T_map_world=self._T_map_odom@FR.invert_T(self._T_world_odom)
        self._frozen=True
        if self._publish_map_odom:
            self._send_static(self._T_map_odom,'map',str(self._params['localization.output_frame_id']))
        if self._world_confirmed:self._publish_latched()
        self._log.info(f'比赛原点按stamp={self._origin_stamp:.6f}冻结，位置={T[:3,3].tolist()}，yaw={yaw:.4f}')

    def _transform_message(self,T,parent,child):
        msg=TransformStamped();msg.header.stamp=self._node.get_clock().now().to_msg()
        msg.header.frame_id=parent;msg.child_frame_id=child
        msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=map(float,T[:3,3])
        q=FR.quat_from_mat(T[:3,:3])
        msg.transform.rotation.w,msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z=map(float,q)
        return msg

    def _send_static(self,T,parent,child):
        self._static_tf.sendTransform(self._transform_message(T,parent,child))

    def _publish_latched(self):
        if self._T_map_world is not None:
            self._pub_t.publish(self._transform_message(self._T_map_world,'map','world'))

    def _tick(self):
        self._try_freeze()
        if not self._frozen or self._last_msg is None:return
        src=self._last_msg;stamp=stamp_seconds(src.header.stamp)
        age=self._node.get_clock().now().nanoseconds*1e-9-stamp
        if age > float(self._params['localization.odom_timeout_s']) or age < -.03:return
        if self._last_published_stamp==stamp:return
        T=self._history.sample(stamp)
        if T is None:return
        result=self._T_map_odom@T
        msg=copy.deepcopy(src);msg.header.frame_id='map'
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,result[:3,3])
        q=FR.quat_from_mat(result[:3,:3])
        msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z=map(float,q)
        # Odometry twist remains expressed in child/base coordinates.
        # Pose covariance is expressed in the parent frame and must rotate.
        C=np.asarray(src.pose.covariance,float).reshape(6,6)
        J=np.zeros((6,6));J[:3,:3]=self._T_map_odom[:3,:3];J[3:,3:]=self._T_map_odom[:3,:3]
        msg.pose.covariance=list(map(float,(J@C@J.T).reshape(36)))
        self._pub_pose.publish(msg);self._last_published_stamp=stamp

    @property
    def frozen(self):return self._frozen
    @property
    def T_map_world(self):return None if self._T_map_world is None else self._T_map_world.copy()
    def shutdown(self):self._log.info(f'比赛系frozen={self._frozen}，有效底盘样本={self._n_pose}')


def main(args=None):
    rclpy.init(args=args);node=NU.create_node(NODE_NAME)
    app=MapFrameBroadcasterNode(node)
    NU.spin_with_finally(node,on_shutdown=app.shutdown)
    node.destroy_node();NU.shutdown();return 0
