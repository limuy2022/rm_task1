"""ROS adapter for the pure registered-cloud mapper. No drivers, TF or commands."""
from __future__ import annotations
from collections import deque
import json
import math
import os
import re
import tempfile
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from gimbal_common.qos_utils import latched_qos, sensor_qos
from .cloud_grid import CloudGrid, PoseHistory, pointcloud2_xyz
from .pgm_io import write_map


def stamp_seconds(stamp):
    return float(stamp.sec)+float(stamp.nanosec)*1e-9


class CloudMappingNode(Node):
    def __init__(self):
        super().__init__('cloud_mapping_node')
        defaults = {
            'cloud_topic':'/red_standard_robot1/cloud_registered',
            'odom_topic':'/red_standard_robot1/aft_mapped_to_init',
            'source_frame':'camera_init', 'pose_child_frame':'body',
            'sensor_origin_in_body':[0.,0.,0.], 'origin_model_confirmed':False,
            'resolution':0.05, 'width_m':20., 'height_m':20., 'origin_xy':[-10.,-10.],
            'floor_z':0., 'floor_confirmed':False, 'obstacle_min':0.15,
            'obstacle_max':1.8, 'ground_tolerance':0.08, 'min_range':0.5,
            'max_range':20., 'angle_bin_deg':0.5, 'pose_max_delta':0.04,
            'max_cloud_age':0.5, 'publish_hz':1., 'autosave_seconds':10.,
            'output_dir':'~/ws_championship/output/team47_mapping',
            'file_prefix':'47_slam_map',
        }
        for key,value in defaults.items(): self.declare_parameter(key,value)
        get = lambda key: self.get_parameter(key).value
        self.frame = str(get('source_frame'))
        if not self.frame or not str(get('pose_child_frame')):
            raise ValueError('source_frame/pose_child_frame cannot be empty')
        self.origin_model_confirmed = bool(get('origin_model_confirmed'))
        self.mapper = CloudGrid(**{k:get(k) for k in (
            'resolution','width_m','height_m','origin_xy','floor_z','floor_confirmed',
            'obstacle_min','obstacle_max','ground_tolerance','min_range','max_range','angle_bin_deg')})
        self.poses = PoseHistory(self.frame,str(get('pose_child_frame')),
                                 get('pose_max_delta'),get('sensor_origin_in_body'))
        self.max_age = float(get('max_cloud_age'))
        self.output = os.path.abspath(os.path.expanduser(str(get('output_dir'))))
        self.prefix = str(get('file_prefix'))
        self.autosave = float(get('autosave_seconds'))
        hz = float(get('publish_hz'))
        if not re.fullmatch(r'[A-Za-z0-9_-]+',self.prefix): raise ValueError('invalid file_prefix')
        if not 0.05<=self.max_age<=2 or not 0.1<=hz<=10 or self.autosave<0:
            raise ValueError('invalid timing parameters')
        if self.autosave and self.autosave<5: raise ValueError('autosave_seconds must be zero or >=5')
        self.pending = deque(maxlen=3)
        self.last_received_stamp = None
        self.last_save = time.monotonic()
        self.last_saved_scan = 0
        self.last_status = {'state':'等待 Point-LIO 点云和位姿', 'frame_id':self.frame}
        self.rejected = 0
        self.last_warn = 0.
        self.map_pub = self.create_publisher(OccupancyGrid,'/team47/slam_map',latched_qos(1))
        self.status_pub = self.create_publisher(String,'/team47/cloud_mapping/status',10)
        self.create_subscription(PointCloud2,str(get('cloud_topic')),self.on_cloud,sensor_qos(5))
        self.create_subscription(Odometry,str(get('odom_topic')),self.on_odom,sensor_qos(5))
        self.create_service(Trigger,'/team47/save_slam_map',self.on_save)
        self.create_timer(1./hz,self.publish)
        self.get_logger().info('只建图：%s；光心由完整 body 位姿与 Point-LIO extrinsic_T 计算；不发布 TF/运动命令'%self.frame)

    def reject(self, reason):
        self.rejected += 1
        self.last_status = {'state':str(reason),'frame_id':self.frame,'rejected':self.rejected}
        if time.monotonic()-self.last_warn > 2:
            self.get_logger().warn(str(reason)); self.last_warn = time.monotonic()

    def on_odom(self,msg):
        try:
            p,q = msg.pose.pose.position,msg.pose.pose.orientation
            self.poses.add(stamp_seconds(msg.header.stamp),msg.header.frame_id,msg.child_frame_id,
                           [p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
            self.process_pending()
        except (ValueError,TypeError,OverflowError) as exc: self.reject(exc)

    def on_cloud(self,msg):
        try:
            stamp=stamp_seconds(msg.header.stamp)
            now=self.get_clock().now().nanoseconds*1e-9
            if msg.header.frame_id!=self.frame: raise ValueError('cloud frame mismatch; refusing frame relabel')
            if not math.isfinite(stamp) or stamp<=0 or now-stamp>self.max_age or stamp-now>0.05:
                raise ValueError('cloud stale/future/invalid timestamp')
            if self.last_received_stamp is not None and stamp<=self.last_received_stamp:
                raise ValueError('out-of-order cloud; restart mapper after replay reset')
            xyz=pointcloud2_xyz(msg)
            self.last_received_stamp=stamp
            if len(self.pending)==self.pending.maxlen: self.reject('点云等待匹配位姿队列已满，丢弃最旧帧')
            self.pending.append((stamp,xyz))
            self.process_pending()
        except (ValueError,TypeError,OverflowError) as exc: self.reject(exc)

    def process_pending(self):
        if not self.origin_model_confirmed:
            self.last_status={'state':'雷达光心模型尚未配置：使用 cloud_mapping.launch.py 读取 Point-LIO 配置',
                              'frame_id':self.frame}
            self.pending.clear(); return
        floor=float(self.get_parameter('floor_z').value)
        confirmed=bool(self.get_parameter('floor_confirmed').value)
        if not math.isfinite(floor): self.reject('floor_z must be finite'); return
        if self.mapper.accepted_scans and (floor!=self.mapper.floor_z or confirmed!=self.mapper.floor_confirmed):
            self.reject('已建图后不能改变地面高度；停止并重启建图节点'); return
        self.mapper.floor_z,self.mapper.floor_confirmed=floor,confirmed
        now=self.get_clock().now().nanoseconds*1e-9
        while self.pending:
            stamp,xyz=self.pending[0]
            if now-stamp>self.max_age:
                self.pending.popleft(); self.reject('点云过期：没有同时间戳的完整位姿'); continue
            origin=self.poses.origin(stamp)
            if origin is None:
                self.last_status={'state':'等待与点云时间相差不超过 %.3fs 的完整 body 位姿'%self.poses.max_delta,
                                  'frame_id':self.frame}; break
            self.pending.popleft()
            try:
                self.last_status={**self.mapper.update(xyz,origin,stamp),
                                  'frame_id':self.frame,'sensor_origin':origin.tolist(),
                                  'floor_z':floor,'rejected':self.rejected}
            except (ValueError,TypeError,OverflowError) as exc: self.reject(exc)

    def publish(self):
        self.process_pending()
        if self.mapper.accepted_scans:
            gm=self.mapper.map
            msg=OccupancyGrid(); msg.header.frame_id=self.frame
            stamp=float(self.mapper.last_stamp)
            msg.header.stamp.sec=int(stamp); msg.header.stamp.nanosec=int(round((stamp-int(stamp))*1e9))
            if msg.header.stamp.nanosec>=1000000000:
                msg.header.stamp.sec+=1; msg.header.stamp.nanosec=0
            msg.info.resolution=gm.resolution; msg.info.width=gm.width; msg.info.height=gm.height
            msg.info.origin.position.x=float(gm.origin_xy[0]); msg.info.origin.position.y=float(gm.origin_xy[1])
            msg.info.origin.position.z=float(self.mapper.floor_z)
            msg.info.origin.orientation.w=1.; msg.data=gm.to_occupancy_data()
            self.map_pub.publish(msg)
            if self.autosave and time.monotonic()-self.last_save>=self.autosave:
                try: self.save()
                except OSError as exc: self.reject('地图保存失败: '+str(exc))
        snapshot=dict(self.last_status)
        if self.mapper.last_stamp is not None:
            age=self.get_clock().now().nanoseconds*1e-9-self.mapper.last_stamp
            snapshot['last_map_cloud_age']=round(age,3)
            input_age=(self.get_clock().now().nanoseconds*1e-9-self.last_received_stamp
                       if self.last_received_stamp is not None else float('inf'))
            if input_age>self.max_age and not self.pending:
                snapshot['state']='点云已停止或过期，保留最后地图'
        status=String(); status.data=json.dumps(snapshot,ensure_ascii=False,allow_nan=False)
        self.status_pub.publish(status)

    def save(self):
        if not self.mapper.accepted_scans: raise ValueError('还没有经过地面确认的有效地图，拒绝保存空图')
        if self.last_saved_scan==self.mapper.accepted_scans: return self.output
        os.makedirs(self.output,exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.cloud-map-',dir=self.output) as tmp:
            paths=write_map(tmp,self.mapper.map,self.prefix+'.pgm',self.prefix+'.yaml')
            with open(paths['yaml'],'a',encoding='utf-8') as f:
                f.write('# Source frame retained; not yet competition start frame.\nframe_id: '+self.frame+'\n')
                # Standard map-server origin is [x, y, yaw], not xyz. Preserve
                # actual plane height separately for 3-D TF/republication.
                f.write('floor_z: %.12g\n' % self.mapper.floor_z)
            for path in paths.values(): os.replace(path,os.path.join(self.output,os.path.basename(path)))
        self.last_save=time.monotonic(); self.last_saved_scan=self.mapper.accepted_scans
        return self.output

    def on_save(self,_request,response):
        try: response.message=self.save(); response.success=True
        except (ValueError,OSError) as exc: response.message=str(exc); response.success=False
        return response


def main(args=None):
    rclpy.init(args=args)
    node=CloudMappingNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        try:
            if node.mapper.accepted_scans: node.save()
        finally:
            node.destroy_node(); rclpy.shutdown()
