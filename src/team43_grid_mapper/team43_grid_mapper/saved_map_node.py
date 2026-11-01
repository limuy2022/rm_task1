"""上午地图跨会话重定位；只发布地图，不改动LIO状态或电机控制。"""
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster
from team43_common import frames
from team43_common.qos_utils import latched_qos, sensor_qos
from .cloud_grid import pointcloud2_xyz
from .saved_map import SavedMap, rotation, angle


class SavedMapNode(Node):
    def __init__(self):
        super().__init__('saved_map_node')
        for k,v in {'map_yaml':'', 'floor_z':0., 'sensor_in_base_xyz':[0.,0.,0.],
                    'sensor_in_base_rpy':[0.,0.,0.], 'run_dir':''}.items():
            self.declare_parameter(k,v)
        get = lambda key:self.get_parameter(key).value
        self.saved = SavedMap(get('map_yaml'))
        self.floor = float(get('floor_z'))
        self.offset = np.asarray(get('sensor_in_base_xyz'), float)
        self.mount = frames.rpy_to_mat(get('sensor_in_base_rpy'))
        self.run_dir = Path(get('run_dir')).expanduser()
        self.odom = None; self.cloud = None; self.hint = None
        self.aligned = False; self.started = False; self.future = None
        self.generation = 0; self.previous = None; self.stable = 0; self.last_attempt = 0.
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.pub = self.create_publisher(OccupancyGrid,'/team43/slam_map',latched_qos(1))
        self.view = self.create_publisher(OccupancyGrid,'/team43/survey_map',latched_qos(1))
        self.status = self.create_publisher(String,'/team43/relocalization/status',latched_qos(1))
        self.tf = StaticTransformBroadcaster(self)
        self.source = self.map_message()
        self.view.publish(self.source)
        self.create_subscription(Odometry,'/red_standard_robot1/aft_mapped_to_init',self.on_odom,sensor_qos(5))
        self.create_subscription(PointCloud2,'/red_standard_robot1/cloud_registered',self.on_cloud,sensor_qos(2))
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.on_hint,10)
        self.create_subscription(Header,'/judge/replay_start',self.on_start,latched_qos(1))
        self.create_timer(.25,self.tick)
        self.emit('等待下午Point-LIO点云；重定位期间可先设置目标进行视觉跟踪')

    def emit(self, state, **values):
        msg = String();msg.data = json.dumps({'state':state,'aligned':self.aligned,**values},ensure_ascii=False)
        self.status.publish(msg)
        self.get_logger().info(msg.data)

    def map_message(self):
        msg = OccupancyGrid();msg.header.frame_id='survey_world'
        msg.header.stamp=self.get_clock().now().to_msg();msg.info.map_load_time=msg.header.stamp
        msg.info.resolution=self.saved.resolution
        msg.info.width,msg.info.height=self.saved.grid.shape
        msg.info.origin.position.x=float(self.saved.origin[0]);msg.info.origin.position.y=float(self.saved.origin[1])
        yaw=self.saved.origin[2]
        msg.info.origin.orientation.z=math.sin(yaw/2);msg.info.origin.orientation.w=math.cos(yaw/2)
        msg.data=self.saved.grid.T.ravel().tolist()
        return msg

    def on_odom(self,msg):
        if msg.header.frame_id=='lio_world' and msg.child_frame_id=='body':self.odom=msg

    def on_cloud(self,msg):
        if msg.header.frame_id=='lio_world':self.cloud=msg

    def on_start(self,msg):
        stamp=msg.stamp.sec+msg.stamp.nanosec*1e-9
        if stamp < self.startup_stamp:return
        self.started=True
        if not self.aligned:self.emit('已开赛但旧图尚未对齐：地图和建议速度可能缺失，应先完成重定位再通知裁判')

    @property
    def startup_stamp(self):
        return self.source.header.stamp.sec+self.source.header.stamp.nanosec*1e-9

    def base_pose(self):
        p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
        R=frames.quat_to_mat([q.w,q.x,q.y,q.z]) @ self.mount.T
        xyz=np.array([p.x,p.y,p.z])-R @ self.offset
        if not np.isfinite(xyz).all():raise ValueError('底盘中心无效')
        return xyz, math.atan2(R[1,0],R[0,0])

    def on_hint(self,msg):
        if self.started or self.aligned:
            self.emit('本轮对齐已固定；改变初始位置需在开赛前重启程序');return
        if msg.header.frame_id!='survey_world':
            self.emit('初始位姿必须在survey_world中：RViz Fixed Frame请选择survey_world');return
        try:
            if self.odom is None:raise ValueError('还没有下午底盘位姿')
            _,heading=self.base_pose()
            q=msg.pose.pose.orientation;R=frames.quat_to_mat([q.w,q.x,q.y,q.z])
            p=msg.pose.pose.position
            hint=np.array([p.x,p.y,angle(math.atan2(R[1,0],R[0,0])-heading)])
            if not np.isfinite(hint).all():raise ValueError('位置提示无效')
            self.hint=hint;self.generation+=1;self.previous=None;self.stable=0
            self.emit('收到初始位置提示：搜索底盘中心±1m、朝向±30度，仍需点云匹配验证')
        except ValueError as exc:self.emit(str(exc))

    def tick(self):
        if self.aligned:return
        if self.future is not None:
            if not self.future.done():return
            future=self.future;self.future=None
            try:result=future.result()
            except Exception as exc:
                self.emit('点云匹配失败',error=str(exc));return
            if self.job_generation!=self.generation:return
            pose=np.asarray(result['pose']);centre=self.job_centre
            trans=pose[:2]-rotation(pose[2]) @ centre[:2]
            transform=np.r_[trans,pose[2]]
            same=(self.previous is not None and np.linalg.norm(transform[:2]-self.previous[:2])<.2
                  and abs(angle(transform[2]-self.previous[2]))<.08)
            self.stable=(self.stable+1 if same else 1) if result['accepted'] else 0
            self.previous=transform if result['accepted'] else None
            self.emit('匹配候选' if result['accepted'] else '匹配不足或存在多解；在RViz指定大致位置后重试',
                      stable_samples=self.stable,**result)
            if self.stable>=3:self.bind(transform,result)
            return
        if self.cloud is None or self.odom is None or time.monotonic()-self.last_attempt<2:return
        now=self.get_clock().now().nanoseconds*1e-9
        cs=self.cloud.header.stamp.sec+self.cloud.header.stamp.nanosec*1e-9
        os=self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9
        if not 0<=now-cs<.5 or abs(cs-os)>.15:return
        try:
            centre,_=self.base_pose();xyz=pointcloud2_xyz(self.cloud)
            delta=xyz[:,:2]-centre[:2];distance=np.linalg.norm(delta,axis=1)
            keep=((xyz[:,2]>self.floor+.15)&(xyz[:,2]<self.floor+1.8)&
                  (distance>.55)&(distance<8)&np.isfinite(xyz).all(axis=1))
            points=delta[keep]
            if len(points)<40:return
            self.job_centre=centre.copy();self.job_generation=self.generation
            self.future=self.pool.submit(self.saved.match,points,None if self.hint is None else self.hint.copy())
            self.last_attempt=time.monotonic()
        except (ValueError,TypeError) as exc:self.emit('当前点云不能用于匹配',error=str(exc))

    def bind(self, transform, result):
        # T_survey_lio；下午LIO的全局原点不同。转换栅格origin，不重采样也不改格子值。
        t,yaw=transform[:2],float(transform[2]);R=rotation(yaw)
        output=copy.deepcopy(self.source);output.header.frame_id='lio_world'
        output.header.stamp=self.get_clock().now().to_msg()
        origin=R.T @ (self.saved.origin[:2]-t);a=angle(self.saved.origin[2]-yaw)
        output.info.origin.position.x=float(origin[0]);output.info.origin.position.y=float(origin[1])
        output.info.origin.orientation.z=math.sin(a/2);output.info.origin.orientation.w=math.cos(a/2)
        tf=TransformStamped();tf.header.frame_id='survey_world';tf.child_frame_id='lio_world'
        tf.header.stamp=output.header.stamp
        tf.transform.translation.x=float(t[0]);tf.transform.translation.y=float(t[1])
        tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2)
        # processing已经发布odom->lio_world，不能再给lio_world添加另一个父节点。
        # 改发lio_world->survey_world（逆变换），供RViz显示旧图与当前点云。
        tf.header.frame_id='lio_world';tf.child_frame_id='survey_world'
        inv=-R.T @ t;tf.transform.translation.x=float(inv[0]);tf.transform.translation.y=float(inv[1])
        tf.transform.rotation.z=math.sin(-yaw/2);tf.transform.rotation.w=math.cos(-yaw/2)
        self.tf.sendTransform(tf);self.pub.publish(output);self.aligned=True
        self.run_dir.mkdir(parents=True,exist_ok=True)
        (self.run_dir/'relocalization.json').write_text(json.dumps(
            {'map_yaml':str(self.saved.path),'T_survey_lio_xy_yaw':transform.tolist(),
             'matching':result,'model':'一次二维初始重定位＋下午独立LIO里程计；不是旧图持续定位'},
            ensure_ascii=False,indent=2)+'\n')
        self.emit('重定位完成；地图对齐已固定，可以检查叠图并通知裁判开赛',transform=transform.tolist())


def main(args=None):
    rclpy.init(args=args);node=SavedMapNode()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.pool.shutdown(wait=True);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
