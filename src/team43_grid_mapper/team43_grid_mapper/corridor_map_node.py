"""从下午当前点云放置平行墙示意地图；失败或缺图不阻塞评分链。"""
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from team43_common.qos_utils import latched_qos, sensor_qos
from .cloud_grid import pointcloud2_xyz
from .corridor import fit_parallel_walls, corridor_grid


class CorridorMapNode(Node):
    def __init__(self):
        super().__init__('corridor_map_node')
        for k,v in {'floor_z':0.,'width_m':2.5,'run_dir':''}.items():
            self.declare_parameter(k,v)
        self.floor = float(self.get_parameter('floor_z').value)
        self.width = float(self.get_parameter('width_m').value)
        self.run_dir = Path(self.get_parameter('run_dir').value)
        self.cloud = None;self.previous = None;self.stable = 0;self.done = False
        self.last_stamp = None
        self.pub = self.create_publisher(OccupancyGrid,'/team43/slam_map',latched_qos(1))
        self.status = self.create_publisher(String,'/team43/corridor/status',latched_qos(1))
        self.create_subscription(PointCloud2,'/red_standard_robot1/cloud_registered',self.on_cloud,sensor_qos(2))
        self.create_timer(.5,self.tick)
        self.emit('简化地图可选：等待当前侧墙点云；CSV与建议速度不等待地图')

    def emit(self,text,**values):
        msg = String();msg.data = json.dumps({'state':text,**values},ensure_ascii=False)
        self.status.publish(msg);self.get_logger().info(msg.data)

    def on_cloud(self,msg):
        if not self.done and msg.header.frame_id == 'lio_world':
            self.cloud = msg

    def tick(self):
        if self.done or self.cloud is None:
            return
        stamp = self.cloud.header.stamp
        key = (stamp.sec,stamp.nanosec)
        age = self.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9
        if key == self.last_stamp or not 0 <= age < 1.:
            return
        self.last_stamp = key
        try:
            xyz = pointcloud2_xyz(self.cloud)
            points = xyz[(xyz[:,2]>self.floor+.15)&(xyz[:,2]<self.floor+1.8),:2]
            fit = fit_parallel_walls(points,expected_width=self.width,min_length=1.5)
            if self.previous is not None:
                delta = (fit['yaw_rad']-self.previous['yaw_rad']+math.pi/2)%math.pi-math.pi/2
                same = (abs(delta)<math.radians(2) and
                        np.max(np.abs(np.array(fit['normal_offsets_m'])-self.previous['normal_offsets_m']))<.15)
            else:
                same = False
            self.stable = self.stable+1 if same else 1
            self.previous = fit
            if self.stable < 3:
                return
            grid,origin = corridor_grid(fit)
            msg = OccupancyGrid();msg.header = self.cloud.header
            msg.info.map_load_time = stamp;msg.info.resolution = .05
            msg.info.width,msg.info.height = grid.shape
            msg.info.origin.position.x,msg.info.origin.position.y = origin[:2]
            msg.info.origin.orientation.z = math.sin(origin[2]/2)
            msg.info.origin.orientation.w = math.cos(origin[2]/2)
            msg.data = grid.T.ravel().tolist()
            self.pub.publish(msg);self.done = True
            self.run_dir.mkdir(parents=True,exist_ok=True)
            (self.run_dir/'corridor_model.json').write_text(json.dumps(
                {**fit,'display_length_m':40.,'not_measured_endpoints':True,
                 'used_for_localization':False,'used_for_advice':False},ensure_ascii=False,indent=2)+'\n')
            self.emit('平行墙示意图已发布；无端墙，不参与定位或建议速度',**fit)
        except (ValueError,TypeError,OverflowError) as exc:
            self.previous = None;self.stable = 0
            self.emit('暂未形成两侧墙示意图；评分链继续运行',reason=str(exc))


def main(args=None):
    rclpy.init(args=args);node = CorridorMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
